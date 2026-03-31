import gc
import math
import ml
import sensor
import struct
import time
import uos
import image
from pyb import LED, UART


MODEL_CANDIDATES = ("trained", "trained.tflite")
LABELS_PATH = "labels.txt"
UART_PORT = 3
UART_BAUDRATE = 115200
MIN_CONFIDENCE = 0.90
SEND_INTERVAL_MS = 1200
STARTUP_SUPPRESS_MS = 3000
REQUIRED_STABLE_FRAMES = 2
MAX_MISSED_FRAMES = 2
WINDOW_SIZE = (240, 240)

COLORS = [
    (255, 0, 0),
    (0, 255, 0),
    (255, 255, 0),
    (0, 0, 255),
    (255, 0, 255),
    (0, 255, 255),
    (255, 255, 255),
]

VOICE_LABELS = {
    "hammer": "hammer",
    "lighter": "lighter",
    "scissors": "scissors",
    "scissor": "scissors",
}

OBJECT_IDS = {
    "lighter": 0x01,
    "scissors": 0x02,
    "hammer": 0x03,
}


def load_model():
    last_error = None

    for name in MODEL_CANDIDATES:
        try:
            if name.endswith(".tflite"):
                load_to_fb = uos.stat(name)[6] > (gc.mem_free() - (64 * 1024))
                return ml.Model(name, load_to_fb=load_to_fb)
            return ml.Model(name)
        except Exception as exc:
            last_error = exc

    raise Exception("Failed to load model (%s)" % last_error)


def load_labels(model):
    try:
        return [line.strip() for line in open(LABELS_PATH) if line.strip()]
    except Exception:
        pass

    try:
        return list(model.labels)
    except Exception:
        return ["background", "hammer", "lighter", "scissors"]


def normalize_label(label_name):
    return VOICE_LABELS.get(label_name.lower(), None)


def send_detection(uart, label_name):
    object_id = OBJECT_IDS.get(label_name, 0x00)
    if object_id != 0x00:
        uart.write(struct.pack("BBBBBBBB", 0xBB, object_id, 0, 0, 0, 0, object_id, 0x55))


def raw_to_confidence(value, dtype_code, scale_q, zero_point):
    if dtype_code == "f":
        return float(value)
    return (float(value) - zero_point) * scale_q


def make_fomo_post_process(min_confidence):
    threshold_list = [(math.ceil(min_confidence * 255), 255)]

    def fomo_post_process(model, inputs, outputs):
        _, oh, ow, oc = model.output_shape[0]

        try:
            dtype_code = model.output_dtype[0]
        except Exception:
            dtype_code = "f"

        try:
            scale_q = model.output_scale[0]
        except Exception:
            scale_q = 1.0

        try:
            zero_point = model.output_zero_point[0]
        except Exception:
            zero_point = 0.0

        x_scale = inputs[0].roi[2] / ow
        y_scale = inputs[0].roi[3] / oh
        scale_xy = min(x_scale, y_scale)

        x_offset = ((inputs[0].roi[2] - (ow * scale_xy)) / 2) + inputs[0].roi[0]
        y_offset = ((inputs[0].roi[3] - (oh * scale_xy)) / 2) + inputs[0].roi[1]

        detections_by_class = [[] for _ in range(oc)]

        for i in range(oc):
            out_array = outputs[0][0, :, :, i]
            heatmap = image.Image(ow, oh, sensor.GRAYSCALE)

            for y in range(oh):
                for x in range(ow):
                    confidence = raw_to_confidence(
                        out_array[y][x], dtype_code, scale_q, zero_point
                    )

                    if confidence < 0.0:
                        confidence = 0.0
                    elif confidence > 1.0:
                        confidence = 1.0

                    heatmap.set_pixel(x, y, int(confidence * 255))

            blobs = heatmap.find_blobs(
                threshold_list,
                x_stride=1,
                y_stride=1,
                area_threshold=1,
                pixels_threshold=1,
            )

            for blob in blobs:
                rect = blob.rect()
                x, y, w, h = rect
                score = heatmap.get_statistics(
                    thresholds=threshold_list, roi=rect
                ).l_mean() / 255.0

                x = int((x * scale_xy) + x_offset)
                y = int((y * scale_xy) + y_offset)
                w = int(w * scale_xy)
                h = int(h * scale_xy)
                detections_by_class[i].append((x, y, w, h, score))

        return detections_by_class

    return fomo_post_process


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing(WINDOW_SIZE)
sensor.skip_frames(time=2000)

uart = UART(UART_PORT, UART_BAUDRATE, timeout_char=100)
led_r = LED(1)
led_g = LED(2)
led_b = LED(3)

model = load_model()
labels = load_labels(model)
fomo_post_process = make_fomo_post_process(MIN_CONFIDENCE)

clock = time.clock()
startup_time = time.ticks_ms()
last_send_time = time.ticks_add(startup_time, -SEND_INTERVAL_MS)
last_sent_label = None
candidate_label = None
candidate_count = 0
missed_count = 0

while True:
    clock.tick()
    img = sensor.snapshot()
    now = time.ticks_ms()
    best_label = None
    best_score = 0.0
    best_box = None
    best_color = None

    led_r.off()
    led_g.off()
    led_b.off()

    for i, detection_list in enumerate(model.predict([img], callback=fomo_post_process)):
        if i == 0:
            continue
        if not detection_list:
            continue

        if i < len(labels):
            raw_label = labels[i]
        else:
            raw_label = "class_%d" % i

        voice_label = normalize_label(raw_label)
        if voice_label is None:
            continue

        color = COLORS[i % len(COLORS)]

        for x, y, w, h, score in detection_list:
            if score < MIN_CONFIDENCE:
                continue

            if score > best_score:
                best_score = score
                best_label = voice_label
                best_box = (x, y, w, h)
                best_color = color

    if best_label is not None:
        if best_label == candidate_label:
            candidate_count += 1
        else:
            candidate_label = best_label
            candidate_count = 1

        missed_count = 0

        x, y, w, h = best_box
        center_x = math.floor(x + (w / 2))
        center_y = math.floor(y + (h / 2))

        img.draw_circle((center_x, center_y, 12), color=best_color, thickness=2)
        img.draw_string(
            center_x,
            center_y - 20,
            "%s: %.0f%%" % (best_label, best_score * 100),
            color=best_color,
            scale=2,
        )

        if best_label == "lighter":
            led_r.on()
        elif best_label == "scissors":
            led_g.on()
        elif best_label == "hammer":
            led_b.on()
    else:
        missed_count += 1
        if missed_count >= MAX_MISSED_FRAMES:
            candidate_label = None
            candidate_count = 0

    if candidate_label is None:
        continue

    if candidate_count < REQUIRED_STABLE_FRAMES:
        continue

    if time.ticks_diff(now, startup_time) < STARTUP_SUPPRESS_MS:
        continue

    if (candidate_label == last_sent_label) and (
        time.ticks_diff(now, last_send_time) < SEND_INTERVAL_MS
    ):
        continue

    send_detection(uart, candidate_label)
    last_sent_label = candidate_label
    last_send_time = now