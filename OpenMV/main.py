import gc
import math
import ml
import sensor
import struct
import time
import uos
import image
from pyb import LED, UART


MODEL_PATH = "trained.tflite"
LABELS_PATH = "labels.txt"
UART_PORT = 3
UART_BAUDRATE = 115200
MIN_CONFIDENCE = 0.90
CLASS_MIN_CONFIDENCE = {
    "hammer": 0.90,
    "lighter": 0.45,
    "scissors": 0.60,
}
SEND_INTERVAL_MS = 1200
STARTUP_SUPPRESS_MS = 3000
REQUIRED_STABLE_FRAMES = 2
MAX_MISSED_FRAMES = 2
DEBUG_SCORE_PRINT_INTERVAL_MS = 200
MIN_SCORE_GAP = 0.12
CLASS_SCORE_BIAS = {
    "hammer": 0.00,
    "lighter": 0.08,
    "scissors": 0.00,
}
CLASS_MIN_SCORE_GAP = {
    "hammer": 0.12,
    "lighter": 0.04,
    "scissors": 0.12,
}
CLASS_REQUIRED_STABLE_FRAMES = {
    "hammer": 2,
    "lighter": 1,
    "scissors": 1,
}
WINDOW_SIZE = (240, 240)
EXPECTED_LABELS = ("background", "hammer", "lighter", "scissors")
DETECTION_LABELS = ("hammer", "lighter", "scissors")

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


last_class_peak_scores = []
last_class_blob_scores = []


def load_model():
    try:
        load_to_fb = uos.stat(MODEL_PATH)[6] > (gc.mem_free() - (64 * 1024))
        return ml.Model(MODEL_PATH, load_to_fb=load_to_fb)
    except Exception as exc:
        raise Exception('Failed to load "%s" (%s)' % (MODEL_PATH, exc))


def load_labels(model):
    try:
        return [line.strip() for line in open(LABELS_PATH) if line.strip()]
    except Exception:
        pass

    try:
        return list(model.labels)
    except Exception:
        return list(EXPECTED_LABELS)


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


def get_label_score(labels, score_list, target_label):
    for idx, raw_label in enumerate(labels):
        if normalize_label(raw_label) == target_label:
            if idx < len(score_list):
                return score_list[idx]
            break
    return 0.0


def get_label_min_confidence(label_name):
    return CLASS_MIN_CONFIDENCE.get(label_name, MIN_CONFIDENCE)


def get_label_required_stable_frames(label_name):
    return CLASS_REQUIRED_STABLE_FRAMES.get(label_name, REQUIRED_STABLE_FRAMES)


def get_label_score_bias(label_name):
    return CLASS_SCORE_BIAS.get(label_name, 0.0)


def get_label_min_score_gap(label_name):
    return CLASS_MIN_SCORE_GAP.get(label_name, MIN_SCORE_GAP)


def make_fomo_post_process(min_confidence):
    threshold_list = [(math.ceil(min_confidence * 255), 255)]

    def fomo_post_process(model, inputs, outputs):
        global last_class_peak_scores
        global last_class_blob_scores

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
        last_class_peak_scores = [0.0 for _ in range(oc)]
        last_class_blob_scores = [0.0 for _ in range(oc)]

        for i in range(oc):
            out_array = outputs[0][0, :, :, i]
            heatmap = image.Image(ow, oh, sensor.GRAYSCALE)
            class_peak_score = 0.0

            for y in range(oh):
                for x in range(ow):
                    confidence = raw_to_confidence(
                        out_array[y][x], dtype_code, scale_q, zero_point
                    )

                    if confidence < 0.0:
                        confidence = 0.0
                    elif confidence > 1.0:
                        confidence = 1.0

                    if confidence > class_peak_score:
                        class_peak_score = confidence

                    heatmap.set_pixel(x, y, int(confidence * 255))

            last_class_peak_scores[i] = class_peak_score

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

                if score > last_class_blob_scores[i]:
                    last_class_blob_scores[i] = score

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
fomo_post_process = make_fomo_post_process(min(CLASS_MIN_CONFIDENCE.values()))

clock = time.clock()
startup_time = time.ticks_ms()
last_send_time = time.ticks_add(startup_time, -SEND_INTERVAL_MS)
last_debug_print_time = time.ticks_add(startup_time, -DEBUG_SCORE_PRINT_INTERVAL_MS)
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
    second_best_label = None
    second_best_score = 0.0
    best_adjusted_score = 0.0
    second_best_adjusted_score = 0.0
    best_box = None
    best_color = None
    frame_best_scores = {}
    frame_best_boxes = {}
    frame_best_colors = {}

    led_r.off()
    led_g.off()
    led_b.off()

    for label_name in DETECTION_LABELS:
        frame_best_scores[label_name] = 0.0
        frame_best_boxes[label_name] = None
        frame_best_colors[label_name] = None

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
            if score < get_label_min_confidence(voice_label):
                continue

            if score > frame_best_scores[voice_label]:
                frame_best_scores[voice_label] = score
                frame_best_boxes[voice_label] = (x, y, w, h)
                frame_best_colors[voice_label] = color

    for label_name in DETECTION_LABELS:
        score = frame_best_scores[label_name]
        box = frame_best_boxes[label_name]

        if box is None:
            continue

        adjusted_score = score + get_label_score_bias(label_name)

        if adjusted_score > best_adjusted_score:
            second_best_adjusted_score = best_adjusted_score
            second_best_score = best_score
            second_best_label = best_label
            best_adjusted_score = adjusted_score
            best_score = score
            best_label = label_name
            best_box = frame_best_boxes[label_name]
            best_color = frame_best_colors[label_name]
        elif adjusted_score > second_best_adjusted_score:
            second_best_adjusted_score = adjusted_score
            second_best_score = score
            second_best_label = label_name

    raw_best_label = best_label
    raw_best_score = best_score
    raw_best_adjusted_score = best_adjusted_score
    raw_second_best_label = second_best_label
    raw_second_best_score = second_best_score
    raw_second_best_adjusted_score = second_best_adjusted_score

    if (best_label is not None) and (
        (raw_best_adjusted_score - raw_second_best_adjusted_score) <
        get_label_min_score_gap(best_label)
    ):
        best_label = None
        best_score = 0.0
        best_box = None
        best_color = None

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

    if time.ticks_diff(now, last_debug_print_time) >= DEBUG_SCORE_PRINT_INTERVAL_MS:
        hammer_peak = get_label_score(labels, last_class_peak_scores, "hammer")
        lighter_peak = get_label_score(labels, last_class_peak_scores, "lighter")
        scissors_peak = get_label_score(labels, last_class_peak_scores, "scissors")
        hammer_blob = get_label_score(labels, last_class_blob_scores, "hammer")
        lighter_blob = get_label_score(labels, last_class_blob_scores, "lighter")
        scissors_blob = get_label_score(labels, last_class_blob_scores, "scissors")

        print(
            "peak h=%.3f l=%.3f s=%.3f | blob h=%.3f l=%.3f s=%.3f | top1=%s raw=%.3f adj=%.3f top2=%s raw=%.3f adj=%.3f gap=%.3f | best=%s %.3f | cand=%s %d"
            % (
                hammer_peak,
                lighter_peak,
                scissors_peak,
                hammer_blob,
                lighter_blob,
                scissors_blob,
                raw_best_label if raw_best_label is not None else "none",
                raw_best_score,
                raw_best_adjusted_score,
                raw_second_best_label if raw_second_best_label is not None else "none",
                raw_second_best_score,
                raw_second_best_adjusted_score,
                raw_best_adjusted_score - raw_second_best_adjusted_score,
                best_label if best_label is not None else "none",
                best_score,
                candidate_label if candidate_label is not None else "none",
                candidate_count,
            )
        )
        last_debug_print_time = now

    if candidate_label is None:
        continue

    if candidate_count < get_label_required_stable_frames(candidate_label):
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
