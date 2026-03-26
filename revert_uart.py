original_code = """import sensor, image, time, ml, math, uos, gc
import struct
from pyb import UART, LED

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((240, 240))
sensor.skip_frames(time=2000)

uart = UART(1, 115200, timeout_char=100)
led_r = LED(1); led_g = LED(2); led_b = LED(3)

net = None
labels = None
min_confidence = 0.90
try:
        net = ml.Model("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
except Exception as e:
        raise Exception('Failed to load "trained.tflite" (' + str(e) + ')')
try:
        labels = [line.rstrip('\\n') for line in open("labels.txt")]
except Exception as e:
        raise Exception('Failed to load "labels.txt" (' + str(e) + ')')
colors = [
        (255,   0,   0), (  0, 255,   0), (255, 255,   0),
        (  0,   0, 255), (255,   0, 255), (  0, 255, 255),
        (255, 255, 255),
]
threshold_list = [(math.ceil(min_confidence * 255), 255)]
def fomo_post_process(model, inputs, outputs):
        ob, oh, ow, oc = model.output_shape[0]
        x_scale = inputs[0].roi[2] / ow
        y_scale = inputs[0].roi[3] / oh
        scale = min(x_scale, y_scale)
        x_offset = ((inputs[0].roi[2] - (ow * scale)) / 2) + inputs[0].roi[0]
        y_offset = ((inputs[0].roi[3] - (ow * scale)) / 2) + inputs[0].roi[1]
        l = [[] for i in range(oc)]
        for i in range(oc):
                out_array = outputs[0][0, :, :, i]
                try:
                        img = image.Image(out_array * 255.0)
                except ValueError:
                        img = image.Image(ow, oh, sensor.GRAYSCALE)
                        for y in range(oh):
                                for x in range(ow):
                                        val = int(out_array[y][x]) + 128
                                        img.set_pixel(x, y, val)
                blobs = img.find_blobs(
                        threshold_list, x_stride=1, y_stride=1, area_threshold=1, pixels_threshold=1
                )
                for b in blobs:
                        rect = b.rect()
                        x, y, w, h = rect
                        score = img.get_statistics(thresholds=threshold_list, roi=rect).l_mean() / 255.0
                        x = int((x * scale) + x_offset)
                        y = int((y * scale) + y_offset)
                        w = int(w * scale)
                        h = int(h * scale)
                        l[i].append((x, y, w, h, score))
        return l
clock = time.clock()
last_send_time = 0
SEND_INTERVAL = 1500

while(True):
        clock.tick()
        img = sensor.snapshot()
        current_time = time.ticks_ms()
        any_object_detected = False

        for i, detection_list in enumerate(net.predict([img], callback=fomo_post_process)):
                if i == 0: continue
                if len(detection_list) == 0: continue
                
                cls_name = labels[i]
                
                for x, y, w, h, score in detection_list:
                        if score < min_confidence: continue
                        
                        any_object_detected = True
                        
                        center_x = math.floor(x + (w / 2))
                        center_y = math.floor(y + (h / 2))
                        
                        img.draw_circle((center_x, center_y, 12), color=colors[i % len(colors)])
                        img.draw_string(center_x, center_y - 20, "%s: %.0f%%" % (cls_name, score * 100), color=colors[i % len(colors)], scale=2)
                        
                        obj_id = 0x00
                        name_lower = cls_name.lower()
                        if "hammer" in name_lower: obj_id = 0x03
                        elif "scissor" in name_lower: obj_id = 0x02
                        elif "lighter" in name_lower or "light" in name_lower: obj_id = 0x01
                            
                        if obj_id != 0x00 and time.ticks_diff(current_time, last_send_time) > SEND_INTERVAL:
                            print("【命中目标！】 物体: %s" % (cls_name))

                            if obj_id == 0x01: led_r.on()
                            if obj_id == 0x02: led_g.on()
                            if obj_id == 0x03: led_b.on()
                            
                            uart.write(struct.pack('BBBBBBBB', 0xBB, obj_id, 0, 0, 0, 0, obj_id, 0x55))
                            time.sleep_ms(150)
                            led_r.off(); led_g.off(); led_b.off()
                            last_send_time = current_time

        if not any_object_detected:
            if time.ticks_diff(current_time, last_send_time) > SEND_INTERVAL:
                print("【未检测到目标】")
                uart.write(struct.pack('BBBBBBBB', 0xBB, 0x04, 0, 0, 0, 0, 0x04, 0x55))
                led_r.on(); led_b.on()
                time.sleep_ms(100)
                led_r.off(); led_b.off()
                last_send_time = current_time

"""

with open(r'E:\main.py', 'w', encoding='utf-8') as f:
    f.write(original_code)

with open(r'd:\KeilMDKARM5.35\smart-car\OpenMV\main.py', 'w', encoding='utf-8') as f:
    f.write(original_code)
print('Done. Switched back to UART(1, 115200)')
