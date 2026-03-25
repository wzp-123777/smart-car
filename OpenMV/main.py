# Edge Impulse - OpenMV FOMO Object Detection Example with UART
import sensor, image, time, struct, math, ml, os
from ml.utils import NMS
from pyb import UART, LED

sensor.reset()  # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)  # Set pixel format to RGB565
sensor.set_framesize(sensor.QVGA)  # Set frame size to QVGA (320x240)
sensor.skip_frames(time=2000)  # Let the camera adjust.

uart = UART(3, 115200, timeout_char=100)
led_r = LED(1); led_g = LED(2); led_b = LED(3)

print("========= Edge Impulse FOMO Model Initializing =========")

# 仅显示置信度达到 90% (0.90) 的对象
min_confidence = 0.90
threshold_list = [(math.ceil(min_confidence * 255), 255)]

# ==== 尝试多种方式加载模型 ====
try:
    # 尝试加载固件内嵌的模型名称 "trained" (一般使用 Edge Impulse 编入的固件叫这个)
    model = ml.Model("trained")
    print("✅ 固件内嵌模型 'trained' 加载完毕！")
    print("包含类别:", model.labels)
except Exception as e:
    print("⚠️ 固件内嵌模型 'trained' 加载失败 [Errno 2] ENOENT")
    print("正在尝试在 OpenMV u盘内查找 .tflite 模型文件...")
    
    model_path = None
    try:
        files = os.listdir('/')
        for f in files:
            if f.endswith('.tflite') or f.endswith('.network'):
                model_path = f
                break
        if not model_path:
            # 兼容寻找您之前存放的 tflite-model 目录
            if 'tflite-model' in files:
                for f in os.listdir('/tflite-model'):
                    if f.endswith('.tflite'):
                        model_path = '/tflite-model/' + f
                        break
    except:
        pass
        
    if model_path:
        print("✅ 找到模型文件: " + model_path + "，正在将其载入显存...")
        model = ml.Model(model_path, load_to_fb=True)
        # 如果是外部模型，尝试自己读取 labels.txt
        try:
            with open('labels.txt', 'r') as lf:
                labels = [line.strip() for line in lf.readlines() if line.strip()]
            model.labels = labels
            print("标签文件 labels.txt 加载成功:", model.labels)
        except:
            print("未找到 labels.txt 或加载失败，使用默认硬编码类别。")
            # 默认给个兼容类别，如果不是这个顺序你需要自己改
            model.labels = ["background", "lighter", "scissor", "hammer"] 
        print("SD卡模型加载成功！🚀")
    else:
        print("❌ 灾难性失败：没有找到内嵌固件，也没有找到任何以 .tflite 结尾的文件！请确认你拷贝进入U盘。")
        while True:
            led_r.toggle()
            time.sleep_ms(150)

colors = [
    (255, 0, 0),    (0, 255, 0),    (255, 255, 0),
    (0, 0, 255),    (255, 0, 255),  (0, 255, 255),
    (255, 255, 255)
]

def fomo_post_process(model, inputs, outputs):
    n, oh, ow, oc = model.output_shape[0]
    nms = NMS(ow, oh, inputs[0].roi)
    for i in range(oc):
        img = image.Image(outputs[0][0, :, :, i] * 255)
        blobs = img.find_blobs(
            threshold_list, x_stride=1, area_threshold=1, pixels_threshold=1    
        )
        for b in blobs:
            rect = b.rect()
            x, y, w, h = rect
            score = (
                img.get_statistics(thresholds=threshold_list, roi=rect).l_mean() / 255.0                                                                                    
            )
            nms.add_bounding_box(x, y, x + w, y + h, score, i)
    return nms.get_bounding_boxes()

clock = time.clock()
last_send_time = 0
SEND_INTERVAL = 1500 # 识别冷却时间 1.5秒

while True:
    clock.tick()
    img = sensor.snapshot()
    current_time = time.ticks_ms()

    # 预测并使用后处理函数提取坐标
    for i, detection_list in enumerate(model.predict([img], callback=fomo_post_process)):                                                                               
        if i == 0:
            continue  # class 0 通常是 background

        if len(detection_list) == 0:
            continue

        cls_name = model.labels[i]

        for (x, y, w, h), score in detection_list:
            if score < 0.90:
                continue # 只有置信度达到90的时候才能进行判断

            center_x = math.floor(x + (w / 2))
            center_y = math.floor(y + (h / 2))

            # 画图圈出物体
            img.draw_circle((center_x, center_y, 12), color=colors[i % len(colors)], thickness=2)                                                                           
            img.draw_string(center_x, center_y - 20, "%s: %.0f%%" % (cls_name, score * 100), color=colors[i % len(colors)], scale=2)                            
            
            # 解析串口 ID 发送给小车
            obj_id = 0x00
            name_lower = cls_name.lower()
            if "hammer" in name_lower:
                obj_id = 0x03
            elif "scissor" in name_lower:
                obj_id = 0x02
            elif "lighter" in name_lower or "light" in name_lower:
                obj_id = 0x01
            else:
                obj_id = 0x00

            if obj_id != 0x00 and time.ticks_diff(current_time, last_send_time) > SEND_INTERVAL:                                                                                
                print("==================================")
                print("【命中目标！】 物体: %s, 置信度: %.1f%%" % (cls_name, score * 100))                                                                                      
                print("==================================")

                if obj_id == 0x01: led_r.on()
                if obj_id == 0x02: led_g.on()
                if obj_id == 0x03: led_b.on()

                # 暂时注释掉串口发送给主板，仅供终端调试使用
                uart.write(struct.pack('BBBBBBBB', 0xBB, obj_id, 0, 0, 0, 0, obj_id, 0x55))
                time.sleep_ms(150)
                led_r.off(); led_g.off(); led_b.off()
                last_send_time = current_time

    img.draw_string(10, 210, "FPS: %d" % clock.fps(), color=(0, 255, 0), scale=2)
