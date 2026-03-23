import sensor, image, time, os, struct, gc, ml
from pyb import UART, LED

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # 320x240
sensor.set_windowing((240, 240))
sensor.skip_frames(time=2000)

uart = UART(3, 115200, timeout_char=100)
led_r = LED(1); led_g = LED(2); led_b = LED(3)

print("========= ML 深度学习正在初始化 =========")

gc.collect()

# 强制写死模型路径（刚刚我们通过命令行查到你U盘里叫这个）
model_path = "tflite-model/tflite_learn_929813_3.tflite"

net = None
labels = ["background", "hammer", "lighter", "scissors"]

# 尝试找标签
try:
    with open("labels.txt", 'r') as f:
        labels = [line.rstrip('\n') for line in f]
    print("模型标签加载成功！包含类别：", labels)
except Exception as e:
    print("系统提示：没找到 labels.txt，暂时使用备用硬编码标签。")

# 加载AI模型文件
try:
    print("正在把训练好的大脑(" + model_path + ")载入显存...")
    net = ml.Model(model_path, load_to_fb=True)
    print("🧠 模型加载完毕！系统正式起飞！")
except Exception as e:
    print("\n【严重报错】加载模型异常！", e)
    # 把 U 盘里所有文件打印出来看看
    try:
        print("当前根目录文件列表:", os.listdir())
        print("tflite-model 文件夹内容:", os.listdir("tflite-model"))
    except:
        pass
    while(True):
        led_r.toggle()
        time.sleep_ms(150)

clock = time.clock()
last_send_time = 0
SEND_INTERVAL = 1500 # 识别冷却时间 1.5秒

while(True):
    clock.tick()
    img = sensor.snapshot()
    current_time = time.ticks_ms()

    obj_id = 0x00
    detected_name = "Unknown"
    highest_score = 0.0

    try:
        out = net.predict([img])
        predictions = out[0].flatten().tolist()
        
        for i in range(len(predictions)):
            if predictions[i] > highest_score:
                highest_score = predictions[i]
                if i < len(labels):
                    detected_name = labels[i]
    except Exception as e:
        print("推理错误，请检查模型:", e)
        time.sleep_ms(1000)
        continue

    img.draw_string(5, 5, "%s: %.1f%%" % (detected_name, highest_score*100), color=(255, 0, 0), scale=2)

    if highest_score > 0.75:
        name_lower = detected_name.lower()
        if "hammer" in name_lower:
            obj_id = 0x03
        elif "scissor" in name_lower:
            obj_id = 0x02
        elif "lighter" in name_lower or "light" in name_lower:
            obj_id = 0x01
        elif "background" in name_lower:
            obj_id = 0x00 

        if obj_id != 0x00 and time.ticks_diff(current_time, last_send_time) > SEND_INTERVAL:
            print("\n==================================")
            print("【深度学习/AI 确信！】 置信度: %.1f%%" % (highest_score*100))
            if obj_id == 0x01: print("🎯 命中：打火机！  发送串口：0x01"); led_r.on()
            if obj_id == 0x02: print("🎯 命中：剪刀！    发送串口：0x02"); led_g.on()
            if obj_id == 0x03: print("🎯 命中：锤子！    发送串口：0x03"); led_b.on()
            print("==================================\n")

            uart.write(struct.pack('BBBBBBBB', 0xBB, obj_id, 0, 0, 0, 0, obj_id, 0x55))
            time.sleep_ms(150)
            led_r.off(); led_g.off(); led_b.off()
            last_send_time = current_time
    else:
        img.draw_string(10, 30, "Not Sure...", color=(255, 255, 0), scale=1)

    img.draw_string(10, 210, "FPS: %d" % clock.fps(), color=(0, 255, 0), scale=1)
