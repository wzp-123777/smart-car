import sensor, image, time, os, struct
from pyb import UART, LED
import tf

# ================= 摄像头初始化 =================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # 很多模型需要QVGA缩小到 96x96 或 128x128
sensor.set_windowing((240, 240))  # 截取中心方形画面送入神经网络(模型大多是正方形输入)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# ================= 串口与硬件 =================
uart = UART(3, 115200, timeout_char=100)
led_red=LED(1); led_green=LED(2); led_blue=LED(3)

HEADER_TX = 0xBB; TAIL = 0x55

# 与 STM32 的对应字典
LABELS_TO_ID = {
    "lighter":  0x01,
    "scissors": 0x02,
    "hammer":   0x03,
    "background": 0x00 # 没识别到目标
}

# ================= 加载模型 =================
# 在你训练好并放入OpenMV U盘后，把模型名字填在这里
# 现在还没训练，先写好骨架，等模型文件有了，取消注释就能跑
MODEL_FILE = "trained_model.tflite"
LABELS_FILE = "labels.txt"

# 尝试加载标签
labels = []
try:
    with open(LABELS_FILE, 'r') as f:
        labels = [line.rstrip('\n') for line in f]
except Exception as e:
    print("等待放入模型和标签文件...")
    labels = ["background", "lighter", "scissors", "hammer"] # 默认后备

def send_result(obj_id):
    # 这里不需要发送坐标了，只需要发送识别到什么物品触发语音即可
    cx, cy = 0, 0
    xh = (cx >> 8) & 0xFF; xl = cx & 0xFF
    yh = (cy >> 8) & 0xFF; yl = cy & 0xFF
    checksum = (obj_id + xh + xl + yh + yl) & 0xFF
    data = struct.pack('BBBBBBBB', HEADER_TX, obj_id, xh, xl, yh, yl, checksum, TAIL)
    uart.write(data)
    print("TF Sent: ID=0x%02X" % obj_id)

clock = time.clock()
last_send_time = 0
SEND_INTERVAL = 2000 

print("Ready to run Edge Impulse TFLite Model!")

while(True):
    clock.tick()
    img = sensor.snapshot()
    current_time = time.ticks_ms()

    # ---------------- 关键的神经网络推理 ----------------
    # try:
    #     # 对当前图像进行推断，返回分类结果对象
    #     # 注意: 这里的 tf.classify 会自动缩放图像去适配模型 (例如 96x96)
    #     objs = tf.classify(MODEL_FILE, img, min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0)
    #     
    #     for obj in objs:
    #         # obj.output() 是一个浮点数组，代表每个分类的概率 (如 [0.1, 0.8, 0.1, 0.0])
    #         predictions = obj.output()
    #         # 找出最大概率的那个的索引
    #         max_val = max(predictions)
    #         max_idx = predictions.index(max_val)
    #         
    #         # 如果最大概率大于 0.7 (70% 确信度)，才认为是真的
    #         if max_val > 0.70:
    #             label_name = labels[max_idx]
    #             img.draw_string(10, 10, "%s: %.2f" % (label_name, max_val), color=(255, 0, 0), scale=2)
    #             
    #             if label_name in LABELS_TO_ID and LABELS_TO_ID[label_name] != 0x00:
    #                 if time.ticks_diff(current_time, last_send_time) > SEND_INTERVAL:
    #                     send_result(LABELS_TO_ID[label_name])
    #                     led_green.on(); time.sleep_ms(50); led_green.off()
    #                     last_send_time = current_time
    # except Exception as e:
    #     # 当还没放入 tflite 模型时，它会在这里抛出异常，只显示画面不闪退
    #     img.draw_string(10, 50, "Please upload trained_model.tflite", color=(255, 255, 0))

    # 为了模拟占位，先画出神经网络必须截断的正方形截取框:
    img.draw_rectangle(0, 0, 240, 240, color=(255,255,255))
    img.draw_string(10, 100, "Waiting for TF Model!", color=(255,100,100), scale=2)

