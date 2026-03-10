# OpenMV H7 Plus 端配套代码
# 文件名: main.py（烧写到OpenMV Flash中）
# 功能: 接收STM32指令，执行图像识别，返回结果
#
# 接线:
#   OpenMV UART3: P4(TX) → STM32 UART1_RX
#                 P5(RX) → STM32 UART1_TX
#   波特率: 115200

import sensor, image, time, struct, math
from pyb import UART, LED

# ==================== 初始化 ====================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

uart = UART(3, 115200, timeout_char=100)

led_red = LED(1)
led_green = LED(2)
led_blue = LED(3)

# ==================== 协议定义 ====================
HEADER_RX  = 0xAA    # 接收帧头（来自STM32）
TAIL       = 0x55    # 帧尾
HEADER_TX  = 0xBB    # 发送帧头（返回给STM32）

CMD_DETECT = 0x01    # 通用检测
CMD_STOP   = 0x02    # 停止
CMD_COLOR  = 0x03    # 颜色识别
CMD_SHAPE  = 0x04    # 形状识别
CMD_QRCODE = 0x05    # 二维码识别

# 物体ID定义（与STM32端一致）
OBJ_NONE     = 0x00
OBJ_CIRCLE   = 0x01
OBJ_TRIANGLE = 0x02
OBJ_SQUARE   = 0x03
OBJ_RED      = 0x10
OBJ_GREEN    = 0x11
OBJ_BLUE     = 0x12

# ==================== 颜色阈值（根据实际环境标定！）====================
# 格式: (L_min, L_max, A_min, A_max, B_min, B_max) - LAB色彩空间
red_threshold   = (30, 100, 40, 80, 10, 70)
green_threshold = (30, 100, -70, -20, 10, 60)
blue_threshold  = (10, 50, -10, 30, -80, -30)

# ==================== 发送函数 ====================
def send_result(obj_id, cx, cy):
    """发送识别结果给STM32"""
    xh = (cx >> 8) & 0xFF
    xl = cx & 0xFF
    yh = (cy >> 8) & 0xFF
    yl = cy & 0xFF
    checksum = (obj_id + xh + xl + yh + yl) & 0xFF
    data = struct.pack('BBBBBBBB', HEADER_TX, obj_id, xh, xl, yh, yl, checksum, TAIL)
    uart.write(data)
    print("TX: ID=0x%02X X=%d Y=%d" % (obj_id, cx, cy))

# ==================== 颜色识别 ====================
def detect_color(img):
    """识别红/绿/蓝色块"""
    best_id = OBJ_NONE
    best_area = 0
    best_cx, best_cy = 160, 120

    for color_id, threshold in [(OBJ_RED, red_threshold),
                                 (OBJ_GREEN, green_threshold),
                                 (OBJ_BLUE, blue_threshold)]:
        blobs = img.find_blobs([threshold], pixels_threshold=200,
                                area_threshold=200, merge=True)
        for b in blobs:
            if b.area() > best_area:
                best_area = b.area()
                best_id = color_id
                best_cx = b.cx()
                best_cy = b.cy()
                img.draw_rectangle(b.rect(), color=(0, 255, 0))

    return best_id, best_cx, best_cy

# ==================== 形状识别（简单版）====================
def detect_shape(img):
    """识别圆形/三角形/矩形"""
    best_id = OBJ_NONE
    best_cx, best_cy = 160, 120

    # 检测圆形
    circles = img.find_circles(threshold=3500, x_margin=10, y_margin=10,
                                r_margin=10, r_min=15, r_max=80)
    if circles:
        c = max(circles, key=lambda x: x.r())
        best_id = OBJ_CIRCLE
        best_cx = c.x()
        best_cy = c.y()
        img.draw_circle(c.x(), c.y(), c.r(), color=(0, 255, 0))
        return best_id, best_cx, best_cy

    # 检测矩形
    rects = img.find_rects(threshold=10000)
    for r in rects:
        corners = r.corners()
        if len(corners) == 4:
            best_id = OBJ_SQUARE
            cx = sum([c[0] for c in corners]) // 4
            cy = sum([c[1] for c in corners]) // 4
            best_cx = cx
            best_cy = cy
            img.draw_rectangle(r.rect(), color=(0, 0, 255))
            return best_id, best_cx, best_cy

    return best_id, best_cx, best_cy

# ==================== 主循环 ====================
print("OpenMV Ready!")
led_green.on()

while True:
    # 检查是否收到STM32指令
    if uart.any():
        data = uart.read(4)  # 读4字节 [AA][CMD][CHK][55]
        if data and len(data) >= 4:
            if data[0] == HEADER_RX and data[3] == TAIL:
                cmd = data[1]
                led_blue.on()
                print("RX CMD: 0x%02X" % cmd)

                img = sensor.snapshot()

                if cmd == CMD_DETECT or cmd == CMD_COLOR:
                    # 颜色识别
                    obj_id, cx, cy = detect_color(img)
                    send_result(obj_id, cx, cy)

                elif cmd == CMD_SHAPE:
                    # 形状识别
                    obj_id, cx, cy = detect_shape(img)
                    send_result(obj_id, cx, cy)

                elif cmd == CMD_QRCODE:
                    # 二维码识别
                    codes = img.find_qrcodes()
                    if codes:
                        # 简单返回第一个二维码
                        qr = codes[0]
                        send_result(0x20, qr.x(), qr.y())
                        print("QR:", qr.payload())
                    else:
                        send_result(OBJ_NONE, 0, 0)

                elif cmd == CMD_STOP:
                    pass  # 停止检测

                led_blue.off()

    time.sleep_ms(10)
