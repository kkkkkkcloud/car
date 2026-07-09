from machine import Pin
import camera
import time

#=====================
# 参数
#=====================

THRESHOLD = 55          # 灰度阈值，可调整
CURVE_THRESHOLD = 400   # 黑色像素差阈值

WIDTH = 160
HEIGHT = 120

#=====================
# 摄像头初始化
#=====================

camera.init(
    0,
    format=camera.GRAYSCALE,
    framesize=camera.FRAME_QQVGA,     #160×120
    xclk_freq=camera.XCLK_20MHz,

    d0=11,
    d1=9,
    d2=8,
    d3=10,
    d4=12,
    d5=18,
    d6=17,
    d7=16,

    vsync=6,
    href=7,
    pclk=13,
    xclk=15,
    siod=4,
    sioc=5,
    reset=-1,
    pwdn=-1
)

print("camera ok")

ROAD_OUT = Pin(45, Pin.OUT)      # 输出给ESP32
straight_cnt = 0
STRAIGHT_CNT_TH = 15            # 连续15次≈750ms

#=====================
# 判断道路
#=====================

def detect_road():

    img = camera.capture()

    left_black = 0
    right_black = 0

    half_w = WIDTH // 2
    half_h = HEIGHT // 2

    # 左上区域
    for y in range(half_h):
        row = y * WIDTH

        for x in range(half_w):
            gray = img[row + x]

            if gray < THRESHOLD:
                left_black += 1

    # 右上区域
    for y in range(half_h):
        row = y * WIDTH

        for x in range(half_w, WIDTH):
            gray = img[row + x]

            if gray < THRESHOLD:
                right_black += 1

    diff = abs(left_black - right_black)

    if diff > CURVE_THRESHOLD:
        state = "CURVE"
    else:
        state = "STRAIGHT"

    return left_black, right_black, diff, state


#=====================
# 主循环
#=====================

while True:

    left, right, diff, state = detect_road()

    if state == "STRAIGHT":
        straight_cnt += 1
    else:
        straight_cnt = 0

    if straight_cnt >= STRAIGHT_CNT_TH:
        ROAD_OUT.value(1)
    else:
        ROAD_OUT.value(0)

    print(state, straight_cnt)

    time.sleep_ms(50)
