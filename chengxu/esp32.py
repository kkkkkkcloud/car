from machine import Pin, PWM, ADC
import time

# ================== 电机基础参数 ==================
MOTOR_MIN = 600
MOTOR_MAX = 1023
PWM_FREQ = 12000

# 电机IO
LEFT_IN1, LEFT_IN2 = 13, 15
RIGHT_IN1, RIGHT_IN2 = 25, 14

left_pwm1 = PWM(Pin(LEFT_IN1), freq=PWM_FREQ, duty=0)
left_pwm2 = PWM(Pin(LEFT_IN2), freq=PWM_FREQ, duty=0)
right_pwm1 = PWM(Pin(RIGHT_IN1), freq=PWM_FREQ, duty=0)
right_pwm2 = PWM(Pin(RIGHT_IN2), freq=PWM_FREQ, duty=0)

road_pin = Pin(22, Pin.IN, Pin.PULL_DOWN)
# ================== 五路循迹ADC ==================
ADC_PINS = [27, 33, 32, 35, 34]
BLACK_THRESHOLD = 130
adc_list = [ADC(Pin(pin)) for pin in ADC_PINS]
for adc in adc_list:
    adc.atten(ADC.ATTN_11DB)

# ================== 直角弯专用参数 ==================
KP = 0.15
KI = 0.0015
KD = 0.35
DEAD_ZONE = 0.15
INTEGRAL_LIMIT = 15

NORMAL_SPEED = 910
FAST_SPEED = 1060

BASE_SPEED = NORMAL_SPEED

boost_flag = False
boost_start = 0
BOOST_TIME = 1000      # 最长1秒

TURN_GAIN = 180
MAX_DIFF = 800       # 【修改】放宽差速限制，允许产生极大的差速来实现单轮反转
SPEED_REDUCE = 70   # 【修改】增大降速系数，偏差大时基础速度大幅降低，便于急转弯

# 全局变量
last_error = 0
integral = 0

# ================== 电机控制 ==================
def set_motor(left_speed, right_speed):
    """
    left_speed/right_speed 正数前进，负数反转后退，0刹车
    有效范围：-MOTOR_MAX ~ MOTOR_MAX，低于MOTOR_MIN绝对值不输出动力
    """
    # 左电机控制
    if left_speed == 0:
        left_pwm1.duty(0)
        left_pwm2.duty(0)
    elif left_speed > 0:
        duty = max(MOTOR_MIN, min(MOTOR_MAX, int(left_speed)))
        left_pwm1.duty(0)
        left_pwm2.duty(duty)
    else:
        duty = max(MOTOR_MIN, min(MOTOR_MAX, int(abs(left_speed))))
        left_pwm1.duty(duty)
        left_pwm2.duty(0)

    # 右电机控制
    if right_speed == 0:
        right_pwm1.duty(0)
        right_pwm2.duty(0)
    elif right_speed > 0:
        duty = max(MOTOR_MIN, min(MOTOR_MAX, int(right_speed)))
        right_pwm1.duty(0)
        right_pwm2.duty(duty)
    else:
        duty = max(MOTOR_MIN, min(MOTOR_MAX, int(abs(right_speed))))
        right_pwm1.duty(duty)
        right_pwm2.duty(0)

def stop():
    set_motor(0, 0)

# ================== 传感器读取 ==================
def read_adc():
    vals = []
    for adc in adc_list:
        val = 0
        for _ in range(3):
            val += adc.read()
        vals.append(val // 3)
    return vals

def get_error():
    v = read_adc()
    weight = [-2, -1, 0, 1, 2]
    sum_w = 0
    cnt = 0
    for i in range(5):
        if v[i] > BLACK_THRESHOLD:
            sum_w += weight[i]
            cnt += 1
    if cnt == 0:
        return None
    return sum_w / cnt

# ================== PID控制 ==================
def pid_control(err):
    global last_error, integral

    if abs(err) < DEAD_ZONE:
        integral = 0
        pid_out = 0
    else:
        integral += err
        integral = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, integral))
        derivative = err - last_error
        pid_out = KP * err + KI * integral + KD * derivative

    last_error = err

    # 偏差越大，整体车速越低（适配直角弯）
    err_abs = abs(err)
    now_base = BASE_SPEED - err_abs * SPEED_REDUCE
    # 【修改】不要用 MOTOR_MIN 限制 now_base，允许它降到 0，否则无法产生负数急转
    now_base = max(0, now_base) 

    # 计算转向差
    diff = pid_out * TURN_GAIN
    diff = max(-MAX_DIFF, min(MAX_DIFF, diff))

    # 实时左右轮转速，此时如果 diff 很大，now_base 很小，就会产生负数
    left_speed  = now_base + diff
    right_speed = now_base - diff

    return int(left_speed), int(right_speed)

# ================== 主循环 ==================
stop()
time.sleep_ms(2000)

while True:
    
    now = time.ticks_ms()

    # CAM要求加速
    if road_pin.value():

        if not boost_flag:
            boost_flag = True
            boost_start = now

    # 超过1秒恢复
    if boost_flag:

        if (not road_pin.value()) or \
           time.ticks_diff(now, boost_start) > BOOST_TIME:

            boost_flag = False

    if boost_flag:
        BASE_SPEED = FAST_SPEED
    else:
        BASE_SPEED = NORMAL_SPEED
    
    err = get_error()
    
    if err is None:
        # 【修改】脱线时，根据最后一次的误差方向，原地反转找线
        if last_error > 0:
            # 之前黑线在右侧，原地右转找线
            set_motor(MOTOR_MIN + 100, -(MOTOR_MIN + 100))
        else:
            # 之前黑线在左侧，原地左转找线
            set_motor(-(MOTOR_MIN + 100), MOTOR_MIN + 100)
        time.sleep_ms(10)
        continue

    l, r = pid_control(err)
    set_motor(l, r)
    # print(f"err:{err:.2f}  l:{l}  r:{r}") # 调试时可取消注释
    time.sleep_ms(20)
