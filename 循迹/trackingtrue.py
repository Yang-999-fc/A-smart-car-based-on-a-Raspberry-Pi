#!/usr/bin/python
# coding=utf-8
# 此代码实现基于树莓派的智能小车循线行驶行为。

import RPi.GPIO as GPIO
import time

# 传感器与电机引脚定义
T_SensorRight = 26       # 右侧循线传感器
T_SensorLeft = 13         # 左侧循线传感器

PWMA = 18               # 左电机PWM控制引脚
AIN1 = 22              # 左电机方向控制A1
AIN2 = 27              # 左电机方向控制A2

PWMB = 23              # 右电机PWM控制引脚
BIN1 = 25              # 右电机方向控制B1
BIN2 = 24              # 右电机方向控制B2

BtnPin = 19             # 启动按钮引脚
Gpin = 5                # 绿色LED引脚
Rpin = 6                # 红色LED引脚

# 电机控制函数
def t_up(speed):
    """两电机正转，前进"""
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2, False)
    GPIO.output(AIN1, True)

    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2, False)
    GPIO.output(BIN1, True)

def t_stop():
    """两电机停止"""
    L_Motor.ChangeDutyCycle(0)
    GPIO.output(AIN2, False)
    GPIO.output(AIN1, False)

    R_Motor.ChangeDutyCycle(0)
    GPIO.output(BIN2, False)
    GPIO.output(BIN1, False)

def t_down(speed):
    """两电机反转，后退"""
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2, True)
    GPIO.output(AIN1, False)

    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2, True)
    GPIO.output(BIN1, False)

def t_left(speed):
    """左电机反转，右电机正转，左转"""
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2, True)
    GPIO.output(AIN1, False)

    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2, False)
    GPIO.output(BIN1, True)

def t_right(speed):
    """左电机正转，右电机反转，右转"""
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2, False)
    GPIO.output(AIN1, True)

    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2, True)
    GPIO.output(BIN1, False)

# 按钮按下等待启动车辆函数
def key_scan():
    """等待按钮按下以启动小车，并指示灯闪烁"""
    while GPIO.input(BtnPin) == False:
        time.sleep(0.01)
    while GPIO.input(BtnPin) == True:
        time.sleep(0.01)
    GPIO.output(Rpin, 1)
    while GPIO.input(BtnPin) == False:
        GPIO.output(Rpin, 0)

# 初始化GPIO设置函数
def setup():
    """初始化GPIO设置"""
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)  # 使用BCM编号方式
    GPIO.setup(Gpin, GPIO.OUT)  # 设置绿色LED为输出
    GPIO.setup(Rpin, GPIO.OUT)  # 设置红色LED为输出
    GPIO.setup(BtnPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # 设置按钮输入并上拉
    GPIO.setup(T_SensorRight, GPIO.IN)  # 右侧传感器设为输入
    GPIO.setup(T_SensorLeft, GPIO.IN)  # 左侧传感器设为输入
    
    # 设置电机控制引脚为输出
    GPIO.setup(AIN2, GPIO.OUT)
    GPIO.setup(AIN1, GPIO.OUT)
    GPIO.setup(PWMA, GPIO.OUT)

    GPIO.setup(BIN1, GPIO.OUT)
    GPIO.setup(BIN2, GPIO.OUT)
    GPIO.setup(PWMB, GPIO.OUT)

# 主函数
if __name__ == '__main__':
    setup()
    key_scan()  # 等待按钮按下启动
    L_Motor = GPIO.PWM(PWMA, 100)  # 初始化左电机PWM，频率100Hz
    L_Motor.start(0)                 # 开始PWM，初始占空比0
    R_Motor = GPIO.PWM(PWMB, 100)  # 初始化右电机PWM，频率100Hz
    R_Motor.start(0)                 # 开始PWM，初始占空比0
    
    try:
        while True:
            SR = GPIO.input(T_SensorRight)  # 读取右侧传感器状态
            SL = GPIO.input(T_SensorLeft)    # 读取左侧传感器状态
            
            if SL and SR:  # 两侧传感器均检测到黑线，直行
                t_up(40)
            elif not SL and SR:  # 左侧传感器未检测到黑线，向左转
                t_left(40)
            elif SL and not SR:  # 右侧传感器未检测到黑线，向右转
                t_right(40)
            else:  # 两侧均未检测到黑线，停车
                t_stop()
                
    except KeyboardInterrupt:  # 捕获Ctrl+C异常，进行GPIO清理
        GPIO.cleanup()  # 清理GPIO设置
