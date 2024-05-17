import numpy as np
import cv2
import camera_config
import random
import math
import RPi.GPIO as GPIO
import time
import sys

def d_avoid():              #避障程序
       t_right(25,1)
       t_stop(0.5)
       t_up(30,2)
       t_stop(0.5)
       t_left(25,1)
       t_stop(0.5)
       t_up(30,4.5)
       t_stop(0.5)
       t_left(25,1)
       t_up(30,2)
       t_right(25,1)

def t_up(speed,t_time):      #前进
        L_Motor.ChangeDutyCycle(speed)
        GPIO.output(AIN2,False)#AIN2
        GPIO.output(AIN1,True) #AIN1

        R_Motor.ChangeDutyCycle(speed)
        GPIO.output(BIN2,False)#BIN2
        GPIO.output(BIN1,True) #BIN1
        time.sleep(t_time)

def t_stop(t_time):             #停止
        L_Motor.ChangeDutyCycle(0)
        GPIO.output(AIN2,False)#AIN2
        GPIO.output(AIN1,False) #AIN1

        R_Motor.ChangeDutyCycle(0)
        GPIO.output(BIN2,False)#BIN2
        GPIO.output(BIN1,False) #BIN1
        time.sleep(t_time)

def t_down(speed,t_time):       #后退
        L_Motor.ChangeDutyCycle(speed)
        GPIO.output(AIN2,True)#AIN2
        GPIO.output(AIN1,False) #AIN1

        R_Motor.ChangeDutyCycle(speed)
        GPIO.output(BIN2,True)#BIN2
        GPIO.output(BIN1,False) #BIN1
        time.sleep(t_time)

def t_left(speed,t_time):       #左转
        L_Motor.ChangeDutyCycle(speed)
        GPIO.output(AIN2,True)#AIN2
        GPIO.output(AIN1,False) #AIN1

        R_Motor.ChangeDutyCycle(speed)
        GPIO.output(BIN2,False)#BIN2
        GPIO.output(BIN1,True) #BIN1
        time.sleep(t_time)

def t_right(speed,t_time):      #右转
        L_Motor.ChangeDutyCycle(speed)
        GPIO.output(AIN2,False)#AIN2
        GPIO.output(AIN1,True) #AIN1

        R_Motor.ChangeDutyCycle(speed)
        GPIO.output(BIN2,True)#BIN2
        GPIO.output(BIN1,False) #BIN1
        time.sleep(t_time)

def keysacn():
    val = GPIO.input(BtnPin)
    while GPIO.input(BtnPin) == False:
        val = GPIO.input(BtnPin)
    while GPIO.input(BtnPin) == True:
        time.sleep(0.01)
        val = GPIO.input(BtnPin)
        if val == True:
            GPIO.output(Rpin,1)
            while GPIO.input(BtnPin) == False:
                GPIO.output(Rpin,0)
        else:
            GPIO.output(Rpin,0)

def setup():        #初始化
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)       # Numbers GPIOs by physical location
    GPIO.setup(Gpin, GPIO.OUT)     # Set Green Led Pin mode to output
    GPIO.setup(Rpin, GPIO.OUT)     # Set Red Led Pin mode to output
    GPIO.setup(BtnPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)    # Set BtnPin's mode is input, and pull up to high level(3.3V)
    GPIO.setup(T_SensorRight,GPIO.IN)
    GPIO.setup(T_SensorLeft,GPIO.IN)

    GPIO.setup(AIN2,GPIO.OUT)
    GPIO.setup(AIN1,GPIO.OUT)
    GPIO.setup(PWMA,GPIO.OUT)

    GPIO.setup(BIN1,GPIO.OUT)
    GPIO.setup(BIN2,GPIO.OUT)
    GPIO.setup(PWMB,GPIO.OUT)

cap = cv2.VideoCapture(0)
cap.set(3, 1280)
cap.set(4, 480)  #打开并设置摄像头


# 鼠标点击显示距离
def onmouse_pick_points(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        threeD = param
        print('\n像素坐标 x = %d, y = %d' % (x, y))
        # print("世界坐标是：", threeD[y][x][0], threeD[y][x][1], threeD[y][x][2], "mm")
        print("世界坐标xyz 是：", threeD[y][x][0]/ 1000.0 , threeD[y][x][1]/ 1000.0 , threeD[y][x][2]/ 1000.0 , "m")

        distance = math.sqrt( threeD[y][x][0] **2 + threeD[y][x][1] **2 + threeD[y][x][2] **2 )
        distance = distance / 1000.0  # mm -> m
        print("距离是：", distance, "m")

WIN_NAME = 'Deep disp'
cv2.namedWindow(WIN_NAME,  cv2.WINDOW_AUTOSIZE)

while True:
  ret, frame = cap.read()
  frame1 = frame[0:480, 0:640]
  frame2 = frame[0:480, 640:1280]  #割开双目图像

  imgL = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)  # 将BGR格式转换成灰度图片
  imgR = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

  # cv2.remap 重映射，就是把一幅图像中某位置的像素放置到另一个图片指定位置的过程。
  # 依据MATLAB测量数据重建无畸变图片
  img1_rectified = cv2.remap(imgL, camera_config.left_map1, camera_config.left_map2, cv2.INTER_LINEAR)
  img2_rectified = cv2.remap(imgR, camera_config.right_map1, camera_config.right_map2, cv2.INTER_LINEAR)

  imageL = cv2.cvtColor(img1_rectified, cv2.COLOR_GRAY2BGR)
  imageR = cv2.cvtColor(img2_rectified, cv2.COLOR_GRAY2BGR)

  # SGBM算法
  blockSize = 8
  img_channels = 3
  stereo = cv2.StereoSGBM_create(minDisparity = 1,
                                   numDisparities = 64,
                                   blockSize = blockSize,
                                   P1 = 8 * img_channels * blockSize * blockSize,
                                   P2 = 32 * img_channels * blockSize * blockSize,
                                   disp12MaxDiff = -1,
                                   preFilterCap = 1,
                                   uniquenessRatio = 10,
                                   speckleWindowSize = 100,
                                   speckleRange = 100,
                                   mode = cv2.STEREO_SGBM_MODE_HH)

  disparity = stereo.compute(img1_rectified, img2_rectified) # 计算视差

  disp = cv2.normalize(disparity, disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)  #归一化函数算法

  threeD = cv2.reprojectImageTo3D(disparity, camera_config.Q, handleMissingValues=True)  #计算三维坐标数据值
  threeD = threeD * 16

  distance = math.sqrt( threeD[260][370][0] **2 + threeD[260][370][1] **2 + threeD[260][370][2] **2 )#计算一点的距离
  distance = distance / 1000.0  # mm -> m
  print("距离是：", distance, "m")

  # threeD[y][x] x:0~640; y:0~480;   !!!!!!!!!!
  cv2.setMouseCallback(WIN_NAME, onmouse_pick_points, threeD) # 鼠标回调事件

  cv2.imshow("left", frame1)
  cv2.imshow("right", frame2)
  # cv2.imshow("left_r", imgL)
  # cv2.imshow("right_r", imgR)
  cv2.imshow(WIN_NAME, disp)  #显示深度图的双目画面

  if __name__ == '__main__':#循迹逻辑
    setup()
    keysacn()
    L_Motor= GPIO.PWM(PWMA,100)
    L_Motor.start(0)
    R_Motor = GPIO.PWM(PWMB,100)
    R_Motor.start(0)
    try:
        while True:
            SR = GPIO.input(T_SensorRight)
            SL = GPIO.input(T_SensorLeft)
            if SL == True and SR == True:
                t_up(40,0)
            elif SL == False and SR ==True:
                t_left(40,0)
            elif SL==True and SR ==False:
                t_right(40,0)
            elif distance == 70:
                d_avoid()
            else:
                t_stop(20)
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        GPIO.cleanup()

  key = cv2.waitKey(1)
  if key == ord("q"):
    break

cap.release()
cv2.destroyAllWindows()
