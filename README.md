# A-smart-car-based-on-a-Raspberry-Pi
基于树莓派的双目视觉智能小车

项目功能介绍：
  本项目以树莓派 4B 作为主控制器，采用双目视觉传感器进行视觉检测，有效获取前方深度信息，并采用三路红外传感器进行循迹。其次，基于张氏标定法对相机进行标定；采用 Bouguet 算法对图像进行校正；采用 SGBM 算法进行立体匹配；采用五点法对障碍物距离进行精密检测，并实现小车的避障功能。同时，运用 PID 控制算法实现小车的自动循迹功能。

硬件：树莓派4B+拓展版
双目相机：HBV-1780-2 S2.0
  采用的是USB免驱双目相机，将该双目相机装到舵机上，并将其USB接口接到树莓派的USB2.0端口上
循迹模块：三路红外
  将左路信号线接入GPIO13，中路信号线接入GPIO19，右路信号线接入GPIO26。并将三路红外传感器的其他两个管脚一个接入电源接口，一个接入地。
  
  <img width="367" alt="image" src="https://github.com/Yang-999-fc/A-smart-car-based-on-a-Raspberry-Pi/assets/57994308/611bb236-39fa-48fc-9ece-f1c11736a6ea">
  
舵机模块：TS90A舵机
  使用树莓派拓展版上的PCA9685芯片来调节PWM控制舵机
  
  <img width="301" alt="image" src="https://github.com/Yang-999-fc/A-smart-car-based-on-a-Raspberry-Pi/assets/57994308/d374939f-5084-47fa-ba25-52350abae132">
  
  PCA9685芯片电路接线图
  
  <img width="299" alt="image" src="https://github.com/Yang-999-fc/A-smart-car-based-on-a-Raspberry-Pi/assets/57994308/4422de5a-e957-407b-a3bc-3f308567a1a1">


  
电机模块：直流减速电机

电机驱动模块：电机驱动芯片TB6612FNG
电机驱动芯片VM连接12V电源，VCC连接5V电源。AO1、AO2、BO1、BO2为电机控制输出端，PWMA、AIN1、BIN1、PWMB、AIN2、BIN2分别连接GPIO18、GPIO22、GPIO25、GPIO23、GPIO27、GPIO24作为控制信号输入端。

<img width="235" alt="image" src="https://github.com/Yang-999-fc/A-smart-car-based-on-a-Raspberry-Pi/assets/57994308/46aa66bf-c246-40ed-9980-0e4c3a3fa6ed">
<img width="257" alt="image" src="https://github.com/Yang-999-fc/A-smart-car-based-on-a-Raspberry-Pi/assets/57994308/5fed6491-0b64-4dc6-b10f-b1c15895a902">

  

蜂鸣器：蜂鸣器连接在GPIO17端口上，一个引脚接+5V电压，一个引脚接地

<img width="248" alt="image" src="https://github.com/Yang-999-fc/A-smart-car-based-on-a-Raspberry-Pi/assets/57994308/ae4b7c46-3ef6-4eed-bdf2-5016755315a6">


树莓派系统配置：
  本次设计使用树莓派控制，树莓派 4B 本身自带 5G 双频 WiFi 模块，能够在电脑端直接通过 VNC 远程界面连接。首先使用网线在 putty 中连接树莓派，putty
  界面中输入指令 sudo raspi-config 进入系统设置。将里面的 VNC 进行使能，便可进入 VNC 界面。如下图所示，在 VNC 界面对树莓派进行配置。我们将其中 SSH、I2C 和 Remote GPIO 使能。SSH 使能后运行树莓派更方便；I2C 可以使控制舵机的 PCA9685 芯片与主控进行通信；Remote GPIO 使能后，便可以控制树莓派的 GPIO 端口。


<img width="295" alt="image" src="https://github.com/Yang-999-fc/A-smart-car-based-on-a-Raspberry-Pi/assets/57994308/d9ea4281-f555-4231-8156-9cba99bf1446">

软硬件调试：
  电机调试
  
    电机调试分为两部分，首先对电机的转动方向进行调试。在直流减速电机安装过程中，考虑到电机在小车上安装的方向与电机线路的正负方向，都有可能接反。所以需要对4个电机的转动方向分别进行检测。并且在检测到异常后，应该检查并重新连接异常电机。当电机单独检查完毕后，应重新检查4个电机整体转动，以确保正确控制小车的方向。

    
    测试第二部分为电机转速调试。在一些因素的干预下，4个电机在同一参数下进行调试，可能会出现转速差异。这将会引起小车控制精度下降。所以需要调节每个电机的PWM占空比，对其转速进行比对。

  舵机调试
  
    舵机在本设计中起到安置双目相机以及确保在系统初始化后使相机居中的作用。在小车测试时，会出现小车碰撞到障碍物或其他物品上的情形。当障碍物碰撞到双目视觉传感器时，会使得舵机带着传感器位移。若舵机位置不归中，下一次测试将会出现很大的误差。

测距流程：

1.相机标定：

  本次设计采用matlab 2016a软件中的相机标定工具箱对相机进行标定。该相机标定工具箱所使用的方法，是张正友教授提出的一种利用平面黑白色棋盘格对相机进行标定的实用方法。称为张氏标定法。
  
2.图像校正:

  在相机标定后，本设计编辑了camera_config.py文件，所以在校正时可以直接调用该文件中的矩阵。开始时调用Opencv库中的stereoRectify函数将左右相机的内外参、畸变系数、旋转矩阵、偏移矩阵都放入其中，来计算投影矩阵和旋转矩阵。再将所得参数放入initUndistortRectifyMap函数，进行计算，将所得结果使用相互对应的形式输出。再使用remap()函数对图片开始校正。对校正后的结果设置平行线，方便对比校正前的图片以得知校正效果。
  
3.获取视差图:

  本设计选用的SGBM匹配算法对图像进行立体匹配计算，同时获取视差图。在实验中本设计将摄像头对准障碍物。为了减少立体匹配的计算量，开始时使用OpenCV视觉库中的cvtColor函数对图像进行灰度化处理。随后利用上文提到的立体校正算法对图像进行校正，使两相机所示图像在同一平面。
完成这些步骤后开始调用OpenCV中的StereoSGBM_create函数来对预处理后的图像进行视差计算，并生成视差图。

4.测距

  得到视差图后，计算视差，将视差图转换为深度图，至此，图像中每个像素点都对应真实世界的距离。

避障逻辑图，本项目主要集中在双目视觉的测距，和循迹上，对避障的精度不高，只是简单设计避障逻辑，后续再做算法


<img width="222" alt="image" src="https://github.com/Yang-999-fc/A-smart-car-based-on-a-Raspberry-Pi/assets/57994308/22f00e0d-5bd6-4a9b-a595-3711bfb97dbf">

循迹模块主要是三路红外传感器实现，采用PID算法控制。


