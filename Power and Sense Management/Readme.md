**Power and Sense Management**
====
In the realm of robotics, effective power and sense management is the linchpin of our robot's performance. Our cutting-edge system ensures optimal resource allocation, revolutionizing how robots interact with their surroundings.

## 1. Power Management: 

<img width="571" alt="Sensor" src="https://github.com/Simplified-Shanto/Team-Integral-Constant-WRO-FE-2025/blob/main/Power%20and%20Sense%20Management/assets/power_management.png">


   At first from lithium ion cell we are fetching 3.7v and being 12v by XL60009. then HW-411A LM2596 are cosuming power to 3.3v for the sensor e.g. Pi Camera, SR04 ultrasonic sensor and gyro-sensor MPU 6050.  Laterby we are suplying 5v to servo 12v to BTS7960 and motor being powered to.
## 2. Sensors: 
<img width="571" alt="Sensor" src="https://github.com/Simplified-Shanto/Team-Integral-Constant-WRO-FE-2025/blob/main/Power%20and%20Sense%20Management/assets/sensor.png">

- #### **Ultrasonic Sensor:**
   <img width="150" alt="" src="https://github.com/Simplified-Shanto/Team-Integral-Constant-WRO-FE-2025/blob/main/Power%20and%20Sense%20Management/assets/sr04.png">

  
   The SR04 is an ultrasonic distance sensor used for measuring distances. We're powering it with a stable 5V supply obtained from an HW-411A LM2596 voltage regulator connected to an 11.1V LiPo battery. This setup ensures reliable distance measurements and efficient power management for our robot.


- #### **Pi Camera:**
  <img width="150" alt="" src="https://github.com/Simplified-Shanto/Team-Integral-Constant-WRO-FE-2025/blob/main/Power%20and%20Sense%20Management/assets/pi.png">

The Raspberry Pi Camera is a high-definition vision module capable of capturing images and streaming video, commonly used in robotics and computer vision applications. In our configuration, we use the Raspberry Pi Camera Module connected to a Raspberry Pi 4B, which serves as the core processing unit for advanced image processing tasks. The camera interfaces directly with the Raspberry Pi via the CSI (Camera Serial Interface) port, enabling high-speed data transfer. 
- #### **gyro-sensor:**
  <img width="150" alt="MPU" src="https://github.com/LabibProjects/Bangladesh_Team-Electrobot/assets/133244520/91c8ec6c-5d88-4726-917d-88b6b141fade">


The MPU6050 is a popular gyro-sensor and accelerometer sensor used for motion sensing and orientation tracking in various applications, including robotics and motion-controlled devices. In our setup, the MPU6050 is powered by a 5V supply sourced from an ESP32 Dev Board, which itself receives power from an HW-411A LM2596 voltage regulator. This HW-411A LM2596 regulator is connected to an XL60009 module, and the entire system is powered by an 3.7V lithium ion cell.
