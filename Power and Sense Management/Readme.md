**Power and Sense Management**
====
In the realm of robotics, effective power and sense management is the linchpin of our robot's performance. Our cutting-edge system ensures optimal resource allocation, revolutionizing how robots interact with their surroundings.

## 1. Power Management: ![Screenshot 2025-07-03 163932](https://github.com/user-attachments/assets/e7bbecae-5366-49e5-855b-807b34e9d7d6)

<img align="center" width="565" alt="Power" src="https://github.com/LabibProjects/Bangladesh_Team-Electrobot/assets/133244520/43bbd256-99ae-4685-831e-1c7c8020c8ad">
">

   At first from lithium ion cell we are fetching 3.7v and being 12v by XL60009. then HW-411A LM2596 are cosuming power to 3.3v for the sensor e.g. Pi Camera, SR04 ultrasonic sensor and gyroscope MPU 6050.  Laterby we are suplying 5v to servo 12v to BTS7960 and motor being powered to.
## 2. Sensors: 
<img width="571" alt="Sensor" src="https://github.com/LabibProjects/Bangladesh_Team-Electrobot/assets/133244520/43bbd256-99ae-4685-831e-1c7c8020c8ad">

- #### **Ultrasonic Sensor:**
   <img width="150" alt="" src="![image](https://github.com/user-attachments/assets/5fb3dc7f-6559-451b-9cf4-848ec7dbd444)
">

  
   The SR04 is an ultrasonic distance sensor used for measuring distances. We're powering it with a stable 5V supply obtained from an HW-411A LM2596 voltage regulator connected to an 11.1V LiPo battery. This setup ensures reliable distance measurements and efficient power management for our robot.


- #### **Pi Camera:**
  <img width="150" alt="" src="![image](https://github.com/user-attachments/assets/31fbd0ef-453f-4fc3-ace3-861052883255)
">

The Raspberry Pi Camera is a high-definition vision module capable of capturing images and streaming video, commonly used in robotics and computer vision applications. In our configuration, we use the Raspberry Pi Camera Module connected to a Raspberry Pi 4B, which serves as the core processing unit for advanced image processing tasks. The camera interfaces directly with the Raspberry Pi via the CSI (Camera Serial Interface) port, enabling high-speed data transfer. 
- #### **Gyroscope:**
  <img width="150" alt="MPU" src="https://github.com/LabibProjects/Bangladesh_Team-Electrobot/assets/133244520/91c8ec6c-5d88-4726-917d-88b6b141fade">


The MPU6050 is a popular gyroscope and accelerometer sensor used for motion sensing and orientation tracking in various applications, including robotics and motion-controlled devices. In our setup, the MPU6050 is powered by a 5V supply sourced from an ESP32 Dev Board, which itself receives power from an HW-411A LM2596 voltage regulator. This HW-411A LM2596 regulator is connected to an XL60009 module, and the entire system is powered by an 3.7V lithium ion cell.
