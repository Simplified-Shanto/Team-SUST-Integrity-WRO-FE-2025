# Autonomous Vehicle | Team Integral Constant | Bangladesh  
---

This repository contains engineering materials for a self-driven autonomous vehicle developed by **Team Integral Constant** from Bangladesh, participating in the **WRO Future Engineers 2025** competition.

---

## 👥 Team Members:

- Nayeem Islam Shanto 
- Dipanjan Ghosh
- L.M. Mahir Labib

---

## 📁 Repository Contents:

- `t-photos/`: Official and candid team photos  
- `v-photos/`: Six vehicle images (top, bottom, and all four sides)  
- `video/`: A file `video.md` containing the driving demo video link  
- `schemes/`: Schematic diagrams (PNG/PDF), including the complete electric system  
- `src/`: Code files for all programmed components  
- `models/`: 3D print, CNC, or laser cutting files (if applicable)  
- `Mobility Management/`: Chassis design, torque & power analysis, motor specs  
- `Power and Sense Management/`: Discussion on the power system and sensors  
- `Obstacle Management/`: Strategies to tackle competition obstacle challenges  
- `other/`: Documentation, datasets, SBC connection instructions, etc.  

---

## 🔍 Introduction

The **"Integral Constant Autonomous Vehicle"** project represents an intelligent and self-guided robotic vehicle, engineered for dynamic environments. Built around the **ESP32-based development board**, it integrates robust sensors, efficient power systems, and optimized software to achieve accurate navigation, lap tracking, and environmental awareness.

The design prioritizes modularity, real-time responsiveness, and adaptability to obstacles on track, making it ideal for WRO Future Engineers 2025.

---

## ⚙️ Electric System Design

Every component was selected through meticulous research and real-world testing. Here's the final hardware list:

- **ESP32 Dev Module** (core microcontroller)
- **0.96” OLED Display**
- **L298N Motor Driver Module**
- **12V DC Geared Motors**
- **MPU6050** (Gyroscope + Accelerometer)
- **IR Sensors** (Track detection & line following)
- **Push Buttons** (Start/Stop interface)
- **XL6009 Boost Converter**
- **MP1584 Buck Converter**
- **3S LiPo Battery (11.1V)**

A well-structured and labeled [schematic diagram](https://github.com/Simplified-Shanto/Team-Integral-Constant-WRO-FE-2025/blob/main/schemes/schematic.png) is available in the `schemes/` folder.

---

----
## Software Setup Procedure

![ide_dl](https://github.com/LabibProjects/Bangladesh_Team-Electrobot/blob/main/other/ide_dl.png)
- At first, we will have to download and install the most up-to-date version of the Arduino IDE on our computer. We can find the software at arduino.cc/en/Main/Software.

![ide_set_1](https://github.com/LabibProjects/Bangladesh_Team-Electrobot/blob/main/other/ide_set_1.png)   |  ![ide_set_2](https://github.com/LabibProjects/Bangladesh_Team-Electrobot/blob/main/other/ide_set_2.png)
- In our Arduino IDE, we need to go to File > Preferences, and then enter the following into the 'Additional Board Manager URLs' field. Finally, we click OK.
`https://espressif.github.io/arduino-esp32/package_esp32_index.json`

![ide_set_3](https://github.com/LabibProjects/Bangladesh_Team-Electrobot/blob/main/other/ide_set_3.png)
- On the left side of our IDE we will have to open our Board Manager . Press install button for the "ESP32 by Espressif Systems".

![ide_set_4](https://github.com/LabibProjects/Bangladesh_Team-Electrobot/blob/main/other/ide_set_4.png)
- Now we will have to select our board so on the upper left side of IDE we will select our board, "ESP32 DEV MODULE" then we will select our prefered board where we have connected our JRC board.

![ide_set_5](https://github.com/LabibProjects/Bangladesh_Team-Electrobot/blob/main/other/ide_set_5.png)
- Finally our task is to upload the code clicking "→" icon we upload our code. 

- Now we can turn on the toggle switch after placing robot on track and it will start doing the laps.
