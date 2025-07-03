# 👥 About the Team – **Team Integral Constant**
---

## 👥 **Team Integral Constants"**

- **Nayeem Islam Shanto**  – Team Leader, Hardware Developer | [islamnayeem386@gmail.com](mailto:islamnayeem386@gmail.com)
- **Dipanjan Ghosh**  –  Schematic and Documentation Developer | [priomghosh33@gmail.com](mailto:priomghosh33@gmail.com)
- **L.M. Mahir Labib** – Software Developer | [labib.programmer@gmail.com](mailto:labib.programmer@gmail.com)

**Team Origin**: Bangladesh

---
---

### 🌟 Why the name *Integral Constant*?

In mathematics, the **integral constant** often appears as “+ C” in integration. It's subtle, usually overlooked, and yet **essential to completeness**. We named our team **Integral Constant** because, like the constant, **we exist quietly but importantly** in every problem we solve — maybe not always visible, but always there, contributing meaningfully.

Also... the value of the integral constant is kind of *weird*, right? That’s us — a bit odd, unpredictable, but always essential.

---

### 🧠 Who We Are

We are a group of passionate young engineers from Bangladesh, driven by curiosity, creativity, and a love for solving real-world problems with technology. Each team member plays a crucial role — from mechanical design to software development to integration testing.

This project reflects not just our technical skills, but our teamwork, perseverance, and belief in building meaningful things with limited resources.

---

### 📁 Our Repository Structure

To keep everything organized and transparent, we structured this GitHub repo with clarity and purpose. Here’s what each folder contains:

- `t-photos/`: Our official team photo and a fun team moment — because engineering is serious fun.
- `v-photos/`: Photos of the robot from all angles — top, bottom, front, back, sides — for full transparency.
- `video/`: Link to our demo and performance video.
- `schemes/`: Wiring and circuit diagrams, showcasing all sensors, modules, and power connections.
- `src/`: All the source code we wrote for robot control, sensors, PID logic, lap counting, and obstacle handling.
- `models/`: 3D printable parts, like sonar mounts and brackets — modeled in Autodesk Fusion 360.
- `Mobility Management/`: Our approach to movement — motor selection, differential system, steering design, and engineering principles.
- `Power and Sense Management/`: Detailed write-up of how power is distributed and how sensors are placed and calibrated.
- `Obstacle Management/`: Strategy for handling red and green blocks, including servo positioning and avoidance decisions.
- `other/`: Extra docs, debugging notes, component datasheets, or setup manuals — basically everything we couldn’t categorize neatly but didn’t want to lose.

---

### Mission Overview for WRO Future Engineers Rounds

<table>
  <tr>
    <td width="50%" valign="top" align="left">
      <h3>🏁 Round 1: Lap Completion</h3>
      <p>In <strong>Round 1</strong>, the robot must autonomously complete <strong>three laps</strong> on a pre-defined track. The goal of this round is for the bot to demonstrate stable navigation and precise lap tracking without any obstacle avoidance requirements.</p>
      <ul>
        <li><strong>Objective</strong>: Complete three laps on the track within the allotted time.</li>
        <li><strong>Key Tasks</strong>: Accurate path-following, speed control, and lap counting.</li>
      </ul>
      <div align="center">
        <br><br><br><br><br>
        <img src="https://github.com/user-attachments/assets/823b29fa-8c92-479e-a78a-9fc96c407858" alt="Round 1 WRO Track" width="250" height="180" />
      </div>
    </td>
    <td width="50%" valign="top" align="left">
      <h3>🏆 Round 2: Lap Completion with Obstacle Avoidance and Parking</h3>
      <p>In <strong>Round 2</strong>, the bot must complete <strong>three laps</strong> while avoiding green and red obstacles:</p>
      <ul>
        <li><strong>Green Obstacles</strong>: The bot should move <strong>left</strong> to avoid.</li>
        <li><strong>Red Obstacles</strong>: The bot should move <strong>right</strong> to avoid.</li>
      </ul>
      <p>After completing the laps, the bot must accurately park within a designated zone.</p>
      <ul>
        <li><strong>Objective</strong>: Complete three laps, avoid obstacles, and park in the designated area.</li>
        <li><strong>Tasks</strong>: Obstacle detection, color-based avoidance, and precision parking.</li>
      </ul>
      <div align="center">
        <img src="https://github.com/user-attachments/assets/b578392d-b443-4315-8fe3-f03af828c39a" alt="Round 2 WRO Track" width="250" height="180" />
      </div>
    </td>
  </tr>
</table>

---
>[!IMPORTANT]
>**Important: WRO Future Engineers Rulebook**
>* **Thorough Reading:** Ensure that you thoroughly read the **WRO Future Engineers 2025 Rulebook** to understand all rules and guidelines.
>* **Official Link:** Access the rulebook here: [🔗 WRO Future Engineers 2025 Rulebook]([https://wro-association.org/competitions/future-engineers/](https://wro-association.org/wp-content/uploads/WRO-2024-Future-Engineers-Self-Driving-Cars-General-Rules.pdf).

---


We built this robot and documentation not just for a competition, but to share, inspire, and collaborate. If you’re here to learn, contribute, or replicate — welcome to the constant.

# 🧩 Hardware Components

<div align="center">

| Component                          | Quantity | Image |
|------------------------------------|:--------:|:-----:|
| **Raspberry Pi 4B**                | 1x       | <img src="https://www.pngkey.com/png/detail/51-511751_raspberry-pi-3-model-b-raspberry-pi-3.png" width="120"/> |
| **ESP32 Dev Board**               | 1x       | <img src="https://hr.mouser.com/images/espressifsystems/lrg/ESP32-DevKitC-32E_SPL.jpg" width="120"/> |
| **HC-SR04 Ultrasonic Sensor**     | 4x       | <img src="https://github.com/Simplified-Shanto/Team-Integral-Constant-WRO-FE-2025/blob/main/Power%20and%20Sense%20Management/assets/sr04.png?raw=true" width="120"/> |
| **Raspberry Pi Camera Module**     | 1x       | <img src="https://github.com/Simplified-Shanto/Team-Integral-Constant-WRO-FE-2025/raw/main/Power%20and%20Sense%20Management/assets/pi.png" width="120"/> |
| **L298N Motor Driver Module**      | 1x       | <img src="https://lastminuteengineers.com/wp-content/uploads/arduino/L298N-Module-Speed-Control-Pins.png" width="120"/> |
| **XL4016 Buck Converter**          | 1x       | <img src="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcSYr0qJPRfCRzA4zGHCbmpqmkJpAZI1X-VQdg&s" width="120"/> |
| **XL6009 Boost Converter**         | 1x       | <img src="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcTcvM16SK7WybpZ8yyatmC3_vKXrEb_ExC5zQ&s" width="120"/> |
| **MG90S Micro Servo Motor**        | 1x       | <img src="https://www.az-delivery.de/cdn/shop/products/mg90s-micro-servomotor-kompatibel-mit-arduino-210293.jpg?v=1679402521&width=1500" width="120"/> |
| **25GA 800RPM DC Geared Motor**    | 1x       | <img src="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcQYv6aRj5U4EZF46Bf0X_Whdv9TKMyS-YCozQ&s" width="120"/> |
| **MPU6050 Gyroscope Sensor**       | 1x       | <img src="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcQPqiDpwavMnuoqXv4AHckHhyJnoHwIkhqRiA&s" width="120"/> |

</div>
---
> ⚠️ **Note**: If any image doesn't load in GitHub's markdown preview, make sure the URL is correct or replace it with your own uploaded image links.



# 🚗 Mobility Management

Mobility is at the heart of our robot's design. We aimed to create a drive and steering system that mimics real-world vehicles while optimizing for speed, torque, and control on WRO tracks.

---

## 🦾 Rear Axle Power Distribution

We designed our rear-wheel drive system around **realistic mechanical dynamics**, utilizing a **LEGO differential gear** instead of a solid axle. This allows each wheel to rotate at different speeds during turns — just like in real cars — improving traction and reducing drag.

### 🔧 Motor Selection:
- We first experimented with standard **TT motors**, but they lacked the torque and RPM needed.
- We upgraded to **25GA 800 RPM DC geared motors**, which offer high torque and fast rotation, suitable for both lap completion and obstacle navigation.

---

## 🔁 Differential System

A **LEGO differential gear** is used on the rear axle. This gear allows independent rotation of the left and right wheels, providing:
- ✅ Smoother turning
- ✅ Reduced wheel slippage
- ✅ Real-world vehicle behavior emulation

The differential is connected directly to the 25GA motor, delivering balanced torque to both wheels.

---

## 🧭 Steering Mechanism

### ⚙️ Initial Design: Ackermann Steering

We initially implemented an **Ackermann Steering System**, commonly used in real vehicles. This system allows the inner and outer wheels to turn at different angles during cornering, minimizing slip and improving efficiency.

**Advantages:**
- ✅ Realistic turning geometry
- ✅ Efficient tire rotation paths

However, the mechanical complexity and setup space required made it difficult to tune within our chassis design.

---

### 🔄 Final Design: Rack and Pinion Steering

To simplify the design and enhance control, we transitioned to a **Rack and Pinion** steering system. This setup:
- Converts rotational motion from the steering motor to linear motion to turn the wheels
- Is simpler to mount and tune
- Offers consistent steering angles and tighter control

**Powered by:** Servo motor controlled by Raspberry Pi GPIO pins

---

## 🛠️ Chassis & Assembly Notes

- **Material**: Custom 3D printed base + LEGO Technic frame
- **Design Tools**: Autodesk Fusion 360 (for custom parts like sonar mounts, motor brackets)
- **Mounting Strategy**:
  - Front: Rack and pinion + servo motor
  - Rear: 25GA motor with LEGO differential
  - Components are evenly distributed for optimal center of mass

---

## 🧪 Testing & Calibration

- ✅ PID tuning done for lap accuracy
- ✅ Rack and pinion tested for max steering angle and stability
- ✅ Motors calibrated for obstacle response delay and RPM

---

## 🔐 Safety Considerations

- All electrical joints are soldered and insulated
- Gear systems are housed to prevent accidental entanglement
- LiPo battery is securely mounted with a cutoff mechanism

---

By combining a **25GA high-RPM motor**, **LEGO differential**, and a **custom rack and pinion system**, our robot delivers realistic movement, efficient turning, and powerful drive — ready for the WRO Future Engineers challenge.
