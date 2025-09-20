# 👥 About the Team – **Team SUST Integrity**
---

## 👥 **Team SUST Integrity**

- **Nayeem Islam Shanto**  – Team Leader, Hardware Developer | [islamnayeem386@gmail.com](mailto:islamnayeem386@gmail.com)
- **Dipanjan Ghosh**  –  Schematic and Documentation Developer | [priomghosh33@gmail.com](mailto:priomghosh33@gmail.com)
- **L.M. Mahir Labib** – Software Developer | [labib.programmer@gmail.com](mailto:labib.programmer@gmail.com)

**Team Origin**: Bangladesh

---
# 📑 Table of Contents  

- [👥 About the Team – Team Integral Constant](#-about-the-team--team-integral-constant)  
  - [🌟 Why the name *Integral Constant*?](#-why-the-name-integral-constant)  
  - [🧠 Who We Are](#-who-we-are)  
  - [📁 Repository Structure](#-our-repository-structure)  
- [🎯 Mission Overview](#mission-overview-for-wro-future-engineers-rounds)  
  - [🏁 Round 1: Lap Completion (Open Round)](#-round-1-lap-completion-open-round)  
  - [🚧 Round 2: Obstacle Challenge Round](#-round-2-obstacle-challenge-round)  
- [🧩 Hardware Components](#-hardware-components)  
- [🚗 Mobility Management](#-mobility-management)  
  - [🦾 Rear Axle Power Distribution](#-rear-axle-power-distribution)  
  - [🔁 Differential System](#-differential-system)  
  - [🧭 Steering Mechanism](#-steering-mechanism)  
- [💻 Raspberry Pi Setup](#-setting-up-raspberry-pi-for-image-processing-with-opencv--python)  
  - [🔑 Enabling & Using VNC](#-enabling--using-vnc-on-raspberry-pi)  
  - [🔎 Static COM Port Setup](#-how-to-set-static-com-port-for-the-low-level-microcontroller)  
  - [📷 Test Webcam Feed](#test-your-webcam-feed)  
  - [⚡ Auto-start Python Script](#running-your-python-script-on-startup-of-raspberry-pi)  
  - [🔌 Safe Shutdown via Serial](#safe-shutdown-for-raspberry-pi-via-serial-command)  
- [🧪 Testing & Calibration](#-testing--calibration)  
- [🔐 Safety Considerations](#-safety-considerations)  
- [🧑‍💻 Obstacle Challenge Strategy](#-strategy-for-the-obstacle-challenge-round)  

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
## 🏁 Round 1: Lap Completion (Open Round)

Here’s the process flow of how the robot completes laps in Round 1:

![Open Round Process](schematic/open%20round%20pc.svg)

---

## 🚧 Round 2: Obstacle Challenge Round

Here’s the process flow of how the robot avoids obstacles in Round 2:

![Obstacle Round Process](schematic/obstacle%20round%20pc.svg)


---

We built this robot and documentation not just for a competition, but to share, inspire, and collaborate. If you’re here to learn, contribute, or replicate — welcome to the constant.

# 🧩 Hardware Components

<div align="center">

| Component                          | Quantity | Image |
|------------------------------------|:--------:|:-----:|
| **Raspberry Pi 4B**                | 1x       | <img src="https://www.pngkey.com/png/detail/51-511751_raspberry-pi-3-model-b-raspberry-pi-3.png" width="120"/> |
| **ESP32 Dev Board**               | 1x       | <img src="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcQWmwffDOI8gw4A1DdMETa1_DUL9Ho8iO5MAA&s" width="120"/> |
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



---

# 🚀 Setting Up Raspberry Pi for Image Processing with OpenCV & Python

You just unboxed a brand-new **Raspberry Pi**. Exciting! 🎉
Now let’s get it ready for image processing projects with OpenCV and Python.

---

## 🛠️ Initial Setup

1. **Download & Install Raspberry Pi Imager** on your computer.
2. In the **OS configuration menu**, set the following:

   * ✅ Hostname
   * ✅ Username
   * ✅ Password
   * ✅ Wi-Fi SSID & Password
   * ✅ Enable SSH
3. ✍️ Write down these credentials — you’ll need them for remote login later.
4. Once the OS image is written, insert the SD card into your Raspberry Pi and let it boot up.

---

## 🎮 Two Ways to Use Your Raspberry Pi

* **Option A**: Connect directly with an HDMI display, keyboard, and mouse.
* **Option B**: Connect remotely from your laptop/PC via **VNC (Virtual Network Computing)**.

We’ll focus on Option B since it’s super handy.

---

# 🔑 Enabling & Using VNC on Raspberry Pi

## 📡 Process 1: Connect via Ethernet Cable

1. Plug an **Ethernet cable** between your Raspberry Pi and your computer.
2. Wait until the **LAN port LEDs blink** — connection established!
3. On your computer, open **Command Prompt/Terminal**.
4. Type:

   ```bash
   ssh username@hostname
   ```

   Example:

   ```bash
   ssh admin@raspi.local
   ```
5. Enter your password when prompted.

💡 **What is SSH?**
SSH (Secure Shell) is a secure protocol that lets you remotely access and manage systems over a network. It encrypts communication to protect your data.

6. Once logged in, run:

   ```bash
   sudo raspi-config
   ```

   This opens a configuration menu:
   ![raspi-config-menu](https://github.com/user-attachments/assets/2ad4bf21-4cc7-443f-a69e-65558994bb8e)

7. Navigate to:
   **3 Interfacing Options → P3 VNC → Enable**
   ![enable-vnc](https://github.com/user-attachments/assets/9552cbb0-12a1-435f-bd0e-9439b577b457)

8. **Reboot** your Raspberry Pi.

9. On your computer, install & open **RealVNC Viewer**.

10. In the search bar, enter your Pi’s **hostname** (e.g., `raspi.local`).

11. First-time login will prompt for **username & password** → tick **“Remember Password”** if desired.

12. 🎉 You should now see the Raspberry Pi’s desktop GUI!

---

## 📶 Process 2: Connect via Wi-Fi (Wireless LAN)

1. Ensure your **Raspberry Pi** and **computer** are on the **same Wi-Fi network**.

   * If using a mobile hotspot, configure it with the same SSID & password you set during OS imaging.
2. Find your Raspberry Pi’s **IP address** on the network.

   * Tools like **Net Analyzer** (mobile app) make this easy.
3. From your computer, open Command Prompt/Terminal and type:

   ```bash
   ssh username@<raspberry-pi-ip>
   ```

   Example:

   ```bash
   ssh admin@10.149.112.111
   ```

   (Note: Here we use the **IP address**, not the hostname.)
4. Follow **Steps 7–12** from **Process 1** to enable VNC.
5. In RealVNC Viewer, enter the Pi’s **IP address**.
6. Log in with your username & password → check **“Remember Password”** if desired.
7. 🎉 The Raspberry Pi’s GUI should now appear on your computer!

---

✨ With VNC ready, you now have full graphical access to your Raspberry Pi — perfect for running **Python + OpenCV** image processing projects remotely!

---


<img width="654" height="368" alt="image" src="https://github.com/user-attachments/assets/2ad4bf21-4cc7-443f-a69e-65558994bb8e" />
  <img width="1120" height="286" alt="image" src="https://github.com/user-attachments/assets/7a67875d-62df-4610-a660-cb7e541b4db3" />
   <img width="554" height="289" alt="image" src="https://github.com/user-attachments/assets/9552cbb0-12a1-435f-bd0e-9439b577b457" />
   <img width="1106" height="293" alt="image" src="https://github.com/user-attachments/assets/1fbf855b-3cce-4c01-bd16-2631259a1b48" />


# How to set static com port for the low level microcontroller?


---

## 🔎 Step 1: Plug in your ESP32 and identify it

First, connect your ESP32 to your Raspberry Pi’s USB port.
Then, open a terminal and run:

```bash
lsusb
```

You’ll see a list of connected USB devices, like this:
![image](https://github.com/user-attachments/assets/d3ebd906-ed12-497b-8c47-4e51c6dc9645)




Take note of the **Vendor ID** (`1a86`) and **Product ID** (`7523`). These are your device’s fingerprints!

Now, let’s find its **serial number** (optional but recommended). Run:

```bash
udevadm info -a -n /dev/ttyUSB0
```

*(Replace `/dev/ttyUSB0` with the actual port assigned to your ESP32.Use the following command to find that)*
![image](https://github.com/user-attachments/assets/e19ba1fd-cdd1-463e-941e-fa568818ab8d)


Look for lines like:

```
ATTRS{idVendor}=="1a86"
ATTRS{idProduct}=="7523"
ATTRS{serial}=="0001"  # Example
```

If your device has a serial number, use it to **uniquely identify** it, especially if you have multiple ESP32s.

---

## ✍️ Step 2: Create the udev rule

Create a new rule file in `/etc/udev/rules.d/`.
You can call it something like:

```bash
sudo nano /etc/udev/rules.d/99-esp32.rules
```

Inside, add this line (replace with your actual values):

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="esp32_serial"
```

If you have a serial number too, add that:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ATTRS{serial}=="0001", SYMLINK+="esp32_serial"
```

This means:

* Whenever a device with **Vendor ID 1a86** and **Product ID 7523** is connected, udev will create a **symlink** (like an alias) at `/dev/esp32_serial`.

---

## 🔄 Step 3: Reload udev and test it

Reload udev rules:

```bash
sudo udevadm control --reload-rules
```

Then **unplug and plug back** your ESP32.

Check:

```bash
ls -l /dev/esp32_serial
```

You should see something like:

```
lrwxrwxrwx 1 root root 7 Jun 10 16:23 /dev/esp32_serial -> ttyUSB0
```

(Or whatever tty device it points to.)

---

## 🐍 Step 4: Use it in your Python script

Now, in your script:

```python
import serial

ser = serial.Serial('/dev/esp32_serial', 115200)
```

This way, no matter what port it lands on—`ttyUSB0`, `ttyUSB1`, `ttyACM0`—you’ll always connect using `/dev/esp32_serial`.

---

# Install python modules like opencv, numpy, time, pyserial using following command: 
sudo apt install python3-opencv python3-numpy python3-serial

## 🚀 Pro Tips

✅ If your ESP32 shows up as `/dev/ttyACM0` instead of `/dev/ttyUSB0`, adjust your `udevadm info` command accordingly.
✅ Use `udevadm monitor` to see real-time device events:

```bash
udevadm monitor --udev
```

---



---

## Test Your Webcam Feed

Before running the full OpenCV script, it’s important to make sure your camera is working properly. This avoids unnecessary debugging later if the issue is just with the webcam feed.

### Steps:

1. Make sure your Raspberry Pi is set up and your webcam is connected.

2. Install OpenCV for Python (if not already done):

   ```bash
   pip install opencv-python
   ```

3. Create a new Python file called `test_camera.py` and paste the following code:

   ```python
   import cv2

   cap = cv2.VideoCapture(0)  # 0 is usually the default camera

   if not cap.isOpened():
       print("Error: Could not open camera.")
       exit()

   while True:
       ret, frame = cap.read()
       if not ret:
           print("Error: Failed to capture frame.")
           break

       cv2.imshow("Webcam Feed", frame)

       # Press 'q' to exit the window
       if cv2.waitKey(1) & 0xFF == ord('q'):
           break

   cap.release()
   cv2.destroyAllWindows()
   ```

4. Run the script:

   ```bash
   python test_camera.py
   ```

5. A window should pop up showing your live webcam feed.

   * If you see the video feed, your camera is working fine.
   * If you don’t, check if your camera is properly connected and detected by the system using:

     ```bash
     ls /dev/video*
     ```

---



---

# Running your Python Script on Startup of Raspberry Pi

If you want your Python script to automatically run when your Raspberry Pi boots up (without needing a monitor, keyboard, or VNC), follow these steps. We’ll use a **systemd service**, which is the recommended modern way.

---

## 1. Locate Your Script

Make sure your Python script is at the correct path. In this guide, we’ll assume your script is here:

```
/home/admin/Desktop/startscript.py
```

Check that the script has a proper shebang at the top (this tells Linux to use Python 3):

```python
#!/usr/bin/env python3
```

---

## 2. Make the Script Executable

Run this command in the terminal:

```bash
chmod +x /home/admin/Desktop/startscript.py
```

---

## 3. Create a systemd Service File

Create a new service definition:

```bash
sudo nano /etc/systemd/system/startscript.service
```

Paste the following:

```ini
[Unit]
Description=Startscript for Line Counter
After=multi-user.target

[Service]
ExecStart=/usr/bin/python3 /home/admin/Desktop/startscript.py
WorkingDirectory=/home/admin/Desktop
StandardOutput=append:/home/admin/startscript.log
StandardError=append:/home/admin/startscript.log
Restart=always
RestartSec=5
User=admin

[Install]
WantedBy=multi-user.target
```

If you want the program run once per boot, make Restart=no and remove the line RestartSec=5. 
This won't allow it to rerun the program if it ends its execution by some internal logic, command or crashes in the same boot session. 


Save and exit (`CTRL+O`, `ENTER`, `CTRL+X`).

---

## 4. Enable the Service

Reload systemd and enable your new service so it runs on every boot:

```bash
sudo systemctl daemon-reload
sudo systemctl enable startscript.service
```

You can start it immediately (without rebooting) using:

```bash
sudo systemctl start startscript.service
```

---

## 5. Check Logs and Status

* Check if your script is running:

  ```bash
  systemctl status startscript.service
  ```
* View live logs:

  ```bash
  journalctl -u startscript.service -f
  ```
* Or check the saved log file:

  ```bash
  cat /home/admin/startscript.log
  ```

---

## 6. Stop or Disable Later

To stop the script from running:

```bash
sudo systemctl stop startscript.service
```

To remove it from startup:

```bash
sudo systemctl disable startscript.service
```

---

✅ That’s it! Your Python script will now start automatically whenever your Raspberry Pi boots up — even without HDMI or network connected.

---




# Safe Shutdown for Raspberry Pi via Serial Command

Normally, if you unplug the power from your Raspberry Pi without shutting it down first, you risk corrupting the SD card or losing data. To prevent this, the Pi should always be shut down properly before the power is removed.

In this setup, we achieve a graceful shutdown without needing a display (HDMI) or remote access (VNC/SSH).

How it works

A low-level microcontroller (e.g., Arduino/ESP) detects a trigger event (such as a button press, sensor signal, or external condition).

It sends a shutdown command over Serial (UART/USB) to the Raspberry Pi.

A Python script running on the Pi (using PySerial
) listens for this command.

Once received, the script executes:

sudo shutdown now


The Pi powers down safely, protecting the filesystem and ensuring no data loss.

Why this matters

✅ Prevents SD card corruption caused by abrupt power cuts

✅ Works without a monitor, keyboard, or VNC session

✅ Can be triggered by just a button press or any hardware event handled by the microcontroller





---
## 🧪 Testing & Calibration

- ✅ PID tuning done for lap accuracy
- ✅ Rack and pinion tested for max steering angle and stability
- ✅ Motors calibrated for obstacle response delay and RPM

---

## 🔐 Safety Considerations

- All electrical joints are soldered and insulated
- Gear systems are housed to prevent accidental entanglement
- Battery is securely mounted with a cutoff mechanism

---

By combining a **25GA high-RPM motor**, **LEGO differential**, and a **custom rack and pinion system**, our robot delivers realistic movement, efficient turning, and powerful drive — ready for the WRO Future Engineers challenge.


---
---

# 🧑‍💻 Strategy for the Obstacle Challenge Round

Our current strategy for the Obstacle Challenge round is to avoid obstacles by changing the setpoint in the PID loop of our robot's main program.

- After the program starts the robot would at first determine the round direction(clockwise or anticlockwise) by observing which line(either orange or blue) comes first
- Then when it encounters a red object, it changes its setpoint from being into the middle towards the right wall(10 cm away from the right wall). The same approach is followed in case of a green object, only the difference being that it now follows the left wall.
- If it does not encounter any object the setpoint is again set to the center.


