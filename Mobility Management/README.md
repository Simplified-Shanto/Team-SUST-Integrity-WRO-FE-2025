# Mobility Management
====

In the realm of mobility management for our vehicle, we meticulously select and implement motors based on engineering principles like speed, torque, and power, while also considering chassis design and component mounting for optimal performance. Assembly instructions and 3D CAD files are available for easy part fabrication.

## Motor Selection and Implementation:

### 1. Motor Selection:
- We initially experimented with **TT motors**, but they did not meet the required torque and speed benchmarks.
- Upgraded to **24GA 800 RPM DC Geared Motors**, offering improved torque output and higher rotational speed for stable and responsive movement.
- To replicate real-world vehicle behavior, we used a **LEGO differential gear** on the rear axle instead of a solid axle. This allows the rear wheels to rotate at different speeds during turns, providing smoother and more realistic cornering.

### 2. Motor Controller:
- The **L298N motor driver** was selected for its high current capacity and smooth PWM-based motor control.
- Integrated seamlessly with our **ESP32-based** microcontroller.

### 3. Implementation:
- Motors were wired to the motor driver, and the driver was interfaced with the ESP32 Dev Board.
- The rear axle was integrated with a LEGO differential system to enable differential wheel speeds during turning.

---

## Front Axle Steering System:

- Initially, we implemented an **Ackermann steering geometry**, which is known for replicating realistic vehicle steering angles and minimizing tire slippage during turns.
- After testing, we transitioned to a **Rack and Pinion steering system**, which offered better mechanical simplicity and precision for our use case.
- The rack and pinion system was designed using 3D CAD software and fabricated to ensure tight control over steering angle and response.

---

## Chassis Design and Selection:

### 1. Chassis Material:
- Constructed from **plastic base material** to ensure weightlessness and durability.

### 2. Chassis Shape:
- Optimized to accommodate all internal components while maintaining stability and proper ground clearance.

### 3. Mounting Components:
- Components were strategically arranged for even weight distribution and accessibility.
- Major mounted components include:
  - 24GA Motors with LEGO Differential Gear
  - Rack and Pinion Steering Mechanism
  - Ultrasonic and IR Sensors
  - MPU6050 Gyroscope Sensor
  - Pi Camera
  - ESP32-based Control Board
  
### 4. Engineering Principles:
- Careful calculation of **center of mass**, **wheelbase**, and **track width** to ensure stability and effective turning.
- The differential and rack & pinion systems improved the maneuverability and reduced strain on tires during sharp turns.

---

## Building and Assembly Instructions:

### 1. Step-by-Step Assembly:
- Detailed step-by-step guide covering chassis assembly, drivetrain mounting, and steering linkage.
- Wiring diagrams are provided for connecting all sensors, motors, and control components.

### 2. 3D Printed Parts:
- Custom-designed components (e.g., sonar mount, wheel rims, rack gear) were modeled using **Autodesk Fusion 360**.
- STL files and recommended materials/settings are available in the `models/` directory.

### 3. Testing and Calibration:
- Procedures for:
  - Steering calibration (center alignment, turning angles)
  - Motor speed tuning
  - Sensor data verification and orientation

### 4. Safety Considerations:
- Emphasized safety while handling LiPo batteries, high-speed motors, and voltage converters.
- All exposed connections were insulated and stress-tested under load.

---
