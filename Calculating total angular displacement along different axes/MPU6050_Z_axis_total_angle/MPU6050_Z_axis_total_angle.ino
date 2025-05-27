#include <Wire.h>          // Enables I2C communication
#include <MPU6050.h>       // Handles MPU6050-specific functions

MPU6050 mpu;               // Create an MPU6050 object

float gyroZ;               // Variable to store Z-axis angular velocity
float angleZ = 0;          // Accumulated Z-axis angular displacement (degrees)

unsigned long lastTime = 0; // Store time of last measurement

void setup() {
  Serial.begin(115200);           // Start serial communication at 115200 bps
  Wire.begin();                   // Begin I2C as master

  mpu.initialize();              // Initialize MPU6050 sensor
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);                   // Halt if sensor not found
  }

  Serial.println("MPU6050 initialized successfully.");
  lastTime = millis();           // Initialize time reference
}

void loop() {
  // Get the current time
  unsigned long currentTime = millis();                // Current time in ms
  float dt = (currentTime - lastTime) / 1000.0;        // Convert to seconds
  lastTime = currentTime;                              // Update lastTime for next loop

  // Read gyroscope raw data (in degrees/sec)
  gyroZ = mpu.getRotationZ() / 131.0;  // Convert raw to deg/sec (131 LSB/(°/s) for ±250°/s range)

  // Integrate angular velocity to find angular displacement
  angleZ += gyroZ * dt;  // Δθ = ω × Δt

  // Output results
  Serial.print("Gyro Z (deg/s): ");
  Serial.print(gyroZ);
  Serial.print(" | Total Angle Z (°): ");
  Serial.println(angleZ);

  delay(10); // Small delay (~100Hz sampling rate)
}
