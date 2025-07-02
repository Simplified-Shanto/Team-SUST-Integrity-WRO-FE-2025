#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Kalman Filter Variables for X-axis
float angleX = 0;          // Estimated angle (X-axis)
float biasX = 0;           // Gyro bias (X-axis)
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;  // Covariance matrix

// Gyro Integration Variables
float totalDisplacementX = 0;  // Cumulative angle (degrees)
unsigned long lastTime = 0;
float dt;                      // Time step (seconds)

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }
  Serial.println("MPU6050 initialized!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  lastTime = micros();
}

void kalmanUpdateX(float accAngleX, float gyroRateX, float dt) {
    // Prediction (Gyro-based)
    angleX += (gyroRateX - biasX) * dt;
    P_00 += dt * (P_11 - P_01 - P_10 + 0.1);  // Process noise
    P_01 -= dt * P_11;
    P_10 -= dt * P_11;
    P_11 += 0.003 * dt;  // Gyro noise

    // Update (Accelerometer-based)
    float y = accAngleX - angleX;
    float S = P_00 + 0.05;  // Measurement noise
    float K_0 = P_00 / S;
    float K_1 = P_10 / S;

    angleX += K_0 * y;
    biasX += K_1 * y;
    P_00 -= K_0 * P_00;
    P_01 -= K_0 * P_01;
    P_10 -= K_1 * P_00;
    P_11 -= K_1 * P_01;
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate dt (time since last reading)
  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0;  // Convert to seconds
  lastTime = now;

  // Accelerometer angle (X-axis)
  float accAngleX = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;

  // Kalman Filter Update
  kalmanUpdateX(accAngleX, g.gyro.x, dt);

  // Integrate gyro rate to get displacement
  totalDisplacementX += g.gyro.x * dt;

  // Print results
  Serial.print("Current Angle (X): ");
  Serial.print(angleX);
  Serial.print("° \t Total Displacement: ");
  Serial.print(totalDisplacementX);
  Serial.println("°");

  delay(10);  // Small delay for stability
}