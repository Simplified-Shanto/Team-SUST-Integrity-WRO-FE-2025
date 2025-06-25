#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("MPU6050 Gyro Test");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  delay(100);
}

void loop() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  Serial.print("Gyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" rad/s, Y: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" rad/s, Z: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" rad/s");

  delay(500);
}
