#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float gyroZOffset = 0;
const int calibrationSamples = 200;

float angle = 0;                       // angle in degrees
const float angleThreshold = 70.0;     // detect turns ≥ 70°
const float stopThreshold = 0.03;      // Below this rad/s = not turning
const float noiseFilterAlpha = 0.8;    // Low-pass filter strength (0.0 to 1.0)

unsigned long lastMicros = 0;
float filteredGyroZ = 0;
bool hasTurned = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("== Sharp Turn Detection ==");

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  delay(200);

  calibrateGyroZ();
  lastMicros = micros();
}

void calibrateGyroZ() {
  Serial.println("Calibrating gyro Z... Stay still");
  float sum = 0;

  for (int i = 0; i < calibrationSamples; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sum += g.gyro.z;
    delay(5);
  }

  gyroZOffset = sum / calibrationSamples;
  Serial.print("Gyro Z offset: ");
  Serial.println(gyroZOffset, 6);
}

void loop() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0;  // delta time in seconds
  lastMicros = now;

  float rawGyroZ = -(g.gyro.z - gyroZOffset); // Inverted for correct turn direction
  filteredGyroZ = noiseFilterAlpha * filteredGyroZ + (1 - noiseFilterAlpha) * rawGyroZ;

  // Only integrate if rotation is above the threshold
  if (abs(filteredGyroZ) > stopThreshold) {
    angle += filteredGyroZ * dt * (180.0 / PI); // rad/s to degrees
  }

  // Check for sharp turn
  if (!hasTurned && abs(angle) >= angleThreshold) {
    hasTurned = true;

    if (angle > 0) {
      Serial.println("🔁 Sharp RIGHT Turn Detected");
    } else {
      Serial.println("↪️ Sharp LEFT Turn Detected");
    }

    // Keep tracking until turn is complete 
  }

  // if stop turning for a while, reset everything
  static unsigned long stillSince = 0;

  if (abs(filteredGyroZ) < stopThreshold) {
    if (stillSince == 0) stillSince = millis();
    else if (millis() - stillSince > 400) {
      hasTurned = false;
      angle = 0;
      stillSince = 0;
    }
  } else {
    stillSince = 0;  // Reset idle timer
  }

  delay(5); // Keep the loop responsive
}
