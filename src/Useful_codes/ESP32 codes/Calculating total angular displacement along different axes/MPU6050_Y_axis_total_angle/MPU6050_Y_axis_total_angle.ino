#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


Adafruit_MPU6050 mpu;

// Kalman Filter Variables
float angleY = 0;          // Estimated angle (Y-axis)
float biasY = 0;           // Gyro bias (Y-axis)
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;  // Covariance matrix

// Gyro Integration Variables
float totalDisplacementY = 0;  // Cumulative angle (degrees)
unsigned long lastTime = 0;
float dt;                      // Time step (seconds)


void kalmanUpdateY(float accAngleY, float gyroRateY, float dt) {
    // Prediction (Gyro-based)
    angleY += (gyroRateY - biasY) * dt;
    P_00 += dt * (P_11 - P_01 - P_10 + 0.1);  // Process noise
    P_01 -= dt * P_11;
    P_10 -= dt * P_11;
    P_11 += 0.003 * dt;  // Gyro noise

    // Update (Accelerometer-based)
    float y = accAngleY - angleY;
    float S = P_00 + 0.05;  // Measurement noise
    float K_0 = P_00 / S;
    float K_1 = P_10 / S;

    angleY += K_0 * y;
    biasY += K_1 * y;
    P_00 -= K_0 * P_00;
    P_01 -= K_0 * P_01;
    P_10 -= K_1 * P_00;
    P_11 -= K_1 * P_01;
}


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


void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate dt (time since last reading)
    unsigned long now = micros();
    dt = (now - lastTime) / 1000000.0;  // Convert to seconds
    lastTime = now;

    // Accelerometer angle (Y-axis)
    float accAngleY = atan2(a.acceleration.x, a.acceleration.z) * 180 / PI;

    // Kalman Filter Update
    kalmanUpdateY(accAngleY, g.gyro.y, dt);

    // Integrate gyro rate to get displacement
    totalDisplacementY += g.gyro.y * dt;

    // Print results
    Serial.print("Current Angle (Y): ");
    Serial.print(angleY);
    Serial.print("° \t Total Displacement: ");
    Serial.print(totalDisplacementY);
    Serial.println("°");

    delay(10);  // Small delay for stability
}