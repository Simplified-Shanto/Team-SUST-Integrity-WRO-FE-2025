#include <ESP32Servo.h>

Servo myServo;

#define SERVO_PIN 17       // Adjust this to your servo's pin
#define SERVO_MIN 0        // Minimum angle
#define SERVO_MID 90       // Middle angle
#define SERVO_MAX 180      // Maximum angle

void setup() {
  myServo.setPeriodHertz(50);  // Standard 50Hz for servos
  myServo.attach(SERVO_PIN, 500, 2400);  // Min and max pulse width
  delay(500);

  myServo.write(SERVO_MIN);
  delay(1000);

  myServo.write(SERVO_MID);
  delay(1000);

  myServo.write(SERVO_MAX);
  delay(1000);

  myServo.write(SERVO_MID);
  delay(1000);
}

void loop() {
  // Nothing here
}
