#include <ESP32Servo.h>

#define SERVO_PIN 17

// === Motor Pins ===
#define IN1_PIN 16
#define IN2_PIN 4
#define PWM_PIN 2


Servo steeringServo;

#define ANGLE_FORWARD 90
#define ANGLE_LEFT 180
#define ANGLE_RIGHT 0

void setup() {
  Serial.begin(115200);

  // Servo setup for ESP32
  ESP32PWM::allocateTimer(0);
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400);

  Serial.println("Type: forward, left, or right");

  // Motor pins
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "forward") {
      steeringServo.write(ANGLE_FORWARD);
      Serial.println("Steering: Forward (90°)");
      delay(100);
      digitalWrite(IN1_PIN, HIGH);
      digitalWrite(IN2_PIN, LOW);
      analogWrite(PWM_PIN, 180);

    } else if (command == "left") {
      steeringServo.write(ANGLE_LEFT);
      Serial.println("Steering: Left (180°)");
      delay(100);
      digitalWrite(IN1_PIN, HIGH);
      digitalWrite(IN2_PIN, LOW);
      analogWrite(PWM_PIN, 180);

    } else if (command == "right") {
      steeringServo.write(ANGLE_RIGHT);
      Serial.println("Steering: Right (0°)");
      delay(100);
      digitalWrite(IN1_PIN, HIGH);
      digitalWrite(IN2_PIN, LOW);
      analogWrite(PWM_PIN, 180);

    } else {
      Serial.println("Invalid command. Use: forward, left, right");
      digitalWrite(IN1_PIN, LOW);
      digitalWrite(IN2_PIN, LOW);
    }
  }
}
