#include <ESP32Servo.h>

// === Motor Pins ===
#define IN1_PIN 16
#define IN2_PIN 4
#define EN_PIN 2          // PWM speed control pin
#define MOTOR_SPEED 180   // 0–255

// === LEDC PWM Config ===
#define MOTOR_PWM_CHANNEL 0
#define MOTOR_PWM_FREQ 5000
#define MOTOR_PWM_RES 8   // 8-bit resolution (0–255)

// === Servo Setup ===
#define SERVO_PIN 17
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2400

Servo steeringServo;

void setup() {
  Serial.begin(115200);

  // === Motor Pins Setup ===
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  // === Setup LEDC PWM for motor speed control ===
  ledcSetup(MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(EN_PIN, MOTOR_PWM_CHANNEL);

  // === Servo Setup ===
 
  steeringServo.attach(SERVO_PIN);

  Serial.println("Send: <motor_state> <servo_angle>");
  Serial.println("Example: 1 90  → Motor ON, servo to 90°");
  Serial.println("Example: 0 180 → Motor OFF, servo to 180°");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int spaceIndex = input.indexOf(' ');
    if (spaceIndex != -1) {
      int motorState = input.substring(0, spaceIndex).toInt();
      int servoAngle = input.substring(spaceIndex + 1).toInt();
      servoAngle = constrain(servoAngle, 0, 180);

      // Move servo
      steeringServo.write(servoAngle);
      Serial.print("Servo angle set to: ");
      Serial.println(servoAngle);

      // Control motor
      if (motorState == 1) {
        digitalWrite(IN1_PIN, HIGH);
        digitalWrite(IN2_PIN, LOW);
        ledcWrite(MOTOR_PWM_CHANNEL, MOTOR_SPEED);
        Serial.println("Motor: ON");
      } else {
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, LOW);
        ledcWrite(MOTOR_PWM_CHANNEL, 0);
        Serial.println("Motor: OFF");
      }
    } else {
      Serial.println("Invalid format. Use: <motor_state> <servo_angle>");
    }
  }
}
