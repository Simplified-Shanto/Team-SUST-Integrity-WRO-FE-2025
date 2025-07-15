#include <Preferences.h>
#include "motors.h"
#include "button.h"
#include "servo_motor.h"
#include "sonar.h"
#include "display.h"

Preferences preferences;
double Kp = 0;
double Kd = 0;
int debugPrint = 0;  //Whether we want to print all the variables to the OLED display.

int wallSide = 0;  // 0 = left wall, 1 = right wall
int wallDistance = 30;


#define LINE_THRESHOLD 1650
#define TARGET_LINE_COUNT 24

int lineCount = 0;
bool lineDetected = false;



void setup() {

  Serial.begin(115200);
  preferences.begin("wrobot", false);
  Kp = preferences.getDouble("Kp", 0);
  Kd = preferences.getDouble("Kd", 0);
  debugPrint = preferences.getInt("dP", 0);  //dP = debugPrint
  forwardSpeed = preferences.getInt("speed", 0);

  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  ESP32PWM::allocateTimer(0);
  steering_servo.setPeriodHertz(50);                     // standard 50 hz servo
  steering_servo.attach(steering_servo_pin, 500, 2400);  // attaches the servo on pin 18 to the servo object
  steering_servo.write(midAngle);
  delay(1000);
  steering_servo.write(rightAngle);
  delay(1000);
  steering_servo.write(midAngle);
  delay(1000);
  steering_servo.write(leftAngle);
  delay(1000);


  ledcSetup(LEDC_CHANNEL, 1000, 8);     // Set LEDC channel, frequency, and resolution
  ledcAttachPin(pwmPin, LEDC_CHANNEL);  // Attach the GPIO pin to the LEDC channel
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextColor(1);
  display.setTextSize(1);
  display.clearDisplay();
}

double value = 0;     // Stores the difference between the left and right readings.
double setpoint = 0;  //The amount of difference in reading of the two ultrasonic sensor we want.
double error = 0;     // The difference between value and setpoint.
double lastError = 0;

int setPoint = 0;  //Setpoint for the difference between the readings of the left and right sonar.

void loop() {
  checkButton();
  if (!gameStarted) return;

  int frontDistance = middleSonar.ping_cm();
  int leftDistance = leftSonar.ping_cm();
  int rightDistance = rightSonar.ping_cm();

  // Fail-safe values
  if (leftDistance == 0) leftDistance = 100;
  if (rightDistance == 0) rightDistance = 100;
  if (frontDistance == 0) frontDistance = 100;

  // Auto wall switch if one side is more stable
  // if (leftDistance < 40 && rightDistance > 60) {
  //   wallSide = 0;  // follow left
  // } else if (rightDistance < 40 && leftDistance > 60) {
  //   wallSide = 1;  // follow right
  // }

  // Obstacle avoidance: if too close in front, make a hard turn
  if (frontDistance < 30) {
    steering_servo.write((wallSide == 0) ? rightAngle : leftAngle);
    goForward(80);
    delay(500);
    return;
  }

  // Wall following logic
  int currentWallDist = (wallSide == 0) ? leftDistance : rightDistance;
  int oppositeWallDist = (wallSide == 0) ? rightDistance : leftDistance;

  // Wall disappeared (likely a corner), turn toward it
  if (currentWallDist > 2 * wallDistance) {
    steering_servo.write((wallSide == 0) ? leftAngle : rightAngle);
    goForward(80);
    delay(400);
    return;
  }

  // PID logic to maintain target distance from wall
  value = currentWallDist;
  error = value - wallDistance;
  double PIDangle = error * Kp + (error - lastError) * Kd;
  lastError = error;

  int steer_angle = midAngle;
  if (PIDangle > 0) {
    steer_angle = midAngle + min(halfAngleRange, PIDangle);
  } else {
    steer_angle = midAngle - min(halfAngleRange, abs(PIDangle));
  }

  steering_servo.write(steer_angle);
  goForward(forwardSpeed);

  int irValue = analogRead(IRpin);

  // Rising edge detection: only count once per line
  if (irValue > LINE_THRESHOLD && !lineDetected) {
    lineCount++;
    lineDetected = true;

    // Optional: short buzzer feedback
    digitalWrite(buzzerPin, HIGH);
    delay(50);
    digitalWrite(buzzerPin, LOW);
  }

  if (irValue <= LINE_THRESHOLD) {
    lineDetected = false;
  }

  // Stop the bot after 24 lines
  if (lineCount >= TARGET_LINE_COUNT) {
    goForward(0);
    steering_servo.write(midAngle);
    gameStarted = 0;

    // Optional: turn on LED or buzzer
    digitalWrite(ledPin, HIGH);
    digitalWrite(buzzerPin, HIGH);
    delay(300);
    digitalWrite(buzzerPin, LOW);
    digitalWrite(ledPin, LOW);
    return;
  }

  // Debug display
  if (debugPrint == 1) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("L:");
    display.print(leftDistance);
    display.print(" R:");
    display.print(rightDistance);
    display.print(" F:");
    display.println(frontDistance);
    display.print("Wall:");
    display.print(wallSide == 0 ? "L" : "R");
    display.print(" D:");
    display.println(wallDistance);
    display.print("PID:");
    display.println(PIDangle);
    display.print("Lines:");
    display.println(lineCount);
    display.display();
  }
}
