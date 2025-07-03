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
int turnFlag = 0;
int turnCount = 0;
double value = 0;     // Stores the difference between the left and right readings.
double error = 0;     // The difference between value and setpoint.
double lastError = 0; 
double setPoint = 0;  // The amount of difference in reading of the two ultrasonic sensor we want.
unsigned long turnTimer = millis();
#define turn_delay 1500  // milliseconds


void checkTurn() {
  if (turnFlag == 0) {
    turnCount += 1;
    turnFlag = 1;
    buzz(100);  //Enable the buzzer for the argument duration
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  preferences.begin("wrobot", false);
  Kp = preferences.getDouble("Kp", 0);
  Kd = preferences.getDouble("Kd", 0);
  debugPrint = preferences.getInt("dP", 0);  //dP = debugPrint
  forwardSpeed = preferences.getInt("speed", 0);
  setPoint = preferences.getDouble("setPoint",0); //The amount of difference in reading of the two ultrasonic sensor we want.

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
  // steering_servo.write(rightAngle);
  // delay(1000);
  // steering_servo.write(midAngle);
  // delay(1000);
  // steering_servo.write(leftAngle);
  // delay(1000);
  ledcSetup(LEDC_CHANNEL, 1000, 8);     // Set LEDC channel, frequency, and resolution
  ledcAttachPin(pwmPin, LEDC_CHANNEL);  // Attach the GPIO pin to the LEDC channel
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextColor(1);
  display.setTextSize(1);
  display.clearDisplay();
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil(';');

    char constant_name = command[0];  //The type of action the remote wants us to take.
    String constant_value_string = command.substring(2, command.length());
    // Serial.print("constant_value_string = ");
    // Serial.println(constant_value_string);
    float constant_value = constant_value_string.toFloat();
    // Serial.print("constant value = ");
    // Serial.println(constant_value); //Though there are more decimal places in the variable,
    //                                 // the print function only prints upto two decimal places
    if (command == "x") {  // Commands the car to stop
      gameStarted = 0;
      goForward(0);
      steering_servo.write(midAngle);
    } else if (command == "y") {
      gameStarted = 1;
      goForward(forwardSpeed);
    } else {
      switch (constant_name) {
        case 'p':  //Proportional of PID
          Kp = constant_value;
          preferences.putDouble("Kp", Kp);
          break;
        case 'd':  //Derivative of PID
          Kd = constant_value;
          preferences.putDouble("Kd", Kd);
          break;
        case 's':  //Speed of vehicle
          forwardSpeed = int(constant_value);
          preferences.putInt("speed", forwardSpeed);
          break;
        case 'D':  //Debug print flag
          debugPrint = int(constant_value);
          preferences.putInt("dP", debugPrint);
          display.clearDisplay();
          display.display();
          // delay(2000);
        case 'S': //Setpoint setup
          setPoint = double(constant_value); 
          preferences.putDouble("setPoint", setPoint);
        default:
          break;
      }
      preferences.end();  // Saves variables to EEPROM
    }
  }

  int frontDistance = middleSonar.ping_cm();
  int leftDistance = leftSonar.ping_cm();
  int rightDistance = rightSonar.ping_cm();
  int backDistance = backSonar.ping_cm();

  if (leftDistance == 0) {
    leftDistance = 200;
    if (turnFlag == 0) {
      turnFlag = 1;
      turnTimer = millis();
      turnCount++;
      digitalWrite(ledPin, HIGH);
    }
  } else if (rightDistance == 0) {
    rightDistance = 200;
    // if (turnFlag == 0) {
    //   turnFlag = 1;
    //   turnTimer = millis();
    //   turnCount++;
    //   digitalWrite(ledPin, HIGH);
    // }
  }

  if (millis() - turnTimer > turn_delay) {
    turnFlag = 0;
    digitalWrite(ledPin, LOW);
  }

  if (turnCount >= 12) {
    gameStarted = 0;
    goForward(0); //Stops the car. 
    digitalWrite(in1Pin, LOW); 
    digitalWrite(in2Pin, LOW); 
    //turnCount = 0;
    turnFlag = 0;
    steering_servo.write(midAngle);
  }

  value = leftDistance - rightDistance;
  error = value - setPoint;
  // int frontConstant = 40;
  //  double frontProportion = 1;
  // double multiplier = (frontConstant - frontDistance) * (frontDistance < 20) * frontProportion;
  //   if (multiplier == 0) { multiplier = 1; }
  double PIDangle = error * Kp + (error - lastError) * Kd;
  lastError = error;

  if (debugPrint == 1) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(middleSonar.ping_cm());
    display.print(" ");
    display.print(rightSonar.ping_cm());
    display.print(" ");
    display.print(leftSonar.ping_cm());
    display.print(" ");
    display.print(backSonar.ping_cm());
    display.print(" ");
    display.print(analogRead(IRpin));
    display.println();
    // display.setCursor(0, 30);
    display.print("ang = ");
    display.println(int(PIDangle));
    display.print("Kp:");
    display.println(Kp);
    display.print("Kd:");
    display.println(Kd);
    display.print("sped = ");
    display.println(forwardSpeed);
    display.print("turns = ");
    display.println(turnCount);
    display.print("sp = "); 
    display.println(setPoint); 
    display.display();
  }

  if (gameStarted == 1) {
    int steer_angle = midAngle;
    if (PIDangle > 0) {
      steer_angle = midAngle + min(halfAngleRange, PIDangle);
    } else {
      steer_angle = midAngle - min(halfAngleRange, abs(PIDangle));
    }
    steering_servo.write(steer_angle);
  } else {
    checkButton();
  }
}
