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

int target_total_lines = 24;
bool stopped = false;  // Add a flag to mark stopped state


void setup() {
  // put your setup code here, to run once:
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
  static String serialBuffer = "";
  static int blueLines = 0;
  static int orangeLines = 0;
  static int totalLines = 0;
  char incoming;

  while (Serial.available()) {  // Read all available chars immediately
    incoming = Serial.read();
    if (incoming != '\n') {
      serialBuffer += incoming;
    } else {
      // Full line received
      Serial.print("Received: ");
      Serial.println(serialBuffer);

      int blueIndex = serialBuffer.indexOf("\"BlueLinesPassed\":");
      int orangeIndex = serialBuffer.indexOf("\"OrangeLinesPassed\":");
      int totalIndex = serialBuffer.indexOf("\"TotalLinesPassed\":");

      if (blueIndex != -1 && orangeIndex != -1 && totalIndex != -1) {
        int blueEnd = serialBuffer.indexOf(",", blueIndex);
        int orangeEnd = serialBuffer.indexOf(",", orangeIndex);
        int totalEnd = serialBuffer.indexOf("}", totalIndex);

        int blueColon = serialBuffer.indexOf(":", blueIndex);
        int orangeColon = serialBuffer.indexOf(":", orangeIndex);
        int totalColon = serialBuffer.indexOf(":", totalIndex);

        String blueStr = serialBuffer.substring(blueColon + 1, blueEnd);
        String orangeStr = serialBuffer.substring(orangeColon + 1, orangeEnd);
        String totalStr = serialBuffer.substring(totalColon + 1, totalEnd);

        blueStr.trim();
        orangeStr.trim();
        totalStr.trim();

        blueLines = blueStr.toInt();
        orangeLines = orangeStr.toInt();
        totalLines = totalStr.toInt();

        if (totalLines >= target_total_lines && !stopped) {
          stopped = true;  // Set stopped flag
          goForward(0);    // Stop the motor immediately
          steering_servo.write(midAngle);
          Serial.println("Stopped immediately due to totalLines reached.");
        }
      }
      serialBuffer = "";
    }
  }

  checkButton();

  // If stopped, skip motor control
  if (stopped) {
    // Just update display if debugPrint enabled
    if (debugPrint == 1) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("Stopped!");
      display.print(" Blue: ");
      display.println(blueLines);
      display.print(" Orange: ");
      display.println(orangeLines);
      display.print(" Total: ");
      display.println(totalLines);
      display.display();
    }
    return;  // skip rest of the loop
  }

  int frontDistance = middleSonar.ping_cm();
  int leftDistance = leftSonar.ping_cm();
  int rightDistance = rightSonar.ping_cm();
  int backDistance = backSonar.ping_cm();

  if (leftDistance == 0) leftDistance = 100;
  if (rightDistance == 0) rightDistance = 100;

  value = leftDistance - rightDistance;
  error = value - setpoint;
  double PIDangle = error * Kp + (error - lastError) * Kd;
  lastError = error;


  if (gameStarted == 1) {
    int steer_angle = midAngle;
    if (PIDangle > 0) {
      steer_angle = midAngle + min(halfAngleRange, PIDangle);
    } else {
      steer_angle = midAngle - min(halfAngleRange, abs(PIDangle));
    }
    steering_servo.write(steer_angle);
  }


    if (debugPrint == 1) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Blue: ");
    display.println(blueLines);
    display.print("Orange: ");
    display.println(orangeLines);
    display.print("Total: ");
    display.println(totalLines);
    display.print("data: ");
    display.println(incoming);

    display.print("ang = ");
    display.println(int(PIDangle));
    display.print("Kp: ");
    display.println(Kp);
    display.print("Kd: ");
    display.println(Kd);
    display.print("speed: ");
    display.println(forwardSpeed);
    display.display();
  }

}
