#include <Wire.h> 
#include <MPU6050.h> 

#include <Preferences.h>
#include "motors.h"
#include "button.h"
#include "servo_motor.h"
#include "sonar.h"
#include "display.h"

MPU6050 mpu;               // Create an MPU6050 object

float gyroZ;               // Variable to store Z-axis angular velocity
float angleZ = 0;          // Accumulated Z-axis angular displacement (degrees)

unsigned long lastTime = 0; // Store time of last measurement

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
//Above one is the initial setPoint which keeps the vehicle centered in a tunnel
double dynamicSetPoint = 0; //This setpoint is assigned after determining the run direction 

String piStatus = "Not ready"; // Whether raspberry pie is ready for image processing

int terminalDistanceThreshold = 80; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  preferences.begin("wrobot", false);
  Kp = preferences.getDouble("Kp", 0);
  Kd = preferences.getDouble("Kd", 0);
  debugPrint = preferences.getInt("dP", 0);  //dP = debugPrint
  forwardSpeed = preferences.getInt("speed", 0);
  dynamicSetPoint = preferences.getDouble("dSetPoint",0); 

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

  mpu.initialize();              // Initialize MPU6050 sensor
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);                   // Halt if sensor not found
  }

  Serial.println("MPU6050 initialized successfully.");
  lastTime = millis();           // Initialize time reference
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
        case 'o': //There's orange line before blue , round is clockwise
          setPoint = dynamicSetPoint; 
          break; 
        case 'b': //There's blue line before orange , round is anticlockwise
          setPoint = (-1)*dynamicSetPoint; 
          break; 
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
          break; 

        case 'S': //dynamic Setpoint setup
          dynamicSetPoint = double(constant_value); 
          terminalDistanceThreshold = dynamicSetPoint; 
          preferences.putDouble("dSetPoint", dynamicSetPoint);
          preferences.end();  // Saves variables to EEPROM
          break; 

        case 'r': // Raspberry pie is ready for 
          piStatus = "ready"; 

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
    leftDistance = terminalDistanceThreshold;
  } else if (rightDistance == 0) {
    rightDistance = terminalDistanceThreshold;
  }

  if (digitalRead(button2Pin)==LOW) {  //For the time being, we'll stop the second button to stop the car and first button to start the car. 
    gameStarted = 0;
    goForward(0); //Stops the car. 
    digitalWrite(in1Pin, LOW); 
    digitalWrite(in2Pin, LOW); 
    steering_servo.write(midAngle);
    delay(300); //Debounce delay
  }

  value = leftDistance - rightDistance;
  error = value - setPoint;
  // int frontConstant = 40;
  // double multiplier = (frontConstant - frontDistance) * (frontDistance < 20) * frontProportion;
  //   if (multiplier == 0) { multiplier = 1; }
  double PIDangle = error * Kp + (error - lastError) * Kd;
  lastError = error;

   // Get the current time
  unsigned long currentTime = millis();                // Current time in ms
  float dt = (currentTime - lastTime) / 1000.0;        // Convert to seconds
  lastTime = currentTime;                              // Update lastTime for next loop

  // Read gyroscope raw data (in degrees/sec)
  gyroZ = mpu.getRotationZ() / 131.0;  // Convert raw to deg/sec (131 LSB/(°/s) for ±250°/s range)

  // Integrate angular velocity to find angular displacement
  angleZ += gyroZ * dt;  // Δθ = ω × Δt

  // Output results
  // Serial.print("Gyro Z (deg/s): ");
  // Serial.print(gyroZ);
  // Serial.print(" | Total Angle Z (°): ");
  // Serial.println(angleZ);

 // delay(10); // Small delay (~100Hz sampling rate)

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
    display.print("ANGZ = ");
    display.println(angleZ);
    display.print("tsd = "); 
    display.println(terminalDistanceThreshold); 
    display.print("PiStatus = ");  // Whether raspi is ready for image processing. 
    display.println(piStatus); 
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



void checkButton()
{ 
  if (digitalRead(buttonPin) == LOW) {
  gameStarted = 1; 
  Serial.print("r"); //Commands the raspberry pie to restart the line order detection process
  setPoint = 0; //Trying to be in the middle when we don't know the game direction
  delay(1000); // Waiting for the raspberry
  goForward(forwardSpeed);
  }
}

