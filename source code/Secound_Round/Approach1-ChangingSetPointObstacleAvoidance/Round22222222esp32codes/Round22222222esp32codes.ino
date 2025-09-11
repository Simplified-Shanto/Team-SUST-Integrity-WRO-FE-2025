#include <Preferences.h>
#include "motors.h"
#include "button.h"
#include "servo_motor.h"
#include "sonar.h"
#include "display.h"

Preferences preferences;
double Kp = 3;  //5
double Kd = 0.3;  //1
double Ki = 0;
int lineInterval = 1000;  //time (ms) to wait in the image processing programme between lines counted.
int stopDelay = 1000;     //when 12 lines are counted, the SBC sends the lap complete command after this fixed delay(ms)
double value = 0;      // Stores the difference between the left and right readings.
double error = 0;      // The difference between value and setpoint.
double lastError = 0;
double PIDangle = 0;
double integralError = 0;

#define parameterCount 8  // Number of configurable parameters

unsigned int redObstacleDistance = 0;
unsigned int greenObstacleDistance = 0;


double setPoint = 0;  // The amount of difference in reading of the two ultrasonic sensor we want.
//Above one is the initial setPoint which keeps the vehicle centered in a tunnel
double dynamicSetPoint = 0;  //This setpoint is assigned after determining the run direction
int setPointMultiplier = 1;  // -1 = round is clockwise 1 = round is anticlockwise
int terminalDistanceThreshold = 200;
short restrictedSteer = 35;
short unrestrictedSteer = 45;


void setup() {
  Serial.begin(115200);
  preferences.begin("wrobot", false);
  preferences.clear();   //Only uncomment it when you have the first round code uploaded currently. Upload this code. Then comment this line again. 
  lineInterval = preferences.getInt("lineInterval", lineInterval);
  stopDelay = preferences.getInt("stopDelay", stopDelay);
  Kp = preferences.getDouble("Kp", Kp);
  Ki = preferences.getDouble("Ki", Ki);
  Kd = preferences.getDouble("Kd", Kd);
  forwardSpeed = preferences.getInt("speed", forwardSpeed);
  restrictedSteer = preferences.getShort("restrictedSteer", restrictedSteer);
  unrestrictedSteer = preferences.getShort("urSteer", unrestrictedSteer);

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
  showParameters();
}

bool button2Flag = 0;
bool button1Flag = 0;
long long pressTime2 = millis();
long long pressTime1 = millis();
bool editParameter = 0;
short parameterIndex = 0;  // 0  = Speed, 1 = Kp, 2 = Kd
short leftDistance = 0;
short rightDistance = 0;
short frontDistance = 0; 


void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil(';');
    handleSerialCommand(command);
  }


  rightDistance = rightSonar.ping_cm();

  leftDistance = leftSonar.ping_cm();

  frontDistance = frontSonar.ping_cm();


  // if(setPoint==0) //There's no obstacle in the vision range. (but we should only check it, while considering the frontDistance, and not in other case. )
  // {

  // if (leftDistance == 0 || (leftDistance!=0 && (frontDistance > 0 && frontDistance < 60 ))) {  //Either any sensor feels a long gap, or the vehicle is very close to the wall, take turning action
  //                                                                        //Make steering freedom proportional to the frontdistance.
  //   if (frontDistance > 0 && frontDistance < 60) {   //Avoiding taking extra turn when the vehicle has already turned, and is now parallel to the wall. 
  //     leftDistance = terminalDistanceThreshold;
  //   } else {  // The vehicle has already turned enough, and is parallel to the wall, which we understand by the front sensor's zero reading, so assume there's a wall on the right side. 
  //     leftDistance = 100 - rightDistance;
  //   }

  // } else if (rightDistance == 0 || (rightDistance!=0 && (frontDistance > 0 && frontDistance < 60))) {

  //  if (frontDistance > 0 && frontDistance < 60) {
  //     rightDistance = terminalDistanceThreshold;
  //   } else {
  //     rightDistance = 100 - leftDistance; // If the vehicle is trying to turn from one extreme side of the tunnel, then we'll be turning only when there's a wall ahead, if there's free space, then we'll stop the turning attempt. 
  //   }

  // }
  // }
  // else //There's some obstacle in the vision range. 
  // {
  //     if (leftDistance == 0) {
  //   leftDistance = terminalDistanceThreshold;
  // } else if (rightDistance == 0) {
  //   rightDistance = terminalDistanceThreshold;
  // }
  // }

/* There's a problem with the following conditioning, if anyhow side sonar fails to be zero when its time to turn, the vehicle will always take left turn. 
  
  If we know the round direction, we can disable taking turn in left or right side at corners, reducing the error possibility. 
  
  Can we anyhow use the gyro sensor to determine the round direction very reliably?  The car goes parallely for the time being at the starting. 

  Can we anyhow avoid obstacle from the right side, or change setPoint properly without knowing the round direction ? Using any opencv mechanism. 

  For now, since we've the camera facing downward, and detecting lines, we can solfe the counter-clockwise turning problem. 

  The counter-clockwise turning problem is arising from the fact that, the frontDistance condition checking is being true before the side sonar being zero condition is hitting the ground. 
  We can also try to solve the problem first by reducing the front distance threshold. So we'll be primarily relying on the side sonars for turning and then in any case if it fails, 
  then the front distance checking will start the turning attempt. With this combine the one side turning only mechanism at corners, and things should work properly. 
  
  */

  

  if(setPoint==0) //There's no obstacle in the vision range. (but we should only check it, while considering the frontDistance, and not in other case. )
  {
  if (leftDistance == 0 || (leftDistance!=0 && (frontDistance > 0 && frontDistance < frontDistanceThreshold ))) {  //Either any sensor feels a long gap, or the vehicle is very close to the wall, take turning action
                                                                         //Make steering freedom proportional to the frontdistance.
    // if (frontDistance > 0 && frontDistance < 60) {   //Avoiding taking extra turn when the vehicle has already turned, and is now parallel to the wall. 
    //   leftDistance = terminalDistanceThreshold;
    // } else {  // The vehicle has already turned enough, and is parallel to the wall, which we understand by the front sensor's zero reading, so assume there's a wall on the right side. 
    //   leftDistance = 100 - rightDistance;
    // }
    leftDistance = terminalDistanceThreshold; 

  } else 

//Either right sensor has detected no-wall (so reading zero )   or     (It has failed to detect that no wall/empty space and not returning zero      and     the front sensor has detected the wall hitting very soon.   
  if (    rightDistance == 0                                    ||    (rightDistance!=0                                                              &&     (frontDistance > 0 && frontDistance < frontDistanceThreshold))) {

  //  if (frontDistance > 0 && frontDistance < 60) {
  //     rightDistance = terminalDistanceThreshold;
  //   } else if(frontDistance==0) {
  //     rightDistance = 100 - leftDistance; // If the vehicle is trying to turn from one extreme side of the tunnel, then we'll be turning only when there's a wall ahead, if there's free space, then we'll stop the turning attempt. 
  //   }

      rightDistance = terminalDistanceThreshold;

  }
  }
  else //There's some obstacle in the vision range. 
  {
      if (leftDistance == 0) {
    leftDistance = terminalDistanceThreshold;
  } else if (rightDistance == 0) {
    rightDistance = terminalDistanceThreshold;
  }
  }


  handleButtonPress();

  integralError += error;
  // optional: limit integral to prevent windup
  if (integralError > 1000) integralError = 1000;
  if (integralError < -1000) integralError = -1000;
  //   67         84,             7   -> When the vehicle follows the right wall
  //  -67         7 ,            84   -> When the vehicle follows the left wall
  value = leftDistance - rightDistance;
  error = value - setPoint;

  PIDangle = error * Kp
             + (error - lastError) * Kd
             + (integralError * Ki);
  lastError = error;


  if (editParameter == 1) { configureParameters(); }

  if (gameStarted == 1) {
    int steer_angle = midAngle;
    if (PIDangle > 0) {
      steer_angle = midAngle - min(halfAngleRange, PIDangle);
    } else {
      steer_angle = midAngle + min(halfAngleRange, abs(PIDangle));
    }
    steering_servo.write(steer_angle);
  }
}

////////////////Obstacle Handling//////////////////////////////////////
void changeSetPoint() {
  if (redObstacleDistance == 0 && greenObstacleDistance == 0) {
    setPoint = 0;
    halfAngleRange = restrictedSteer;
    digitalWrite(ledPin, LOW);
    digitalWrite(buzzerPin, LOW);
    
  } else if (redObstacleDistance > greenObstacleDistance) {
    setPoint = 67 * setPointMultiplier;  //Green obstacle is near the vehicle, so it will try to follow the left wall
    halfAngleRange = unrestrictedSteer;  // Increasing steering freedom
    digitalWrite(ledPin, HIGH);          // Indicator for detecting red obstacle.
    digitalWrite(buzzerPin, LOW);
  } else if (redObstacleDistance < greenObstacleDistance) {
    setPoint = -67 * setPointMultiplier;
    halfAngleRange = unrestrictedSteer;  //Increasing steering freedom
    digitalWrite(buzzerPin, HIGH);
    digitalWrite(ledPin, LOW);
  }
}

void showParameters() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("P:");
  display.print(Kp);
  display.print(" I:");
  display.print(Ki);
  display.print(" D:");
  display.println(Kd);

  display.print("Speed: ");
  display.println(forwardSpeed);
  display.print("rSteer: ");
  display.println(restrictedSteer);
  display.print("urSteer: ");
  display.println(unrestrictedSteer);
  display.print("Line Intv: ");
  display.println(lineInterval);
  display.print("stopDel: ");
  display.println(stopDelay);


  display.display();
}

void handleSerialCommand(String command) {

  char constant_name = command[0];  //The type of action the remote wants us to take.
  String constant_value_string = command.substring(2, command.length());
  float constant_value = constant_value_string.toFloat();

  if (command == "x") {  // Commands the car to stop
    stopGame();
  } else if (command == "y") {
    startGame();
  } else {
    switch (constant_name) {
      case 'r':  // Raspberry pie is ready for
        Serial.print("a:");
        Serial.println(lineInterval);
        Serial.print("b:");
        Serial.println(stopDelay);
        delay(500);
        digitalWrite(ledPin, HIGH);
        break;
      case 'b':  // Blue line is encountered before the orange line in the runtime of the python script -> round is anti-clockwise
        setPointMultiplier = 1;
        changeSetPoint();  //Fix the sign of the setpoint
        break;
      case 'o':  // Orange line is encountered before the blue line in the runtie of the python script -> round is clockwise
        setPointMultiplier = -1;
        changeSetPoint();  //Fix the sign of the setpoint
        break;
      case 'R':                                     //Red obstacle's distance
        redObstacleDistance = int(constant_value);  // obstacle distance will be 0 when it is beyond the vision range of the vehicle
        changeSetPoint();
        break;
      case 'G':                                       //Green obstacle's distance
        greenObstacleDistance = int(constant_value);  // obstacle distance will be 0 when it is beyond the vision range of the vehicle
        changeSetPoint();
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

      case 'm':  //MidAngle setup of the steering servo
        Serial.println(int(constant_value));
        steering_servo.write(int(constant_value));
        break;

      default:
        break;
    }
  }
}

void configureParameters() {
  display.clearDisplay();
  display.setCursor(0, 0);

  if (parameterIndex <= 2) {
    display.setCursor(0, 0);
    display.print("L:");
    display.print(leftDistance);
    display.print(" F:");
    display.print(frontDistance); 
    display.print(" R:");
    display.println(rightDistance);

    display.print("A:");
    display.print(int(PIDangle));
    display.print("* SP:");
    display.println(setPoint);
  }
  if (parameterIndex <= 3) {
    if (editParameter == 1 && parameterIndex == 0) { display.print("> "); }
    display.print("Speed: ");
    display.println(forwardSpeed);
    display.println();
  }

  if (parameterIndex <= 4) {
    if (editParameter == 1 && parameterIndex == 1) { display.print("> "); }
    display.print("Kp:");
    display.println(Kp);
    display.println();
  }

  if (parameterIndex <= 5) {
    if (editParameter == 1 && parameterIndex == 2) { display.print("> "); }
    display.print("Ki:");
    display.println(Ki);
    display.println();
  }

  if (parameterIndex <= 6) {
    if (editParameter == 1 && parameterIndex == 3) { display.print("> "); }
    display.print("Kd:");
    display.println(Kd);
    display.println();
  }

  if (parameterIndex <= 7) {
    if (editParameter == 1 && parameterIndex == 4) { display.print("> "); }
    display.print("LineIntv: ");
    display.println(lineInterval);
    display.println();
  }

  if (parameterIndex <= 8) {
    if (editParameter == 1 && parameterIndex == 5) { display.print("> "); }
    display.print("StopDel: ");
    display.println(stopDelay);
    display.println();
  }

  if (parameterIndex <= 9) {
    if (editParameter == 1 && parameterIndex == 6) { display.print("> "); }
    display.print("rSteer: ");
    display.println(restrictedSteer);
    display.println();
  }

  if (parameterIndex <= 10) {
    if (editParameter == 1 && parameterIndex == 7) { display.print("> "); }
    display.print("urSteer: ");
    display.println(unrestrictedSteer);
    display.println();
  }

  display.println();
  display.display();
}


void handleButtonPress() {
  if (button2Flag == 0 && digitalRead(button2Pin) == LOW) {  //Detected a press for button2
    button2Flag = 1;
    pressTime2 = millis();
    delay(300);
  }

  if (button2Flag == 1 && digitalRead(button2Pin) == HIGH) {  //Detected the release for button2
    long gap = millis() - pressTime2;
    if (gap < 500) {
      if (editParameter == 0) {
        stopGame();
      } else if (editParameter == 1) {
        switch (parameterIndex) {
          case 0:
            forwardSpeed -= (forwardSpeed >= 2) ? 2 : 0;
            break;
          case 1:
            Kp -= 0.5;
            break;
          case 2:
            Ki -= 0.05;
            break;
          case 3:
            Kd -= 0.05;
            break;
          case 4:
            lineInterval -= 50;
            break;
          case 5:
            stopDelay -= (stopDelay >= 50) ? 50 : 0;
            break;
          case 6:
            restrictedSteer -= (restrictedSteer > 0) ? 1 : 0;
            break;
          case 7:
            unrestrictedSteer -= (unrestrictedSteer > 0) ? 1 : 0;
            break;
          default:
            break;
        }
      }

    } else if (gap <= 1500)  // Going one step up in the edit parameter menu
    {
      parameterIndex--;
      if (parameterIndex < 0) { parameterIndex = parameterCount - 1; }
    }

    else if (gap > 2500) {
      Serial.write('d');  //Commands the SBC to shutdown
    }
    button2Flag = 0;  //Resetting the flag to detect next presses.
  }

  if (button1Flag == 0 && digitalRead(buttonPin) == LOW)  // We've detected a press in button1
  {
    button1Flag = 1;
    pressTime1 = millis();
    delay(300);
  }

  if (button1Flag == 1 && digitalRead(buttonPin) == HIGH)  // We've detected a release, and will take action according to the pressTime.
  {
    long gap = millis() - pressTime1;
    if (gap < 500) {
      if (editParameter == 0) {
        if (gameStarted == 0) {
          startGame();
        }
      } else if (editParameter == 1) {
        switch (parameterIndex) {
          case 0:
            forwardSpeed += 2;
            break;
          case 1:
            Kp +=0.5;
            break;
          case 2:
            Ki += 0.05;
            break;
          case 3:
            Kd += 0.05;
            break;
          case 4:
            lineInterval += 50;
            break;
          case 5:
            stopDelay += 50;
            break;
          case 6:
            restrictedSteer += (restrictedSteer < maxSteer) ? 1 : 0;
            break;
          case 7:
            unrestrictedSteer += (unrestrictedSteer < maxSteer) ? 1 : 0;
            break;

          default:
            break;
        }
      }
    } else if (gap < 1500) {
      parameterIndex = (parameterIndex + 1) % parameterCount;
    } else if (gap < 3000) {
      if (editParameter == 1) {
        saveParameters();
        editParameter = 0;
        // Updates the associated variables in the SBC
        Serial.print("a:");
        Serial.println(lineInterval);
        Serial.print("b:");
        Serial.println(stopDelay);
        showParameters();

      } else if (editParameter == 0) {
        editParameter = 1;
      }
    }
    button1Flag = 0;
  }
}


void saveParameters() {
  preferences.putDouble("Kp", Kp);
  preferences.putDouble("Ki", Ki);
  preferences.putDouble("Kd", Kd);
  preferences.putInt("speed", forwardSpeed);
  preferences.putInt("lineInterval", lineInterval);
  preferences.putInt("stopDelay", stopDelay);
  preferences.putShort("urSteer", unrestrictedSteer);
  preferences.putShort("restrictedSteer", restrictedSteer);
}

void startGame() {
  gameStarted = 1;
  Serial.print("r");  //Commands the raspberry pie to restart the line order detection process
  delay(500);         // Waiting for the raspberry
  goForward(forwardSpeed);
  digitalWrite(ledPin, LOW);  // Turning Off the LED to use it as the high setpoint monitor.
}

void stopGame() {
  gameStarted = 0;
  goForward(0);  //Stops the car.
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  steering_servo.write(midAngle);
}
