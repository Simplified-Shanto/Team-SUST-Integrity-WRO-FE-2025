//Board: esp32 by Espressif Systems. Version: 1.0.3
//ESP32Servo by Kevin, Harrington, John K. Benett. Version: 3.0.6
//NewPing by Tim Eckel. Version: 1.9.7
//Adafruit GFX Library by Adafruit. Version: 1.12.0
//Adafruit SSD1306 by Adafruit. Version: 2.5.13

#include <Preferences.h>
#include "motors.h"
#include "button.h"
#include "servo_motor.h"
#include "sonar.h"
#include "display.h"

//Tested parameter values1: Speed: 150,160, 170  Kp: 3, Kd: 0.3 , Ki = 0, steerAngle = 40

Preferences preferences;
float Kp = 3;    //4
float Kd = 0.3;  //0.3
float Ki = 0;
int lineInterval = 500;  //time (ms) to wait in the image processing programme between lines counted.
int stopDelay = 1000;    //when 12 lines are counted, the SBC sends the lap complete command after this fixed delay(ms)
float value = 0;         // Stores the difference between the left and right readings.
float error = 0;         // The difference between value and setpoint.
float lastError = 0;
float PIDangle = 0;
float integralError = 0;
int restrictedSteer = 40;  // maximum allowed angle change in left or right direction for steering.
int unrestrictedSteer = 45;
int steerAngle = restrictedSteer;


#define parameterCount 9  // Number of parameters to display, configure or both.

float setPoint = 0;  // The amount of difference in reading of the two ultrasonic sensor we want.
//Above one is the initial setPoint which keeps the vehicle centered in a tunnel
float dynamicSetPoint = 0;  //This setpoint is assigned after determining the run direction

int terminalDistanceThreshold = 80;

void setup() {
  Serial.begin(115200);
  preferences.begin("wrobot", false);
  //preferences.clear(); //Only uncomment it when you have the second round code uploaded currently. Upload this code. Then comment this line again in the further upload of this same code.
  lineInterval = preferences.getInt("lineInterval", lineInterval);
  stopDelay = preferences.getInt("stopDelay", stopDelay);
  Kp = preferences.getFloat("Kp", Kp);
  Ki = preferences.getFloat("Ki", Ki);
  Kd = preferences.getFloat("Kd", Kd);
  forwardSpeed = preferences.getInt("speed", forwardSpeed);
  restrictedSteer = preferences.getInt("rSteer", restrictedSteer);
  unrestrictedSteer = preferences.getInt("urSteer", unrestrictedSteer);


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

  setupDisplay();
}

bool button2Flag = 0;
bool button1Flag = 0;
long long pressTime2 = millis();
long long pressTime1 = millis();
bool editParameter = 0;
short parameterIndex = 1;  //0 = Angle demonstration , 1 = speed
short leftDistance = 0;
short rightDistance = 0;
short frontDistance = 0;
int roundDirection = 0;  // 0 = not detected yet, -1 = cw 1 = ccw

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil(';');
    handleSerialCommand(command);
  }

  rightDistance = rightSonar.ping_cm();
  //delay(sonarReadingDelay);

  leftDistance = leftSonar.ping_cm();
  //delay(sonarReadingDelay);

  frontDistance = frontSonar.ping_cm();
  //delay(sonarReadingDelay);

  // //Mechanism 0:
  // if (rightDistance==0) { rightDistance== terminalDistanceThreshold; }
  // else if(leftDistance==0) { leftDistance = terminalDistanceThreshold; }


  //Mechanism 1:  The following mechanism fails in the case where's there an extension in the center boundry, and the vehicle takes turn when near the outerboarder, the front distance threshold gets true and takes turn in
  // first condition mentioned below.

  //  if (rightDistance == 0 || (rightDistance != 0 && (frontDistance != 0 && frontDistance < frontDistanceThreshold))) {
  //   rightDistance = terminalDistanceThreshold;
  // }
  // else if (leftDistance == 0 || (leftDistance != 0 && (frontDistance != 0 && frontDistance < frontDistanceThreshold))) {
  //   leftDistance = terminalDistanceThreshold;
  // }

  //Mechanism 2:

  if (rightDistance == 0 || leftDistance == 0 || (frontDistance != 0 && frontDistance < frontDistanceThreshold))  //Take a turning action on any of these events.
  {
    steerAngle = unrestrictedSteer;
    if (rightDistance == 0 && leftDistance != 0)  // 1 0
    {
      rightDistance = terminalDistanceThreshold;
      digitalWrite(buzzerPin, LOW);
      //  digitalWrite(ledPin, LOW);
    } else if (leftDistance == 0 && rightDistance != 0)  //0 1
    {
      leftDistance = terminalDistanceThreshold;
      digitalWrite(buzzerPin, LOW);
      //  digitalWrite(ledPin, LOW);
    } else if (leftDistance != 0 && rightDistance != 0)  //1 1
    {
      if (leftDistance > rightDistance) {
        leftDistance = terminalDistanceThreshold;
      } else if (rightDistance > leftDistance) {
        rightDistance = terminalDistanceThreshold;
      }
      //  digitalWrite(ledPin, HIGH);
      digitalWrite(buzzerPin, LOW);
    } else if (leftDistance == 0 && rightDistance == 0) {  //0 0 Both sonar sensor is reading zero, so we are resorting to the round direction for taking turns.
      if (gameStarted == 1) {
        digitalWrite(buzzerPin, HIGH);
      }
      if (roundDirection == -1) {  //cw round
        rightDistance = terminalDistanceThreshold;
      }
    } else if(roundDirection == 1) {  //ccw round
      leftDistance = terminalDistanceThreshold;
    }
  } else {
    steerAngle = restrictedSteer;
  }

  // 0 0 - If anyhow both left and right side senor reads zero, then we'll do nothing, just register such events with a buzzer beep. the vehicle will go straight.
  // Here's a few choice we have at our hand now
  // 0. Increase sonar sensor reading range, to avoid such mishap
  //    Sonar range = 100;   Frequent mishap
  //    Sonar range = 120;   Still both sensor gets zero at the narrow tunnel entry, but they were for very shortperiod
  //                         and unable to distract the vehicle.
  //    Sonar range = 130;   Detected significantly in the ccw turn in the extended tunnel entry point
  //                         Peforms decent in high speed runs with short period of both sensor 0 reading detecting at the entry point of the narrow tunnel.
  //    One reasone for such event might be in the narrow tunnel entry point, there's too much interference of the ultrosinc sounds from the 3 sonar sensors, because
  //    no 0 reading event is occurring in the no - extension tunnel entry points.
  // 1. Introduce delay betwee sensor readings and see if the misreading can be avoided.
  //    delay = 10ms * 3 = 30 ms;   The issue is persistent, pid response is slow.        No hitting.
  //    delay = 15ms * 3 = 45 ms;   The issue is persistent for the ccw entry point only. No hitting.
  //    delay = 20ms * 3 = 60 ms;   Issue is persistent. PID response is extremely slow - Wall hitting for high speed(120).which we can afford for a high speed first round.
  // 2. Detect round order at the beginning via camera, and just take turn in one side - no condition checking for the other side. - safest way to avoid those wall hitting. -
  ///   We must try this method - and after implementing and with this we can also try single sensor pid (and the front sensor reading to detect turns. )
  //    This will be particularly useful in the second round, where we'll consider a left turn/right turn only when encountering an obstacle, and for corners we'll just give
  //    a fixed turn

  // // Mechanism 3: Taking turn in one side only after detecting the round direction
  // if((forwardSpeed/2)%2==0){ //cw round
  //   if(rightDistance==0 || (frontDistance!=0 && frontDistance < frontDistanceThreshold))
  //   {
  //     rightDistance = terminalDistanceThreshold;
  //   }
  // }
  // else { //ccw round
  //   if(leftDistance==0 || (frontDistance!=0 && frontDistance < frontDistanceThreshold))
  //   {
  //     leftDistance = terminalDistanceThreshold;
  //   }

  // }

  handleButtonPress();

  value = leftDistance - rightDistance;
  error = value - setPoint;
  // accumulate integral error
  integralError += error;

  // optional: limit integral to prevent windup
  if (integralError > 1000) integralError = 1000;
  if (integralError < -1000) integralError = -1000;

  // int frontConstant = 40;
  // double multiplier = (frontConstant - frontDistance) * (frontDistance < 20) * frontProportion;
  //   if (multiplier == 0) { multiplier = 1; }
  PIDangle = error * Kp
             + (error - lastError) * Kd
             + (integralError * Ki);
  lastError = error;


  if (editParameter == 1) { configureParameters(); }
  if (gameStarted == 1) {
    writeAngleToServo();
  }
}



void setupDisplay() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextColor(1);
  display.setTextSize(1);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Kp:");
  display.println(Kp);
  display.print("Ki:");
  display.println(Ki);
  display.print("Kd:");
  display.println(Kd);
  display.print("Speed: ");
  display.println(forwardSpeed);
  display.print("Line Intv: ");
  display.println(lineInterval);
  display.print("stopDel: ");
  display.println(stopDelay);
  display.print("rSteer: ");
  display.println(restrictedSteer);
  display.print("urSteer: ");
  display.println(unrestrictedSteer);
  display.display();
}

void handleSerialCommand(String command) {
  char constant_name = command[0];  //The type of action the remote wants us to take.
  String constant_value_string = command.substring(2, command.length());
  float constant_value = constant_value_string.toFloat();

  if (command == "x") {  // Commands the car to stop
    goForward(0);        //Stops the car, but still steers to handle the velocity resulted from momentum.
  } else if (command == "y") {
    startGame();
  } else if (command == "z") {
    stopGame();      //Stops the game, including steering. 
  } else {
    switch (constant_name) {
      case 'p':  //Proportional of PID
        Kp = constant_value;
        preferences.putFloat("Kp", Kp);
        break;
      case 'd':  //Derivative of PID
        Kd = constant_value;
        preferences.putFloat("Kd", Kd);
        break;
      case 's':  //Speed of vehicle
        forwardSpeed = int(constant_value);
        preferences.putInt("speed", forwardSpeed);
        break;

      case 'r':  // Raspberry pie is ready for
        Serial.print("a:");
        Serial.println(lineInterval);
        Serial.print("b:");
        Serial.println(stopDelay);
        delay(500);
        digitalWrite(ledPin, HIGH);
        break;

      case 'b':  // Blue line is encountered before the orange line in the runtime of the python script -> round is counter-clockwise
        roundDirection = 1;
        break;

      case 'o':  // Orange line is encountered before the blue line in the runtie of the python script -> round is clockwise
        roundDirection = -1;
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
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("L:");
    display.print(leftDistance);
    display.print(" F:");
    display.print(frontDistance);
    display.print(" R:");
    display.println(rightDistance);

    display.print("Round dir: "); // RO = round order
    if(roundDirection==0) { display.println(" ---- "); //Not detected yet. 
    }
    else if (roundDirection==1) { display.println("ccw"); }
    else if(roundDirection==-1) { display.println("cw");  }

    if (editParameter == 1 && parameterIndex == 0) {
      writeAngleToServo();
      display.print("> ");
    }
    display.print("A: ");
    display.print(int(PIDangle));
    display.println(" deg");
    
    display.println(); 

  }


  if (parameterIndex <= 3) {
    if (editParameter == 1 && parameterIndex == 1) { display.print("> "); }
    display.print("Speed: ");
    display.println(forwardSpeed);
    display.println();
  }

  if (parameterIndex <= 4) {
    if (editParameter == 1 && parameterIndex == 2) { display.print("> "); }
    display.print("Kp:");
    display.println(Kp);
    display.println();
  }

  if (parameterIndex <= 5) {
    if (editParameter == 1 && parameterIndex == 3) { display.print("> "); }
    display.print("Ki:");
    display.println(Ki);
    display.println();
  }

  if (parameterIndex <= 6) {
    if (editParameter == 1 && parameterIndex == 4) { display.print("> "); }
    display.print("Kd:");
    display.println(Kd);
    display.println();
  }

  if (parameterIndex <= 7) {
    if (editParameter == 1 && parameterIndex == 5) { display.print("> "); }
    display.print("LineIntv: ");
    display.println(lineInterval);
    display.println();
  }


  if (parameterIndex <= 8) {
    if (editParameter == 1 && parameterIndex == 6) { display.print("> "); }
    display.print("StopDel: ");
    display.println(stopDelay);
    display.println();
  }

  if (parameterIndex <= 9) {
    if (editParameter == 1 && parameterIndex == 7) { display.print("> "); }
    display.print("rSteer:");
    display.println(restrictedSteer);
    display.println();
  }

  if (parameterIndex <= 10) {
    if (editParameter == 1 && parameterIndex == 8) { display.print("> "); }
    display.print("urSteer:");
    display.println(unrestrictedSteer);
    display.println();
  }



  display.println();
  display.display();
}

void handleButtonPress() {
  if (button2Flag == 0 && digitalRead(button2Pin) == LOW) {  //For the time being, we'll stop the second button to stop the car and first button to start the car.
    button2Flag = 1;
    pressTime2 = millis();
    delay(300);
  }

  if (button2Flag == 1 && digitalRead(button2Pin) == HIGH) {
    long gap = millis() - pressTime2;
    if (gap < 500) {
      if (editParameter == 0) {
        stopGame();
        digitalWrite(buzzerPin, LOW);
      } else if (editParameter == 1) {
        switch (parameterIndex) {
          case 1:
            forwardSpeed -= (forwardSpeed >= 2) ? 2 : 0;
            break;
          case 2:
            Kp -= 0.5;
            break;
          case 3:
            Ki -= 0.05;
            break;
          case 4:
            Kd -= 0.05;
            break;
          case 5:
            lineInterval -= 50;
            break;
          case 6:
            stopDelay -= 50;
            break;
          case 7:
            restrictedSteer -= (restrictedSteer >= 1) ? 1 : 0;
            break;
          case 8:
            unrestrictedSteer -= (unrestrictedSteer >= 1) ? 1 : 0;
            break;

          default:
            break;
        }
      }

    } else if (gap <= 1500)  //Going one step up in the edit parameter menu
    {
      parameterIndex--;
      if (parameterIndex < 0) {
        parameterIndex = parameterCount - 1;
      }
    } else if (gap > 2500)  // If presstime exceeds 3 second, we'll shutdown the raspi, won't wait for the release
    {
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
          case 1:
            forwardSpeed += (forwardSpeed >= 2) ? 2 : 0;
            break;
          case 2:
            Kp += 0.5;
            break;
          case 3:
            Ki += 0.05;
            break;
          case 4:
            Kd += 0.05;
            break;
          case 5:
            lineInterval += 50;
            break;
          case 6:
            stopDelay += 50;
            break;
          case 7:
            restrictedSteer += (restrictedSteer < unrestrictedSteer) ? 1 : 0;
            break;
          case 8:
            unrestrictedSteer += (unrestrictedSteer < maxSteer) ? 1 : 0;

          default:
            break;
        }
      }
    } else if (gap < 1500) {
      parameterIndex = (parameterIndex + 1) % parameterCount;
      if (parameterIndex == 1 || parameterIndex == parameterCount - 1)  //Resetting the servo position when the Angle menu is not in focus.
      {
        steering_servo.write(midAngle);
      }
    } else if (gap < 3000) {
      if (editParameter == 1) {
        steering_servo.write(midAngle);  //Resetting the servo position
        roundDirection = 0; // We determined the round direction inside this parameter configuration session as a test, after the configurations are done, we're resetting it again for a fresh lap start. 
        preferences.putFloat("Kp", Kp);
        preferences.putFloat("Ki", Ki);
        preferences.putFloat("Kd", Kd);
        preferences.putInt("speed", forwardSpeed);
        preferences.putInt("lineInterval", lineInterval);
        preferences.putInt("stopDelay", stopDelay);
        preferences.putInt("rSteer", restrictedSteer);
        preferences.putInt("urSteer", unrestrictedSteer);
        editParameter = 0;
        // Updates the associated variables in the SBC
        Serial.print("a:");
        Serial.println(lineInterval);
        Serial.print("b:");
        Serial.println(stopDelay);

        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("Speed: ");
        display.println(forwardSpeed);
        display.print("Kp:");
        display.println(Kp);
        display.print("Ki:");
        display.println(Ki);
        display.print("Kd:");
        display.println(Kd);
        display.print("Line Intv: ");
        display.println(lineInterval);
        display.print("StopDel: ");
        display.println(stopDelay);
        display.print("rSteer: ");
        display.println(restrictedSteer);
        display.print("urSteer: ");
        display.println(unrestrictedSteer);

        display.display();
      } else if (editParameter == 0) {
        editParameter = 1;
        stopGame();
        Serial.print("r"); // We need to determine the round order 
      }
    }
    button1Flag = 0;
  }
}


void writeAngleToServo() {
  int steer_angle = midAngle;
  if (PIDangle > 0) {
    steer_angle = midAngle - min(steerAngle, int(PIDangle));
  } else {
    steer_angle = midAngle + min(steerAngle, int(abs(PIDangle)));
  }
  steering_servo.write(steer_angle);
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
