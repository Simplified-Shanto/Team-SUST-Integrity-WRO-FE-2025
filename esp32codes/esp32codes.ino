#include "motors.h"
#include "button.h"
#include "servo_motor.h"
#include "sonar.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  pinMode(ledPin, OUTPUT); 
  pinMode(buttonPin, INPUT_PULLUP); 

  ESP32PWM::allocateTimer(0);
  steering_servo.setPeriodHertz(50);    // standard 50 hz servo
	steering_servo.attach(steering_servo_pin, 500, 2400); // attaches the servo on pin 18 to the servo object
  steering_servo.write(midAngle); 

  ledcSetup(LEDC_CHANNEL, 1000, 8); // Set LEDC channel, frequency, and resolution
  ledcAttachPin(pwmPin, LEDC_CHANNEL); // Attach the GPIO pin to the LEDC channel
  pinMode(in1Pin,  OUTPUT); 
  pinMode(in2Pin,  OUTPUT);   
}

double error = 0;

int setPoint = 0;  //Setpoint for the difference between the readings of the left and right sonar.

double maxError = 60;  // Considering that the maximum difference
                       //  of reading of the two sensors is 20 cm in the tunnel



void loop() {
  checkButton(); 
  // Serial.print(middleSonar.ping_cm()); 
  // Serial.print(" "); 
  // Serial.print(rightSonar.ping_cm()); 
  // Serial.print(" "); 
  // Serial.print(leftSonar.ping_cm()); 
  // Serial.println(); 

   byte steer_angle = 0;
  if (gameStarted == 1) {
    int frontDistance = middleSonar.ping_cm();
    int leftDistance = leftSonar.ping_cm(); 
    int rightDistance = rightSonar.ping_cm(); 
     Serial.print(" left distance = ");
  Serial.print(leftDistance);
   Serial.print(" right distance = ");
  Serial.print(rightDistance);
  Serial.println(); 
    if (frontDistance != 0 && frontDistance < frontDistanceThreshold) {
      Serial.println("Time to turn!"); 
      steering_servo.write(leftAngle);
      while (middleSonar.ping_cm() != 0) {
        checkButton();
        delay(30); 
        //Serial.println("In the while loop");
      }
      steering_servo.write(midAngle);
    } else if ( leftDistance >= rightDistance) {
      error = leftDistance - rightDistance;
      steer_angle = (error / maxError) * halfAngleRange;
      steer_angle = midAngle + steer_angle;
      Serial.print("steer = ");
      Serial.print(steer_angle);
      Serial.println();   
    } else if (leftDistance  < rightDistance) {
      error = rightDistance - leftDistance ;
      steer_angle = (error / maxError) * halfAngleRange;
      steer_angle = midAngle - steer_angle;
     // Serial.println("Calculating right going angle."); 
    }
    //erial.println(steer_angle); 
    if (steer_angle <= leftAngle and steer_angle >= rightAngle) {
      steering_servo.write(steer_angle);
    }
  }

  

}
