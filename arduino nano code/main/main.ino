#include "sonar_sensor.h"
#include "motors.h"
#include "servo_motor.h"
#include "button.h"


void setup() {
  Serial.begin(9600);  // Open serial monitor at 115200 baud to see ping results.
  steering_servo.attach(steering_servo_pin);

  pinMode(pwmPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  steering_servo.write(midAngle);
}


double error = 0;

int setPoint = 0;  //Setpoint for the difference between the readings of the left and right sonar.

double maxError = 60;  // Considering that the maximum difference
                       //  of reading of the two sensors is 20 cm in the tunnel



void loop() {
  checkButton();
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