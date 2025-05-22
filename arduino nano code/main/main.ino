#include "sonar_sensor.h"
#include "motors.h"
#include "servo_motor.h"




void setup() {
  Serial.begin(9600); // Open serial monitor at 115200 baud to see ping results.
  steering_servo.attach(steering_servo_pin);
  pinMode(pwmPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  steering_servo.write(midAngle);
  goForward(150); 
}

double error = 0;

double maxError = 60; // Considering that the maximum difference
                    //  of reading of the two sensors is 20 cm in the tunnel
void loop() {
  if(leftSonar.ping_cm() >= rightSonar.ping_cm())
  {
   error = leftSonar.ping_cm() - rightSonar.ping_cm(); 

  }
  else
  {
    error = rightSonar.ping_cm() - leftSonar.ping_cm(); 
  }
  byte steer_angle = (error / maxError) * halfAngleRange;



  // Proportional response to the error. 
  
  if(leftSonar.ping_cm() >= rightSonar.ping_cm() and maxError > error)
  {
    steering_servo.write(midAngle + steer_angle);
    
  }
  else if(leftSonar.ping_cm() < rightSonar.ping_cm() and maxError > error)
  {
     
    steering_servo.write(midAngle - steer_angle); 
  }

  Serial.print("Left = ");
  Serial.print(leftSonar.ping_cm());
  Serial.print(" Right = ");
  Serial.print(rightSonar.ping_cm()); 
  Serial.print(" Error = ");
  Serial.print(error);
  Serial.print(" Steer = ");
  Serial.print(steer_angle); 
  Serial.println(); 
}