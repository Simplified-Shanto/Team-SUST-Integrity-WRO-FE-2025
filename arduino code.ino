#include <Servo.h>

#define servoPin 9

Servo steering_servo;

// Pin definitions
const int pwmPin = 5; // PWM pin for motor speed

const int in1Pin = 4; // Direction control
const int in2Pin = 6; // Direction control

#define leftAngle 20
#define midAngle 70
#define rightAngle 120
#define forward_motor_speed 150
#define forward_travel_delay 200
#define steering_delay    500

void setup()
{
  pinMode(pwmPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);

  steering_servo.attach(9);
  steering_servo.write(midAngle);
  delay(1000);
  steering_servo.write(leftAngle);
  delay(1000);
  steering_servo.write(midAngle); 
  delay(1000); 
  steering_servo.write(rightAngle); 
  delay(1000); 
  steering_servo.write(midAngle); 

}

void loop()
{

  // Forward
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  analogWrite(pwmPin, forward_motor_speed); // Speed: 0 to 255

  delay(forward_travel_delay);

  steering_servo.write(leftAngle);
  delay(steering_delay);
  steering_servo.write(midAngle);
  delay(forward_travel_delay);
  steering_servo.write(leftAngle);
  delay(steering_delay);
  steering_servo.write(midAngle);
  delay(forward_travel_delay);
  steering_servo.write(leftAngle); 
  delay(steering_delay);

}

