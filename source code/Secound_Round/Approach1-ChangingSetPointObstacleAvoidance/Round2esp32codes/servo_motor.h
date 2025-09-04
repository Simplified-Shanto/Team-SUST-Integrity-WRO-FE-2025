#include <ESP32Servo.h>

Servo steering_servo;
#define steering_servo_pin 17
// #define rightAngle 25
// #define midAngle 70
// #define leftAngle 115
double halfAngleRange = 45; //Maximum angle it can go from the centre. 
double steeringFreedom = 65; 

#define midAngle 92
#define leftAngle midAngle - halfAngleRange
#define rightAngle midAngle + halfAngleRange

