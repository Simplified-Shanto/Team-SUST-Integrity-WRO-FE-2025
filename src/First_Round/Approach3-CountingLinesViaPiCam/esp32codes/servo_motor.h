#include <ESP32Servo.h>

Servo steering_servo;

#define steering_servo_pin 17
// #define rightAngle 25
// #define midAngle 70
// #define leftAngle 115

#define rightAngle 20
#define midAngle 90
#define leftAngle 160
double halfAngleRange = 70;