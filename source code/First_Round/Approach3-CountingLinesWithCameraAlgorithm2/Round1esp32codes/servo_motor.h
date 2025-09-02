#include <ESP32Servo.h>

Servo steering_servo;

#define steering_servo_pin 17
// #define rightAngle 25
// #define midAngle 70
// #define leftAngle 115
double halfAngleRange = 45;


#define midAngle 83
#define leftAngle midAngle - halfAngleRange
#define rightAngle midAngle + halfAngleRange
