#include <ESP32Servo.h>

Servo steering_servo;
#define steering_servo_pin 17
// #define rightAngle 25
// #define midAngle 70
// #define leftAngle 115

#define maxSteer 65  //Maximum possible steering  - limited by the steering mechanism and the hardware setup. 

#define midAngle 92
#define leftAngle midAngle - halfAngleRange
#define rightAngle midAngle + halfAngleRange

