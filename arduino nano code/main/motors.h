// Pin definitions
#define pwmPin 5 // PWM pin for motor speed
#define in1Pin 4 // Direction control
#define in2Pin 6 // Direction control

void goForward(int speed)
{
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, speed); 
}