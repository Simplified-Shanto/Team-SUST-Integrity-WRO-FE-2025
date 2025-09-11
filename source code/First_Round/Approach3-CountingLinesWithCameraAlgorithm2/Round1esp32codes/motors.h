// Pin definitions
#define pwmPin 2 // PWM pin for motor speed
#define in1Pin 4 // Direction control
#define in2Pin 16 // Direction control
#define LEDC_CHANNEL 6 // LEDC channel to use

int forwardSpeed = 150; // Out of 255


void goForward(int speed)
{
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    ledcWrite(LEDC_CHANNEL, speed);
}