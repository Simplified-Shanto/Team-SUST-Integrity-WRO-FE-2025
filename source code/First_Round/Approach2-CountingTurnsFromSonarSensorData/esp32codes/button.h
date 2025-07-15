#define buttonPin 13
#define button2Pin 19  //Not used yet
#define ledPin    15
#define buzzerPin 26
byte gameStarted = 0;


void buzz(int duration)
{
  digitalWrite(buzzerPin, HIGH);
  delay(duration); 
  digitalWrite(buzzerPin, LOW); 
}

void checkButton()
{ 
   if (digitalRead(buttonPin) == LOW) {
   gameStarted = 1; 
  delay(300); 
  goForward(forwardSpeed);
  }
}
