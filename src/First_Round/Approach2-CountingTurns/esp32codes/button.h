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
    digitalWrite(ledPin, HIGH);
    delay(300); //debounce delay
    if(gameStarted==1)
    {
      goForward(0);
      gameStarted = 0; 
    } else {
      goForward(forwardSpeed); 
      gameStarted = 1;
    }
    digitalWrite(ledPin, LOW);
  }
}
