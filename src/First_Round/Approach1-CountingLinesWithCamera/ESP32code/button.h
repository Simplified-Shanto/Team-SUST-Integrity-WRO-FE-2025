#define buttonPin 13
#define button2Pin 19  //Not used yet
#define ledPin    15
#define buzzerPin 26
byte gameStarted = 0;

void checkButton()
{
  if (digitalRead(buttonPin) == LOW) {
    digitalWrite(ledPin, HIGH);
    delay(10); //debounce delay
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
