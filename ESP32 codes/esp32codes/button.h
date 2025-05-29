#define buttonPin 13
#define ledPin    15
byte gameStarted = 0;


void checkButton()
{
  if (digitalRead(buttonPin) == LOW) {
    delay(1); //debounce delay
    digitalWrite(ledPin, HIGH);
    if(gameStarted==1)
    {
      goForward(0);
      gameStarted = 0; 
    } else {
      goForward(forwardSpeed); 
      gameStarted = 1;
    }
    
    delay(500);
    digitalWrite(ledPin, LOW);
  }
}
