#include <NewPing.h>

#define MAX_DISTANCE 100  // Maximum distance (in cm) to ping

// Define sonar sensors: trigger pin, echo pin, max distance
NewPing middleSonar(33, 33, MAX_DISTANCE); 
NewPing leftSonar(32, 32, MAX_DISTANCE);
NewPing rightSonar(23, 23, MAX_DISTANCE);
NewPing backSonar(25, 25, MAX_DISTANCE);

void setup() {
  Serial.begin(115200);
}

void loop() {
  int frontDistance = middleSonar.ping_cm();
  int leftDistance = leftSonar.ping_cm();
  int rightDistance = rightSonar.ping_cm();
  int backDistance = backSonar.ping_cm();

  Serial.print("Front: ");
  Serial.print(frontDistance);
  Serial.print(" cm\tLeft: ");
  Serial.print(leftDistance);
  Serial.print(" cm\tRight: ");
  Serial.print(rightDistance);
  Serial.print(" cm\tBack: ");
  Serial.print(backDistance);
  Serial.println(" cm");

  delay(200);
}
