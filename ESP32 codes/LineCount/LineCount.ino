#define IRpin 36  

int blueCount = 0;
int yellowCount = 0;

bool previouslyDetected = false;
String lastColor = "";

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Line detection started...");
}

void loop() {
  int irValue = analogRead(IRpin);

  String currentColor = "";

  // Detect color
  if (irValue >= 1100 && irValue <= 1200) {
    currentColor = "blue";
  } else if (irValue > 1200 && irValue <= 1300) {
    currentColor = "yellow";
  } else {
    currentColor = "";
  }

  // Count only when entering a new colored line
  if (currentColor != "" && !previouslyDetected) {
    previouslyDetected = true;
    lastColor = currentColor;

    if (currentColor == "blue") {
      blueCount++;
      Serial.println("🔵 Blue line detected!");
    } else if (currentColor == "yellow") {
      yellowCount++;
      Serial.println("🟡 Yellow line detected!");
    }

    // Show updated count
    Serial.print("Total Blue: ");
    Serial.print(blueCount);
    Serial.print(" | Total Yellow: ");
    Serial.print(yellowCount);
    Serial.print(" | Total: ");
    Serial.println(blueCount + yellowCount);
  }

  // Reset detection when sensor moves back to white or other surface
  if (currentColor == "") {
    previouslyDetected = false;
    lastColor = "";
  }

  delay(50);  
}
