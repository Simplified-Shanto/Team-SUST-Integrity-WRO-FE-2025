#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define MENU_BUTTON 19
#define GAME_BUTTON 13

#define LONG_PRESS_DURATION 1500
#define DOUBLE_CLICK_TIME 400

enum Mode {
  GAME_MODE,
  MENU_MODE,
  EDIT_MODE,
  SENSOR_TEST_MODE
};

Mode currentMode = GAME_MODE;

// Menu value structure
struct MenuValue {
  const char* name;
  float value;
  bool isFloat;
};

MenuValue menuValues[] = {
  { "Kp", 0.50, true }, { "Kd", 1.20, true }, { "Ki", 0.00, true }, { "Wall Sid", 10, false }, { "Wall Dis", 12, false }, { "White Surf", 0, false }, { "Blue Line", 0, false }, { "Yellow Line", 0, false }, { "Servo Min", 0, false }, { "Servo Mid", 90, false }, { "Servo Max", 180, false }
};
const int valueCount = sizeof(menuValues) / sizeof(menuValues[0]);

// Sensor value structure
struct SensorData {
  const char* name;
  int value;
};

SensorData sensorValues[] = {
  { "L Sonar", 10 }, { "R Sonar", 11 }, { "F Sonar", 9 }, { "B Sonar", 13 }, { "IR", 1 }, { "Red L Counter", 0 }, { "Yellow L Counter", 0 }, { "Total L Counter", 0 }, { "Serial val", 255 }
};


const int sensorItemCount = sizeof(sensorValues) / sizeof(sensorValues[0]);
int sensorTopItem = 0;

String serialValStr = "";

const char* menuItems[] = {
  "Kp", "Kd", "Ki", "Wall Sid", "Wall Dis",
  "White Surf", "Blue Line", "Yellow Line", "Servo Min", "Servo Mid", "Servo Max",
  "Calibrate", "Sensor Test", "Run Test", "Motor Test", "Reset"
};
const int menuLength = sizeof(menuItems) / sizeof(menuItems[0]);
const int itemsPerPage = 5;

int selectedItem = 0;
int topItem = 0;

bool menuButtonState = HIGH, lastMenuButtonState = HIGH;
bool gameButtonState = HIGH, lastGameButtonState = HIGH;

unsigned long lastMenuDown = 0, lastClickTime = 0;
unsigned long lastGameDown = 0;
bool menuWaitingForSecondClick = false;

void setup() {
  pinMode(MENU_BUTTON, INPUT_PULLUP);
  pinMode(GAME_BUTTON, INPUT_PULLUP);
  Serial.begin(115200);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (1)
      ;
  }
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  showGameScreen();
}

void loop() {
  handleMenuButton();
  handleGameButton();

  if (Serial.available() > 0) {
    serialValStr = "";
    while (Serial.available() > 0) {
      char c = Serial.read();
      serialValStr += c;
    }
    if (serialValStr.length() > 20) {
      serialValStr = serialValStr.substring(serialValStr.length() - 20);
    }
  }

  if (currentMode == SENSOR_TEST_MODE) {
    drawSensorMenu();
  }
}

bool hasValue(int index) {
  const char* name = menuItems[index];
  for (int i = 0; i < valueCount; i++) {
    if (strcmp(menuValues[i].name, name) == 0) return true;
  }
  return false;
}

MenuValue* getMenuValueByName(const char* name) {
  for (int i = 0; i < valueCount; i++) {
    if (strcmp(menuValues[i].name, name) == 0) return &menuValues[i];
  }
  return nullptr;
}

void updateMenuValue(const char* name, float delta) {
  MenuValue* ptr = getMenuValueByName(name);
  if (ptr) {
    if (ptr->isFloat) {
      ptr->value += delta;
    } else {
      ptr->value += (delta > 0) ? 1 : -1;
    }
  }
}

int* getSensorByName(const char* name) {
  for (int i = 0; i < sensorItemCount; i++) {
    if (strcmp(sensorValues[i].name, name) == 0) return &sensorValues[i].value;
  }
  return nullptr;
}

void updateSensorValue(const char* name, int delta) {
  int* ptr = getSensorByName(name);
  if (ptr) *ptr += delta;
}

void handleMenuButton() {
  menuButtonState = digitalRead(MENU_BUTTON);
  unsigned long now = millis();

  if (menuButtonState == LOW && lastMenuButtonState == HIGH)
    lastMenuDown = now;

  if (menuButtonState == HIGH && lastMenuButtonState == LOW) {
    unsigned long pressDuration = now - lastMenuDown;

    if (pressDuration >= LONG_PRESS_DURATION) {
      if (currentMode == GAME_MODE) {
        currentMode = MENU_MODE;
        drawMenu();
      } else if (currentMode == SENSOR_TEST_MODE) {
        currentMode = MENU_MODE;
        drawMenu();
      } else {
        currentMode = GAME_MODE;
        showGameScreen();
      }
    } else {
      if (menuWaitingForSecondClick && (now - lastClickTime <= DOUBLE_CLICK_TIME)) {
        menuWaitingForSecondClick = false;
        if (currentMode == MENU_MODE) {
          if (hasValue(selectedItem)) {
            currentMode = EDIT_MODE;
            drawEditScreen();
          } else if (strcmp(menuItems[selectedItem], "Sensor Test") == 0) {
            currentMode = SENSOR_TEST_MODE;
            drawSensorMenu();
          }
        }
      } else {
        menuWaitingForSecondClick = true;
        lastClickTime = now;
      }
    }
  }

  if (menuWaitingForSecondClick && (now - lastClickTime > DOUBLE_CLICK_TIME)) {
    menuWaitingForSecondClick = false;
    if (currentMode == MENU_MODE) {
      selectedItem = (selectedItem + 1) % menuLength;
      updateScroll();
      drawMenu();
    } else if (currentMode == EDIT_MODE) {
      updateMenuValue(menuItems[selectedItem], 0.1);
      drawEditScreen();
    } else if (currentMode == SENSOR_TEST_MODE) {
      sensorTopItem = (sensorTopItem + 1) % sensorItemCount;
      drawSensorMenu();
    }
  }

  lastMenuButtonState = menuButtonState;
}

void handleGameButton() {
  gameButtonState = digitalRead(GAME_BUTTON);
  unsigned long now = millis();

  if (gameButtonState == LOW && lastGameButtonState == HIGH)
    lastGameDown = now;

  if (gameButtonState == HIGH && lastGameButtonState == LOW) {
    unsigned long pressDuration = now - lastGameDown;

    if (pressDuration >= LONG_PRESS_DURATION) {
      if (currentMode == EDIT_MODE || currentMode == SENSOR_TEST_MODE) {
        currentMode = MENU_MODE;
        drawMenu();
      }
    } else {
      if (currentMode == MENU_MODE) {
        selectedItem = (selectedItem - 1 + menuLength) % menuLength;
        updateScroll();
        drawMenu();
      } else if (currentMode == EDIT_MODE) {
        updateMenuValue(menuItems[selectedItem], -0.1);
        drawEditScreen();
      } else if (currentMode == SENSOR_TEST_MODE) {
        sensorTopItem = (sensorTopItem - 1 + sensorItemCount) % sensorItemCount;
        drawSensorMenu();
      }
    }
  }

  lastGameButtonState = gameButtonState;
}

void updateScroll() {
  if (selectedItem < topItem) topItem = selectedItem;
  else if (selectedItem >= topItem + itemsPerPage)
    topItem = selectedItem - itemsPerPage + 1;
}

void drawSensorMenu() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("== Sensor Test ==");

  int lineHeight = 11;

  for (int i = 0; i < itemsPerPage && (sensorTopItem + i) < sensorItemCount; i++) {
    int index = sensorTopItem + i;
    int y = 11 + i * lineHeight;
    display.setCursor(0, y);
    display.print(sensorValues[index].name);

    if (strcmp(sensorValues[index].name, "Serial val") == 0) {
      String valToShow = serialValStr;
      if (valToShow.length() == 0) valToShow = "(empty)";
      if (valToShow.length() > 20) valToShow = valToShow.substring(valToShow.length() - 20);
      int xRight = SCREEN_WIDTH - 6 * valToShow.length();
      display.setCursor(xRight, y);
      display.print(valToShow);
    } else {
      char valStr[12];
      sprintf(valStr, "%d", sensorValues[index].value);
      int xRight = SCREEN_WIDTH - 6 * strlen(valStr);
      display.setCursor(xRight, y);
      display.print(valStr);
    }
  }

  display.display();
}

void drawMenu() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("== Menu Mode ==");

  int lineHeight = 11;
  for (int i = 0; i < itemsPerPage && (topItem + i) < menuLength; i++) {
    int itemIndex = topItem + i;
    int y = 11 + i * lineHeight;
    display.setCursor(0, y);
    display.print(itemIndex == selectedItem ? "> " : "  ");
    const char* name = menuItems[itemIndex];
    display.print(name);
    MenuValue* val = getMenuValueByName(name);
    if (val) {
      char buf[8];
      if (val->isFloat) {
        dtostrf(val->value, 5, 2, buf);
      } else {
        sprintf(buf, "%d", (int)val->value);
      }
      display.setCursor(SCREEN_WIDTH - 6 * strlen(buf), y);
      display.print(buf);
    }
  }

  display.display();
}

void drawEditScreen() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("== Edit Mode ==");

  const char* name = menuItems[selectedItem];
  display.setCursor(0, 20);
  display.print(name);

  MenuValue* val = getMenuValueByName(name);
  if (val) {
    char buf[10];
    if (val->isFloat) {
      dtostrf(val->value, 6, 2, buf);
    } else {
      sprintf(buf, "%d", (int)val->value);
    }
    display.setCursor(0, 40);
    display.print("Value: ");
    display.print(buf);
  }

  display.display();
}

void showGameScreen() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("== Game Mode ==");
  display.println("Ready to play...");
  display.display();
}