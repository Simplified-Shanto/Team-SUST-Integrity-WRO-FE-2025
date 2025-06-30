// DisplayManager.cpp
#include "DisplayManager.h"
#include "MenuManager.h"
#include <Wire.h>

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

extern Mode currentMode;
extern String serialValStr;
extern SensorData sensorValues[];
extern const int sensorItemCount;
extern int sensorTopItem;
extern const char* menuItems[];
extern const int menuLength;
extern int selectedItem;
extern int topItem;
extern const int itemsPerPage;

void showGameScreen() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("== Game Mode ==");
  display.println("Ready to play...");
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

void showResetConfirmation() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Values Reset!");
  display.display();
  delay(1000);
  drawMenu();
}
