// main.ino
#include "DisplayManager.h"
#include "MenuManager.h"
#include "InputHandler.h"

void setup() {
  pinMode(MENU_BUTTON, INPUT_PULLUP);
  pinMode(GAME_BUTTON, INPUT_PULLUP);
  Serial.begin(115200);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (1);
  }

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  EEPROM.begin(4 * valueCount);
  loadMenuValuesFromEEPROM();
  showGameScreen();
}

void loop() {
  handleMenuButton();
  handleGameButton();
  readSerialInput();

  if (currentMode == SENSOR_TEST_MODE) {
    drawSensorMenu();
  }
}
