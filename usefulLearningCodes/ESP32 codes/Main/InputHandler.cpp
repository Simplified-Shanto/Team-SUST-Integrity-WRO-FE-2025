// InputHandler.cpp
#include "InputHandler.h"
#include "MenuManager.h"
#include "DisplayManager.h"

bool menuButtonState = HIGH, lastMenuButtonState = HIGH;
bool gameButtonState = HIGH, lastGameButtonState = HIGH;

unsigned long lastMenuDown = 0, lastClickTime = 0;
unsigned long lastGameDown = 0;
bool menuWaitingForSecondClick = false;

String serialValStr = "";

void readSerialInput() {
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
}

void handleMenuButton() {
  menuButtonState = digitalRead(MENU_BUTTON);
  unsigned long now = millis();

  if (menuButtonState == LOW && lastMenuButtonState == HIGH)
    lastMenuDown = now;

  if (menuButtonState == HIGH && lastMenuButtonState == LOW) {
    unsigned long pressDuration = now - lastMenuDown;

    if (pressDuration >= LONG_PRESS_DURATION) {
      if (currentMode == GAME_MODE || currentMode == SENSOR_TEST_MODE) {
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
          } else if (strcmp(menuItems[selectedItem], "Reset") == 0) {
            resetMenuValuesToDefault();
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
