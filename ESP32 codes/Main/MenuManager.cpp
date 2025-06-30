// MenuManager.cpp
#include "MenuManager.h"
#include "DisplayManager.h"
#include <string.h>
#include <math.h>

MenuValue menuValues[] = {
  { "Kp", 0.50, true }, { "Kd", 1.20, true }, { "Ki", 0.00, true },
  { "Game Round", 0, false }, { "Wall Sid", 0, false }, { "Wall Dis", 12, false },
  { "White Surf", 1450, false }, { "Blue Line", 1150, false }, { "Yellow Line", 1250, false },
  { "Servo Min", 0, false }, { "Servo Mid", 90, false }, { "Servo Max", 180, false }
};

SensorData sensorValues[] = {
  { "L Sonar", 10 }, { "R Sonar", 11 }, { "F Sonar", 9 }, { "B Sonar", 13 },
  { "IR", 1 }, { "Red L Counter", 0 }, { "Yellow L Counter", 0 }, { "Total L Counter", 0 },
  { "Serial val", 255 }
};

const char* menuItems[] = {
  "Kp", "Kd", "Ki", "Game Round", "Wall Sid", "Wall Dis",
  "White Surf", "Blue Line", "Yellow Line", "Servo Min", "Servo Mid", "Servo Max",
  "Calibrate", "Sensor Test", "Run Test", "Motor Test", "Reset"
};

float defaultMenuValues[] = { 0.50, 1.20, 0.00, 0, 0, 12, 1450, 1150, 1250, 0, 90, 180 };

const int valueCount = sizeof(menuValues) / sizeof(menuValues[0]);
const int sensorItemCount = sizeof(sensorValues) / sizeof(sensorValues[0]);
const int menuLength = sizeof(menuItems) / sizeof(menuItems[0]);
const int itemsPerPage = 5;

Mode currentMode = GAME_MODE;
int selectedItem = 0;
int topItem = 0;
int sensorTopItem = 0;

void loadMenuValuesFromEEPROM() {
  int addr = 0;
  for (int i = 0; i < valueCount; i++) {
    float val;
    EEPROM.get(addr, val);
    if (!isnan(val)) menuValues[i].value = val;
    addr += sizeof(float);
  }
}

void saveMenuValuesToEEPROM() {
  int addr = 0;
  for (int i = 0; i < valueCount; i++) {
    EEPROM.put(addr, menuValues[i].value);
    addr += sizeof(float);
  }
  EEPROM.commit();
}

void resetMenuValuesToDefault() {
  for (int i = 0; i < valueCount; i++) {
    menuValues[i].value = defaultMenuValues[i];
  }
  saveMenuValuesToEEPROM();
  showResetConfirmation();
}

MenuValue* getMenuValueByName(const char* name) {
  for (int i = 0; i < valueCount; i++) {
    if (strcmp(menuValues[i].name, name) == 0) return &menuValues[i];
  }
  return nullptr;
}

bool hasValue(int index) {
  const char* name = menuItems[index];
  for (int i = 0; i < valueCount; i++) {
    if (strcmp(menuValues[i].name, name) == 0) return true;
  }
  return false;
}

void updateMenuValue(const char* name, float delta) {
  MenuValue* ptr = getMenuValueByName(name);
  if (ptr) {
    if (ptr->isFloat) {
      ptr->value += delta;
    } else {
      ptr->value += (delta > 0) ? 1 : -1;
    }
    saveMenuValuesToEEPROM();
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
