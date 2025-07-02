// MenuManager.h
#ifndef MENU_MANAGER_H
#define MENU_MANAGER_H

#include <EEPROM.h>

struct MenuValue {
  const char* name;
  float value;
  bool isFloat;
};

struct SensorData {
  const char* name;
  int value;
};

enum Mode {
  GAME_MODE,
  MENU_MODE,
  EDIT_MODE,
  SENSOR_TEST_MODE
};

extern MenuValue menuValues[];
extern const int valueCount;
extern float defaultMenuValues[];
extern const char* menuItems[];
extern const int menuLength;
extern SensorData sensorValues[];
extern const int sensorItemCount;
extern Mode currentMode;
extern int selectedItem, topItem, sensorTopItem;
extern const int itemsPerPage;

void loadMenuValuesFromEEPROM();
void saveMenuValuesToEEPROM();
void resetMenuValuesToDefault();
MenuValue* getMenuValueByName(const char* name);
bool hasValue(int index);
void updateMenuValue(const char* name, float delta);
int* getSensorByName(const char* name);
void updateSensorValue(const char* name, int delta);

#endif
