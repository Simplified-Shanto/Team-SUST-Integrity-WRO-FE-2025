// InputHandler.h
#ifndef INPUT_HANDLER_H
#define INPUT_HANDLER_H

#include <Arduino.h>

#define MENU_BUTTON 19
#define GAME_BUTTON 13
#define LONG_PRESS_DURATION 1500
#define DOUBLE_CLICK_TIME 400

void handleMenuButton();
void handleGameButton();
void readSerialInput();
void updateScroll();

#endif
