#pragma once

#include "stdio.h"

#define ON  1
#define OFF 0

#define GREEN       2
#define PURPLE      3
#define RAINBOW     4

#define GREENSTATE      1
#define PURPLESTATE     2
#define RAINBOWSTATE    3

int lastState = 1;
int colorstate = 1;

void init_led_handler(void);

void setLED(uint8_t on_off);
void setColor(uint8_t color);