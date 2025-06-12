#pragma once
#include "stdio.h"

#define GPIO_BTN_VOL_UP   11
#define GPIO_BTN_VOL_DOWN 12
#define GPIO_BTN_MODE1    14
#define GPIO_BTN_MODE2    13

#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_BTN_VOL_UP) | (1ULL<<GPIO_BTN_VOL_DOWN) | (1ULL<<GPIO_BTN_MODE1) | (1ULL<<GPIO_BTN_MODE2))

#define BTN_VOL_UP   0
#define BTN_VOL_DOWN 1
#define BTN_MODE1    2
#define BTN_MODE2    3

bool getBtnState(uint8_t button);

void init_button_handler();