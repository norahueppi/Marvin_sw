#pragma once
#include "stdio.h"

#define GPIO_BTN_VOL_UP 12

#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_BTN_VOL_UP))

#define BTN_VOL_UP 0

bool getBtnState(uint8_t button);

void init_button_handler();