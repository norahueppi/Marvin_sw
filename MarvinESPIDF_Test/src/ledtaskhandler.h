#pragma once

#include "stdio.h"

#define ON  1
#define OFF 0

void init_led_handler(void);

void setLED(uint8_t on_off);