#pragma once

#include "stdio.h"

#define GPIO_I2C_SDA        0
#define GPIO_I2C_SCL        2

void init_tof_handler(uint16_t updatetime, uint16_t kilo_iterations, uint8_t spad, uint16_t threshold_low_mm, uint16_t threshold_high_mm);

bool getSensor();
uint16_t tmf8820_distance();