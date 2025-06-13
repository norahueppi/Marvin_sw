#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "toftaskhandler.h"

#define TAG "DETECTED"

EventGroupHandle_t tofeventgroup;

void tofTask (void* param)
{
    vTaskDelay(10/portTICK_PERIOD_MS);
}

bool getSensor(uint8_t detected)
{
    return false;
}

void init_tof_taskhandler (void)
{
    tofeventgroup = xEventGroupCreate();
    xTaskCreate(tofTask, "toftask", 4*2048, NULL, 12, NULL);
}