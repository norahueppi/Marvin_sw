#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "audiotaskhandler.h"

#define TAG "MUSIC_ON"

EventGroupHandle_t audioeventgroup;

void audioTask (void* param)
{
    vTaskDelay(10/portTICK_PERIOD_MS);
}

void init_audio_taskhandler (void)
{
    audioeventgroup = xEventGroupCreate;
    xTaskCreate(audioTask, "audiotaks", 4*2048, NULL, 11, NULL);
}