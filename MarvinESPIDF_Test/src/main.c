#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define TAG "BIT_EMPFANGEN"

#define BIT_0	( 1 << 0 )
#define BIT_4	( 1 << 4 )
#define ALL_BITS 0xFFFFFFFF

#define BTN1 11
#define BTN2 12
#define BTN3 13
#define BTN4 14

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<BTN1) | (1ULL<<BTN2) | (1ULL<<BTN3) | (1ULL<<BTN4))

EventGroupHandle_t xEventGroup;

void BitUebergeberTask1(void* param)
{
    for(;;)
    {
        xEventGroupSetBits(xEventGroup, BIT0);
        vTaskDelay(300/portTICK_PERIOD_MS);
    }

}

void BitUebergeberTask2(void* param)
{
    for(;;)
    {
        xEventGroupSetBits(xEventGroup, BIT4);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }

}

void BitEmpfangerTask(void* param)
{
    for(;;)
    {
        uint32_t eventbits = xEventGroupGetBits(xEventGroup);
        ESP_LOGI(TAG, "Anzahl Bits: %i", (int)eventbits);

        xEventGroupClearBits(xEventGroup, ALL_BITS);

        vTaskDelay(200/portTICK_PERIOD_MS);
    }
}


void app_main() 
{
    xEventGroup = xEventGroupCreate();
                //Subroutine            //Name                //Stacksize     //Parameters    //Priority      //Taskhandle
    xTaskCreate(BitUebergeberTask1,     "BitUebergeber1",     2048,           NULL,           10,             NULL);

                //Subroutine            //Name                //Stacksize     //Parameters    //Priority      //Taskhandle
    xTaskCreate(BitUebergeberTask2,     "BitUebergeber2",     2048,           NULL,           11,             NULL);

                //Subroutine            //Name                //Stacksize     //Parameters    //Priority      //Taskhandle
    xTaskCreate(BitEmpfangerTask,       "BitEmpfanger",       2*2048,         NULL,           12,             NULL);

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;

}