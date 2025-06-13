#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "buttentaskhandler.h"
#include "ledtaskhandler.h"

#define TAG "BTN_PUSHED"

// #define BIT_0	( 1 << 0 )
// #define BIT_4	( 1 << 4 )
// #define ALL_BITS 0x00FFFFFF

#define BTN1 11
#define BTN2 12
#define BTN3 13
#define BTN4 14

// int BTN1_last = 0;
// int BTN2_last = 0;
// int BTN3_last = 0;
// int BTN4_last = 0;

#define GPIO_INPUT_PIN_SEL  ((1ULL<<BTN1) | (1ULL<<BTN2) | (1ULL<<BTN3) | (1ULL<<BTN4))

// EventGroupHandle_t xEventGroup;

// void BitUebergeberTask1(void* param)
// {
//     ESP_LOGI(TAG, "Start BtnTask");
//     for(;;)
//     {
//         if((BTN1 != BTN1_last)&&(BTN1 == 1))
//         {
//             ESP_LOGI(TAG, "BTN1 ist gedrückt");
//             xEventGroupSetBits(xEventGroup, BIT0);
//         }
    
//         vTaskDelay(300/portTICK_PERIOD_MS);
//     }

// }

// void BitUebergeberTask2(void* param)
// {
//     for(;;)
//     {
//         // xEventGroupSetBits(xEventGroup, BIT4);
//         vTaskDelay(100/portTICK_PERIOD_MS);
//     }

// }

// void BitEmpfangerTask(void* param)
// {
//     uint32_t eventbits;
//     for(;;)
//     {
//         eventbits = xEventGroupGetBits(xEventGroup);
//         ESP_LOGI(TAG, "Anzahl Bits: %i", (int)eventbits);

//         xEventGroupClearBits(xEventGroup, ALL_BITS);

//         vTaskDelay(200/portTICK_PERIOD_MS);
//     }
// }


void app_main() 
{
    // vTaskDelay(1000/portTICK_PERIOD_MS);
    // xEventGroup = xEventGroupCreate();
    //             //Subroutine            //Name                //Stacksize     //Parameters    //Priority      //Taskhandle
    // xTaskCreate(BitUebergeberTask1,     "BitUebergeber1",     4*2048,           NULL,           10,             NULL);

    //             //Subroutine            //Name                //Stacksize     //Parameters    //Priority      //Taskhandle
    // xTaskCreate(BitUebergeberTask2,     "BitUebergeber2",     4*2048,           NULL,           11,             NULL);

    //             //Subroutine            //Name                //Stacksize     //Parameters    //Priority      //Taskhandle
    // xTaskCreate(BitEmpfangerTask,       "BitEmpfanger",       4*2048,         NULL,           12,             NULL);
    vTaskDelay(3000/portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "startup");
    init_button_handler();
    init_led_handler();
    setLED(OFF);

    for(;;)
    {
       

        if (getBtnState(BTN_VOL_DOWN))
        {
            ESP_LOGI(TAG, "BTN_VOL_UP: Pressed");
            setLED(ON);
        }

        if (getBtnState(BTN_VOL_UP))
        {
            ESP_LOGI(TAG, "BTN_VOL_UP: Pressed");
            setLED(OFF);
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }

}