#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "buttentaskhandler.h"

#define TAG "BTN_PUSHED"

EventGroupHandle_t buttoneventgroup;

#define BTN_VOL_UP_BITMASK      1 << 0
#define BTN_VOL_DOWN_BITMASK    1 << 1
#define BTN_MODE1_BITMASK       1 << 2
#define BTN_MODE2_BITMASK       1 << 3

typedef struct
{
    uint8_t gpio;
    uint8_t bitmask;
    uint8_t count;
}button_t;

button_t button_data[4];


void buttontask(void* param)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    button_data[BTN_VOL_UP].gpio = GPIO_BTN_VOL_UP;
    button_data[BTN_VOL_UP].bitmask = BTN_VOL_UP_BITMASK;
    button_data[BTN_VOL_UP].count = 0;

    button_data[BTN_VOL_DOWN].gpio = GPIO_BTN_VOL_DOWN;
    button_data[BTN_VOL_DOWN].bitmask = BTN_VOL_DOWN_BITMASK;
    button_data[BTN_VOL_DOWN].count = 0;
    
    button_data[BTN_MODE1].gpio = GPIO_BTN_MODE1;
    button_data[BTN_MODE1].bitmask = BTN_MODE1;
    button_data[BTN_MODE1].count = 0;

    button_data[BTN_MODE2].gpio = GPIO_BTN_MODE2;
    button_data[BTN_MODE2].bitmask = BTN_MODE2;
    button_data[BTN_MODE2].count = 0;

    ESP_LOGW(TAG, "BTN_VOL_UP: Start");

    for(;;)
    {
        for(int i = 0; i < 4; i++)
        {
            if(gpio_get_level(button_data[i].gpio) == false)
            {
                ESP_LOGE(TAG, "BTN_VOL_UP: Detected");
                button_data[i].count ++;
            }
            else
            {
                if (button_data[i].count >= 1)
                {
                    ESP_LOGI(TAG, "BTN_VOL_UP: Send Bits");
                    xEventGroupSetBits(buttoneventgroup, BTN_VOL_UP_BITMASK);
                    xEventGroupSetBits(buttoneventgroup, BTN_VOL_DOWN_BITMASK);
                    xEventGroupSetBits(buttoneventgroup, BTN_MODE1_BITMASK);
                    xEventGroupSetBits(buttoneventgroup, BTN_MODE2_BITMASK);
                }
                button_data[i].count = 0;
                
            }
        }
        vTaskDelay(200/portTICK_PERIOD_MS);
    }
}

bool getBtnState(uint8_t button)
{
    ESP_LOGI(TAG, "get Buttonstate");
    if (xEventGroupGetBits(buttoneventgroup) & BTN_VOL_UP_BITMASK)
    {
        xEventGroupClearBits(buttoneventgroup, BTN_VOL_UP_BITMASK);
        return true;       
    }

    if (xEventGroupGetBits(buttoneventgroup) & BTN_VOL_DOWN_BITMASK)
    {                                                 
        xEventGroupClearBits(buttoneventgroup, BTN_VOL_DOWN_BITMASK);
        return true;       
    }

    if (xEventGroupGetBits(buttoneventgroup) & BTN_MODE1_BITMASK)
    {
        xEventGroupClearBits(buttoneventgroup, BTN_MODE1_BITMASK);
        return true;       
    }

    if (xEventGroupGetBits(buttoneventgroup) & BTN_MODE2_BITMASK)
    {
        xEventGroupClearBits(buttoneventgroup, BTN_MODE2_BITMASK);
        return true;       
    }
    return false;
}

void init_button_handler()
{
    buttoneventgroup = xEventGroupCreate();
    xTaskCreate(buttontask, "buttontask", 4*2048, NULL, 10, NULL);
}