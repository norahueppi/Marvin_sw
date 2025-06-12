#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "buttontaskhandler.h"

void app_main() 
{
    for(;;)
    {
        getBtnState(BTN_VOL_UP);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}