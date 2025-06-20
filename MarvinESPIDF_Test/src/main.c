#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_pm.h"
#include "driver/gpio.h"

#include "buttentaskhandler.h"
#include "ledtaskhandler.h"
#include "toftaskhandler.h"
#include "audioplayer.h"

#define TAG "MAIN"

#define MODE1   1
#define MODE2   2


uint8_t volume = 100;

int counterLED = 0;

int Mode = 1;
int predictablenumber = 0;

void app_main()
{
    vTaskDelay(3000/portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "startup");
    init_button_handler();
    init_led_handler();

    init_tof_handler(200, 25, 6, 100, 1500);
    vTaskDelay(2000/portTICK_PERIOD_MS);
    powerEnable(true);
    amplifierEnable(true);
    vTaskDelay(100);
    initAudioPlayer();
    initSDCard();

    vTaskDelay(100);
    setVolumeMain(volume);
    setVolumeOut1(volume);
    setVolumeOut2(volume);

    setLED(OFF);

    for(;;)
    {
        if (!isPlayingMP3())
        {
            if(getSensor())
            {
                setLED(ON);
                setColor(RAINBOW);

                counterLED = 150;

                ESP_LOGI(TAG, "distace: %i", tmf8820_distance());
                if(initSDCard() == ESP_OK)
                {
                    int nfiles = readFileListFromSD();
                    if(nfiles > 0)
                    {
                        switch (Mode)
                        {
                            case MODE1:
                                playMP3(filelist[predictablenumber].name);
                                break;

                            case MODE2:
                                int randomnumber = rand() % nfiles;
                                playMP3(filelist[randomnumber].name);
                                break;
                        }
                    }
                }
            }
        }

        counterLED --;

        if (counterLED < 1)
        {
            ESP_LOGI(TAG, "Help");
            colorstate = lastState;
        }

        if (getBtnState(BTN_MODE1))
        {
            Mode = 1;
            ESP_LOGI(TAG,"Switch: %i", Mode);
            // setLED(ON);
            // setColor(PURPLE);
        }

        if (getBtnState(BTN_MODE2))
        {
            Mode = 2;
            ESP_LOGI(TAG,"Switch: %i", Mode);
            setLED(ON);
            setColor(GREEN);
        }

        if (getBtnState(BTN_VOL_DOWN))
        {
            volume = volume - 5;
            setVolumeMain(volume);
            setVolumeOut1(volume);
            setVolumeOut2(volume);

            // ESP_LOGI(TAG, "BTN_VOL_DOWM:  %i", volume);
            // setLED(ON);
        }

        if (getBtnState(BTN_VOL_UP))
        {
            volume = volume + 5;
            setVolumeMain(volume);
            setVolumeOut1(volume);
            setVolumeOut2(volume);

            // ESP_LOGI(TAG, "BTN_VOL_UP:  %i", volume);
            // setLED(OFF);
        }
        vTaskDelay(50/portTICK_PERIOD_MS);
    }
}