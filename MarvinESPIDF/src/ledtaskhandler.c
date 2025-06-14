#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"
#include "driver/gpio.h"

#include "ledtaskhandler.h"

EventGroupHandle_t ledeventgroup;

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM      6
#define V_LED_GPIO                  38

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<V_LED_GPIO))

#define EXAMPLE_LED_NUMBERS         4
#define EXAMPLE_CHASE_SPEED_MS      10

#define ON_BITMASK  1 << 0
#define OFF_BITMASL 1 << 1

static const char *TAG = "LED";

static uint8_t led_strip_pixels[EXAMPLE_LED_NUMBERS * 3];

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */

 /*GET COLOR*/
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}


/*LED TASK*/
void ledTask (void* param)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(V_LED_GPIO, true);

    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    uint16_t hue = 0;
    uint16_t start_rgb = 0;

    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_channel_handle_t led_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    ESP_LOGI(TAG, "Install led strip encoder");
    rmt_encoder_handle_t led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    ESP_LOGI(TAG, "Start LED rainbow chase");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop

    };
    for (;;) {
        if (xEventGroupGetBits(ledeventgroup) & ON_BITMASK)
        {
            ESP_LOGW(TAG, "LED on");
            // if ()
            // {
                for (int i = 0; i < 3; i++)
                {
                    for (int j = i; j < EXAMPLE_LED_NUMBERS; j += 3)
                    {
                        // Build RGB pixels
                        hue = 0 * 360 / 50 + start_rgb;
                        led_strip_hsv2rgb(hue, 100, 10, &red, &green, &blue);
                        led_strip_pixels[j * 3 + 0] = green;
                        led_strip_pixels[j * 3 + 1] = blue;
                        led_strip_pixels[j * 3 + 2] = red;
                    }
                    // Flush RGB values to LEDs
                    vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
                    // memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
                    // ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                    // ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
                }
            // }    
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            start_rgb += 5;
        }
        else 
        {
            memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
        }
        vTaskDelay(200/portTICK_PERIOD_MS);
    }
}

void setLED(uint8_t on_off)
{
    ESP_LOGI(TAG, "on_off");
    if (on_off == ON)
    {
        xEventGroupSetBits(ledeventgroup, ON_BITMASK);
    }
    else
    {
        xEventGroupClearBits(ledeventgroup, ON_BITMASK);
    }

}

/*INIT TASK*/
void init_led_handler(void)
{
    ledeventgroup = xEventGroupCreate();
    xTaskCreate(ledTask, "ledtask", 4*2048, NULL, 11, NULL);
}
