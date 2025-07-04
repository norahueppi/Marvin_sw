
#include "stdio.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "soc/gpio_sig_map.h"
#include "driver/i2s_std.h"
#include "i2smanager.h"

#define TAG "i2smanager"

#define I2S_BUFF_SIZE                   2048

static i2s_chan_handle_t                tx_chan;        // I2S tx channel handler
static i2s_chan_handle_t                rx_chan;        // I2S rx channel handler

uint8_t *w_buf;
size_t w_bytes = I2S_BUFF_SIZE;

void i2smanager_init(uint32_t samplerate, i2s_data_bit_width_t bits_per_sample, uint8_t i2s_channel_nums)
{   
    ESP_LOGI(TAG, "i2smanager_init");
    /* Setp 1: Determine the I2S channel configuration and allocate both channels
     * The default configuration can be generated by the helper macro,
     * it only requires the I2S controller id and I2S role */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, &rx_chan));

    /* Step 2: Setting the configurations of standard mode, and initialize rx & tx channels
     * The slot configuration and clock configuration can be generated by the macros
     * These two helper macros is defined in 'i2s_std.h' which can only be used in STD mode.
     * They can help to specify the slot and clock configurations for initialization or re-configuring */
    ESP_LOGI(TAG, "Config I2S: SR: %i - BPS:%i", (int)samplerate, (int)bits_per_sample);
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(samplerate),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(bits_per_sample, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = GPIO_I2S_MCLK,    // some codecs may require mclk signal, this example doesn't need it
            .bclk = GPIO_I2S_BCLK,
            .ws   = GPIO_I2S_LRCLK,
            .dout = GPIO_I2S_DOUT,
            .din  = GPIO_I2S_DIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    /* Initialize the channels */
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));

    w_buf = (uint8_t *)calloc(1, I2S_BUFF_SIZE);
    assert(w_buf); // Check if w_buf allocation success

    ESP_ERROR_CHECK(i2s_channel_preload_data(tx_chan, w_buf, I2S_BUFF_SIZE, &w_bytes));
    /* Enable the TX channel */
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
}

esp_err_t i2smanager_read(void* data, size_t size, size_t *bytes_read, TickType_t ticks_to_wait) {
    return 0;
}
esp_err_t i2smanager_write(uint8_t *data, size_t size, size_t *bytes_written, TickType_t ticks_to_wait) {
    return i2s_channel_write(tx_chan, data, I2S_BUFF_SIZE, bytes_written, ticks_to_wait);
}