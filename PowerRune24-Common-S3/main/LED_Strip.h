/**
 *
 */
#define LED_Strip_IO 18
#define LED_Strip_Channel 0

// LED_WS2812B相关库
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_err.h"

// TAG
const char *TAG = "LED_Strip";

struct LED_color_info_t
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

class LED_Strip
{
private:
    static rmt_channel_handle_t LED_encoder_handle;
    static uint8_t LED_Strip_num;

protected:
    LED_color_info_t *LED_Strip_color;
    uint16_t LED_Strip_length;

public:
    LED_Strip(gpio_num_t io_num, rmt_channel_t channel = RMT_CHANNEL_3, uint16_t LED_Strip_length); // Channel 3 支持DMA
    static size_t LED_encoder(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state);
    // 单灯颜色
    LED_color_info_t &operator[](uint16_t index);
    // 全链同颜色
    esp_err_t set_color(uint8_t red, uint8_t green, uint8_t blue);
    // 单灯颜色
    esp_err_t set_color(uint16_t index, uint8_t red, uint8_t green, uint8_t blue);
    esp_err_t refresh();
    esp_err_t clear();
    ~LED_Strip();
};

rmt_channel_handle_t LED_Strip::LED_encoder_handle = NULL;
uint8_t LED_Strip::LED_Strip_num = 0;

static size_t LED_encoder(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state){
    
}

LED_Strip::LED_Strip(gpio_num_t io_num, rmt_channel_t channel, uint16_t LED_Strip_length)
{
    this->LED_Strip_length = LED_Strip_length;
    this->LED_Strip_color = new LED_color_info_t[LED_Strip_length];

    ESP_LOGI(TAG, "Initialzing LED_Strip Driver")
    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = io_num,
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .resolution_hz = 10000000,      // 10Mhz (Period: 0.1uS) for WS2812B
        .trans_queue_depth = 4,         // We want to be able to queue 4 items
        .flags = {.with_dma = 1},
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &LED_encoder_handle));
    ESP_LOGI(TAG, "Installing LED_Strip encoder");
    rmt_encoder_handle_t led_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    ESP_LOGI(TAG, "Enabling RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));
    LED_Strip_num++;
}