/**
 * @file main.cpp
 * @brief 单元测试入口
 * @version 0.1
 * @date 2024-01-23
 * @note 本文件不会被编译到固件中，只用于测试Common里面的通用类型库。
 */
#include <stdio.h>
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_event_loop.h>

const char *TAG = "main";

// LED_WS2812B相关定义
#define LED_Strip_IO 18
#define LED_Strip_Channel 0

// LED_WS2812B相关库
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_err.h"

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
    LED_Strip(gpio_num_t io_num, rmt_channel_t channel=RMT_CHANNEL_3, uint16_t LED_Strip_length);
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

LED_Strip::LED_Strip(gpio_num_t io_num, rmt_channel_t channel, uint16_t LED_Strip_length)
{
    if (LED_Strip_num == 0)
    {
        rmt_config_t config;
        config.rmt_mode = RMT_MODE_TX;
        config.channel = (rmt_channel_t)channel;
        config.gpio_num = (gpio_num_t)io_num;
        config.mem_block_num = 1;
        config.tx_config.loop_en = false;
        config.tx_config.carrier_en = false;
        config.tx_config.idle_output_en = true;
        config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
        config.clk_div = 8;
        ESP_ERROR_CHECK(rmt_config(&config));
        ESP_ERROR_CHECK(rmt_driver_install((rmt_channel_t)channel, 0, 0));
        ESP_ERROR_CHECK(rmt_translator_init((rmt_channel_t)channel, LED_encoder, &LED_encoder_handle));
    }
    LED_Strip_num++;
    this->LED_Strip_length = LED_Strip_length;
    this->LED_Strip_color = new LED_color_info_t[LED_Strip_length];
}

extern "C" void app_main(void)
{
    // 在此编写单元测试代码
    ESP_LOGI("main", "Unit Test Start");
}
