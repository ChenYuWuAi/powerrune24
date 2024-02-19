/**
 * @file LED_Strip.h
 * @brief LED_Strip驱动
 * @version 1.1
 * @date 2024-02-18
 * @author CH
 * @note 改编自官方示例
 */
// #define ENABLE_DMA // S3库需要使能该宏定义，C3库不需要
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

#include <string.h>

struct LED_color_info_t
{
    uint8_t green;
    uint8_t red;
    uint8_t blue;
};

class LED_Strip
{
private:
    static rmt_channel_handle_t LED_tx_channel_handle;
    static uint8_t LED_Strip_num;
    static LED_Strip *LED_Strip_instance;
    static rmt_transmit_config_t tx_config;
    static rmt_encoder_handle_t led_encoder;
    typedef struct
    {
        rmt_encoder_t base;
        rmt_encoder_t *bytes_encoder;
        rmt_encoder_t *copy_encoder;
        int state;
        rmt_symbol_word_t reset_code;
    } rmt_led_strip_encoder_t;

    static size_t rmt_encode_led_strip(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state);

    static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder);

    static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t *encoder);

    static esp_err_t rmt_new_led_strip_encoder(rmt_encoder_handle_t *ret_encoder);

protected:
    LED_color_info_t *LED_Strip_color;
    uint16_t LED_Strip_length;
    uint8_t LED_Brightness;

public:
    LED_Strip(const gpio_num_t io_num = GPIO_NUM_11, uint16_t LED_Strip_length = 1);
    // 单灯颜色
    LED_color_info_t &operator[](uint16_t index);
    // 全链同颜色
    esp_err_t set_color(uint8_t red, uint8_t green, uint8_t blue);
    esp_err_t set_color(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness);
    // 单灯颜色
    esp_err_t set_color_index(uint16_t index, uint8_t red, uint8_t green, uint8_t blue);
    esp_err_t set_color_index(uint16_t index, uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness);
    esp_err_t refresh(uint8_t block = 1);
    esp_err_t clear_pixels();
    esp_err_t set_brightness_filter(uint8_t brightness);

    ~LED_Strip();
};
