/**
 * @file LED_Strip.h
 * @brief LED_Strip驱动
 * @version 0.1
 * @date 2024-01-24
 * @author CH
 * @note 改编自官方示例
 */
#pragma once
#ifdef CONFIG_IDF_TARGET_ESP32S3
#define ENABLE_DMA // S3库需要使能该宏定义，C3库不需要
#endif
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

#include <string.h>

// 调试标签
const char *TAG_LED_STRIP = "LED_Strip";

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

    static size_t rmt_encode_led_strip(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
    {
        rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
        rmt_encoder_handle_t bytes_encoder = led_encoder->bytes_encoder;
        rmt_encoder_handle_t copy_encoder = led_encoder->copy_encoder;
        rmt_encode_state_t session_state = RMT_ENCODING_RESET;
        rmt_encode_state_t state = RMT_ENCODING_RESET;
        size_t encoded_symbols = 0;
        switch (led_encoder->state)
        {
        case 0: // send RGB data
            encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
            if (session_state & RMT_ENCODING_COMPLETE)
            {
                led_encoder->state = 1; // switch to next state when current encoding session finished
            }
            if (session_state & RMT_ENCODING_MEM_FULL)
            {
                state = rmt_encode_state_t(state | RMT_ENCODING_MEM_FULL);
                goto out; // yield if there's no free space for encoding artifacts
            }
        // fall-through
        case 1: // send reset code
            encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_encoder->reset_code,
                                                    sizeof(led_encoder->reset_code), &session_state);
            if (session_state & RMT_ENCODING_COMPLETE)
            {
                led_encoder->state = RMT_ENCODING_RESET; // back to the initial encoding session
                state = rmt_encode_state_t(state | RMT_ENCODING_COMPLETE);
            }
            if (session_state & RMT_ENCODING_MEM_FULL)
            {
                state = rmt_encode_state_t(state | RMT_ENCODING_MEM_FULL);
                goto out; // yield if there's no free space for encoding artifacts
            }
        }
    out:
        *ret_state = state;
        return encoded_symbols;
    }

    static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder)
    {
        rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
        rmt_del_encoder(led_encoder->bytes_encoder);
        rmt_del_encoder(led_encoder->copy_encoder);
        free(led_encoder);
        return ESP_OK;
    }

    static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t *encoder)
    {
        rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
        rmt_encoder_reset(led_encoder->bytes_encoder);
        rmt_encoder_reset(led_encoder->copy_encoder);
        led_encoder->state = RMT_ENCODING_RESET;
        return ESP_OK;
    }

    static esp_err_t rmt_new_led_strip_encoder(rmt_encoder_handle_t *ret_encoder)
    {
        esp_err_t ret = ESP_OK;
        rmt_led_strip_encoder_t *led_encoder = NULL;
        uint16_t reset_ticks = 250; // reset code duration defaults to 50us

        // different led strip might have its own timing requirements, following parameter is for WS2812
        rmt_bytes_encoder_config_t bytes_encoder_config = {
            rmt_symbol_word_t{
                3,
                1,
                9,
                0,
            }, // 0 code
            rmt_symbol_word_t{
                9,
                1,
                3,
                0,
            },   // 1 code
            {1}, // MSB first
        };

        rmt_copy_encoder_config_t copy_encoder_config = {};

        led_encoder = (rmt_led_strip_encoder_t *)calloc(1, sizeof(rmt_led_strip_encoder_t));
        ESP_GOTO_ON_FALSE(led_encoder, ESP_ERR_NO_MEM, err, TAG_LED_STRIP, "no mem for led strip encoder");

        led_encoder->base.encode = rmt_encode_led_strip;
        led_encoder->base.del = rmt_del_led_strip_encoder;
        led_encoder->base.reset = rmt_led_strip_encoder_reset;

        ESP_GOTO_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder->bytes_encoder), err, TAG_LED_STRIP, "create bytes encoder failed");
        ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &led_encoder->copy_encoder), err, TAG_LED_STRIP, "create copy encoder failed");

        led_encoder->reset_code = {0, reset_ticks, 0, reset_ticks};
        *ret_encoder = &led_encoder->base;
        return ESP_OK;
    err:
        if (led_encoder)
        {
            if (led_encoder->bytes_encoder)
            {
                rmt_del_encoder(led_encoder->bytes_encoder);
            }
            if (led_encoder->copy_encoder)
            {
                rmt_del_encoder(led_encoder->copy_encoder);
            }
            free(led_encoder);
        }
        return ret;
    }

protected:
    LED_color_info_t *LED_Strip_color;
    uint16_t LED_Strip_length;
    uint8_t LED_Brightness;

public:
    LED_Strip(gpio_num_t io_num = GPIO_NUM_11, uint16_t LED_Strip_length = 1);
    // 单灯颜色
    LED_color_info_t &operator[](uint16_t index);
    // 全链同颜色
    esp_err_t set_color(uint8_t red, uint8_t green, uint8_t blue);
    esp_err_t set_color(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness);
    // 单灯颜色
    esp_err_t set_color(uint16_t index, uint8_t red, uint8_t green, uint8_t blue);
    esp_err_t set_color(uint16_t index, uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness);
    esp_err_t refresh(uint8_t block = 1);
    esp_err_t clear_pixels();
    esp_err_t set_brightness_filter(uint8_t brightness);

    ~LED_Strip();
};

rmt_channel_handle_t LED_Strip::LED_tx_channel_handle = NULL;
uint8_t LED_Strip::LED_Strip_num = 0;
rmt_transmit_config_t LED_Strip::tx_config;
rmt_encoder_handle_t LED_Strip::led_encoder;

LED_Strip::LED_Strip(gpio_num_t io_num, uint16_t LED_Strip_length)
{
    this->LED_Strip_length = LED_Strip_length;
    // malloc
    this->LED_Strip_color = new LED_color_info_t[LED_Strip_length];
    memset(this->LED_Strip_color, 0, sizeof(LED_color_info_t) * LED_Strip_length);
    if (LED_Strip_num == 0)
    {
#ifdef ENABLE_DMA
        rmt_tx_channel_config_t tx_chan_config = {
            .gpio_num = io_num,
            .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
            .resolution_hz = 10000000,      // 10Mhz (Period: 0.1uS) for WS2812B
            .mem_block_symbols = 2046,
            .trans_queue_depth = 4, // We want to be able to queue 4 items
            .flags = {.with_dma = 1},
        };
#endif
#ifndef ENABLE_DMA
        rmt_tx_channel_config_t tx_chan_config = {
            .gpio_num = io_num,
            .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
            .resolution_hz = 10000000,      // 10Mhz (Period: 0.1uS) for WS2812B
            .mem_block_symbols = 192,
            .trans_queue_depth = 4, // We want to be able to queue 4 items
        };
#endif
        tx_config = {
            .loop_count = 0, // no transfer loop
        };

        ESP_LOGI(TAG_LED_STRIP, "Initialzing LED_Strip Driver");
        ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &LED_tx_channel_handle));

        ESP_LOGI(TAG_LED_STRIP, "Installing LED_Strip encoder");
        ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&led_encoder));

        ESP_LOGI(TAG_LED_STRIP, "Enabling RMT TX channel");
        ESP_ERROR_CHECK(rmt_enable(LED_tx_channel_handle));
    }
    LED_Strip_num++;
}

LED_color_info_t &LED_Strip::operator[](uint16_t index)
{
    if (index >= this->LED_Strip_length)
    {
        return this->LED_Strip_color[0];
    }
    return this->LED_Strip_color[index];
}

esp_err_t LED_Strip::set_color(uint8_t red, uint8_t green, uint8_t blue)
{
    for (uint16_t i = 0; i < this->LED_Strip_length; i++)
    {
        this->LED_Strip_color[i].green = green;
        this->LED_Strip_color[i].red = red;
        this->LED_Strip_color[i].blue = blue;
    }
    return ESP_OK;
}

esp_err_t LED_Strip::set_color(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness)
{
    for (uint16_t i = 0; i < this->LED_Strip_length; i++)
    {
        this->LED_Strip_color[i].green = green * brightness / 255.0;
        this->LED_Strip_color[i].red = red * brightness / 255.0;
        this->LED_Strip_color[i].blue = blue * brightness / 255.0;
    }
    return ESP_OK;
}

esp_err_t LED_Strip::set_color(uint16_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    if (index >= this->LED_Strip_length)
    {
        return ESP_ERR_INVALID_ARG;
    }
    this->LED_Strip_color[index].red = red;
    this->LED_Strip_color[index].green = green;
    this->LED_Strip_color[index].blue = blue;
    return ESP_OK;
}

esp_err_t LED_Strip::set_color(uint16_t index, uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness)
{
    if (index >= this->LED_Strip_length)
    {
        return ESP_ERR_INVALID_ARG;
    }
    this->LED_Strip_color[index].red = red * brightness / 255.0;
    this->LED_Strip_color[index].green = green * brightness / 255.0;
    this->LED_Strip_color[index].blue = blue * brightness / 255.0;
    return ESP_OK;
}

esp_err_t LED_Strip::refresh(uint8_t block) // 默认阻塞
{
    rmt_encoder_reset(led_encoder);
    ESP_ERROR_CHECK(rmt_transmit(LED_tx_channel_handle, led_encoder, LED_Strip_color, 3 * LED_Strip_length, &tx_config));
    if (block)
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(LED_tx_channel_handle, portMAX_DELAY));
    return ESP_OK;
}

esp_err_t LED_Strip::clear_pixels()
{
    for (uint16_t i = 0; i < this->LED_Strip_length; i++)
    {
        this->LED_Strip_color[i].red = 0;
        this->LED_Strip_color[i].green = 0;
        this->LED_Strip_color[i].blue = 0;
    }
    return ESP_OK;
}

/**
 * @brief 遍历所有像素点，将颜色值按比例缩小
 * @param brightness 亮度值，0-255
 */
esp_err_t LED_Strip::set_brightness_filter(uint8_t brightness) // 0-255
{
    this->LED_Brightness = brightness;
    for (uint16_t i = 0; i < this->LED_Strip_length; i++)
    {
        this->LED_Strip_color[i].red = this->LED_Strip_color[i].red * this->LED_Brightness / 255.0;
        this->LED_Strip_color[i].green = this->LED_Strip_color[i].green * this->LED_Brightness / 255.0;
        this->LED_Strip_color[i].blue = this->LED_Strip_color[i].blue * this->LED_Brightness / 255.0;
    }
    return ESP_OK;
}

LED_Strip::~LED_Strip()
{

    delete[] this->LED_Strip_color;
    LED_Strip_num--;
    if (LED_Strip_num == 0)
    {
        rmt_disable(LED_tx_channel_handle);
        rmt_del_encoder(led_encoder);
        rmt_del_channel(LED_tx_channel_handle);
    }
}
