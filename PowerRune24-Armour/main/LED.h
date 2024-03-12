/**
 * @file LED.h
 * @brief LED类，用于控制小型LED灯，有三种操作模式：常亮、呼吸灯、按编码闪烁
 * @version 1
 * @date 2024-02-16
 * @author CH
 */
#pragma once
#ifndef _LED_H_
#define _LED_H_
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_attr.h>
#include <string.h>

#define LED_MAX_DUTY 4095
#define TASK_LED_PRIORITY 10

/**
 * @brief LED类，用于控制小型LED灯，有三种操作模式：常亮、呼吸灯、按编码闪烁
 */
enum LED_MODE
{
    LED_MODE_ON = 0,
    LED_MODE_FADE = 1,
    LED_MODE_BLINK = 2,
};

class LED
{
private:
    ledc_channel_config_t ledc_channel;
    gpio_num_t gpio_num;
    uint8_t mode;
    uint8_t blink_code; // 闪烁编码，每秒钟闪烁blink_code次
    uint8_t fade_up;
    uint8_t invert;
    TaskHandle_t task_handle;
    // Led Mutex
    static SemaphoreHandle_t ledMutex;

public:
    static IRAM_ATTR bool fade_cb(const ledc_cb_param_t *param, void *user_arg);

    /**
     * @brief 状态机更新函数
     */
    static void task_LED(void *pvParameter);

    esp_err_t ledc_init(gpio_num_t gpio_num, uint8_t invert = 1);

    esp_err_t led_gpio_init(gpio_num_t gpio_num);

    /**
     * @brief Construct a new LED object
     *
     * @param gpio_num GPIO编号
     * @param invert 是否反转输出
     * @param mode LED模式，0为常亮/常灭，1为呼吸灯，2为按编码闪烁
     * @param blink_code 闪烁编码，闪烁blink_code次，常亮/常灭时控制开关
     */
    LED(gpio_num_t gpio_num, uint8_t invert = 1, uint8_t mode = 0, uint8_t blink_code = 0);

    /**
     * @brief 设置LED模式
     *
     * @param mode LED模式，0为常亮/常灭，1为呼吸灯，2为按编码闪烁
     * @param blink_code 闪烁编码，闪烁blink_code次，常亮/常灭时控制开关
     */
    esp_err_t set_mode(uint8_t mode, uint8_t blink_code = 0);

    /**
     * @brief 设置闪烁编码
     *
     * @param blink_code 闪烁编码，每秒钟闪烁blink_code次
     */
    esp_err_t set_blink_code(uint8_t blink_code);

    ~LED();
};

extern LED *led;

#endif