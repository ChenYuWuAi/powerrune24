/**
 * @file LED.h
 * @brief LED类，用于控制小型LED灯，有三种操作模式：常亮、呼吸灯、按编码闪烁
 * @version 1
 * @date 2024-01-25
 * @author CH
 */
#pragma once
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_attr.h>
#include <string.h>

#define LED_MAX_DUTY 4095

// LOG TAG
static const char *TAG_LED = "LED";

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

public:
    static IRAM_ATTR bool fade_cb(const ledc_cb_param_t *param, void *user_arg)
    {
        LED *led = (LED *)user_arg;
        led->fade_up = !led->fade_up;

        return 1;
    }

    /**
     * @brief 状态机更新函数
     */
    static void task_LED(void *pvParameter)
    {
        LED *led = (LED *)pvParameter;
        while (1)
        {
            switch (led->mode)
            {
            case 0:
                ESP_ERROR_CHECK(gpio_set_level(led->gpio_num, !led->invert));
                // delete task
                vTaskDelete(NULL);
                break;
            case 1:
                if (led->fade_up)
                    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LED_MAX_DUTY, 1000);
                else
                    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 1000);
                ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
                break;
            case 2:
                // 闪烁编码 0 为常亮 其他数为1000ms内闪烁code次，其他时候关闭，延时用vTaskDelay
                if (led->blink_code == 0)
                {
                    ESP_ERROR_CHECK(gpio_set_level(led->gpio_num, !led->invert));
                }
                else
                {
                    for (int i = 0; i < led->blink_code; i++)
                    {
                        ESP_ERROR_CHECK(gpio_set_level(led->gpio_num, !led->invert));
                        vTaskDelay(150 / portTICK_PERIOD_MS);
                        ESP_ERROR_CHECK(gpio_set_level(led->gpio_num, led->invert));
                        vTaskDelay(150 / portTICK_PERIOD_MS);
                    }
                    vTaskDelay(1500 / portTICK_PERIOD_MS);
                }
                break;
            }
        }
        vTaskDelete(NULL);
    }

    esp_err_t ledc_init(gpio_num_t gpio_num, uint8_t invert = 1)
    {
        /*
         * Prepare and set configuration of timers
         * that will be used by LED Controller
         */
        ledc_timer_config_t ledc_timer;
        ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;    // timer mode
        ledc_timer.duty_resolution = LEDC_TIMER_13_BIT; // resolution of PWM duty
        ledc_timer.timer_num = LEDC_TIMER_0;            // timer index
        ledc_timer.freq_hz = 5000;                      // frequency of PWM signal
        ledc_timer.clk_cfg = LEDC_AUTO_CLK;             // Auto select the source clock

        // Set configuration of timer0 for high speed channels
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

        memset(&ledc_channel, 0, sizeof(ledc_channel));
        ledc_channel.channel = LEDC_CHANNEL_0;
        ledc_channel.gpio_num = gpio_num;
        ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
        ledc_channel.hpoint = 0;
        ledc_channel.timer_sel = LEDC_TIMER_0;
        ledc_channel.duty = 0;
        ledc_channel.flags.output_invert = invert;

        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

        ledc_fade_func_install(0);
        ledc_cbs_t callbacks = {
            .fade_cb = LED::fade_cb,
        };
        ESP_ERROR_CHECK(ledc_cb_register(ledc_channel.speed_mode, ledc_channel.channel, &callbacks, this));
        return ESP_OK;
    }

    esp_err_t led_gpio_init(gpio_num_t gpio_num, uint8_t invert = 1)
    {
        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.pin_bit_mask = 1ULL << gpio_num;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        return gpio_config(&io_conf);
    }

    /**
     * @brief Construct a new LED object
     *
     * @param gpio_num GPIO编号
     * @param invert 是否反转输出
     * @param mode LED模式，0为常亮，1为呼吸灯，2为按编码闪烁
     * @param blink_code 闪烁编码，闪烁blink_code次
     */
    LED(gpio_num_t gpio_num, uint8_t invert = 1, uint8_t mode = 0, uint8_t blink_code = 0)
    {
        this->invert = invert;
        this->gpio_num = gpio_num;
        this->mode = mode;
        this->blink_code = blink_code;
        this->fade_up = 1;
        if (mode == 1)
        {
            ledc_init(gpio_num, invert);
        }
        else
        {
            led_gpio_init(gpio_num, invert);
        }

        ESP_LOGI(TAG_LED, "LED mode: %d, blink_code: %d", mode, blink_code);
        // create task when mode is not 0
        if (mode != 0)
            xTaskCreate(LED::task_LED, "LED_blink_task", 8192, this, tskIDLE_PRIORITY, &task_handle);
    }

    /**
     * @brief 设置LED模式
     *
     * @param mode LED模式，0为常亮，1为呼吸灯，2为按编码

    */
    esp_err_t set_mode(uint8_t mode)
    {
        // check args
        if (mode > 2)
            return ESP_ERR_INVALID_ARG;
        if (mode != 1 && this->mode == 1)
        {
            // deinit ledc
            ledc_fade_func_uninstall();
            // init gpio
            ESP_ERROR_CHECK(led_gpio_init(gpio_num, invert));
        }
        else if (mode == 1 && this->mode != 1)
        {
            // deinit gpio
            ESP_ERROR_CHECK(gpio_reset_pin(gpio_num));
            // init ledc
            ESP_ERROR_CHECK(ledc_fade_func_install(0));
            ledc_cbs_t callbacks = {
                .fade_cb = LED::fade_cb,
            };
            ESP_ERROR_CHECK(ledc_cb_register(ledc_channel.speed_mode, ledc_channel.channel, &callbacks, this));
        }
        this->mode = mode;
        // create task when mode is not 0
        if (mode != 0)
            xTaskCreate(LED::task_LED, "LED_blink_task", 8192, this, tskIDLE_PRIORITY, &task_handle);
        ESP_LOGI(TAG_LED, "LED mode: %d, blink_code: %d", mode, blink_code);
        return ESP_OK;
    }

    /**
     * @brief 设置闪烁编码
     *
     * @param blink_code 闪烁编码，每秒钟闪烁blink_code次
     */
    esp_err_t set_blink_code(uint8_t blink_code)
    {
        // check args
        if (blink_code > 5)
            return ESP_ERR_INVALID_ARG;
        this->blink_code = blink_code;
        ESP_LOGI(TAG_LED, "LED mode: %d, blink_code: %d", mode, blink_code);
        return ESP_OK;
    }

    ~LED()
    {
        vTaskDelete(task_handle);
        ledc_stop(LEDC_LOW_SPEED_MODE, ledc_channel.channel, 0);
        ledc_fade_func_uninstall();
    }
};