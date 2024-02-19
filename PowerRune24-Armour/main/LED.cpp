/**
 * @file LED.cpp
 * @brief LED class, used to control small LED lights, with three operation modes: always on, breathing light, and blinking according to the code
 * @version 1.0
 * @date 2024-02-16
 */
#include "LED.h"
// LOG TAG
static const char *TAG_LED = "LED";
// LED class static instance
LED *led = NULL;
// LED Mutex
SemaphoreHandle_t LED::ledMutex = NULL;

IRAM_ATTR bool LED::fade_cb(const ledc_cb_param_t *param, void *user_arg)
{
    LED *led = (LED *)user_arg;
    led->fade_up = !led->fade_up;

    return 1;
}

void LED::task_LED(void *pvParameter)
{
    LED *led = (LED *)pvParameter;
    TickType_t last_wake = 0;
    while (1)
    {
        last_wake = xTaskGetTickCount();
        switch (led->mode)
        {
        case 0:
            ESP_ERROR_CHECK(gpio_set_level(led->gpio_num, led->invert ? (!led->blink_code) : led->blink_code));
            // suspend task
            vTaskSuspend(NULL);
            break;
        case 1:
            if (led->fade_up)
                ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LED_MAX_DUTY, 1000);
            else
                ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 1000);
            // 获取操作锁，如果失败则重新进入循环
            if (xSemaphoreTake(led->ledMutex, 0) != pdTRUE)
            {
                vTaskDelay(50 / portTICK_PERIOD_MS);
                continue;
            }
            ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
            // 释放操作锁
            xSemaphoreGive(led->ledMutex);
            break;
        case 2:
            // 闪烁编码 0 为常闪 其他数为周期闪烁code次，其他时候关闭，延时用vTaskDelay
            if (led->blink_code == 0)
            {
                ESP_ERROR_CHECK(gpio_set_level(led->gpio_num, !(led->invert)));
                vTaskDelayUntil(&last_wake, 100 / portTICK_PERIOD_MS);
                ESP_ERROR_CHECK(gpio_set_level(led->gpio_num, led->invert));
                vTaskDelayUntil(&last_wake, 150 / portTICK_PERIOD_MS);
            }
            else
            {
                for (int i = 0; i < led->blink_code; i++)
                {
                    ESP_ERROR_CHECK(gpio_set_level(led->gpio_num, !(led->invert)));
                    vTaskDelayUntil(&last_wake, 150 / portTICK_PERIOD_MS);
                    ESP_ERROR_CHECK(gpio_set_level(led->gpio_num, led->invert));
                    vTaskDelayUntil(&last_wake, 150 / portTICK_PERIOD_MS);
                }
                vTaskDelay(1500 / portTICK_PERIOD_MS);
            }
            break;
        }
    }
    vTaskDelete(NULL);
}

esp_err_t LED::ledc_init(gpio_num_t gpio_num, uint8_t invert)
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

esp_err_t LED::led_gpio_init(gpio_num_t gpio_num)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = 1ULL << gpio_num;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    return gpio_config(&io_conf);
}

LED::LED(gpio_num_t gpio_num, uint8_t invert, uint8_t mode, uint8_t blink_code)
{
    this->invert = invert;
    this->gpio_num = gpio_num;
    this->mode = mode;
    this->blink_code = blink_code;
    this->fade_up = 1;
    if (mode == 1)
        ESP_ERROR_CHECK(ledc_init(gpio_num, invert));
    else
        ESP_ERROR_CHECK(led_gpio_init(gpio_num));

    ESP_LOGI(TAG_LED, "LED mode: %d, blink_code: %d", mode, blink_code);

    ledMutex = xSemaphoreCreateMutex(); // 创建互斥锁
    xTaskCreate(task_LED, "LED_blink_task", 8192, this, TASK_LED_PRIORITY, &task_handle);
}

esp_err_t LED::set_mode(uint8_t mode, uint8_t blink_code)
{
    // mutex
    if (xSemaphoreTake(ledMutex, portMAX_DELAY) != pdTRUE)
        return ESP_ERR_INVALID_STATE;
    // check args
    if (mode > 2)
        return ESP_ERR_INVALID_ARG;
    if (mode != 1 && this->mode == 1)
    {
        // deinit ledc
        ledc_fade_func_uninstall();
        // reset gpio
        ESP_ERROR_CHECK(gpio_reset_pin(gpio_num));
        // init gpio
        ESP_ERROR_CHECK(led_gpio_init(gpio_num));
    }
    else if (mode == 1 && this->mode != 1)
    {
        // deinit gpio
        ESP_ERROR_CHECK(gpio_reset_pin(gpio_num));
        ESP_ERROR_CHECK(ledc_init(gpio_num, invert));
    }
    this->mode = mode; // check args
    if (blink_code > 5)
        return ESP_ERR_INVALID_ARG;
    this->blink_code = blink_code;
    // 释放操作锁
    xSemaphoreGive(ledMutex);
    // 如果任务被挂起，重新启动任务
    vTaskResume(task_handle);
    ESP_LOGI(TAG_LED, "LED mode: %d, blink_code: %d", mode, blink_code);
    return ESP_OK;
}

esp_err_t LED::set_blink_code(uint8_t blink_code)
{
    // check args
    if (blink_code > 5)
        return ESP_ERR_INVALID_ARG;
    this->blink_code = blink_code;
    ESP_LOGI(TAG_LED, "LED mode: %d, blink_code: %d", mode, blink_code);
    // 如果任务被挂起，重新启动任务
    vTaskResume(task_handle);
    return ESP_OK;
}

LED::~LED()
{
    vTaskDelete(task_handle);
    ledc_stop(LEDC_LOW_SPEED_MODE, ledc_channel.channel, 0);
    ledc_fade_func_uninstall();
}