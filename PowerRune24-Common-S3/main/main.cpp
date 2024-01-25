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
#include <LED_Strip.h>
#include <LED.h>
#include <DEMUX.h>
#include <driver/gpio.h>

const char *TAG = "Unit Test";

// 测试LED_Strip
LED_Strip LED_Strip_0(GPIO_NUM_11, 301);
LED_Strip LED_Strip_1(GPIO_NUM_11, 86);
LED_Strip LED_Strip_2(GPIO_NUM_11, 92);

gpio_num_t DEMUX_IO[3] = {GPIO_NUM_14, GPIO_NUM_21, GPIO_NUM_38};
gpio_num_t DEMUX_IO_enable = GPIO_NUM_13;
DEMUX<3> DEMUX_LED(DEMUX_IO, DEMUX_IO_enable);

// LED update task for 3 LED strips
void LED_update_task(void *pvParameter)
{

    while (1)
    {
        DEMUX_LED = 0;
        LED_Strip_0.refresh();
        DEMUX_LED = 1;
        LED_Strip_1.refresh();
        DEMUX_LED = 2;
        LED_Strip_2.refresh();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void)
{
    // 在此编写单元测试代码
    ESP_LOGI(TAG, "Unit Test Start");

    // LED Blink Code
    LED blink(GPIO_NUM_48, 1, 2, 5);

    // GPIO 0 for test
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = 1ULL << GPIO_NUM_0;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // 测试DEMUX
    // GPIO14/21/38, GPIO13
    // LSB - - - MSB, EN
    DEMUX_LED.enable();
    DEMUX_LED = 0;

    // // Red
    // LED_color_info_t LED_Red;
    // LED_Red = {0, 255, 0};

    // // Green
    // LED_color_info_t LED_Green;
    // LED_Green = {255, 0, 0};

    // // Blue
    // LED_color_info_t LED_Blue;
    // LED_Blue = {0, 0, 255};

    // create task
    TaskHandle_t LED_update_task_handle;
    xTaskCreatePinnedToCore(&LED_update_task, "LED_update_task", 2048, NULL, 5, &LED_update_task_handle, 1);

    while (1)
    {
        // fade in&out
        for (int i = 0; i < 128; i++)
        {
            LED_Strip_0.set_color(0, 0, i);
            LED_Strip_1.set_color(0, 0, i);
            LED_Strip_2.set_color(0, 0, i);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        for (int i = 128; i > 0; i--)
        {
            LED_Strip_0.set_color(0, 0, i);
            LED_Strip_1.set_color(0, 0, i);
            LED_Strip_2.set_color(0, 0, i);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    vTaskDelete(LED_update_task_handle);

    // clear
    LED_Strip_0.clear_pixels();
    LED_Strip_1.clear_pixels();
    LED_Strip_2.clear_pixels();
    DEMUX_LED = 0;
    LED_Strip_0.refresh();
    DEMUX_LED = 1;
    LED_Strip_1.refresh();
    DEMUX_LED = 2;
    LED_Strip_2.refresh();

    ESP_LOGI(TAG, "Unit Test End");
}
