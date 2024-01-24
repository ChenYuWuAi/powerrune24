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
#include <DEMUX.h>

const char *TAG = "main";

extern "C" void app_main(void)
{
    // 在此编写单元测试代码
    ESP_LOGI("main", "Unit Test Start");

    // 测试DEMUX
    // GPIO14/21/38, GPIO13
    // LSB - - - MSB, EN
    // gpio_num_t DEMUX_IO[3] = {GPIO_NUM_14, GPIO_NUM_21, GPIO_NUM_38};
    // gpio_num_t DEMUX_IO_enable = GPIO_NUM_13;
    // DEMUX<3> DEMUX_LED(DEMUX_IO, DEMUX_IO_enable);

    // DEMUX_LED.enable();
    // DEMUX_LED = 1;

    // 测试LED_Strip
    LED_Strip LED_Strip_1(GPIO_NUM_10, 49);
    LED_color_info_t LED_Blue;
    LED_Blue = {0, 0, 255};
    for (int i = 0; i < 50; i++)
    {
        memcpy(&LED_Strip_1[i], &LED_Blue, sizeof(LED_color_info_t));
    }
    LED_Strip_1.set_brightness_filter(10);

    LED_Strip_1.refresh();
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_LOGI("main", "Unit Test End");
}
