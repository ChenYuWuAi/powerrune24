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



extern "C" void app_main(void)
{
    // 在此编写单元测试代码
    ESP_LOGI("main", "Unit Test Start");
}
