/**
 * @file main.cpp
 * @brief 单元测试入口
 * @version 0.2
 * @date 2024-02-02
 * @note 本文件不会被编译到固件中，只用于测试Common里面的通用类型库。
 */
#include "main.h"

const char *TAG = "Main";

/**
 * @note beacon timeout处理函数
 */
void beacon_timeout(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{

    // 开始空闲状态
    esp_event_post_to(pr_events_loop_handle, PRA, PRA_STOP_EVENT, NULL, 0, portMAX_DELAY);
    return;
}

extern "C" void app_main(void)
{
    // LED and LED Strip init
    led = new LED(GPIO_NUM_48, 1, LED_MODE_ON, 1);

    // 启动事件循环
    esp_event_loop_args_t loop_args = {
        .queue_size = 10,
        .task_name = "pr_events_loop",
        .task_priority = 5,
        .task_stack_size = 4096,
    };
    ESP_ERROR_CHECK(esp_event_loop_create(&loop_args, &pr_events_loop_handle));

    // Firmware init
    Firmware firmware;

    // ESP-NOW init
    espnow_protocol = new ESPNowProtocol(beacon_timeout);

    ESP_LOGI(TAG, "ESP Now Start");
    // ID设置完成，开启led blink
    if (config->get_config_info_pt()->armour_id != 0xFF)
        led->set_mode(LED_MODE_BLINK, config->get_config_info_pt()->armour_id);
    else
        led->set_mode(LED_MODE_BLINK, 0);

    // Armour init
    PowerRune_Armour armour;

    // 注册大符通讯协议事件
    // 发送事件
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_HIT_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_BEGIN_EVENT, Firmware::global_pr_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_COMPLETE_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, CONFIG_COMPLETE_EVENT, ESPNowProtocol::tx_event_handler, NULL));

    // Register beacon timeout event handlers.
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, BEACON_TIMEOUT_EVENT, beacon_timeout, NULL));

    vTaskSuspend(NULL);
}
/* 大符Armour灯带库测试代码
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
*/