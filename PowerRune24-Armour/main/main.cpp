/**
 * @file main.cpp
 * @brief 主函数
 * @version 0.2
 * @date 2024-02-02
 */
#include "main.h"

const char *TAG = "Main";

/**
 * @note beacon timeout处理函数
 */
void beacon_timeout(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    // 失去连接灯效
    led->set_mode(LED_MODE_ON, 1);
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