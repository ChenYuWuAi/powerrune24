/**
 * @file "main.cpp"
 * @note 本文件存放大符主控的操作逻辑代码
 */
#include "main.h"

// ops服务
int IS_PRM_SPEED_STABLE = 0;
int IS_HIT = 0;
int hitted_ID = 0;

// 排序法生成不重复随机数列
void generate_rand_sequence(uint8_t *rand_sequence, int length)
{
    int i, j, temp;
    for (i = 0; i < length; i++)
    {
        rand_sequence[i] = i + 1;
    }
    for (i = 0; i < length; i++)
    {
        j = esp_random() % length;
        temp = rand_sequence[i];
        rand_sequence[i] = rand_sequence[j];
        rand_sequence[j] = temp;
    }
}

/**
 * @brief PRA_START_EVENT 事件处理函数
 * @note 事件处理函数，仅做打印事件数据

*/
static void pra_start(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    PRA_START_EVENT_DATA *data = (PRA_START_EVENT_DATA *)event_data;
    // 打印事件数据
    ESP_LOGI(TAG_MAIN, "PRA_START_EVENT: address = %d, data_len = %d", data->address, data->data_len);
    ESP_LOGI(TAG_MAIN, "PRA_START_EVENT: mode = %d, color = %d", data->mode, data->color);

    // 停止空闲状态
    vTaskSuspend(led_animation_task_handle);
    if (data->color == 0)
    {
        led_strip->set_color(config->get_config_info_pt()->brightness, 0, 0);
    }
    else
    {
        led_strip->set_color(0, 0, config->get_config_info_pt()->brightness);
    }
    led_strip->refresh();
    return;
}

/**
 * @note beacon timeout处理函数
 */
void beacon_timeout(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{

    // 开始空闲状态
    vTaskSuspend(led_animation_task_handle);
    led_strip->clear_pixels();
    vTaskResume(led_animation_task_handle);
    if (event_data != NULL) // 自注销
    {
        // beacon注定会timeout，先注销beacon timeout事件，挂起beacon任务
        esp_event_handler_unregister_with(pr_events_loop_handle, PRC, BEACON_TIMEOUT_EVENT, beacon_timeout);
    }
    return;
}

/**
 * @brief run_task
 * @note 大符运行任务代码
 */
void run_task(void *pvParameter)
{
    const u_int8_t *value;
    u_int16_t len;

    // 生成随机数列
    uint8_t rune_start_sequence[5];
    generate_rand_sequence(rune_start_sequence, 5);
    char log_string[100];

    esp_ble_gatts_get_attr_value(ops_handle_table[RUN_VAL], &len, &value);

    ESP_LOGI(TAG_MAIN, "Run Triggered:");
    ESP_LOGI(TAG_MAIN, "Color : %s", value[0] ? "Blue" : "Red");
    ESP_LOGI(TAG_MAIN, "Mode : %s", value[1] ? "Small" : "Big");
    ESP_LOGI(TAG_MAIN, "Circulation : %s", value[2] ? "Enabled" : "Disabled");
    // 发送indicator日志
    sprintf(log_string, "PowerRune Start with Color: %s, Mode: %s, Circulation %s.", value[0] ? "Blue" : "Red", value[1] ? "Small" : "Big", value[2] ? "Enabled" : "Disabled");
    ESP_ERROR_CHECK(esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[RUN_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false));

    // 打印随机数列
    ESP_LOGI(TAG_MAIN, "Rune Start Sequence: %i, %i, %i, %i, %i", rune_start_sequence[0], rune_start_sequence[1], rune_start_sequence[2], rune_start_sequence[3], rune_start_sequence[4]);

    // 发送indicator日志
    sprintf(log_string, "Rune Sequence: %i, %i, %i, %i, %i", rune_start_sequence[0], rune_start_sequence[1], rune_start_sequence[2], rune_start_sequence[3], rune_start_sequence[4]);
    ESP_ERROR_CHECK(esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[RUN_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false));

    // 设置颜色
    vTaskSuspend(led_animation_task_handle);
    if (value[0] == 0)
    {
        led_strip->set_color(config->get_config_info_pt()->brightness, 0, 0);
    }
    else
    {
        led_strip->set_color(0, 0, config->get_config_info_pt()->brightness);
    }
    led_strip->refresh();

    sprintf(log_string, "Starting Motor...");
    ESP_ERROR_CHECK(esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[RUN_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false));

    // // 启动并等待PRM稳定
    // esp_event_post_to(pr_events_loop_handle, PRM, PRM_START_EVENT, NULL, 0, portMAX_DELAY);
    // while (IS_PRM_SPEED_STABLE == 0)
    // {
    //     vTaskDelay(100 / portTICK_PERIOD_MS);
    // }
    sprintf(log_string, "Motor speed stable.");
    ESP_ERROR_CHECK(esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[RUN_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false));


    // 发送START事件
    PRA_START_EVENT_DATA pra_start_event_data = {
        .address = 0,
        .data_len = 4,
        .mode = value[1],
        .color = value[0],
    };
    esp_event_post_to(pr_events_loop_handle, PRA, PRA_START_EVENT, &pra_start_event_data, sizeof(PRA_START_EVENT_DATA), portMAX_DELAY);
    // 等待通信
    xEventGroupWaitBits(espnow_protocol->send_state, espnow_protocol->SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);

    // int hit_count = 0;
    // for (; hit_count < 5; hit_count++)
    // {
    //     int res;
    //     int all_stop_arg = 0;
    //     res = start_pra_and_wait_hit(rune_start_sequence[hit_count]);
    //     if (res == 0)
    //     {
    //         ESP_LOGI(TAG_MAIN, "击错");
    //         esp_event_post(PRM, PRA_STOP_EVENT, &all_stop_arg, sizeof(int), portMAX_DELAY);
    //         char a[6] = "wrong";
    //         esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[RUN_VAL], sizeof(a), (uint8_t *)a, false);
    //         hit_count = 0;
    //         break;
    //     }
    //     else if (res == -1)
    //     {
    //         ESP_LOGI(TAG_MAIN, "超时");
    //         esp_event_post(PRM, PRA_STOP_EVENT, &all_stop_arg, sizeof(int), portMAX_DELAY);
    //         char a[5] = "miss";
    //         esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[RUN_VAL], sizeof(a), (uint8_t *)a, false);
    //         hit_count = 0;
    //         break;
    //     }
    // }
    // if (hit_count == 4)
    // {
    //     ESP_LOGI(TAG_MAIN, "完成");
    //     esp_event_post(PRM, PRA_COMPLETE_EVENT, NULL, 0, portMAX_DELAY);
    // }
    // else
    // {
    //     ESP_LOGI(TAG_MAIN, "未完成");
    // }
    // if (value[2] == 1)
    // {
    //     ESP_LOGI(TAG_MAIN, "自动循环");
    //     esp_event_post(RUN_EVENTS, RUN_EVENT_WRITE, NULL, 0, portMAX_DELAY);
    // }
    vTaskDelete(NULL);
}

void ota_task(void *pvParameter)
{
    OTA_BEGIN_EVENT_DATA ota_begin_event_data;
    // 队列接收，等待所有设备OTA完成
    OTA_COMPLETE_EVENT_DATA ota_complete_event_data;
    // 发送indicator日志
    char log_string[100] = "Starting OTA Operation";
    esp_err_t err = ESP_OK;

    ESP_ERROR_CHECK(esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[OTA_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false));

    // 置位队列listening bit
    // 生成队列
    Firmware::ota_complete_queue = xQueueCreate(1, sizeof(OTA_COMPLETE_EVENT_DATA));
    assert(Firmware::ota_complete_queue != NULL);
    xEventGroupSetBits(Firmware::ota_event_group, Firmware::OTA_COMPLETE_LISTENING_BIT);
    // 命令各个设备开始OTA，先暂停ESP_NOW收发，然后重新启动ESP_NOW收发
    for (size_t i = 0; i < 1; i++) // TODO: 把这里改成已连接设备数
    {
        // 字符串打印到log_string
        sprintf(log_string, "Triggering OTA for device %d", i);
        ESP_ERROR_CHECK(esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[OTA_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false));
        ota_begin_event_data.address = i;
        esp_event_post_to(pr_events_loop_handle, PRC, OTA_BEGIN_EVENT, &ota_begin_event_data, sizeof(OTA_BEGIN_EVENT_DATA), portMAX_DELAY);
        // 等待ACK
        xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    }
    for (size_t i = 0; i < 1; i++) // TODO: 把这里改成已连接设备数
    {
        xQueueReceive(Firmware::ota_complete_queue, &ota_complete_event_data, portMAX_DELAY);
        esp_log_buffer_hex(TAG_MAIN, &ota_complete_event_data, sizeof(OTA_COMPLETE_EVENT_DATA));
        if (ota_complete_event_data.status != ESP_OK)
        {
            if (ota_complete_event_data.status == ESP_ERR_NOT_SUPPORTED)
                sprintf(log_string, "OTA for device %i skipped", ota_complete_event_data.address);
            else
                sprintf(log_string, "OTA for device %i failed [%s]", ota_complete_event_data.address, esp_err_to_name(ota_complete_event_data.status));

            ESP_ERROR_CHECK(esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[OTA_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false));
        }
        else
        {
            sprintf(log_string, "OTA for device %i complete", ota_complete_event_data.address);
            ESP_ERROR_CHECK(esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[OTA_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false));
        }
    }
    // 更新自己
    ota_begin_event_data.address = 0x06;
    // LED Strip 清空
    vTaskSuspend(led_animation_task_handle);
    led_strip->clear_pixels();
    led_strip->refresh();
    sprintf(log_string, "Starting OTA for PowerRune Server");
    ESP_ERROR_CHECK(esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[OTA_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false));

    esp_event_post_to(pr_events_loop_handle, PRC, OTA_BEGIN_EVENT, &ota_begin_event_data, sizeof(OTA_BEGIN_EVENT_DATA), portMAX_DELAY);
    // 等待队列接收
    xQueueReceive(Firmware::ota_complete_queue, &ota_complete_event_data, portMAX_DELAY);
    esp_log_buffer_hex(TAG_MAIN, &ota_complete_event_data, sizeof(OTA_COMPLETE_EVENT_DATA));
    if (ota_complete_event_data.status != ESP_OK)
    {
        if (ota_complete_event_data.status == ESP_ERR_NOT_SUPPORTED)
            sprintf(log_string, "OTA for server skipped");
        else
            sprintf(log_string, "OTA for server failed [%s]", esp_err_to_name(ota_complete_event_data.status));

        ESP_ERROR_CHECK(esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[OTA_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false));
    }
    else
    {
        sprintf(log_string, "OTA for server complete, ready for restart");
        ESP_ERROR_CHECK(esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[OTA_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false));
    }
    led->set_mode(LED_MODE_FADE, 0);
    // 更新完成，准备重启
    if (ota_complete_event_data.status == ESP_OK)
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        esp_restart();
    }
    vTaskResume(led_animation_task_handle);
    sprintf(log_string, "OTA operation complete");
    ESP_ERROR_CHECK(esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[OTA_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false));
    xEventGroupClearBits(Firmware::ota_event_group, Firmware::OTA_COMPLETE_LISTENING_BIT);
    vQueueDelete(Firmware::ota_complete_queue);
    vTaskDelete(NULL);
}

// PowerRune_Events handles
static void pra_stop(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    switch (*((int *)event_data))
    {
    case 0:
        break;
    default:
        break;
    }
}

void led_animation_task(void *pvParameter)
{
    uint16_t sequence[] = {1, 2, 3, 4, 5, 13, 20, 27, 34, 41, 47, 46, 45, 44, 43, 35, 28, 21, 14, 7};
    uint8_t sequence_len = sizeof(sequence) / sizeof(sequence[0]); // 20
    uint8_t line_len = 15;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        for (int i = 0; i < sequence_len; i++)
        {
            for (int j = 0; j < line_len; j++)
            {
                // TODO：把亮度改成从Config中获取的
                led_strip->set_color_index(sequence[(i + j) % sequence_len], 20, 0, 0);
            }
            led_strip->refresh();
            vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);
            // 只需要关闭第一个灯
            led_strip->set_color_index(sequence[i], 0, 0, 0);
            led_strip->refresh();
        }
    }
}

extern "C" void app_main(void)
{
    // LED and LED Strip init
    led = new LED(GPIO_NUM_2);
    led_strip = new LED_Strip(GPIO_NUM_8, 49);

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

    // 注册大符通讯协议事件
    // 发送事件
#if CONFIG_POWERRUNE_TYPE == 1 // RLogo
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_BEGIN_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, CONFIG_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_START_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_STOP_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_START_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_UNLOCK_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_BEGIN_EVENT, Firmware::global_pr_event_handler, NULL));
#endif
#if CONFIG_POWERRUNE_TYPE == 0 // Armour
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_HIT_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_BEGIN_EVENT, Firmware::global_pr_event_handler, NULL));
#endif
#if CONFIG_POWERRUNE_TYPE == 2 // Motor
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_DISCONNECT_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_SPEED_STABLE_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_START_DONE_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_UNLOCK_DONE_EVENT, ESPNowProtocol::tx_event_handler, NULL));
#endif
#if CONFIG_POWERRUNE_TYPE != 1 // 除了主控外的设备
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_COMPLETE_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, CONFIG_COMPLETE_EVENT, ESPNowProtocol::tx_event_handler, NULL));
#endif

    // BLE Start
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    // 获取ble默认配置
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();

    // GATT的回调注册
    esp_ble_gatts_register_callback(gatts_event_handler);
    // GAP事件的函数
    esp_ble_gap_register_callback(gap_event_handler);
    // 注册APP
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
    {
        ESP_LOGE(TAG_MAIN, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    ESP_LOGI(TAG_MAIN, "BLE Started.");

    // 注册事件handle
    // spp服务
    // Register url event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(URL_EVENTS, URL_EVENT_READ, url_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(URL_EVENTS, URL_EVENT_WRITE, url_write_handler, NULL, NULL));
    // Register mac event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(MAC_EVENTS, MAC_EVENT_READ, mac_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(MAC_EVENTS, MAC_EVENT_WRITE, mac_write_handler, NULL, NULL));
    // Register ssid event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(SSID_EVENTS, SSID_EVENT_READ, ssid_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(SSID_EVENTS, SSID_EVENT_WRITE, ssid_write_handler, NULL, NULL));
    // Register wifi event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(Wifi_EVENTS, Wifi_EVENT_READ, wifi_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(Wifi_EVENTS, Wifi_EVENT_WRITE, wifi_write_handler, NULL, NULL));
    // Register aota event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(AOTA_EVENTS, AOTA_EVENT_READ, aota_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(AOTA_EVENTS, AOTA_EVENT_WRITE, aota_write_handler, NULL, NULL));
    // Register lit event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(LIT_EVENTS, LIT_EVENT_READ, lit_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(LIT_EVENTS, LIT_EVENT_WRITE, lit_write_handler, NULL, NULL));
    // Register strip_lit event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(STRIP_LIT_EVENTS, STRIP_LIT_EVENT_READ, strip_lit_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(STRIP_LIT_EVENTS, STRIP_LIT_EVENT_WRITE, strip_lit_write_handler, NULL, NULL));
    // Register r_lit event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(R_LIT_EVENTS, R_LIT_EVENT_READ, r_lit_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(R_LIT_EVENTS, R_LIT_EVENT_WRITE, r_lit_write_handler, NULL, NULL));
    // Register matrix_lit event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(MATRIX_LIT_EVENTS, MATRIX_LIT_EVENT_READ, matrix_lit_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(MATRIX_LIT_EVENTS, MATRIX_LIT_EVENT_WRITE, matrix_lit_write_handler, NULL, NULL));
    // Register pid event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(PID_EVENTS, PID_EVENT_READ, pid_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(PID_EVENTS, PID_EVENT_WRITE, pid_write_handler, NULL, NULL));
    // Register armour_id event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(ARMOUR_ID_EVENTS, ARMOUR_ID_EVENT_READ, armour_id_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(ARMOUR_ID_EVENTS, ARMOUR_ID_EVENT_WRITE, armour_id_write_handler, NULL, NULL));
    // ops服务
    // Register run event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(RUN_EVENTS, RUN_EVENT_WRITE, run_write_handler, NULL, NULL));
    // Register gpa event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(GPA_EVENTS, GPA_EVENT_READ, gpa_read_handler, NULL, NULL));
    // Register unlk event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(UNLK_EVENTS, UNLK_EVENT_WRITE, unlk_write_handler, NULL, NULL));
    // Register stop event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(STOP_EVENTS, STOP_EVENT_WRITE, stop_write_handler, NULL, NULL));
    // Register ota event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(OTA_EVENTS, OTA_EVENT_WRITE, ota_write_handler, NULL, NULL));
    // PowerRune_Events
    // Register pra_start event handlers.
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_START_EVENT, pra_start, NULL));
    // Register pra_stop event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(PRA, PRA_STOP_EVENT, pra_stop, NULL, NULL));

    // 启动LED动画，表示大符初始化完成
    xTaskCreate(led_animation_task, "led_animation_task", 2048, NULL, 5, &led_animation_task_handle);

    vTaskSuspend(NULL);
}