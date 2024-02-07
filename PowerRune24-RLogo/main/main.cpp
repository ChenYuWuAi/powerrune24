/**
 * @file "main.cpp"
 * @note 本文件存放大符主控的操作逻辑代码
 */
#include "main.h"

// ops服务
int IS_PRM_SPEED_STABLE = 0;
int IS_HIT = 0;
int hitted_ID = 0;

// 等待击打
int start_pra_and_wait_hit(uint8_t expected_ID)
{
    esp_event_post(PRA, PRA_START_EVENT, NULL, 0, portMAX_DELAY);

    int old_time = xTaskGetTickCount();
    while (IS_HIT == 0)
    {
        if (xTaskGetTickCount() - old_time >= 2500)
        {
            esp_event_post(PRA, PRA_STOP_EVENT, NULL, 0, portMAX_DELAY);
            return -1;
        }
    }
    IS_HIT = 0;
    if (expected_ID == hitted_ID)
    {
        esp_event_post(PRA, PRA_HIT_EVENT, NULL, 0, portMAX_DELAY);
        return 1;
    }
    else
    {
        return 0;
    }
}

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
 * @brief run_task
 * @note 大符运行任务代码
 */
void run_task(void *pvParameter)
{
    const u_int8_t *value;
    u_int16_t len;

    esp_ble_gatts_get_attr_value(ops_handle_table[RUN_VAL], &len, &value);

    printf("run特征值:\n");
    printf("value0(Red0/Blue1) = %d\r\n", value[0]);
    printf("Mode(Big0/Small1) = %d\r\n", value[1]);
    printf("Circulation = %d\r\n\n", value[2]);

    // 生成随机数列
    uint8_t rune_start_sequence[5];
    generate_rand_sequence(rune_start_sequence, 5);

    // 打印随机数列
    printf("Rune Start Sequence: ");
    for (int i = 0; i < 5; i++)
    {
        printf("%d ", rune_start_sequence[i]);
    }
    printf("\n");

    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[RUN_VAL], sizeof(rune_start_sequence), (uint8_t *)rune_start_sequence, false);

    // 等待PRM稳定
    esp_event_post(PRM, PRM_START_EVENT, NULL, 0, portMAX_DELAY);
    while (IS_PRM_SPEED_STABLE == 0)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    int hit_count = 0;
    for (; hit_count < 5; hit_count++)
    {
        int res;
        int all_stop_arg = 0;
        res = start_pra_and_wait_hit(rune_start_sequence[hit_count]);
        if (res == 0)
        {
            printf("击错\n");
            esp_event_post(PRM, PRA_STOP_EVENT, &all_stop_arg, sizeof(int), portMAX_DELAY);
            char a[6] = "wrong";
            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[RUN_VAL], sizeof(a), (uint8_t *)a, false);
            hit_count = 0;
            break;
        }
        else if (res == -1)
        {
            printf("超时\n");
            esp_event_post(PRM, PRA_STOP_EVENT, &all_stop_arg, sizeof(int), portMAX_DELAY);
            char a[5] = "miss";
            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[RUN_VAL], sizeof(a), (uint8_t *)a, false);
            hit_count = 0;
            break;
        }
    }
    if (hit_count == 4)
    {
        printf("完成\n");
        esp_event_post(PRM, PRA_COMPLETE_EVENT, NULL, 0, portMAX_DELAY);
    }
    else
    {
        printf("未完成\n");
    }
    if (value[2] == 1)
    {
        printf("自动循环\n");
        esp_event_post(RUN_EVENTS, RUN_EVENT_WRITE, NULL, 0, portMAX_DELAY);
    }
    vTaskDelete(NULL);
}

void ota_task(void *pvParameter)
{
    esp_event_post(PRC, OTA_BEGIN_EVENT, NULL, 0, portMAX_DELAY);

    int old_time = xTaskGetTickCount();
    // while (/*条件*/)
    // {
    //     if (xTaskGetTickCount() - old_time >= 2500)
    //     {//超时
    //         esp_event_post(PRA, PRA_STOP_EVENT, NULL, 0, portMAX_DELAY);
    //     }
    // }
    vTaskDelete(NULL);
    // esprestart();
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

extern "C" void app_main(void)
{
    led = new LED(GPIO_NUM_2);

    // 启动事件循环
    esp_event_loop_args_t loop_args = {
        .queue_size = 10,
        .task_name = "pr_events_loop",
        .task_priority = 5,
        .task_stack_size = 4096,
        .task_core_id = 1,
    };
    ESP_ERROR_CHECK(esp_event_loop_create(&loop_args, &pr_events_loop_handle));

    // Firmware init
    Firmware firmware;
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
    // Register armor_id event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(ARMOR_ID_EVENTS, ARMOR_ID_EVENT_READ, armor_id_read_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(ARMOR_ID_EVENTS, ARMOR_ID_EVENT_WRITE, armor_id_write_handler, NULL, NULL));
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
    // Register pra_stop event handlers.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(PRA, PRA_STOP_EVENT, pra_stop, NULL, NULL));

    vTaskSuspend(NULL);
}