
#include "main.h"

/**
 * @note beacon timeout处理函数
 */
void beacon_timeout(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    // 立刻停止电机
    ESP_LOGE(TAG, "beacon_timeout, stopping motor");
    esp_event_post_to(pr_events_loop_handle, PRM, PRM_STOP_EVENT, NULL, 0, portMAX_DELAY);
    return;
}

// define event base handler function
static void PRM_event_handler(void *handler_args, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    // set handler_args to motor_3508
    Motor *motor_3508 = (Motor *)handler_args;
    esp_err_t err = ESP_OK;
    if (event_base == PRC)
    {
        if (event_id == OTA_BEGIN_EVENT)
        {
            ESP_LOGI(TAG, "OTA_BEGIN_EVENT, stopping motor");
            // 同PRM_STOP_EVENT
            motor_3508->disable_motor(CONFIG_DEFAULT_MOTOR_ID);
            return;
        }
        else if (event_id == CONFIG_EVENT)
        {
            CONFIG_EVENT_DATA *config_data = (CONFIG_EVENT_DATA *)event_data;
            PowerRune_Motor_config_info_t *config_info = &config_data->config_motor_info;
            // 同PRM_STOP_EVENT
            motor_3508->reset(config_info->kp, config_info->ki, config_info->kd, config_info->i_max, config_info->d_max, config_info->out_max);
            ESP_LOGI(TAG, "CONFIG_EVENT, resetting PID: Kp: %f, Ki: %f, Kd: %f, Imax: %f, Dmax: %f, Outmax: %f", config_info->kp, config_info->ki, config_info->kd, config_info->i_max, config_info->d_max, config_info->out_max);
            return;
        }
    }
    switch (event_id)
    {
    case PRM_UNLOCK_EVENT:
    {
        ESP_LOGI(TAG, "PRM_UNLOCK_EVENT");

        err = motor_3508->unlock_motor(CONFIG_DEFAULT_MOTOR_ID);
        // post PRM_UNLOCK_DONE_EVENT
        PRM_UNLOCK_DONE_EVENT_DATA unlock_done_data = {
            .status = err,
        };
        ESP_ERROR_CHECK(esp_event_post_to(pr_events_loop_handle, PRM, PRM_UNLOCK_DONE_EVENT, &unlock_done_data, sizeof(PRM_UNLOCK_DONE_EVENT_DATA), portMAX_DELAY));
        break;
    }
    case PRM_START_EVENT:
    {
        struct PRM_START_EVENT_DATA *start_data = (struct PRM_START_EVENT_DATA *)event_data;
        ESP_LOGI(TAG, "Starting motor with mode: %d, clockwise: %d", start_data->mode, start_data->clockwise);
        if (start_data->mode == PRA_RUNE_SMALL_MODE)
        {
            // set speed to 1140 in rpm
            err = motor_3508->set_speed(CONFIG_DEFAULT_MOTOR_ID, start_data->clockwise ? 1140 : -1140);
        }
        else if (start_data->mode == PRA_RUNE_BIG_MODE)
        {
            ESP_LOGI(TAG, "Amplitude: %f, omega: %f, offset: %f", start_data->amplitude, start_data->omega, start_data->offset);
            // clockwise
            if (start_data->clockwise == PRM_DIRECTION_CLOCKWISE)
            {
                start_data->amplitude = -start_data->amplitude;
                start_data->offset = -start_data->offset;
            }
            // set motor status to MOTOR_TRACE_SIN_PENDING
            err = motor_3508->set_speed(CONFIG_DEFAULT_MOTOR_ID, start_data->amplitude, start_data->omega, start_data->offset);
        }
        else
        {
            err = ESP_ERR_INVALID_ARG;
        };
        PRM_START_DONE_EVENT_DATA start_done_data = {
            .status = err,
            .mode = start_data->mode,
        };
        // post PRM_START_DONE_EVENT
        ESP_ERROR_CHECK(esp_event_post_to(pr_events_loop_handle, PRM, PRM_START_DONE_EVENT, &start_done_data, sizeof(PRM_START_DONE_EVENT_DATA), portMAX_DELAY));
        break;
    }
    case PRM_STOP_EVENT:
        ESP_LOGI(TAG, "PRM_STOP_EVENT");
        motor_3508->disable_motor(CONFIG_DEFAULT_MOTOR_ID);
        break;
    default:
        break;
    };
};

extern "C" void app_main(void)
{
    // LED and LED Strip init
    led = new LED(GPIO_NUM_2, 1, LED_MODE_ON, 1);

    // 启动事件循环
    esp_event_loop_args_t loop_args = {
        .queue_size = 10,
        .task_name = "pr_events_loop",
        .task_priority = 5,
        .task_stack_size = 8192,
    };
    ESP_ERROR_CHECK(esp_event_loop_create(&loop_args, &pr_events_loop_handle));

    // Firmware init
    Firmware firmware;

    // ESP-NOW init
    espnow_protocol = new ESPNowProtocol(beacon_timeout);

    ESP_LOGI(TAG, "ESP Now Start");

    uint8_t motor_counts = 1;
    // 电机控制器初始化 和 id 数组
    // 一个电机，ID为1
    uint8_t id[motor_counts] = {1};
    Motor motor_3508(id, motor_counts, GPIO_NUM_4, GPIO_NUM_5,
                     config->get_config_info_pt()->kp,
                     config->get_config_info_pt()->ki,
                     config->get_config_info_pt()->kd,
                     2000.0, // P_max
                     config->get_config_info_pt()->i_max,
                     config->get_config_info_pt()->d_max,
                     config->get_config_info_pt()->out_max);

    // 注册大符通讯协议事件
    // 发送事件
#if CONFIG_POWERRUNE_TYPE == 1 // RLogo
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_BEGIN_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, CONFIG_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_START_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_STOP_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_COMPLETE_EVENT, ESPNowProtocol::tx_event_handler, NULL));
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
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_BEGIN_EVENT, Firmware::global_pr_event_handler, NULL));
#endif
#if CONFIG_POWERRUNE_TYPE != 1 // 除了主控外的设备
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_COMPLETE_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, CONFIG_COMPLETE_EVENT, ESPNowProtocol::tx_event_handler, NULL));
#endif

    // register event PRM handler, transfer motor_3508 to handler_args
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, ESP_EVENT_ANY_ID, PRM_event_handler, &motor_3508));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, ESP_EVENT_ANY_ID, PRM_event_handler, &motor_3508));

    /*
        Unit Test
        // Unit Test, Post PRM_UNLOCK_EVENT
        ESP_LOGI(TAG, "posting PRM_UNLOCK_EVENT");
        ESP_ERROR_CHECK(esp_event_post_to(pr_events_loop_handle, PRM, PRM_UNLOCK_EVENT, NULL, 0, portMAX_DELAY));
        // Unit Test, Post PRM_START_EVENT
        ESP_LOGI(TAG, "posting PRM_START_EVENT");
        struct PRM_START_EVENT_DATA start_data = {
            .mode = PRA_RUNE_SMALL_MODE,
            .clockwise = PRM_DIRECTION_ANTICLOCKWISE,
        };
        ESP_ERROR_CHECK(esp_event_post_to(pr_events_loop_handle, PRM, PRM_START_EVENT, &start_data, sizeof(PRM_START_EVENT_DATA), portMAX_DELAY));
        // Wait for 5S
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        // Unit Test, Post PRM_STOP_EVENT
        ESP_LOGI(TAG, "posting PRM_STOP_EVENT");
        ESP_ERROR_CHECK(esp_event_post_to(pr_events_loop_handle, PRM, PRM_STOP_EVENT, NULL, 0, portMAX_DELAY));
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        // Unit Test, Post PRM_START_EVENT
        ESP_LOGI(TAG, "posting PRM_START_EVENT");
        start_data = {
            .mode = PRA_RUNE_SMALL_MODE,
            .clockwise = PRM_DIRECTION_CLOCKWISE,
        };
        ESP_ERROR_CHECK(esp_event_post_to(pr_events_loop_handle, PRM, PRM_START_EVENT, &start_data, sizeof(PRM_START_EVENT_DATA), portMAX_DELAY));
        // Wait for 5S
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        // Unit Test, Post PRM_STOP_EVENT
        ESP_LOGI(TAG, "posting PRM_STOP_EVENT");
        ESP_ERROR_CHECK(esp_event_post_to(pr_events_loop_handle, PRM, PRM_STOP_EVENT, NULL, 0, portMAX_DELAY));
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        // Unit Test, Post PRM_START_EVENT
        ESP_LOGI(TAG, "posting PRM_START_EVENT");
        start_data.mode = PRA_RUNE_BIG_MODE;
        start_data.clockwise = PRM_DIRECTION_ANTICLOCKWISE;
        // 数据全给正为逆时针
        start_data.amplitude = 1.045;
        start_data.omega = 1.884;
        start_data.offset = 2.090 - start_data.amplitude;
        ESP_ERROR_CHECK(esp_event_post_to(pr_events_loop_handle, PRM, PRM_START_EVENT, &start_data, sizeof(PRM_START_EVENT_DATA), portMAX_DELAY));

        // Wait for 10S
        vTaskDelay(10000 / portTICK_PERIOD_MS);

        // Unit Test, Post PRM_STOP_EVENT
        ESP_LOGI(TAG, "posting PRM_STOP_EVENT");
        ESP_ERROR_CHECK(esp_event_post_to(pr_events_loop_handle, PRM, PRM_STOP_EVENT, NULL, 0, portMAX_DELAY));
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        // unlock motor
        ESP_LOGI(TAG, "posting PRM_UNLOCK_EVENT");
        ESP_ERROR_CHECK(esp_event_post_to(pr_events_loop_handle, PRM, PRM_UNLOCK_EVENT, NULL, 0, portMAX_DELAY));

        // Unit Test, Post PRM_START_EVENT
        ESP_LOGI(TAG, "posting PRM_START_EVENT");
        start_data.mode = PRA_RUNE_BIG_MODE;
        start_data.clockwise = PRM_DIRECTION_CLOCKWISE;
        // 数据全给正为逆时针
        start_data.amplitude = 1.045;
        start_data.omega = 1.884;
        start_data.offset = 2.090 - start_data.amplitude;
        ESP_ERROR_CHECK(esp_event_post_to(pr_events_loop_handle, PRM, PRM_START_EVENT, &start_data, sizeof(PRM_START_EVENT_DATA), portMAX_DELAY));

        // Wait for 10S
        vTaskDelay(10000 / portTICK_PERIOD_MS);

        // // Unit Test, Post PRM_STOP_EVENT
        // ESP_LOGI(TAG, "posting PRM_STOP_EVENT");
        // ESP_ERROR_CHECK(esp_event_post_to(pr_events_loop_handle, PRM, PRM_STOP_EVENT, NULL, 0, portMAX_DELAY));
    */
    while (1)
    {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}
