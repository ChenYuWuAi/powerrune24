/**
 * @file "main.cpp"
 * @note 本文件存放大符主控的操作逻辑代码
 */
#include "main.h"

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

void unlock_done_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    SemaphoreHandle_t unlock_done_sem = (SemaphoreHandle_t)handler_args;
    PRM_UNLOCK_DONE_EVENT_DATA *data = (PRM_UNLOCK_DONE_EVENT_DATA *)event_data;
    // 打印事件数据
    ESP_LOGI(TAG_MAIN, "Motor Unlock Done, result: %s", esp_err_to_name(data->status));
    // 发送indicator日志
    char log_string[25];
    sprintf(log_string, (data->status == ESP_OK) ? "Motor unlocked." : "Fail to unlock motor.");
    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[RUN_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
    // 释放信号量
    xSemaphoreGive(unlock_done_sem);
    // 自注销
    esp_event_handler_unregister_with(pr_events_loop_handle, PRM, PRM_UNLOCK_DONE_EVENT, unlock_done_event_handler);
    return;
}

void unlock_task(void *pvParameter)
{
    char log_string[] = "Unlocking Motor...";
    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[RUN_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);

    SemaphoreHandle_t unlock_done_sem = xSemaphoreCreateBinary();
    // 注册事件
    esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_UNLOCK_DONE_EVENT, unlock_done_event_handler, unlock_done_sem);
    // 发送START事件
    PRM_UNLOCK_EVENT_DATA prm_unlock_event_data;
    esp_event_post_to(pr_events_loop_handle, PRM, PRM_UNLOCK_EVENT, &prm_unlock_event_data, sizeof(PRM_UNLOCK_EVENT_DATA), portMAX_DELAY);
    // 等待通信
    xEventGroupWaitBits(espnow_protocol->send_state, espnow_protocol->SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    // 等待信号量
    xSemaphoreTake(unlock_done_sem, portMAX_DELAY);
    vSemaphoreDelete(unlock_done_sem);
    vTaskDelete(NULL);
}

// PowerRune_Events handles
static void pra_stop(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    if (led_animation_task_handle == NULL)
        return;

    while (eTaskGetState(led_animation_task_handle) == eSuspended)
    {
        vTaskResume(led_animation_task_handle);
        // 发送重置通知
        xTaskNotifyGive(led_animation_task_handle);
        vTaskDelay(100);
    }
    return;
}

void stop_task(void *pvParameter)
{
    // 发送停止位
    TaskHandle_t run_task_handle = xTaskGetHandle("run_task");
    if (run_task_handle != NULL)
        while (eTaskGetState(run_task_handle) == eTaskState::eRunning ||
               eTaskGetState(run_task_handle) == eTaskState::eBlocked)
        {
            ESP_LOGI(TAG_MAIN, "Sending STOP to run_task");
            if (hit_timer != NULL)
                // stop timer
                xTimerStop(hit_timer, portMAX_DELAY);
            // run_queue
            PRA_HIT_EVENT_DATA hit_done_data;
            hit_done_data.address = 10;
            if (run_queue != NULL)
                xQueueSend(run_queue, &hit_done_data, portMAX_DELAY);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    // STOP
    char log_string[35] = "Stopping Armour...";
    PRA_STOP_EVENT_DATA pra_stop_event_data;
    for (uint8_t i = 0; i < 0; i++)
    {
        pra_stop_event_data.address = i;
        esp_event_post_to(pr_events_loop_handle, PRA, PRA_STOP_EVENT, &pra_stop_event_data, sizeof(PRA_STOP_EVENT_DATA), portMAX_DELAY);
        // 等待ACK
        xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    }
    pra_stop(NULL, NULL, 0, NULL);
    // sprintf(log_string, "Armour stopped, stopping motor");
    // esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[STOP_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
    // // 发送STOP到PRM
    // PRM_STOP_EVENT_DATA prm_stop_event_data;
    // esp_event_post_to(pr_events_loop_handle, PRM, PRM_STOP_EVENT, &prm_stop_event_data, sizeof(PRM_STOP_EVENT_DATA), portMAX_DELAY);
    // // 等待ACK
    // xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    // sprintf(log_string, "Motor stopped");
    // esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[STOP_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);

    vTaskDelete(NULL);
}

// PRA_HIT_EVENT 事件处理函数
void hit_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    PRA_HIT_EVENT_DATA *data = (PRA_HIT_EVENT_DATA *)event_data;
    // 释放信号量
    xQueueSend((QueueHandle_t)handler_args, data, portMAX_DELAY);
    return;
}

void hit_timer_callback(TimerHandle_t xTimer)
{
    PRA_HIT_EVENT_DATA hit_done_data;
    xQueueSend(run_queue, &hit_done_data, portMAX_DELAY);
}

void prm_speed_stable_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    SemaphoreHandle_t motor_done_sem = (SemaphoreHandle_t)handler_args;
    // 释放信号量
    xSemaphoreGive(motor_done_sem);
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
    // 随机数列
    uint8_t rune_start_sequence[5];
    char log_string[100];
    // 事件队列
    run_queue = xQueueCreate(5, sizeof(PRA_HIT_EVENT_DATA));
    // FreeRTOS计时器
    hit_timer = xTimerCreate("hit_timer", 2500 / portTICK_PERIOD_MS, pdFALSE, (void *)0, hit_timer_callback);

    // 电机信号量
    motor_done_sem = xSemaphoreCreateBinary();
    // 注册事件
    esp_ble_gatts_get_attr_value(ops_handle_table[RUN_VAL], &len, &value);

    ESP_LOGI(TAG_MAIN, "Run Triggered:");
    ESP_LOGI(TAG_MAIN, "Color : %s", value[0] ? "Blue" : "Red");
    ESP_LOGI(TAG_MAIN, "Mode : %s", value[1] ? "Small" : "Big");
    ESP_LOGI(TAG_MAIN, "Circulation : %s", value[2] ? "Enabled" : "Disabled");
    ESP_LOGI(TAG_MAIN, "Direction : %s", value[3] ? "Clockwise" : "Anti-Clockwise");
    // 发送indicator日志
    sprintf(log_string, "PowerRune Start with Color %s, Mode %s, Circulation %s, Direction %s.", value[0] ? "Blue" : "Red", value[1] ? "Small" : "Big", value[2] ? "Enabled" : "Disabled", value[3] ? "Anti-Clockwise" : "Clockwise");
    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[RUN_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);

    // 设置颜色
    vTaskSuspend(led_animation_task_handle);
    if (value[0] == 0)
        led_strip->set_color(config->get_config_info_pt()->brightness, 0, 0);
    else
        led_strip->set_color(0, 0, config->get_config_info_pt()->brightness);

    led_strip->refresh();

    /* 生成随机速度参数
   速度目标函数为： spd = a ∗ sin(𝜔𝜔 ∗ 𝑡𝑡) + 𝑏𝑏，其中 spd 的单位
   为 rad/s， t 的单位为 s， a 的取值范围为 0.780~1.045，ω的取值范围为 1.884~2.000，
   b 始终满足 b=2.090-a。每次大能量机关进入可激活状态时，所有参数重置，
   其中 t 重置为 0， a 和ω重置为取值范围内任意值。*/

    PRM_START_EVENT_DATA prm_start_event_data;
    prm_start_event_data.clockwise = value[3];
    // 大符模式
    if (value[1] == PRA_RUNE_BIG_MODE)
    {
        float a = (esp_random() % 266 + 780) / 1000.0;
        float omega = (esp_random() % 116 + 1884) / 1000.0;
        float b = 2.090 - a;
        prm_start_event_data.mode = PRA_RUNE_BIG_MODE;
        prm_start_event_data.amplitude = a;
        prm_start_event_data.omega = omega;
        prm_start_event_data.offset = b;
        ESP_LOGI(TAG_MAIN, "Starting motor in SIN tracing, amp = %f, omega = %f, b = %f", prm_start_event_data.amplitude, prm_start_event_data.omega, prm_start_event_data.offset);
        sprintf(log_string, "Starting motor in SIN tracing, amp = %f, omega = %f, b = %f", prm_start_event_data.amplitude, prm_start_event_data.omega, prm_start_event_data.offset);
    }
    else
    {
        prm_start_event_data.mode = PRA_RUNE_SMALL_MODE;
        ESP_LOGI(TAG_MAIN, "Starting motor in constant speed mode");
        sprintf(log_string, "Starting motor in constant speed mode");
    }
    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[RUN_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
    // 注册事件
    // esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_SPEED_STABLE_EVENT, prm_speed_stable_event_handler, motor_done_sem);
    // 启动并等待PRM稳定
    // esp_event_post_to(pr_events_loop_handle, PRM, PRM_START_EVENT, &prm_start_event_data, sizeof(PRM_START_EVENT_DATA), portMAX_DELAY);
    // // 等待ACK
    // xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    // // 等待信号量
    // xSemaphoreTake(motor_done_sem, portMAX_DELAY);
    // esp_event_handler_unregister_with(pr_events_loop_handle, PRM, PRM_SPEED_STABLE_EVENT, prm_speed_stable_event_handler);

    ESP_LOGI(TAG_MAIN, "Motor speed stable.");
    sprintf(log_string, "Motor speed stable.");
    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[RUN_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);

    // 事件、等待PRA_HIT_EVENT信号量
    esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_HIT_EVENT, hit_event_handler, run_queue);
    PRA_HIT_EVENT_DATA hit_done_data;
    bool hit_state = 1; // 0: 未完成 1: 完成
    bool circulation = value[2];
    uint8_t last_first_activation_armour = 0;
    do
    {
        uint8_t score = 0;
        do
            generate_rand_sequence(rune_start_sequence, 5);
        // 检测是否和上一次重复
        while (rune_start_sequence[0] == last_first_activation_armour);

        last_first_activation_armour = rune_start_sequence[0];

        // 打印随机数列
        ESP_LOGI(TAG_MAIN, "Rune Sequence: %i, %i, %i, %i, %i", rune_start_sequence[0], rune_start_sequence[1], rune_start_sequence[2], rune_start_sequence[3], rune_start_sequence[4]);

        // 发送indicator日志
        sprintf(log_string, "Rune Sequence: %i, %i, %i, %i, %i", rune_start_sequence[0], rune_start_sequence[1], rune_start_sequence[2], rune_start_sequence[3], rune_start_sequence[4]);
        esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[RUN_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);

        // 发送START事件
        PRA_START_EVENT_DATA pra_start_event_data = {
            .address = 0,
            .data_len = sizeof(PRA_START_EVENT_DATA),
            .mode = value[1],
            .color = value[0],
        };
        ESP_LOGI(TAG_MAIN, "Starting Armour...");
        sprintf(log_string, "Starting Armour...");
        for (uint8_t i = 0; i < 1; i++) // TODO: 把这里改成已连接设备数
        {
            uint8_t expected_id = 1; // 测试：等待1号靶子响应

            pra_start_event_data.address = 0; // 测试：等待1号靶子响应
            esp_event_post_to(pr_events_loop_handle, PRA, PRA_START_EVENT, &pra_start_event_data, sizeof(PRA_START_EVENT_DATA), portMAX_DELAY);
            // 等待通信
            xEventGroupWaitBits(espnow_protocol->send_state, espnow_protocol->SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
            // 开启FreeRTOS计时器
            xTimerReset(hit_timer, portMAX_DELAY);
            xTimerStart(hit_timer, portMAX_DELAY);
            xQueueReceive(run_queue, &hit_done_data, portMAX_DELAY);
            // 关闭FreeRTOS计时器
            xTimerStop(hit_timer, portMAX_DELAY);
            if (hit_done_data.address != expected_id - 1)
            {
                if (hit_done_data.address == 0xFF)
                {
                    // 超时
                    ESP_LOGE(TAG_MAIN, "Timeout hit from armour %d, activation failed", expected_id);
                    sprintf(log_string, "Timeout hit from armour %d, activation failed", expected_id);
                    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[RUN_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
                    PRA_STOP_EVENT_DATA pra_stop_event_data;
                    // 发送STOP到所有已激活设备
                    for (int8_t j = i; j >= 0; j--)
                    {
                        pra_stop_event_data.address = rune_start_sequence[j] - 1; // TODO：这里应该是个数组
                        esp_event_post_to(pr_events_loop_handle, PRA, PRA_STOP_EVENT, &pra_stop_event_data, sizeof(PRA_STOP_EVENT_DATA), portMAX_DELAY);
                        // 等待ACK
                        xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
                    }
                    hit_state = 0; // 未完成
                    break;
                }
                else if (hit_done_data.address == 10)
                {
                    // 收到PRA_STOP_EVENT，停止
                    ESP_LOGI(TAG_MAIN, "PRA_STOP_EVENT received, stopping");
                    sprintf(log_string, "PRA_STOP_EVENT received, stopping");
                    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[RUN_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
                    circulation = 0;
                    hit_state = 0;
                    break;
                }
                else
                {
                    ESP_LOGI(TAG_MAIN, "Mistaken hit from armour %d, expected %d", hit_done_data.address + 1, expected_id);
                    sprintf(log_string, "Mistaken hit from armour %d, expected %d", hit_done_data.address + 1, expected_id);
                    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[RUN_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
                    // 发送STOP到所有已激活设备
                    for (int8_t j = i; j >= 0; j--)
                    {
                        PRA_STOP_EVENT_DATA pra_stop_event_data;
                        pra_stop_event_data.address = rune_start_sequence[j] - 1; // TODO：这里应该是个数组
                        esp_event_post_to(pr_events_loop_handle, PRA, PRA_STOP_EVENT, &pra_stop_event_data, sizeof(PRA_STOP_EVENT_DATA), portMAX_DELAY);
                        // 等待ACK
                        xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
                    }
                    hit_state = 0;
                    break;
                }
            }
            else
            {
                score += hit_done_data.score;
                ESP_LOGI(TAG_MAIN, "Hit from armour %d score +%d", expected_id, hit_done_data.score);
                sprintf(log_string, "Hit from armour %d score +%d", expected_id, hit_done_data.score);
                esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[RUN_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
            }
        }
        // 发送PRA_COMPLETE_EVENT
        if (hit_state)
        {
            score_vector.push_back(score);
            uint8_t len = score_vector.size();
            // 迭代器保存最近10次成绩，不满10次的用0补齐
            if (len <= 10)
            {
                for (uint8_t i = 0; i < 10 - len; i++)
                    // 从尾到头写入ops_gpa_val
                    ops_gpa_val[i] = score_vector[len - i - 1];

                for (uint8_t i = 10 - len; i < 10; i++)
                    ops_gpa_val[i] = 0;
            }
            else
                for (uint8_t i = 0; i < 10; i++)
                    // 从尾到头写入ops_gpa_val
                    ops_gpa_val[i] = score_vector[len - i - 1];

            esp_ble_gatts_set_attr_value(ops_handle_table[GPA_VAL], sizeof(ops_gpa_val), ops_gpa_val);

            ESP_LOGI(TAG_MAIN, "[Score: %d]PowerRune Activated Successfully", score);
            sprintf(log_string, "[Score: %d]PowerRune Activated Successfully", score);
            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[RUN_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
            PRA_COMPLETE_EVENT_DATA pra_complete_event_data;
            for (uint8_t i = 0; i < 1; i++) // TODO: 把这里改成已连接设备数
            {
                pra_complete_event_data.address = i;
                esp_event_post_to(pr_events_loop_handle, PRA, PRA_COMPLETE_EVENT, &pra_complete_event_data, sizeof(PRA_COMPLETE_EVENT_DATA), portMAX_DELAY);
                // 等待ACK
                xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
            }
            // 等待灯效结束，开启下一轮或者停止
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            // 清屏
            // 发送STOP到所有已激活设备
            PRA_STOP_EVENT_DATA pra_stop_event_data;
            for (uint8_t j = 0; j < 1; j++)
            {
                pra_stop_event_data.address = 0; // TODO：这里应该是个数组
                esp_event_post_to(pr_events_loop_handle, PRA, PRA_STOP_EVENT, &pra_stop_event_data, sizeof(PRA_STOP_EVENT_DATA), portMAX_DELAY);
                // 等待ACK
                xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
            }
        }
        else
        {
            ESP_LOGW(TAG_MAIN, "PowerRune Activation Failed");
            sprintf(log_string, "PowerRune Activation Failed");
            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[RUN_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
        }

        hit_state = 1;
    } while (circulation);

    ESP_LOGI(TAG_MAIN, "PowerRune Run Complete");
    sprintf(log_string, "PowerRune Run Complete");
    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[RUN_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);

    // 恢复空闲灯效
    pra_stop(NULL, NULL, 0, NULL);
    // 发送STOP到PRM
    // PRM_STOP_EVENT_DATA prm_stop_event_data;
    // esp_event_post_to(pr_events_loop_handle, PRM, PRM_STOP_EVENT, &prm_stop_event_data, sizeof(PRM_STOP_EVENT_DATA), portMAX_DELAY);
    // // 等待ACK
    // xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    // 注销事件、删除队列、删除计时器
    esp_event_handler_unregister_with(pr_events_loop_handle, PRA, PRA_HIT_EVENT, hit_event_handler);
    vQueueDelete(run_queue);
    xTimerDelete(hit_timer, portMAX_DELAY);

    vTaskDelete(NULL);
}

void ota_task(void *pvParameter)
{
    OTA_BEGIN_EVENT_DATA ota_begin_event_data;
    // 队列接收，等待所有设备OTA完成
    OTA_COMPLETE_EVENT_DATA ota_complete_event_data;
    // 发送indicator日志
    char log_string[100] = "Starting OTA Operation";

    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[OTA_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);

    // 发送STOP到所有设备
    for (size_t i = 0; i < 1; i++) // TODO: 把这里改成已连接设备数
    {
        PRA_STOP_EVENT_DATA pra_stop_event_data;
        pra_stop_event_data.address = i;
        esp_event_post_to(pr_events_loop_handle, PRA, PRA_STOP_EVENT, &pra_stop_event_data, sizeof(PRA_STOP_EVENT_DATA), portMAX_DELAY);
        // 等待ACK
        xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    }
    pra_stop(NULL, NULL, 0, NULL);
    // PRM_STOP_EVENT_DATA prm_stop_event_data;
    // esp_event_post_to(pr_events_loop_handle, PRM, PRM_STOP_EVENT, &prm_stop_event_data, sizeof(PRM_STOP_EVENT_DATA), portMAX_DELAY);
    // // 等待ACK
    // xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);

    // 置位队列listening bit
    // 生成队列
    Firmware::ota_complete_queue = xQueueCreate(5, sizeof(OTA_COMPLETE_EVENT_DATA));
    assert(Firmware::ota_complete_queue != NULL);
    xEventGroupSetBits(Firmware::ota_event_group, Firmware::OTA_COMPLETE_LISTENING_BIT);
    // 命令各个设备开始OTA，先暂停ESP_NOW收发，然后重新启动ESP_NOW收发
    for (size_t i = 0; i < 1; i++) // TODO: 把这里改成已连接设备数
    {
        // 字符串打印到log_string
        sprintf(log_string, "Triggering OTA for device %d", i);
        esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[OTA_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
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

            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[OTA_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
        }
        else
        {
            sprintf(log_string, "OTA for device %i complete", ota_complete_event_data.address);
            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[OTA_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
        }
    }
    // 更新自己
    ota_begin_event_data.address = 0x06;
    // LED Strip 清空
    vTaskSuspend(led_animation_task_handle);
    led_strip->clear_pixels();
    led_strip->refresh();
    sprintf(log_string, "Starting OTA for PowerRune Server");
    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[OTA_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);

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

        esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[OTA_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
    }
    else
    {
        sprintf(log_string, "OTA for server complete, ready for restart");
        esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[OTA_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
    }
    led->set_mode(LED_MODE_FADE, 0);
    // 更新完成，准备重启
    if (ota_complete_event_data.status == ESP_OK)
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        esp_restart();
    }
    pra_stop(NULL, NULL, 0, NULL);
    sprintf(log_string, "OTA operation complete");
    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, ops_handle_table[OTA_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
    xEventGroupClearBits(Firmware::ota_event_group, Firmware::OTA_COMPLETE_LISTENING_BIT);
    vQueueDelete(Firmware::ota_complete_queue);
    vTaskDelete(NULL);
}

void config_task(void *pvParameter)
{
    CONFIG_EVENT_DATA config_event_data;
    memcpy(&config_event_data.config_common_info, config->get_config_common_info_pt(), sizeof(PowerRune_Common_config_info_t));
    memcpy(&config_event_data.config_motor_info, config->get_config_motor_info_pt(), sizeof(PowerRune_Motor_config_info_t));
    memcpy(&config_event_data.config_armour_info, config->get_config_armour_info_pt(ARMOUR1), sizeof(PowerRune_Armour_config_info_t));
    char log_string[100];
    if (pvParameter == NULL || *(uint8_t *)pvParameter == ARMOUR1)
    {
        // 发送notify，统一发送到URL_VAL
        sprintf(log_string, "Sending configuration to armour devices");
        esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[URL_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
        // 发送给所有装甲板设备
        for (uint8_t i = 0; i < 1; i++)
        {
            config_event_data.config_armour_info.armour_id = i + 1;
            config_event_data.address = i;
            esp_event_post_to(pr_events_loop_handle, PRC, CONFIG_EVENT, &config_event_data, sizeof(CONFIG_EVENT_DATA), portMAX_DELAY);
            // 等待ACK
            xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
        }
        sprintf(log_string, "Configuration sent to all armour devices");
        esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[URL_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
    }
    if (pvParameter == NULL || *(uint8_t *)pvParameter == MOTOR)
    {
        // 发送给电机设备
        sprintf(log_string, "Sending configuration to motor device");
        esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[URL_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
        config_event_data.address = MOTOR;
        esp_event_post_to(pr_events_loop_handle, PRC, CONFIG_EVENT, &config_event_data, sizeof(CONFIG_EVENT_DATA), portMAX_DELAY);
        // 等待ACK
        xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
        sprintf(log_string, "Configuration sent to motor device");
        esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[URL_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
    }
    vTaskDelete(NULL);
}

void reset_armour_id_task(void *pvParameter)
{
    char log_string[] = "Resetting Armour IDs";
    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[ARMOUR_ID_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
    ESPNowProtocol::reset_armour_id();
    sprintf(log_string, "Armour IDs reset");
    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[ARMOUR_ID_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
    vTaskDelete(NULL);
}

void led_animation_task(void *pvParameter)
{
    uint16_t sequence[] = {1, 2, 3, 4, 5, 13, 20, 27, 34, 41, 47, 46, 45, 44, 43, 35, 28, 21, 14, 7};
    uint8_t sequence_len = sizeof(sequence) / sizeof(sequence[0]); // 20
    uint8_t line_len = 15;
    const PowerRune_Rlogo_config_info_t *config_rlogo = config->get_config_info_pt();
    while (1)
    {
        config_rlogo = config->get_config_info_pt();
        for (int i = 0; i < sequence_len; i++)
        {
            for (int j = 0; j < line_len; j++)
            {
                led_strip->set_color_index(sequence[(i + j) % sequence_len], config_rlogo->brightness, 0, 0);
            }
            led_strip->refresh();
            vTaskDelay(100);
            // 只需要关闭第一个灯
            led_strip->set_color_index(sequence[i], 0, 0, 0);
            led_strip->refresh();
            // 接受任务通知检查是否重置
            if (ulTaskNotifyTake(pdTRUE, 0))
            {
                // 重置
                i = 0;
                led_strip->clear_pixels();
                led_strip->refresh();
            }
        }
    }
}

// GATTS最终的回调函数
void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *)param;
    uint8_t res;

    ESP_LOGI(GATTS_TABLE_TAG, "event = %x\n", event);
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        // 注册事件
        esp_ble_gap_set_device_name(DEVICE_NAME);
        esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, sizeof(spp_adv_data));

        // 创建属性表
        esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
        ESP_LOGD(TAG_BLE, "注册spp属性表结束\n");
        esp_ble_gatts_create_attr_tab(ops_gatt_db, gatts_if, OPS_IDX_NB, OPS_SVC_INST_ID);
        ESP_LOGD(TAG_BLE, "注册ops属性表结束\n");
        break;
    case ESP_GATTS_READ_EVT:
    {
        // read事件
        res = find_char_and_desr_index(p_data->read.handle);
        switch (res)
        {
        case URL_VAL:
        {
            // URL_read事件
            // 回复READ
            ESP_LOGD(TAG_BLE, "URL_read事件\n");
            break;
        }
        case URL_CFG:
            // URL_cfg
            ESP_LOGD(TAG_BLE, "URL_cfg(read)\n");
            break;
        case SSID_VAL:
        {
            // SSID_read事件
            ESP_LOGD(TAG_BLE, "SSID_read事件\n");

            break;
        }
        case SSID_CFG:
            // SSID_cfg
            ESP_LOGD(TAG_BLE, "SSID_cfg(read)\n");
            break;
        case Wifi_VAL:
        {
            // Wifi_read事件
            ESP_LOGD(TAG_BLE, "Wifi_read事件\n");

            break;
        }
        case Wifi_CFG:
            // Wifi_cfg
            ESP_LOGD(TAG_BLE, "Wifi_cfg(read)\n");
            break;
        case AOTA_VAL:
        {
            // AOTA_read事件
            ESP_LOGD(TAG_BLE, "AOTA_read事件\n");

            break;
        }
        case AOTA_CFG:
            // AOTA_cfg
            ESP_LOGD(TAG_BLE, "AOTA_cfg(read)\n");
            break;
        case LIT_VAL:
        {
            // LIT_read事件，回复brightness
            ESP_LOGD(TAG_BLE, "LIT_read事件\n");

            break;
        }
        case LIT_CFG:
            // LIT_cfg
            ESP_LOGD(TAG_BLE, "LIT_cfg(read)\n");
            break;
        case ARM_LIT_VAL:
        {
            // ARM_LIT_read事件
            ESP_LOGD(TAG_BLE, "ARM_LIT_read事件\n");

            break;
        }
        case ARM_LIT_CFG:
            // ARM_LIT_cfg
            ESP_LOGD(TAG_BLE, "ARM_LIT_cfg(read)\n");
            break;
        case R_LIT_VAL:
        {
            // R_LIT_read事件
            ESP_LOGD(TAG_BLE, "R_LIT_read事件\n");

            break;
        }
        case R_LIT_CFG:
            // R_LIT_cfg
            ESP_LOGD(TAG_BLE, "R_LIT_cfg(read)\n");
            break;
        case MATRIX_LIT_VAL:
        {
            // MATRIX_LIT_read事件
            ESP_LOGD(TAG_BLE, "MATRIX_LIT_read事件\n");

            break;
        }
        case MATRIX_LIT_CFG:
            // MATRIX_LIT_cfg
            ESP_LOGD(TAG_BLE, "MATRIX_LIT_cfg(read)\n");
            break;
        case PID_VAL:
        {
            // PID_read事件，发送顺序：P, I, D, I_MAX, D_MAX, OUT_MAX
            ESP_LOGD(TAG_BLE, "PID_read事件\n");

            break;
        }
        case PID_CFG:
            // PID_cfg
            ESP_LOGD(TAG_BLE, "PID_cfg(read)\n");
            break;
        case ARMOUR_ID_CFG:
            // ARMOUR_ID_cfg
            ESP_LOGD(TAG_BLE, "ARMOUR_ID_cfg(read)\n");
            break;
        case (uint8_t)GPA_VAL + (uint8_t)SPP_IDX_NB:
        {
            // GPA_read事件
            ESP_LOGD(TAG_BLE, "GPA_read事件\n");

            ESP_LOGD(TAG_BLE, "GPA_read事件结束\n");
            break;
        }
        default:
            ESP_LOGD(TAG_BLE, "未知read事件\n");
        }
        break;
    }
    case ESP_GATTS_WRITE_EVT:
    {
        // write事件
        res = find_char_and_desr_index(p_data->write.handle);
        ESP_LOGD(TAG_BLE, "write事件  pdata handle: %d\n", res);
        if (p_data->write.is_prep == false)
        {
            switch (res)
            {
            case URL_VAL:
                // URL_write事件
                ESP_LOGD(TAG_BLE, "URL_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case URL_CFG:
                // URL_cfg
                ESP_LOGD(TAG_BLE, "URL_cfg(write)\n");
                break;
            case SSID_VAL:
                // SSID_write事件
                ESP_LOGD(TAG_BLE, "SSID_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case SSID_CFG:
                // SSID_cfg
                ESP_LOGD(TAG_BLE, "SSID_cfg(write)\n");
                break;
            case Wifi_VAL:
                // Wifi_write事件
                ESP_LOGD(TAG_BLE, "Wifi_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case Wifi_CFG:
                // Wifi_cfg
                ESP_LOGD(TAG_BLE, "Wifi_cfg(write)\n");
                break;
            case AOTA_VAL:
                // AOTA_write事件
                ESP_LOGD(TAG_BLE, "AOTA_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case AOTA_CFG:
                // AOTA_cfg
                ESP_LOGD(TAG_BLE, "AOTA_cfg(write)\n");
                break;
            case LIT_VAL:
                // LIT_write事件
                ESP_LOGD(TAG_BLE, "LIT_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case LIT_CFG:
                // LIT_cfg
                ESP_LOGD(TAG_BLE, "LIT_cfg(write)\n");
                break;
            case ARM_LIT_VAL:
                // ARM_LIT_write事件
                ESP_LOGD(TAG_BLE, "ARM_LIT_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case ARM_LIT_CFG:
                // ARM_LIT_cfg
                ESP_LOGD(TAG_BLE, "ARM_LIT_cfg(write)\n");
                break;
            case R_LIT_VAL:
                // R_LIT_write事件
                ESP_LOGD(TAG_BLE, "R_LIT_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case R_LIT_CFG:
                // R_LIT_cfg
                ESP_LOGD(TAG_BLE, "R_LIT_cfg(write)\n");
                break;
            case MATRIX_LIT_VAL:
                // MATRIX_LIT_write事件
                ESP_LOGD(TAG_BLE, "MATRIX_LIT_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case MATRIX_LIT_CFG:
                // MATRIX_LIT_cfg
                ESP_LOGD(TAG_BLE, "MATRIX_LIT_cfg(write)\n");
                break;
            case PID_VAL:
                // PID_write事件
                ESP_LOGD(TAG_BLE, "PID_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case PID_CFG:
                // PID_cfg
                ESP_LOGD(TAG_BLE, "PID_cfg(write)\n");
                break;
            case ARMOUR_ID_VAL:
                // ARMOUR_ID_write事件
                ESP_LOGD(TAG_BLE, "ARMOUR_ID_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case ARMOUR_ID_CFG:
                // ARMOUR_ID_cfg
                ESP_LOGD(TAG_BLE, "ARMOUR_ID_cfg(write)\n");
                break;
            case (uint8_t)RUN_VAL + (uint8_t)SPP_IDX_NB:
                // RUN_write事件
                ESP_LOGD(TAG_BLE, "RUN_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case (uint8_t)RUN_CFG + (uint8_t)SPP_IDX_NB:
                // RUN_cfg
                ESP_LOGD(TAG_BLE, "RUN_cfg(write)\n");
                break;
            case (uint8_t)UNLK_VAL + (uint8_t)SPP_IDX_NB:
                // UNLK_write事件
                ESP_LOGD(TAG_BLE, "UNLK_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case (uint8_t)UNLK_CFG + (uint8_t)SPP_IDX_NB:
                // UNLK_cfg
                ESP_LOGD(TAG_BLE, "UNLK_cfg(write)\n");
                break;
            case (uint8_t)STOP_VAL + (uint8_t)SPP_IDX_NB:
                // STOP_write事件
                ESP_LOGD(TAG_BLE, "STOP_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case (uint8_t)STOP_CFG + (uint8_t)SPP_IDX_NB:
                // STOP_cfg
                ESP_LOGD(TAG_BLE, "STOP_cfg(write)\n");
                break;
            case (uint8_t)OTA_VAL + (uint8_t)SPP_IDX_NB:
                // OTA_write事件
                ESP_LOGD(TAG_BLE, "OTA_write事件\n");
                esp_ble_gatts_set_attr_value(p_data->write.handle, p_data->write.len, p_data->write.value);
                break;
            case (uint8_t)OTA_CFG + (uint8_t)SPP_IDX_NB:
                // OTA_cfg
                ESP_LOGD(TAG_BLE, "OTA_cfg(write)\n");
                break;
            default:
                ESP_LOGD(TAG_BLE, "未知write事件\n");
            }
        }
        else if ((p_data->write.is_prep == true))
        {
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_PREP_WRITE_EVT : handle = %d\n", res);
            store_wr_buffer(p_data);
        }
        break;
    }
    case ESP_GATTS_MTU_EVT:
        spp_mtu_size = p_data->mtu.mtu;
        break;
    case ESP_GATTS_CONF_EVT:
        // 经过esp_ble_gatts_send_indicate会到此处
        ESP_LOGD(TAG_BLE, "经过send_indicate后\n");
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
        spp_conn_id = p_data->connect.conn_id;
        spp_gatts_if = gatts_if;
        is_connected = true;
        memcpy(&spp_remote_bda, &p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        is_connected = false;
        enable_data_ntf = false;
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GATTS_OPEN_EVT:
        break;
    case ESP_GATTS_CANCEL_OPEN_EVT:
        break;
    case ESP_GATTS_CLOSE_EVT:
        break;
    case ESP_GATTS_LISTEN_EVT:
        break;
    case ESP_GATTS_CONGEST_EVT:
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        if (param->add_attr_tab.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        }
        // 创建属性表后启动服务
        else if (param->add_attr_tab.svc_inst_id == 0)
        {
            memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
            esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);

            ESP_LOGD(TAG_BLE, "spp_handle_table[0] = %d\n", spp_handle_table[0]);
        }
        else if (param->add_attr_tab.svc_inst_id == 1)
        {
            memcpy(ops_handle_table, param->add_attr_tab.handles, sizeof(ops_handle_table));
            esp_ble_gatts_start_service(ops_handle_table[OPS_IDX_SVC]);

            ESP_LOGD(TAG_BLE, "ops_handle_table[0] = %d\n", ops_handle_table[0]);
        }
        break;
    }
    case ESP_GATTS_SET_ATTR_VAL_EVT:
    {
        // 当设置属性表完成时，到这里
        res = find_char_and_desr_index(param->set_attr_val.attr_handle);
        switch (res)
        {
        case URL_VAL:
        {
            // Update URL 设置
            // 获取特征值
            uint16_t len;
            const uint8_t *value;
            esp_ble_gatts_get_attr_value(spp_handle_table[URL_VAL], &len, &value);
            strcpy(config->get_config_common_info_pt()->URL, (char *)value);
            // 启动config_task
            xTaskCreate(config_task, "config_task", 4096, NULL, 5, NULL);
            break;
        }
        case SSID_VAL:
        {
            // SSID
            uint16_t len;
            const uint8_t *value;
            esp_ble_gatts_get_attr_value(spp_handle_table[SSID_VAL], &len, &value);
            strcpy(config->get_config_common_info_pt()->SSID, (char *)value);
            // 启动config_task
            xTaskCreate(config_task, "config_task", 4096, NULL, 5, NULL);
            break;
        }
        case Wifi_VAL:
        {
            // Wifi
            uint16_t len;
            const uint8_t *value;
            esp_ble_gatts_get_attr_value(spp_handle_table[Wifi_VAL], &len, &value);
            strcpy(config->get_config_common_info_pt()->SSID_pwd, (char *)value);
            // 启动config_task
            xTaskCreate(config_task, "config_task", 4096, NULL, 5, NULL);
            break;
        }
        case AOTA_VAL:
        {
            // AOTA
            uint16_t len;
            const uint8_t *value;
            esp_ble_gatts_get_attr_value(spp_handle_table[AOTA_VAL], &len, &value);
            config->get_config_common_info_pt()->auto_update = *value ? 1 : 0;
            config->save();
            // 启动config_task
            xTaskCreate(config_task, "config_task", 4096, NULL, 5, NULL);
            break;
        }
        case LIT_VAL:
        {
            // LIT
            uint16_t len;
            const uint8_t *value;
            esp_ble_gatts_get_attr_value(spp_handle_table[LIT_VAL], &len, &value);
            config->get_config_armour_info_pt(ARMOUR1)->brightness = *value;
            // 启动config_task
            static uint8_t config_armour_only = ARMOUR1;
            xTaskCreate(config_task, "config_task", 4096, &config_armour_only, 5, NULL);
            break;
        }
        case ARM_LIT_VAL:
        {
            // ARM_LIT
            uint16_t len;
            const uint8_t *value;
            esp_ble_gatts_get_attr_value(spp_handle_table[ARM_LIT_VAL], &len, &value);
            config->get_config_armour_info_pt(ARMOUR1)->brightness_proportion_edge = *value;
            // 启动config_task
            static uint8_t config_armour_only = ARMOUR1;
            xTaskCreate(config_task, "config_task", 4096, &config_armour_only, 5, NULL);
            break;
        }
        case R_LIT_VAL:
        {
            // R_LIT
            uint16_t len;
            const uint8_t *value;
            esp_ble_gatts_get_attr_value(spp_handle_table[R_LIT_VAL], &len, &value);
            config->get_config_info_pt()->brightness = *value;
            config->save();
            char log_string[] = "Configuration sent to RLogo device";
            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[URL_VAL], strlen(log_string) + 1, (uint8_t *)log_string, false);
            break;
        }
        case MATRIX_LIT_VAL:
            // MATRIX_LIT
            uint16_t len;
            const uint8_t *value;
            esp_ble_gatts_get_attr_value(spp_handle_table[MATRIX_LIT_VAL], &len, &value);
            config->get_config_armour_info_pt(ARMOUR1)->brightness_proportion_matrix = *value;
            // 启动config_task
            xTaskCreate(config_task, "config_task", 4096, NULL, 5, NULL);
            break;
        case PID_VAL:
        {
            static uint8_t config_motor_only = MOTOR;
            uint16_t len;
            const uint8_t *value;
            esp_ble_gatts_get_attr_value(spp_handle_table[PID_VAL], &len, &value);
            // PID
            ESP_LOGI(TAG_BLE, "PID_val: %f, %f, %f, %f, %f, %f", *(float *)(value + 0), *(float *)(value + 1 * sizeof(float)), *(float *)(value + 2 * sizeof(float)), *(float *)(value + 3 * sizeof(float)), *(float *)(value + 4 * sizeof(float)), *(float *)(value + 5 * sizeof(float)));
            memcpy(&config->get_config_motor_info_pt()->kp, value + 0, sizeof(float));
            memcpy(&config->get_config_motor_info_pt()->ki, value + 1 * sizeof(float), sizeof(float));
            memcpy(&config->get_config_motor_info_pt()->kd, value + 2 * sizeof(float), sizeof(float));
            memcpy(&config->get_config_motor_info_pt()->i_max, value + 3 * sizeof(float), sizeof(float));
            memcpy(&config->get_config_motor_info_pt()->d_max, value + 4 * sizeof(float), sizeof(float));
            memcpy(&config->get_config_motor_info_pt()->out_max, value + 5 * sizeof(float), sizeof(float));

            // 启动config_task
            xTaskCreate(config_task, "config_task", 4096, &config_motor_only, 5, NULL);
            break;
        }
        case ARMOUR_ID_VAL:
            // ARMOUR_ID
            xTaskCreate(reset_armour_id_task, "reset_armour_id_task", 4096, NULL, 5, NULL);
            break;
        case (uint8_t)RUN_VAL + (uint8_t)SPP_IDX_NB:
            // RUN
            xTaskCreate((TaskFunction_t)run_task, "run_task", 4096, NULL, 10, NULL);
            break;
        case (uint8_t)UNLK_VAL + (uint8_t)SPP_IDX_NB:
            // UNLK
            xTaskCreate((TaskFunction_t)unlock_task, "unlock_task", 4096, NULL, 10, NULL);
            break;
        case (uint8_t)STOP_VAL + (uint8_t)SPP_IDX_NB:
            xTaskCreate((TaskFunction_t)stop_task, "stop_task", 4096, NULL, 10, NULL);
            break;
        case (uint8_t)OTA_VAL + (uint8_t)SPP_IDX_NB:
            // OTA
            xTaskCreate((TaskFunction_t)ota_task, "ota_task", 8192, NULL, 10, NULL);
            break;
        }
        break;
    }
    default:
        break;
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
        .task_priority = 2,
        .task_stack_size = 4096,
    };
    ESP_ERROR_CHECK(esp_event_loop_create(&loop_args, &pr_events_loop_handle));

    // Firmware init
    Firmware firmware;

    // ESP-NOW init
    espnow_protocol = new ESPNowProtocol();

    // 注册大符通讯协议事件
    // 发送事件
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_BEGIN_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, CONFIG_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_START_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_STOP_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRA, PRA_COMPLETE_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_START_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_UNLOCK_EVENT, ESPNowProtocol::tx_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRM, PRM_STOP_EVENT, ESPNowProtocol::tx_event_handler, NULL));

    ESP_ERROR_CHECK(esp_event_handler_register_with(pr_events_loop_handle, PRC, OTA_BEGIN_EVENT, Firmware::global_pr_event_handler, NULL));

    // BLE 特征值设置
    memcpy(url_val, config->get_config_common_info_pt()->URL, 100);
    memcpy(ssid_val, config->get_config_common_info_pt()->SSID, 20);
    memcpy(wifi_val, config->get_config_common_info_pt()->SSID_pwd, 20);
    aota_val[0] = config->get_config_common_info_pt()->auto_update;
    lit_val[0] = config->get_config_armour_info_pt(ARMOUR1)->brightness;
    arm_lit_val[0] = config->get_config_armour_info_pt(ARMOUR1)->brightness_proportion_edge;
    r_lit_val[0] = config->get_config_info_pt()->brightness;
    matrix_lit_val[0] = config->get_config_armour_info_pt(ARMOUR1)->brightness_proportion_matrix;
    memcpy(pid_val, &(config->get_config_motor_info_pt()->kp), sizeof(float));
    memcpy(pid_val + 1 * sizeof(float), &(config->get_config_motor_info_pt()->ki), sizeof(float));
    memcpy(pid_val + 2 * sizeof(float), &(config->get_config_motor_info_pt()->kd), sizeof(float));
    memcpy(pid_val + 3 * sizeof(float), &(config->get_config_motor_info_pt()->i_max), sizeof(float));
    memcpy(pid_val + 4 * sizeof(float), &(config->get_config_motor_info_pt()->d_max), sizeof(float));
    memcpy(pid_val + 5 * sizeof(float), &(config->get_config_motor_info_pt()->out_max), sizeof(float));

    // BLE Start
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    // 获取ble默认配置
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_config_t ble_cfg = {
        .ssp_en = false,
    };
    esp_bluedroid_init_with_cfg(&ble_cfg);
    esp_bluedroid_enable();

    // GATT的回调注册
    esp_ble_gatts_register_callback(gatts_event_handler);
    // GAP事件的函数
    esp_ble_gap_register_callback(gap_event_handler);
    // 注册APP
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
        ESP_LOGE(TAG_MAIN, "set local  MTU failed, error code = %x", local_mtu_ret);

    ESP_LOGI(TAG_MAIN, "BLE Started.");

    // 发送STOP到各设备以复位
    // PRM_STOP_EVENT_DATA stop_event_data;
    // esp_event_post_to(pr_events_loop_handle, PRM, PRM_STOP_EVENT, &stop_event_data, sizeof(PRM_STOP_EVENT_DATA), portMAX_DELAY);
    // xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);

    // 发送STOP到各装甲板设备
    PRA_STOP_EVENT_DATA pra_stop_event_data;
    for (uint8_t i = 0; i < 1; i++) // TODO: 改成5
    {
        pra_stop_event_data.address = i;
        esp_event_post_to(pr_events_loop_handle, PRA, PRA_STOP_EVENT, &pra_stop_event_data, sizeof(PRA_STOP_EVENT_DATA), portMAX_DELAY);
        xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    }
    // 电机停止
    // PRM_STOP_EVENT_DATA prm_stop_event_data;
    // esp_event_post_to(pr_events_loop_handle, PRM, PRM_STOP_EVENT, &prm_stop_event_data, sizeof(PRM_STOP_EVENT_DATA), portMAX_DELAY);
    // xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);

    // 启动LED动画，表示大符初始化完成
    xTaskCreate(led_animation_task, "led_animation_task", 2048, NULL, 5, &led_animation_task_handle);

    // 解锁电机
    // PRM_UNLOCK_EVENT_DATA unlock_event_data;
    // esp_event_post_to(pr_events_loop_handle, PRM, PRM_UNLOCK_EVENT, &unlock_event_data, sizeof(PRM_UNLOCK_EVENT_DATA), portMAX_DELAY);
    // xEventGroupWaitBits(ESPNowProtocol::send_state, ESPNowProtocol::SEND_ACK_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);

    vTaskSuspend(NULL);
}