#include <stdio.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/twai.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_event_base.h>
#include "PowerRune_Events.h"
#include "motor_ctrl.h"

// ESP_EVENT_DECLARE_BASE(PRM);
// enum
// {
//     PRM_UNLOCK_EVENT,
//     PRM_UNLOCK_DONE_EVENT,
//     PRM_START_EVENT,
//     PRM_START_DONE_EVENT,
//     PRM_SPEED_STABLE_EVENT,
//     PRM_STOP_EVENT,
//     PRM_DISCONNECT_EVENT,
// };

// typedef enum
// {
//     MOTOR_DISCONNECTED,
//     MOTOR_DISABLED_LOCKED,
//     MOTOR_DISABLED,
//     MOTOR_NORMAL,
//     MOTOR_NORMAL_PENDING,
//     MOTOR_TRACE_SIN_PENDING,
//     MOTOR_TRACE_SIN_STABLE,
// } motor_status_t;

// event loop TAG
static const char *TAG = "PRM";

// create event loop with task PRM
esp_event_loop_handle_t loop_PRM;

// check event loop: loop_with_PRM
static void check_loop(void *handler_args, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "Set done.");
};

// define event base
// ESP_EVENT_DEFINE_BASE(PRM);

// define event base handler function
static void PRM_event_handler(void *handler_args, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    // set handler_args to motor_3508
    Motor *motor_3508 = (Motor *)handler_args;
    esp_err_t err = ESP_OK;

    switch (event_id)
    {
    case PRM_UNLOCK_EVENT:
        ESP_LOGI(TAG, "PRM_UNLOCK_EVENT");

        err = motor_3508->unlock_motor(CONFIG_DEFAULT_MOTOR_ID);
        // post PRM_UNLOCK_DONE_EVENT
        ESP_ERROR_CHECK(esp_event_post_to(loop_PRM, PRM, PRM_UNLOCK_DONE_EVENT, &err, sizeof(esp_err_t), portMAX_DELAY));
        break;
    case PRM_START_EVENT:
    {
        ESP_LOGI(TAG, "PRM_START_EVENT");
        struct PRM_START_DONE_EVENT_DATA *start_data = (struct PRM_START_DONE_EVENT_DATA *)event_data;
        if (start_data->mode == 0)
        {
            // set motor status to MOTOR_NORMAL_PENDING
            motor_3508->set_motor_status(CONFIG_DEFAULT_MOTOR_ID, MOTOR_NORMAL_PENDING);
            // set speed to 60 in rpm
            motor_3508->set_speed(CONFIG_DEFAULT_MOTOR_ID, 150);
        }
        else if (start_data->mode == 1)
        {
            // set motor status to MOTOR_TRACE_SIN_PENDING
            motor_3508->set_motor_status(CONFIG_DEFAULT_MOTOR_ID, start_data->amplitude, start_data->omega, start_data->offset);
        }
        else
        {
            err = ESP_ERR_INVALID_ARG;
        };
        // post PRM_START_DONE_EVENT
        ESP_ERROR_CHECK(esp_event_post_to(loop_PRM, PRM, PRM_START_DONE_EVENT, NULL, 0, portMAX_DELAY));
        break;
    }
    case PRM_STOP_EVENT:
        ESP_LOGI(TAG, "PRM_STOP_EVENT");
        motor_3508->set_motor_status(CONFIG_DEFAULT_MOTOR_ID, MOTOR_DISABLED);
        break;
    default:
        break;
    };
};

extern "C" void app_main(void)
{
    // TODO 使用event_data传入数据?
    // 电机数量
    uint8_t motor_counts = 1;
    // 电机控制器初始化 和 id 数组
    // 一个电机，ID为1
    uint8_t id[motor_counts] = {1};
    Motor motor_3508(id, motor_counts);

    ESP_LOGI(TAG, "setting up");

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // set event loop args
    esp_event_loop_args_t loop_PRM_args = {
        .queue_size = 4,
        .task_name = "PRM",
        .task_priority = uxTaskPriorityGet(NULL),
        .task_stack_size = 8192,
        .task_core_id = tskNO_AFFINITY};

    // create event loop with task PRM
    ESP_ERROR_CHECK(esp_event_loop_create(&loop_PRM_args, &loop_PRM));

    // register event PRM handler, transfer motor_3508 to handler_args
    ESP_ERROR_CHECK(esp_event_handler_register_with(loop_PRM, PRM, ESP_EVENT_ANY_ID, PRM_event_handler, &motor_3508));
    // register event loop check handler
    ESP_ERROR_CHECK(esp_event_handler_register_with(loop_PRM, PRM, ESP_EVENT_ANY_ID, check_loop, NULL));

    // post check_loop event
    ESP_LOGI(TAG, "posting check_loop event");
    ESP_ERROR_CHECK(esp_event_post_to(loop_PRM, TAG, 1, NULL, 0, portMAX_DELAY));

    // Unit Test, Post PRM_UNLOCK_EVENT
    ESP_LOGI(TAG, "posting PRM_UNLOCK_EVENT");
    ESP_ERROR_CHECK(esp_event_post_to(loop_PRM, PRM, PRM_UNLOCK_EVENT, NULL, 0, portMAX_DELAY));
    // Unit Test, Post PRM_START_EVENT
    ESP_LOGI(TAG, "posting PRM_START_EVENT");
    struct PRM_START_DONE_EVENT_DATA start_data = {
        .mode = 0,
    };
    ESP_ERROR_CHECK(esp_event_post_to(loop_PRM, PRM, PRM_START_EVENT, &start_data, sizeof(struct PRM_START_DONE_EVENT_DATA), portMAX_DELAY));
    // Wait for 10S
    vTaskDelay(10000 / portTICK_PERIOD_MS);

    // Unit Test, Post PRM_STOP_EVENT
    ESP_LOGI(TAG, "posting PRM_STOP_EVENT");
    ESP_ERROR_CHECK(esp_event_post_to(loop_PRM, PRM, PRM_STOP_EVENT, NULL, 0, portMAX_DELAY));
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Unit Test, Post PRM_START_EVENT
    ESP_LOGI(TAG, "posting PRM_START_EVENT");
    start_data.mode = 1;
    start_data.amplitude = 1.045;
    start_data.omega = 1.884;
    start_data.offset = 2.090 - start_data.amplitude;
    ESP_ERROR_CHECK(esp_event_post_to(loop_PRM, PRM, PRM_START_EVENT, &start_data, sizeof(struct PRM_START_DONE_EVENT_DATA), portMAX_DELAY));

    // Wait for 10S
    vTaskDelay(10000 / portTICK_PERIOD_MS);

    // Unit Test, Post PRM_STOP_EVENT
    ESP_LOGI(TAG, "posting PRM_STOP_EVENT");
    ESP_ERROR_CHECK(esp_event_post_to(loop_PRM, PRM, PRM_STOP_EVENT, NULL, 0, portMAX_DELAY));

    while (1)
    {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }

    // ESP_ERROR_CHECK(esp_event_post_to(loop_with_task, TAG, 1, NULL, 0, portMAX_DELAY));
}
