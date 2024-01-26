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
static const char* TAG = "PRM";

// create event loop with task PRM
esp_event_loop_handle_t loop_with_PRM;

// check event loop: loop_with_PRM
static void check_loop(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
    esp_event_loop_handle_t loop = (esp_event_loop_handle_t)handler_args;
    ESP_LOGI(TAG, "Event in loop: %p ", loop);
};

// define event base
// ESP_EVENT_DEFINE_BASE(PRM);

// define event base handler function
static void PRM_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
    if (base == PRM)
    {
        switch (id)
        {
        case PRM_UNLOCK_EVENT:
            ESP_LOGI(TAG, "PRM_UNLOCK_EVENT");
            break;
        case PRM_UNLOCK_DONE_EVENT:
            ESP_LOGI(TAG, "PRM_UNLOCK_DONE_EVENT");
            break;
        case PRM_SPEED_STABLE_EVENT:
            ESP_LOGI(TAG, "PRM_SPEED_STABLE_EVENT");
            break;
        case PRM_START_EVENT:
            ESP_LOGI(TAG, "PRM_START_EVENT");
            break;
        case PRM_START_DONE_EVENT:
            ESP_LOGI(TAG, "PRM_START_DONE_EVENT");
            break;
        case PRM_STOP_EVENT:
            ESP_LOGI(TAG, "PRM_STOP_EVENT");
            break;
        case PRM_DISCONNECT_EVENT:
            ESP_LOGI(TAG, "PRM_DISCONNECT_EVENT");
            break;
        default:
            break;
        }
    }
};


extern "C" void app_main(void)
{
    uint8_t motor_counts = 1; // 电机数量
    // 电机控制器初始化
    // id 数组
    uint8_t id[motor_counts] = {1}; // 一个电机，ID为1

    Motor motor_3508(id, motor_counts, GPIO_NUM_4, GPIO_NUM_5);
    motor_3508.unlock_motor(1);
    motor_3508.set_speed(1, 2000);

    ESP_LOGI(TAG, "setting up");

    // set event loop args
    esp_event_loop_args_t loop_with_PRM_args = {
        .queue_size = 5,
        .task_name = "PowerruneMotor",
        .task_priority = uxTaskPriorityGet(NULL),
        .task_stack_size = 3072,
        .task_core_id = tskNO_AFFINITY
    };
    
    // while (1)
    // {
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }

    // // Create the event loops
    // ESP_ERROR_CHECK(esp_event_loop_create(&loop_with_task_args, &loop_with_task));
    // ESP_ERROR_CHECK(esp_event_loop_create(&loop_without_task_args, &loop_without_task));

    // // Register the handler for task iteration event. Notice that the same handler is used for handling event on different loops.
    // // The loop handle is provided as an argument in order for this example to display the loop the handler is being run on.
    // ESP_ERROR_CHECK(esp_event_handler_instance_register_with(loop_with_task, TAG, ESP_EVENT_ANY_ID, check_loop, loop_with_task, NULL));

    // ESP_ERROR_CHECK(esp_event_post_to(loop_with_task, TAG, 1, NULL, 0, portMAX_DELAY));

}
