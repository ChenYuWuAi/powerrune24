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
static void PRM_event_handler(void* handler_args, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    // set handler_args to motor_3508
    Motor* motor_3508 = (Motor*)handler_args;

    if (event_base == PRM)
    {
        switch (event_id)
        {
        case PRM_UNLOCK_EVENT:
            ESP_LOGI(TAG, "PRM_UNLOCK_EVENT");
            // TODO if motor status is MOTOR_DISABLED_LOCKED
            // TODO
            // set motor status to MOTOR_DISABLED
            motor_3508->unlock_motor(1);
            // TODO motor_3508->set_motor_status(1, MOTOR_DISABLED);
            // post PRM_UNLOCK_DONE_EVENT
            ESP_ERROR_CHECK(esp_event_post_to(loop_with_PRM, PRM, PRM_UNLOCK_DONE_EVENT, NULL, 0, portMAX_DELAY));
            break;
        case PRM_START_EVENT:
            ESP_LOGI(TAG, "PRM_START_EVENT");
            // set motor status to MOTOR_NORMAL_PENDING & MOTOR_TRACE_SIN_PENDING
            motor_3508->set_motor_status(1, MOTOR_NORMAL_PENDING);
            motor_3508->set_motor_status(1, MOTOR_TRACE_SIN_PENDING);
            // post PRM_START_DONE_EVENT
            ESP_ERROR_CHECK(esp_event_post_to(loop_with_PRM, PRM, PRM_START_DONE_EVENT, NULL, 0, portMAX_DELAY));
            break;
        case PRM_STOP_EVENT:
            ESP_LOGI(TAG, "PRM_STOP_EVENT");
            break;
        case PRM_DISCONNECT_EVENT:
            // if motor status is MOTOR_DISABLED_LOCKED, post PRM_DISCONNECT_EVENT
            if(motor_3508->get_motor_status(1) == MOTOR_DISABLED_LOCKED)
            {
                ESP_ERROR_CHECK(esp_event_post_to(loop_with_PRM, PRM, PRM_DISCONNECT_EVENT, NULL, 0, portMAX_DELAY));
            };
            break;
        default:
            // if motor status is MOTOR_DISCONNECTED or MOTOR_NORMAL_PENDING or MOTOR_DISABLED or MOTOR_TRACE_SIN_STABLE, post PRM_DISCONNECT_EVENT, set motor status to MOTOR_DISABLED_LOCKED
            if(((motor_3508->get_motor_status(1) == MOTOR_DISCONNECTED) || (motor_3508->get_motor_status(1) == MOTOR_NORMAL_PENDING) || (motor_3508->get_motor_status(1) == MOTOR_DISABLED) || (motor_3508->get_motor_status(1) == MOTOR_TRACE_SIN_STABLE)))
            {
                ESP_ERROR_CHECK(esp_event_post_to(loop_with_PRM, PRM, PRM_DISCONNECT_EVENT, NULL, 0, portMAX_DELAY));
                motor_3508->set_motor_status(1, MOTOR_DISABLED_LOCKED);
            };
            break;
        };
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
    motor_3508.unlock_motor(1);
    motor_3508.set_speed(1, 2000);

    ESP_LOGI(TAG, "setting up");

    // set event loop args
    esp_event_loop_args_t loop_with_PRM_args = {
        .queue_size = 7,
        .task_name = "PRM",
        .task_priority = uxTaskPriorityGet(NULL),
        .task_stack_size = 3072,
        .task_core_id = tskNO_AFFINITY
    };
    
    // create event loop with task PRM
    ESP_ERROR_CHECK(esp_event_loop_create(&loop_with_PRM_args, &loop_with_PRM));

    // register event PRM handler, transfer 
    ESP_ERROR_CHECK(esp_event_handler_instance_register_with(loop_with_PRM, PRM, ESP_EVENT_ANY_ID, PRM_event_handler, &motor_3508, NULL));
    
    ESP_ERROR_CHECK(esp_event_post(PRM, PRM_UNLOCK_EVENT, &motor_3508, sizeof(motor_3508), portMAX_DELAY));
    // while (1)
    // {
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }

    // ESP_ERROR_CHECK(esp_event_post_to(loop_with_task, TAG, 1, NULL, 0, portMAX_DELAY));

}
