#include <stdio.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/twai.h>
#include <esp_err.h>
#include <esp_log.h>
#include "event_source.h"
#include "esp_event_base.h"
#include "motor_ctrl.h"

// static const char* TAG_EVENT = "powerrune_motor";

// event loops
// esp_event_loop_handle_t loop_with_task;

// void check_loop(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
//     ESP_LOGI(TAG, "OK.");
// }
// *esp_event_handler_t)(void* event_handler_arg,
//                                         esp_event_base_t event_base,
//                                         int32_t event_id,
//                                         void* event_data)

extern "C" void app_main(void)
{
    uint8_t motor_counts = 1; // 电机数量
    // 电机控制器初始化
    // id 数组
    uint8_t id[motor_counts] = {1}; // 一个电机，ID为1

    Motor motor_3508(id, motor_counts, GPIO_NUM_4, GPIO_NUM_5);
    motor_3508.unlock_motor(1);
    motor_3508.set_speed(1, 2000);
    // 
    
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // ESP_LOGI(TAG, "setting up");

    // esp_event_loop_args_t loop_with_task_args = {
    //     .queue_size = 5,
    //     .task_name = "PowerruneMotor", // task will be created
    //     .task_priority = uxTaskPriorityGet(NULL),
    //     .task_stack_size = 3072,
    //     .task_core_id = tskNO_AFFINITY
    // };

    // esp_event_loop_args_t loop_without_task_args = {
    //     .queue_size = 5,
    //     .task_name = NULL // no task will be created
    // };

    // // Create the event loops
    // ESP_ERROR_CHECK(esp_event_loop_create(&loop_with_task_args, &loop_with_task));
    // ESP_ERROR_CHECK(esp_event_loop_create(&loop_without_task_args, &loop_without_task));

    // // Register the handler for task iteration event. Notice that the same handler is used for handling event on different loops.
    // // The loop handle is provided as an argument in order for this example to display the loop the handler is being run on.
    // ESP_ERROR_CHECK(esp_event_handler_instance_register_with(loop_with_task, TAG, ESP_EVENT_ANY_ID, check_loop, loop_with_task, NULL));

    // ESP_ERROR_CHECK(esp_event_post_to(loop_with_task, TAG, 1, NULL, 0, portMAX_DELAY));

}
