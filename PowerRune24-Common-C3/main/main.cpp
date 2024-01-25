#include "motor_ctrl.h"

extern "C" void app_main(void)
{
    uint8_t motor_counts = 1; // 电机数量
    // 电机控制器初始化
    // id 数组
    uint8_t id[motor_counts] = {1}; // 一个电机，ID为1

    Motor motor_3508(id, motor_counts);
    motor_3508.unlock_motor(1);
    motor_3508.set_speed(1, 1000);
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}