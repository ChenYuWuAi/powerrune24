#include "motor_ctrl.h"

#define DEBUG_NO_PID
#define DEBUG_NO_PID_CURRENT 1000

extern "C" void app_main(void)
{
    Motor motor_3508;
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}