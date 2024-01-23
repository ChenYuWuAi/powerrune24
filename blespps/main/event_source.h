
#include "esp_event.h"

// Declare an event base
//声明一个事件
ESP_EVENT_DECLARE_BASE(BLE_EVENTS);             // declaration of the event family
//在枚举中声明事件的IDs
enum {                                          // declaration of the specific events under the event family
    LED_EVENT_READ,            
    LED_EVENT_WRITE,                             
};

// // Declarations for event source 2: task
// #define TASK_ITERATIONS_COUNT        5       // number of times the task iterates
// #define TASK_ITERATIONS_UNREGISTER   3       // count at which the task event handler is unregistered
// #define TASK_PERIOD                  500     // period of the task loop in milliseconds

// ESP_EVENT_DECLARE_BASE(TASK_EVENTS);         // declaration of the task events family

// enum {
//     TASK_ITERATION_EVENT,                    // raised during an iteration of the loop within the task
// };