/**
 * @file PowerRune_Armour.h
 * @brief PowerRune_Armour类的头文件
 * @version 0.2
 * @date 2024-02-18
 * @note 本文件用于维护装甲板上的所有LED、按键以及它们的事件处理
 */
#pragma once
#ifndef __POWERRUNE_ARMOUR_H__
#define __POWERRUNE_ARMOUR_H__
#include "LED_Strip.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "DEMUX.h"
#include "driver/gpio.h"
#include "firmware.h"
#include "PowerRune_Events.h"
#include "espnow_protocol.h"

#define MATRIX_REFRESH_PERIOD 100
#define BLINK_DELAY 83

// GPIO定义
// 扳机IO: 1 2 4 5 6 7 10 12 8 9
const gpio_num_t TRIGGER_IO[] = {GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7, GPIO_NUM_10, GPIO_NUM_12, GPIO_NUM_8, GPIO_NUM_9};
const uint8_t TRIGGER_IO_TO_SCORE[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
const gpio_num_t DEMUX_IO[] = {GPIO_NUM_14, GPIO_NUM_21, GPIO_NUM_38};
const gpio_num_t DEMUX_IO_enable = GPIO_NUM_13;
const gpio_num_t STRIP_IO = GPIO_NUM_11;

// LED_Strip_Enum
enum LED_Strip_Enum
{
    LED_STRIP_MAIN_ARMOUR,
    LED_STRIP_UPPER,
    LED_STRIP_LOWER,
    LED_STRIP_ARM,
    LED_STRIP_MATRIX,
};
// LED_Strip_State_t
enum LED_Strip_State_t
{
    LED_STRIP_DEBUG,
    LED_STRIP_IDLE,
    LED_STRIP_TARGET,
    LED_STRIP_HIT,
    LED_STRIP_BLINK,
};

struct LED_Strip_FSM_t
{
    RUNE_COLOR color = PR_RED;
    RUNE_MODE mode = PRA_RUNE_BIG_MODE;
    LED_Strip_State_t LED_Strip_State = LED_STRIP_DEBUG;
    uint8_t score = 0; // 默认值为0
};

class PowerRune_Armour
{
private:
    // 装甲板图案数组
    // 靶状图案点灯序号，超级长，不要展开
    constexpr static const uint16_t target_pic[] = {
        30,
        0,
        10,
        20,
        40,
        41,
        42,
        43,
        44,
        45,
        46,
        47,
        48,
        49,
        50,
        51,
        52,
        53,
        54,
        55,
        56,
        57,
        58,
        59,
        60,
        61,
        62,
        63,
        64,
        65,
        66,
        67,
        68,
        69,
        70,
        71,
        72,
        73,
        74,
        75,
        76,
        77,
        78,
        79,
        80,
        81,
        82,
        92,
        102,
        112,
        122,
        131,
        140,
        149,
        158,
        167,
        176,
        185,
        194,
        195,
        196,
        197,
        198,
        199,
        200,
        201,
        202,
        203,
        204,
        205,
        206,
        207,
        208,
        209,
        210,
        211,
        212,
        213,
        214,
        219,
        224,
        229,
        246,
        247,
        248,
        249,
        250,
        251,
        252,
        253,
    };
    // 命中图案，各环截止点[环首LED]
    constexpr static const uint16_t hit_ring_cutoff[] = {0, 40, 82, 122, 158, 194, 214, 234, 246, 254, 258};
    // 流水灯数组图案
    constexpr static bool single_arrow[] = {
        0,
        0,
        1,
        0,
        0,
        0,
        1,
        1,
        1,
        0,
        1,
        1,
        0,
        1,
        1,
        1,
        0,
        0,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
    };

    // LED_Strip_FSM_t
    static LED_Strip_FSM_t state;
    // LED_Strip初始化
    static LED_Strip *led_strip[5];
    // DEMUX初始化
    static DEMUX demux_led;
    // ISR Mutex
    static SemaphoreHandle_t ISR_mutex;
    // LED更新任务
    static void LED_update_task(void *pvParameter);
    static void restart_ISR_task(void *pvParameter);
    // LED更新任务句柄
    static TaskHandle_t LED_update_task_handle;
    // 状态机更新信号量
    static SemaphoreHandle_t LED_Strip_FSM_Semaphore;
    // GPIO初始化
    static inline void GPIO_init();
    // GPIO中断启动
    static inline void GPIO_ISR_enable();
    // GPIO ISR处理
    static void IRAM_ATTR GPIO_ISR_handler(void *arg);
    // 装甲板启动，含红蓝方和大小符
    static inline void trigger(RUNE_MODE mode, RUNE_COLOR color);
    // 装甲板命中
    static inline void hit(uint8_t score);
    // 装甲板清除
    static inline void clear_armour(bool refresh = true);
    // 装甲板激活完毕
    static inline void blink();
    // 装甲板DEBUG
    static inline void debug();
    // 装甲板停止
    static inline void stop();
    // GPIO轮询&滤波服务
    static void GPIO_polling_service(void *pvParameter);

public:
    PowerRune_Armour();
    static void global_pr_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
};

#endif