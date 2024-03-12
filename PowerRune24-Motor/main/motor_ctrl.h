/**
 * @file motor_ctrl.h
 * @brief 电机控制
 * @version 1.0
 * @date 2024-01-25
 */

#pragma once
#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/twai.h>
#include <esp_err.h>
#include <esp_log.h>
#include <PID.h>
#include <PowerRune_Events.h>

extern esp_event_loop_handle_t pr_events_loop_handle;

// speed_trace_sin_info
typedef struct
{
    TickType_t start_tick; // 开始时间
    float amplitude;       // 振幅
    float omega;           // 角频率
    float offset;          // 偏移量0
} speed_trace_sin_info_t;

// motor_status
typedef enum
{
    // 需要发消息给主控拒绝其他操作
    MOTOR_DISCONNECTED,

    // 需要调用unlock_motor命令使电机运转
    MOTOR_DISABLED_LOCKED,

    // 待机，可以进入MOTOR_NORMAL状态, 可以进入MOTOR_TRACE_SIN_PENDING状态
    MOTOR_DISABLED,

    // 运转中
    MOTOR_NORMAL,

    // 速度正在改变中
    MOTOR_NORMAL_PENDING,

    // state: 电机启动需要时间, 到达正弦转速后启动LED,
    // 此时进入MOTOR_TRACE_SIN_PENDING状态
    MOTOR_TRACE_SIN_PENDING,

    // state: 电机正弦转速跟随稳定，可以启动LED
    MOTOR_TRACE_SIN_STABLE,

} motor_status_t;

// motor_info
typedef struct
{
    int16_t set_speed;
    int16_t speed;
    int16_t current;
    int16_t torque;
    int16_t temp;
    uint8_t motor_id;
    speed_trace_sin_info_t speed_trace_sin_info;
    motor_status_t motor_status;
    uint32_t last_received;
} motor_info_t;

class Motor : public PID
{
private:
    gpio_num_t TX_TWAI_GPIO;
    gpio_num_t RX_TWAI_GPIO;

    // current[0:3], 存储电流数据
    struct current_info_t
    {
        int16_t iq1;
        int16_t iq2;
        int16_t iq3;
        int16_t iq4;
    };

    /**
     * @brief 根据对应电机ID将输入的current存入到current_info对应的结构体成员中
     */
    static esp_err_t set_current(uint8_t motor_id, int16_t current, current_info_t &current_info);

    /**
     * @brief 使用current_info结构体向tx_msg存入电流数据, 此前需要使用set_current()初始化current_info
     */
    static void send_motor_current(current_info_t &current_info);

    // 当速度当前值与速度设定值之间的误差在一定的FreeRTOS系统tick的时间段内小于一定范围时
    // 电机状态从MOTOR_NORMAL_PENDING转换为MOTOR_NORMAL, 或者电机状态从MOTOR_TRACE_SIN_PENDING转换为MOTOR_TRACE_SIN_STABLE
    static void speed_stable_check();

public:
    /**
     * @brief 初始化电机, 需要在主函数手动设置motor_counts, TX和RX的IO口, PID初始化是否正确? 数据输入包含PID初始化
     */
    Motor(uint8_t *motor_id, uint8_t motor_counts = 1, gpio_num_t TX_TWAI_GPIO = GPIO_NUM_4, gpio_num_t RX_TWAI_GPIO = GPIO_NUM_5, float Kp = 4, float Ki = 0.2, float Kd = 0.6, float Pmax = 2000, float Imax = 2000, float Dmax = 2000, float max = 6000);

    /**
     * @brief 设置电机ID, 使电机ID与motor_info内顺序一致
     * @param index, ID(0-1)
     */
    esp_err_t set_id(uint8_t index, uint8_t id);

    /**
     * @brief 电机状态从DISABLED_LOCKED转换为DISABLED
     */
    esp_err_t unlock_motor(uint8_t motor_id);

    /**
     * @brief 解除电机控制
     */
    void disable_motor(uint8_t motor_id);

    /**
     * @brief 设置电机速度, 如果电机状态为DISABLED_LOCKED, 则返回ESP_ERR_NOT_SUPPORTED
     */
    esp_err_t set_speed(uint8_t motor_id, int16_t speed);
    /**
     * @brief 设置电机速度跟踪正弦曲线, 如果电机状态为DISABLED_LOCKED, 则返回ESP_ERR_NOT_SUPPORTED
     */
    esp_err_t set_speed(uint8_t motor_id, float amplitude, float omega, float offset);

    /**
     * @brief get motor status
     */
    motor_status_t get_motor_status(uint8_t motor_id);

protected:
    static motor_info_t *motor_info;
    static size_t motor_counts;

    /**
     * @brief 使用CAN获取电机信息: 速度, 电流, 温度; 并更新状态
     */
    static void state_check();

    /**
     * @brief FreeRTOS Task: 控制电机运动，同时接受电机状态
     */
    static void task_motor(void *args);
};

#endif