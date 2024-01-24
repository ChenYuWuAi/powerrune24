/*
File Name:      motor_crtl.h

Function List:  void task_motor(void *args)
                void state_check()
                esp_err_t unlock_motor(uint8_t motor_id)
                void disable_motor(uint8_t motor_id)
                esp_err_t set_speed(uint8_t motor_id, int16_t speed)
                esp_err_t set_speed_trace(uint8_t motor_id, float amplitude, float omega, float offset)

*/
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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/twai.h>
#include <esp_err.h>
#include <math.h>
#include <esp_log.h>
#include "simple_pid.h"

const char *TAG_TWAI = "TWAI";

// speed_trace_sin_info
typedef struct
{
    float set_speed;
    float speed_angular;
    float offset;
} speed_trace_sin_info_t;

// motor_status
typedef enum
{
    // 需要发消息给主控拒绝其他操作
    MOTOR_DISCONNECTED,

    // state: 无控制信号输入; 可以set_speed(), set_speed_trace(),
    // 可以进入MOTOR_NORMAL状态, 可以进入MOTOR_TRACE_SIN_PENDING状态,
    // 需要手动输入unlock_motor命令使电机运转
    MOTOR_DISABLED_LOCKED,

    // state:  无控制信号输入; 可以set_speed(), set_speed_trace(),
    // 可以进入MOTOR_NORMAL状态, 可以进入MOTOR_TRACE_SIN_PENDING状态,
    // 含义: ????
    MOTOR_DISABLED,

    // 含义: ????
    // 与MOTOR_TRACE_SIN_STABLE区别
    MOTOR_NORMAL,

    // state: 电机启动需要时间, 到达转速后启动LED,
    // 此时进入MOTOR_TRACE_SIN_PENDING状态
    //??此状态只针对正弦形式的速度吗??
    MOTOR_TRACE_SIN_PENDING,

    // state: 电机转速稳定
    //??此状态只针对正弦形式的速度吗??
    MOTOR_TRACE_SIN_STABLE,

} motor_status_t;

// motor_info
typedef struct
{
    int set_speed;
    int set_current;
    int speed;
    int current;
    int torque;
    int temp;
    uint8_t motor_id;
    speed_trace_sin_info_t speed_trace_sin_info;
    motor_status_t motor_status;
    uint32_t last_received;
} motor_info_t;

class Motor
{
private:
    gpio_num_t TX_TWAI_GPIO;
    gpio_num_t RX_TWAI_GPIO;

    // PID crtl
    static PID *pid;

    struct current_info_t
    {
        int16_t iq1;
        int16_t iq2;
        int16_t iq3;
        int16_t iq4;
    }; // current[0:3]

    static esp_err_t set_current(uint8_t motor_id, int16_t current, current_info_t &current_info)
    {
        for (size_t i = 0; i < motor_counts; i++)
        {
            motor_info[motor_id].set_current = current;
        }
        return ESP_OK;
    };

    static void send_motor_current(current_info_t &current_info)
    {
        twai_message_t tx_msg = {8, 0x200, 0, 0};

        tx_msg.data[0] = current_info.iq1 >> 8;
        tx_msg.data[1] = current_info.iq1;
        tx_msg.data[2] = current_info.iq2 >> 8;
        tx_msg.data[3] = current_info.iq2;
        tx_msg.data[4] = current_info.iq3 >> 8;
        tx_msg.data[5] = current_info.iq3;
        tx_msg.data[6] = current_info.iq4 >> 8;
        tx_msg.data[7] = current_info.iq4;

        ESP_ERROR_CHECK(twai_transmit(&tx_msg, portMAX_DELAY));
    }

protected:
    static motor_info_t *motor_info;
    static size_t motor_counts;

    /**
     * @brief FreeRTOS Task: 控制电机运动，同时接受电机状态
     */
    static void task_motor(void *args)
    {
        Motor *motor_ctrl = (Motor *)args;
        current_info_t current_info;
        memset(&current_info, 0, sizeof(current_info_t));
        while (1)
        {
            state_check();
            for (size_t i = 0; i < motor_counts; i++)
            {

                switch (motor_ctrl->motor_info[i].motor_status)
                {
                case MOTOR_DISABLED_LOCKED:
                case MOTOR_DISCONNECTED:
                case MOTOR_DISABLED:
                    set_current(motor_info[i].motor_id, 0, current_info);
                    break;

                case MOTOR_NORMAL:
// TODO
// set_current(pid->PID_CALC(motor_ctrl->motor_info[i].speed));
#ifdef DEBUG_NO_PID
                    motor_info[i].set_current = DEBUG_NO_PID_CURRENT;
#endif

                    set_current(motor_info[i].motor_id, motor_info[i].set_current, current_info);
                    break;

                default:
                    break;
                }
            }
            send_motor_current(current_info);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        vTaskDelete(NULL);
    };

    // get motor state from TWAI
    static void state_check()
    {

        twai_message_t twai_msg_re;
        static uint32_t current_tick = 0;
        current_tick = xTaskGetTickCount();
        // while receive queue is not empty
        while (twai_receive(&twai_msg_re, pdMS_TO_TICKS(1)) == ESP_OK)
        {

            ESP_LOGI(TAG_TWAI, "TWAI message received .\n");

            uint8_t motor_id = twai_msg_re.identifier & 0xF;

            for (size_t i = 0; i < motor_counts; i++)
            {
                if (motor_info[i].motor_id == motor_id)
                {
                    motor_info[i].speed = (twai_msg_re.data[2] << 8) | twai_msg_re.data[3];
                    motor_info[i].current = (twai_msg_re.data[4] << 8) | twai_msg_re.data[5];
                    motor_info[i].temp = twai_msg_re.data[6];

                    // check motor status
                    if (motor_info[i].last_received - current_tick > 10)
                    {
                        motor_info[i].motor_status = MOTOR_DISCONNECTED;
                    }
                    else if (motor_info[i].motor_status == MOTOR_DISCONNECTED)
                    {
                        motor_info[i].motor_status = MOTOR_DISABLED_LOCKED;
                    }

                    // update reveive time
                    motor_info[i].last_received = xTaskGetTickCount();

                    ESP_LOGI(TAG_TWAI, "Motor %d State: Speed %d, Current %d, Temp %d .\n",
                             motor_id, motor_info[i].speed, motor_info[i].current, motor_info[i].temp);
                }
            }
        }
    }

public:
    // motor init 。
    Motor(uint8_t motor_counts = 1, gpio_num_t TX_TWAI_GPIO = GPIO_NUM_4, gpio_num_t RX_TWAI_GPIO = GPIO_NUM_5)
    {
        this->TX_TWAI_GPIO = TX_TWAI_GPIO;
        this->RX_TWAI_GPIO = RX_TWAI_GPIO;
        TaskHandle_t TWAI_handle;

        twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_500KBITS();
        twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_TWAI_GPIO, RX_TWAI_GPIO, TWAI_MODE_NORMAL);

        if (twai_driver_install(&general_config, &timing_config, &filter_config) != ESP_OK)
        {
            ESP_LOGE("TWAI", "Failed to install TWAI driver");
        }

        if (twai_start() != ESP_OK)
        {
            ESP_LOGE("TWAI", "Failed to start TWAI driver.");
        }

        ESP_LOGI("TWAI", "TWAI initialized.");

        BaseType_t task_created = xTaskCreate(this->task_motor, "task_motor", 2048, this, 5, NULL);
        if (task_created == pdPASS)
        {
            ESP_LOGI(TAG_TWAI, "TWAI task created.\n");
        }
        else
        {
            ESP_LOGI(TAG_TWAI, "Failed to create TWAI task.\n");
        }

        pid = new PID[motor_counts];
        motor_info = new motor_info_t[motor_counts];
        this->motor_counts = motor_counts;
        // init motor_info
        for (size_t i = 0; i < motor_counts; i++)
        {
            motor_info[i].motor_id = i;
            motor_info[i].set_speed = 0;
            motor_info[i].set_current = 0;
            motor_info[i].speed = 0;
            motor_info[i].current = 0;
            motor_info[i].torque = 0;
            motor_info[i].temp = 0;
            motor_info[i].motor_status = MOTOR_DISABLED_LOCKED;
            motor_info[i].last_received = xTaskGetTickCount();
        }
    }

    esp_err_t unlock_motor(uint8_t motor_id)
    {
        esp_err_t ret = ESP_ERR_INVALID_ARG;
        for (size_t i = 0; i < motor_counts; i++)
        {
            if (motor_info[i].motor_id == motor_id)
            {
                motor_info[i].motor_status = MOTOR_DISABLED;
                ESP_LOGI(TAG_TWAI, "Motor %d unlocked.\n", i);
                break;
                return ESP_OK;
            }
        }
        return ret;
    }

    /**
     * @brief 解除电机控制
     */
    void disable_motor(uint8_t motor_id)
    {
        for (size_t i = 0; i < motor_counts; i++)
        {
            if (motor_info[i].motor_id == motor_id)
            {
                motor_info[i].set_speed = 0;
                motor_info[i].set_current = 0;
                motor_info[i].speed = 0;
                motor_info[i].current = 0;
                motor_info[i].torque = 0;
                motor_info[i].temp = 0;
                motor_info[i].motor_status = MOTOR_DISABLED;
                ESP_LOGI(TAG_TWAI, "Motor %d disabled.\n", i);
                break;
            }
        }
    };

    //!!NOT SURE!! set motor speed, state:???
    esp_err_t set_speed(uint8_t motor_id, int16_t speed) // if locked return ESP_ERR_NOT_SUPPORTED
    {
        for (size_t i = 0; i < motor_counts; i++)
        {
            // locked
            if (motor_info[i].motor_id == motor_id)
            {
                if (motor_info[i].motor_status == MOTOR_DISABLED_LOCKED)
                {
                    ESP_LOGW(TAG_TWAI, "Motor %d is locked.\n", motor_id);
                    return ESP_ERR_NOT_SUPPORTED;
                }
                if (speed > 0)
                {
                    motor_info[i].motor_status = MOTOR_NORMAL;
                }
                else
                {
                    motor_info[i].motor_status = MOTOR_DISABLED;
                }
                motor_info[i].set_speed = speed;
                ESP_LOGI(TAG_TWAI, "Motor %d speed %d.\n", i, speed);
                return ESP_OK;
            }
        }
        // cannot find motor_id
        ESP_LOGI(TAG_TWAI, "Invalid ID.\n");
        return ESP_ERR_INVALID_ARG;
    };

    // set speed trace, state:???
    esp_err_t set_speed_trace(uint8_t motor_id, float amplitude, float omega, float offset)
    {
        for (size_t i = 0; i < motor_counts; i++)
        {
            if (motor_info[i].motor_id == motor_id)
            {
                motor_info[i].speed_trace_sin_info.set_speed = offset;
                motor_info[i].speed_trace_sin_info.speed_angular = 2 * M_PI * omega;
                motor_info[i].speed_trace_sin_info.offset = offset;
                motor_info[i].motor_status = MOTOR_TRACE_SIN_PENDING;
                return ESP_OK;
            }
        }
        // cannot find motor_id
        ESP_LOGI(TAG_TWAI, "Invalid ID.\n");
        return ESP_ERR_INVALID_ARG;
    };
};

#endif