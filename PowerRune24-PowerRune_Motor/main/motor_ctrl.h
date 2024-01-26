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
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/twai.h>
#include <esp_err.h>
#include <esp_log.h>
#include <PID.h>
// #include "MiniPID.h"

// #define DEBUG_NO_PID
// #define DEBUG_NO_PID_CURRENT 250

const char *TAG_TWAI = "TWAI";
extern esp_event_loop_handle_t loop_PRM;

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

class Motor : PID
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
    static esp_err_t set_current(uint8_t motor_id, int16_t current, current_info_t &current_info)
    {
        for (size_t i = 0; i < motor_counts; i++)
        {
            if (motor_info[i].motor_id == motor_id)
            {
                switch (motor_id)
                {
                case 1:
                    current_info.iq1 = current;
                    break;
                case 2:
                    current_info.iq2 = current;
                    break;
                case 3:
                    current_info.iq3 = current;
                    break;
                case 4:
                    current_info.iq4 = current;
                    break;
                default:
                    break;
                }
                break;
            }
        }
        return ESP_OK;
    };

    /**
     * @brief 使用current_info结构体向tx_msg存入电流数据, 此前需要使用set_current()初始化current_info
     */
    static void send_motor_current(current_info_t &current_info)
    {
        twai_message_t tx_msg;
        tx_msg.extd = 0;
        tx_msg.rtr = 0;
        tx_msg.self = 0;
        tx_msg.dlc_non_comp = 0;
        tx_msg.data_length_code = 8;
        tx_msg.identifier = 0x200;

        ESP_LOGI(TAG_TWAI, "Motor 1: %i, %i, %i, %i", current_info.iq1, current_info.iq2, current_info.iq3, current_info.iq4);
        tx_msg.data[0] = current_info.iq1 >> 8;
        tx_msg.data[1] = current_info.iq1;
        tx_msg.data[2] = current_info.iq2 >> 8;
        tx_msg.data[3] = current_info.iq2;
        tx_msg.data[4] = current_info.iq3 >> 8;
        tx_msg.data[5] = current_info.iq3;
        tx_msg.data[6] = current_info.iq4 >> 8;
        tx_msg.data[7] = current_info.iq4;

        ESP_ERROR_CHECK(twai_transmit(&tx_msg, portMAX_DELAY));
    };

    // 当速度当前值与速度设定值之间的误差在一定的FreeRTOS系统tick的时间段内小于一定范围时
    // 电机状态从MOTOR_NORMAL_PENDING转换为MOTOR_NORMAL, 或者电机状态从MOTOR_TRACE_SIN_PENDING转换为MOTOR_TRACE_SIN_STABLE
    static void speed_stable_check()
    {
        for (size_t i = 0; i < motor_counts; i++)
        {
            if (motor_info[i].motor_status == MOTOR_NORMAL_PENDING)
            {
                if (abs(motor_info[i].speed - motor_info[i].set_speed) < 100)
                {
                    motor_info[i].motor_status = MOTOR_NORMAL;
                    ESP_LOGI(TAG_TWAI, "Motor %d speed stable.", motor_info[i].motor_id);
                }
            }
            else if (motor_info[i].motor_status == MOTOR_TRACE_SIN_PENDING)
            {
                if (abs(motor_info[i].speed - motor_info[i].set_speed) < 100)
                {
                    motor_info[i].motor_status = MOTOR_TRACE_SIN_STABLE;
                    ESP_LOGI(TAG_TWAI, "Motor %d speed stable.", motor_info[i].motor_id);
                };
            };
        };
    };

public:
    /**
     * @brief 初始化电机, 需要在主函数手动设置motor_counts, TX和RX的IO口, PID初始化是否正确? 数据输入包含PID初始化
     */
    Motor(uint8_t *motor_id, uint8_t motor_counts = 1, gpio_num_t TX_TWAI_GPIO = GPIO_NUM_4, gpio_num_t RX_TWAI_GPIO = GPIO_NUM_5, float Kp = 1.2, float Ki = 0.2, float Kd = 0.5, float Pmax = 1000, float Imax = 1000, float Dmax = 1000, float max = 2000) : PID(Kp, Ki, Kd, Pmax, Imax, Dmax, max)
    {
        this->motor_counts = motor_counts;
        motor_info = new motor_info_t[motor_counts];
        memset(motor_info, 0, sizeof(motor_info_t) * motor_counts);
        this->TX_TWAI_GPIO = TX_TWAI_GPIO;
        this->RX_TWAI_GPIO = RX_TWAI_GPIO;

        twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_1MBITS();
        twai_filter_config_t filter_config = {
            .acceptance_code = (0),
            .acceptance_mask = (0xFFFFFFFF),
            .single_filter = true,
        };
        twai_general_config_t general_config = {
            .mode = TWAI_MODE_NO_ACK,
            .tx_io = TX_TWAI_GPIO,
            .rx_io = RX_TWAI_GPIO,
            .clkout_io = ((gpio_num_t)-1),
            .bus_off_io = ((gpio_num_t)-1),
            .tx_queue_len = 5,
            .rx_queue_len = 1,
            .alerts_enabled = 0x00000000,
            .clkout_divider = 0,
            .intr_flags = (1 << 1),
        };

        if (twai_driver_install(&general_config, &timing_config, &filter_config) != ESP_OK)
        {
            ESP_LOGE("TWAI", "Failed to install TWAI driver");
        }

        if (twai_start() != ESP_OK)
        {
            ESP_LOGE("TWAI", "Failed to start TWAI driver.");
        }

        ESP_LOGI("TWAI", "TWAI initialized.");

        // init motor_info
        for (size_t i = 0; i < motor_counts; i++)
        {
            motor_info[i].motor_id = i;
            motor_info[i].set_speed = 0;
            motor_info[i].speed = 0;
            motor_info[i].current = 0;
            motor_info[i].torque = 0;
            motor_info[i].temp = 0;
            motor_info[i].motor_status = MOTOR_DISABLED_LOCKED;
            motor_info[i].last_received = xTaskGetTickCount();
        }

        for (size_t i = 0; i < motor_counts; i++)
        {
            ESP_ERROR_CHECK(set_id(i, motor_id[i]));
            ESP_LOGI(TAG_TWAI, "Motor %d ID set to %d.\n", i, motor_id[i]);
        }

        BaseType_t task_created = xTaskCreate(Motor::task_motor, "task_motor", 8192, this, 10, NULL);
        if (task_created == pdPASS)
        {
            ESP_LOGI(TAG_TWAI, "TWAI task created.\n");
        }
        else
        {
            ESP_LOGI(TAG_TWAI, "Failed to create TWAI task.\n");
        }
    }

    /**
     * @brief 设置电机ID, 使电机ID与motor_info内顺序一致
     * @param index, ID(0-1)
     */
    esp_err_t set_id(uint8_t index, uint8_t id)
    {
        if (id > 4)
            return ESP_ERR_INVALID_ARG;
        motor_info[index].motor_id = id;
        return ESP_OK;
    }

    /**
     * @brief 电机状态从DISABLED_LOCKED转换为DISABLED
     */
    esp_err_t unlock_motor(uint8_t motor_id)
    {
        esp_err_t ret = ESP_ERR_INVALID_ARG;
        for (size_t i = 0; i < motor_counts; i++)
        {
            if (motor_info[i].motor_id == motor_id)
            {
                // Return ESP_NOT_SUPPORTED when MOTOR_DISCONNECTED
                if (motor_info[i].motor_status == MOTOR_DISCONNECTED)
                    return ESP_ERR_NOT_SUPPORTED;

                motor_info[i].motor_status = MOTOR_DISABLED;
                ESP_LOGI(TAG_TWAI, "Motor %d unlocked.\n", motor_id);
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

    /**
     * @brief 设置电机速度, 如果电机状态为DISABLED_LOCKED, 则返回ESP_ERR_NOT_SUPPORTED
     */
    esp_err_t set_speed(uint8_t motor_id, int16_t speed)
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
                    motor_info[i].motor_status = MOTOR_NORMAL_PENDING;
                }
                else
                {
                    motor_info[i].motor_status = MOTOR_DISABLED;
                }
                motor_info[i].set_speed = speed;
                ESP_LOGI(TAG_TWAI, "Motor %d speed %d.\n", motor_id, speed);
                return ESP_OK;
            }
        }
        // cannot find motor_id
        ESP_LOGI(TAG_TWAI, "Invalid ID.\n");
        return ESP_ERR_INVALID_ARG;
    };

    /**
     * @brief 设置电机速度跟踪正弦曲线, 如果电机状态为DISABLED_LOCKED, 则返回ESP_ERR_NOT_SUPPORTED
     */
    esp_err_t set_speed_trace(uint8_t motor_id, float amplitude, float omega, float offset)
    {
        for (size_t i = 0; i < motor_counts; i++)
        {
            if (motor_info[i].motor_id == motor_id)
            {
                if (motor_info[i].motor_status == MOTOR_DISABLED_LOCKED)
                {
                    ESP_LOGW(TAG_TWAI, "Motor %d is locked.\n", motor_id);
                    return ESP_ERR_NOT_SUPPORTED;
                }
                motor_info[i].speed_trace_sin_info.amplitude = amplitude;
                motor_info[i].speed_trace_sin_info.omega = omega;
                motor_info[i].speed_trace_sin_info.offset = offset;
                motor_info[i].motor_status = MOTOR_TRACE_SIN_PENDING;
                motor_info[i].speed_trace_sin_info.start_tick = xTaskGetTickCount();
                ESP_LOGI(TAG_TWAI, "Motor %d speed trace sin: amplitude %f, omega %f, offset %f.\n", motor_id, amplitude, omega, offset);
                return ESP_OK;
            }
        }
        // cannot find motor_id
        ESP_LOGI(TAG_TWAI, "Invalid ID.\n");
        return ESP_ERR_INVALID_ARG;
    };

    /**
     * @brief get motor status
     */
    motor_status_t get_motor_status(uint8_t motor_id)
    {
        return motor_info[motor_id].motor_status;
    };

    /**
     * @brief set motor status
     */
    esp_err_t set_motor_status(uint8_t motor_id, motor_status_t motor_status)
    {
        for (size_t i = 0; i < motor_counts; i++)
        {
            if (motor_info[i].motor_id == motor_id)
            {
                motor_info[i].motor_status = motor_status;
                return ESP_OK;
            };
        };
        return ESP_ERR_INVALID_ARG;
    };

    esp_err_t set_motor_status(uint8_t motor_id, float amplitude, float omega, float offset)
    {
        for (size_t i = 0; i < motor_counts; i++)
        {
            if (motor_info[i].motor_id == motor_id)
            {
                motor_info[i].speed_trace_sin_info.amplitude = amplitude;
                motor_info[i].speed_trace_sin_info.omega = omega;
                motor_info[i].speed_trace_sin_info.offset = offset;
                motor_info[i].motor_status = MOTOR_TRACE_SIN_PENDING;
                motor_info[i].speed_trace_sin_info.start_tick = xTaskGetTickCount();
                return ESP_OK;
            };
        };
        return ESP_ERR_INVALID_ARG;
    };

protected:
    static motor_info_t *motor_info;
    static size_t motor_counts;

    /**
     * @brief 使用CAN获取电机信息: 速度, 电流, 温度; 并更新状态
     */
    static void state_check()
    {
        static uint32_t current_tick = 0;
        current_tick = xTaskGetTickCount();
        if (current_tick % 1000 == 0)
            ESP_LOGI(TAG_TWAI, "Current tick: %lu", current_tick);

        twai_message_t twai_msg_re;

        // receive队列是否为空
        if (twai_receive(&twai_msg_re, 100) == ESP_OK)
        {
            // 获取电机ID
            uint8_t motor_id = twai_msg_re.identifier - 0x200;

            for (size_t i = 0; i < motor_counts; i++)
            {
                // update reveive time
                motor_info[i].last_received = xTaskGetTickCount();
                if (motor_info[i].motor_id == motor_id)
                {
                    motor_info[i].speed = (int16_t)(twai_msg_re.data[2] << 8 | twai_msg_re.data[3]);
                    motor_info[i].current = (int16_t)(twai_msg_re.data[4] << 8 | twai_msg_re.data[5]);
                    motor_info[i].temp = twai_msg_re.data[6];

                    // 将DISCONNECTED状态更新为DISABLED_LOCKED状态
                    if ((motor_info[i].motor_status == MOTOR_DISCONNECTED) & (current_tick - motor_info[i].last_received < 100))
                    {
                        motor_info[i].motor_status = MOTOR_DISABLED_LOCKED;
                    }
                    // 按照一定的时间间隔打印电机状态
                    ESP_LOGI(TAG_TWAI, "Motor %d State: Speed %d, Current %d, Temp %d .", motor_id, motor_info[i].speed, motor_info[i].current, motor_info[i].temp);
                }
            }
        }
        else
        {
            // 接收队列为空则将电机状态更新为DISCONNECTED
            for (size_t i = 0; i < motor_counts; i++)
            {
                // check motor status
                if (current_tick - motor_info[i].last_received > 1000)
                {
                    motor_info[i].motor_status = MOTOR_DISCONNECTED;
                    esp_event_post_to(loop_PRM, PRM, PRM_DISCONNECT_EVENT, NULL, 0, portMAX_DELAY);
                    ESP_LOGI(TAG_TWAI, "Motor %d disconnected.", motor_info[i].motor_id);
                }
            }
        }
    }

    /**
     * @brief FreeRTOS Task: 控制电机运动，同时接受电机状态
     */
    static void
    task_motor(void *args)
    {
        Motor *motor_ctrl = (Motor *)args;
        current_info_t current_info;
        memset(&current_info, 0, sizeof(current_info_t));
        // 服务缓启动
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        float PID_CURRENT = 0;

        while (1)
        {
            state_check();
            speed_stable_check();
            for (size_t i = 0; i < motor_counts; i++)
            {
                //     if (xTaskGetTickCount() % 1000 == 0)
                //         ESP_LOGI(TAG_TWAI, "Motor %d status: %d", motor_info[i].motor_id, motor_info[i].motor_status);

                // check motor status
                switch (motor_ctrl->motor_info[i].motor_status)
                {
                case MOTOR_DISABLED_LOCKED:
                case MOTOR_DISCONNECTED:
                case MOTOR_DISABLED:
                    set_current(motor_info[i].motor_id, 0, current_info);
                    break;

                case MOTOR_NORMAL:
                case MOTOR_NORMAL_PENDING:
                    PID_CURRENT = (motor_ctrl->get_output(motor_ctrl->motor_info[i].speed, motor_ctrl->motor_info[i].set_speed));
                    set_current(motor_info[i].motor_id, PID_CURRENT, current_info);
                    break;

                case MOTOR_TRACE_SIN_PENDING:
                case MOTOR_TRACE_SIN_STABLE:
                    // Generate SIN speed
                    motor_info[i].set_speed =
                        (motor_info[i].speed_trace_sin_info.amplitude *
                             sinf(motor_info[i].speed_trace_sin_info.omega *
                                  (xTaskGetTickCount() - motor_info[i].speed_trace_sin_info.start_tick) / (float)CONFIG_FREERTOS_HZ) +
                         motor_info[i].speed_trace_sin_info.offset) *
                        3420 / M_PI;
                    PID_CURRENT = motor_ctrl->get_output(motor_ctrl->motor_info[i].speed, motor_info[i].set_speed);
                    set_current(motor_info[i].motor_id, PID_CURRENT, current_info);
                    break;
                default:
                    break;
                };
            };
            send_motor_current(current_info);
            vTaskDelay(2 / portTICK_PERIOD_MS);
            // LOG MOTOR_INFO_t
            ESP_LOGI(TAG_TWAI, "Motor 1: %i, %i, %i, %i, %i, %i, %i", motor_info[0].set_speed, motor_info[0].speed, motor_info[0].current, motor_info[0].torque, motor_info[0].temp, motor_info[0].motor_id, motor_info[0].motor_status);
        };
        vTaskDelete(NULL);
    };
};

motor_info_t *Motor::motor_info = NULL;
size_t Motor::motor_counts = 0;

#endif