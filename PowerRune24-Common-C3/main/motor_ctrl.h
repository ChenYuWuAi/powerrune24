/*
File Name:      motor_crtl.h

Function List:  add_motor(uint8_t motor_id)
                del_motor(uint8_t motor_id)
                disable_motor(uint8_t motor_id)
                set_speed(uint8_t motor_id, int16_t speed)
                set_speed_trace(uint8_t motor_id, float amplitude, float omega, float offset)
                set_current(uint8_t motor_id, int16_t current)
                motor(gpio_num_t io_TWAI_tx, gpio_num_t io_TWAI_rx)
                task_motor(void* args)
                state_check(void* args)

*/

#pragma once
#ifndef _MOTOR_H_
#define _MOTOR_H_

#ifdef  __cplusplus
extern  "C"  {
#endif

#include <stdio.h>
#include <string.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/twai.h>
#include <esp_err.h> 
#include "simple_pid.h"

//speed_trace_sin_info
typedef struct {
    int set_speed;
    float amplitude;
    float omega;
    float speed_angular;
    float offset;
}speed_trace_sin_info_t;

//motor_info
typedef struct {
    int set_speed;
    int set_current;
    int speed;
    int current;
    int torque;
    int temp;
    uint8_t motor_id;
    speed_trace_sin_info_t speed_trace_sin_info;
    //motor_status_t motor_status;
    
}motor_info_t;

//motor_status
typedef enum {
    //需要发消息给主控拒绝其他操作
    MOTOR_DISCONNECTED,

    //state: 无控制信号输入; 可以set_speed(), set_speed_trace(), 
    //可以进入MOTOR_NORMAL状态, 可以进入MOTOR_TRACE_SIN_PENDING状态,
    //需要手动输入unlock_motor命令使电机运转
    MOTOR_DISABLED_LOCKED,

    //state:  无控制信号输入; 可以set_speed(), set_speed_trace(), 
    //可以进入MOTOR_NORMAL状态, 可以进入MOTOR_TRACE_SIN_PENDING状态,
    //含义: ????
    MOTOR_DISABLED,

    //含义: ????
    //与MOTOR_TRACE_SIN_STABLE区别
    MOTOR_NORMAL,

    //state: 电机启动需要时间, 到达转速后启动LED,
    //此时进入MOTOR_TRACE_SIN_PENDING状态
    //??此状态只针对正弦形式的速度吗??
    MOTOR_TRACE_SIN_PENDING,

    //state: 电机转速稳定
    //??此状态只针对正弦形式的速度吗??
    MOTOR_TRACE_SIN_STABLE,

}motor_status_t;

class motor {
protected:
    motor_info_t* motor_info;
    size_t motor_counts = 0;

    motor(gpio_num_t io_TWAI_tx, gpio_num_t io_TWAI_rx){//TWAI

    };
    
    virtual void task_motor(void* args){
        
    };

    
    virtual void state_check(void* args){    

    };

private:  
    static TaskHandle_t TWAI_handle;

    gpio_num_t io_TWAI_tx;
    gpio_num_t io_TWAI_rx;

    //PID crtl
    PID* motor_PID;

    esp_err_t set_current(uint8_t motor_id, int16_t current) {
       
    };

public:
    //new motor, state:???
    void add_motor(uint8_t motor_id) {
        for(size_t i=0; i < motor_counts; i++) {
            if(motor_info[i].motor_id == motor_id){
                return;
            }
        }

        //new motor
        motor_info_t new_motor;
        new_motor.motor_id = motor_id;
        new_motor.set_speed = 0;
        new_motor.set_current = 0;
        new_motor.speed = 0;
        new_motor.current = 0;
        new_motor.torque = 0;
        new_motor.temp = 0;
        //new_motor.motor_status = MOTOR_DISCONNECTED;

        //increase motor_info array, add new motor
        motor_info_t* temp = new motor_info_t[motor_counts+1];
        memcpy(temp, motor_info, motor_counts * sizeof(motor_info_t));
        delete[] motor_info;
        motor_info = temp;
        motor_info[motor_counts] = new_motor;
        motor_counts += 1;
        printf("New motor added.\n");

    };

    //delete motor, state:???
    void del_motor(uint8_t motor_id) {
        for(size_t i = 0; i < motor_counts; i++) {
            if(motor_info[i].motor_id == motor_id) {
                size_t t = i;
                for(size_t j = i; j < motor_counts - 1; j++) {
                    motor_info[j] = motor_info[j + 1];

                }
                motor_counts -= 1;
                printf("Motor %d deleted.\n",&t);

                break;

            }

        }

    };

    //!!NOT SURE!! physically disable motor, state:???
    void disable_motor(uint8_t motor_id) {
       for(size_t i = 0; i < motor_counts; i++) { 
            if(motor_info[i].motor_id == motor_id) {
                motor_info[i].set_speed = 0;
                motor_info[i].set_current = 0;
                motor_info[i].speed = 0;
                motor_info[i].current = 0;
                motor_info[i].torque = 0;
                motor_info[i].temp = 0;
                //motor_info[i].motor_status = MOTOR_DISABLED;
                printf("Motor %d disabled.\n",&i);

                break;

            }

        } 
    };

    //!!NOT SURE!! set motor speed, state:???
    esp_err_t set_speed(uint8_t motor_id, int16_t speed) {
        for(size_t i = 0; i < motor_counts; i++) {
            if(motor_info[i].motor_id == motor_id) {
                motor_info[i].set_speed = speed;
                //motor_info[i].motor_status = MOTOR_DISABLED_LOCKED;
                printf("Motor %d speed %d.\n",&i, &speed);

                return ESP_OK;
                //break;

            }

        }

        //cannot find motor_id
        printf("Invalid ID.\n");
        return ESP_ERR_INVALID_ARG;
    };

    //set speed trace, state:???
    esp_err_t set_speed_trace(uint8_t motor_id, float amplitude, float omega, float offset) {
       for(size_t i = 0; i < motor_counts; i++) {
            if(motor_info[i].motor_id == motor_id) {      
                //motor_info[i].motor_status = MOTOR_TRACE_SIN_PENDING;
                motor_info[i].speed = 0;
                motor_info[i].speed_trace_sin_info.set_speed = motor_info[i].set_speed;  // 设置期望速度
                motor_info[i].speed_trace_sin_info.amplitude = amplitude; 
                motor_info[i].speed_trace_sin_info.omega = omega;  
                motor_info[i].speed_trace_sin_info.speed_angular = 2 * M_PI * omega;  
                motor_info[i].speed_trace_sin_info.offset = offset;  

                return ESP_OK;
            }
        }

        //cannot find motor_id
        printf("Invalid ID.\n");
        return ESP_ERR_INVALID_ARG;
    };

};

#ifdef  __cplusplus
}
#endif

#endif