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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/twai.h>
#include <esp_err.h> 
#include "twai_init.h"
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
    //int torque;
    int temp;
    uint8_t motor_id;
    speed_trace_sin_info_t speed_trace_sin_info;
    motor_status_t motor_status;
    
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

private:  
    gpio_num_t TX_TWAI_GPIO;
    gpio_num_t RX_TWAI_GPIO;

    //PID crtl
    //PID* motor_PID;
    
    esp_err_t set_current(uint8_t motor_id, int16_t current) {
        for(size_t i=0; i < motor_counts; i++) {
            if(motor_info[i].motor_id == motor_id){
                motor_info[i].set_current == current;
            }
        }
    };

protected:
    motor_info_t* motor_info;
    size_t motor_counts;

    //start motor TWAI
    virtual void task_motor(void* args){
        twai_message_t twai_msg_tr;
        memset(&twai_msg_tr, 0, sizeof(twai_message_t));

        while(1) {
            for(size_t i = 0; i < motor_counts; i++) {
                int16_t current_val = motor_info[i].set_current;
                twai_msg_tr.data[i*2] = (current_val >> 8) & 0xFF;
                twai_msg_tr.data[i*2 + 1] = current_val & 0xFF;
            }
        
            twai_msg_tr.identifier= 0x200;
            twai_msg_tr.data_length_code = 8;
            twai_msg_tr.flags = TWAI_MSG_FLAG_NONE;

            esp_err_t err = twai_transmit(&twai_msg_tr, pdMS_TO_TICKS(20));
        
            if (err == ESP_OK){
                printf("Transmit successful.\n");
            } else{
                printf("Transmit failed with error: %d\n", err);
            }
        
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    };

    //get motor state from TWAI
    virtual void state_check(void* args){    
        while(1) {
        twai_message_t twai_msg_re;
        esp_err_t receive_status = twai_receive(&twai_msg_re, pdMS_TO_TICKS(20));
            
        if(receive_status == ESP_OK){
            printf("TWAI message received .\n");

            uint8_t motor_id = twai_msg_re.identifier & 0xF; 

            for(size_t i = 0; i < motor_counts; i++) {
                if(motor_info[i].motor_id == motor_id) {       
                    motor_info[i].speed = (twai_msg_re.data[2] << 8) | twai_msg_re.data[3]; 
                    motor_info[i].current = (twai_msg_re.data[4] << 8) | twai_msg_re.data[5]; 
                    motor_info[i].temp = twai_msg_re.data[6]; 

                    printf("Motor %d State: Speed %d, Current %d, Temp %d .\n", 
                            motor_id, motor_info[i].speed, motor_info[i].current, motor_info[i].temp);
                }
            }
        }else if(receive_status == ESP_ERR_TIMEOUT){
            printf("TWAI Receive timed out.\n");
        }else{
            printf("Failed to receive TWAI message.\n");
        }

        vTaskDelay(pdMS_TO_TICKS(20)); 

        }

    };

    //motor init 。
    motor(gpio_num_t TX_TWAI_GPIO, gpio_num_t RX_TWAI_GPIO){
        this->TX_TWAI_GPIO = TX_TWAI_GPIO;
        this->RX_TWAI_GPIO = RX_TWAI_GPIO;

        twai_init(this->TX_TWAI_GPIO, this->RX_TWAI_GPIO);

        //
        BaseType_t task_created = xTaskCreate(this->task_motor, "task_motor", 2048, NULL, 5, &(this->TWAI_handle));
        if(task_created == pdPASS) {
            printf("TWAI task created.\n");
        }else {
            printf("Failed to create TWAI task.\n");
        }

        motor_counts = 0;
        motor_info = new motor_info_t[motor_counts];
    }


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
        //new_motor.torque = 0;
        new_motor.temp = 0;
        //new_motor.motor_status = MOTOR_DISABLED

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
                printf("Motor %d deleted.\n", &t);

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
                motor_info[i].speed_trace_sin_info.set_speed = motor_info[i].set_speed;
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