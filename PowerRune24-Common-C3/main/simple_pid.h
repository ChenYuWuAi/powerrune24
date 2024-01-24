#pragma once
#ifndef _PID_H_
#define _PID_H_

#include "stdint.h"

class PID
{
private:
    typedef struct _PID_TypeDef
    {
        float target;
        float lastNoneZeroTarget;
        float kp;
        float ki;
        float kd;

        float measure;  // 测量值
        float err;      // 误差
        float last_err; // 上次误差

        float pout;
        float iout;
        float dout;

        float output; // 本次输出
        // float last_output;      //上次输出

        float MaxOutput;     // 输出限幅
        float IntegralLimit; // 积分限幅
        float DeadBand;      // 死区（绝对值）
        // float ControlPeriod;  //控制周期
        float Max_Err; // 最大误差

        uint32_t thistime;
        uint32_t lasttime;
        uint8_t dtime;

    } PID_TypeDef;

public:
    PID();

    void PID_INIT(PID_TypeDef *pid,
                  uint16_t maxout,
                  uint16_t intergral_limit,
                  float deadband,
                  int16_t max_err,
                  int16_t target,
                  float kp,
                  float ki,
                  float kd);

    void PID_RESET(PID_TypeDef *pid,
                   float kp,
                   float ki,
                   float kd);

    void PID_TARGET(PID_TypeDef *pid,
                    uint16_t new_target);

    float PID_CALC(PID_TypeDef *pid,
                   int16_t measure);
};

#endif