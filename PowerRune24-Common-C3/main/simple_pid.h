#pragma once
#ifndef _PID_H_
#define _PID_H_

#ifdef  __cplusplus
extern  "C"  {
#endif

#include <stdio.h>

#define TAG "PID"

//PID variables
typedef struct{
    float Kp;
    float Ki;
    float Kd;
    float p;
    float i;
    float d;
    float p_MAX;
    float i_MAX;
    float d_MAX;
    float out_MAX;
    float out;
    float err; 
}PID_var;

//PID keys
typedef enum {
    kp,
    ki,
    kd,
    p_max,
    i_max,
    d_max,
    out_max
}PID_key;

class PID {
protected:
    //init PID variables
    PID(PID_var *pid, float Kp, float Ki, float Kd, float p_MAX, float i_MAX, float d_MAX, float out_MAX){
        pid->Kp = Kp;
        pid->Ki = Ki;
        pid->Kd = Kd;
        pid->p_MAX = p_MAX;
        pid->i_MAX = i_MAX;
        pid->d_MAX = d_MAX;
        pid->out_MAX = out_MAX;
        pid->err = 0.0f;

    };
public: 
    //modify PID variables??
    /*
    void modify(PID_var* pid, PID_key key, float value){    
        switch (key) {
        case kp:
            pid->Kp = value;
            break;
        case ki:
            pid->Ki = value;
            break;
        case kd:
            pid->Kd = value;
            break;
        case p_max:
            pid->p_MAX = value;
            break;
        case i_max:
            pid->i_MAX = value;
            break;
        case d_max:
            pid->d_MAX = value;
            break;
        case out_max:
            pid->out_MAX = value;
            break;
        default:
            
            break;
    }
    };
    */
   
    //calculate PID
    float calc(PID_var* pid, float input, float target) {
        float error = target - input;
        pid->p = pid->Kp * pid->err;
        pid->i += pid->Ki * pid->err;
        if (pid->i > pid->i_MAX) {
            pid->i = pid->i_MAX;
        } else if (pid->i < -pid->i_MAX) {
            pid->i = -pid->i_MAX;
        }
        pid->d = pid->Kd * (error - pid->err);
        pid->err = error;

        pid->out = pid->p + pid->i + pid->d;
        if (pid->out > pid->out_MAX) {
            pid->out = pid->out_MAX;
        } else if (pid->out < -pid->out_MAX) {
            pid->out = -pid->out_MAX;
        }
    
        return pid->out;

    };

};

#ifdef  __cplusplus
}
#endif

#endif