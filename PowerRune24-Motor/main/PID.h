/**
 * @file PID.h
 * @brief PID控制器
 * @version 0.1
 */
#ifndef __PID_H__
#define __PID_H__

class PID
{
private:
    float Kp, Ki, Kd;
    float Pout, Iout, Dout;
    float Imax, Dmax;
    float error, error_last;
    float outputMax;
    float target;
    float output;

public:
    PID(float Kp, float Ki, float Kd, float Pmax, float Imax, float Dmax, float max);
    float get_output(float feedback_input, float target_input);
    void reset();
    void reset(float Kp, float Ki, float Kd, float Imax, float Dmax, float max); // 重置PID参数
};

#endif