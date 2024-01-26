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
    PID(float Kp, float Ki, float Kd, float Pmax, float Imax, float Dmax, float max)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->Imax = Imax;
        this->Dmax = Dmax;
        this->outputMax = max;

        error = 0;
        error_last = 0;
        Pout = 0;
        Iout = 0;
        Dout = 0;
        output = 0;
    };

    // get output, return float, input float feedback, input float target
    float get_output(float feedback_input, float target_input)
    {
        target = target_input;
        error = target - feedback_input;

        Pout = Kp * error;
        Iout += Ki * error;
        Dout = Kd * (error - error_last);

        if (Iout > Imax)
            Iout = Imax;
        else if (Iout < -Imax)
            Iout = -Imax;

        if (Dout > Dmax)
            Dout = Dmax;
        else if (Dout < -Dmax)
            Dout = -Dmax;

        output = Pout + Iout + Dout;

        if (output > outputMax)
            output = outputMax;
        else if (output < -outputMax)
            output = -outputMax;

        error_last = error;

        return output;
    }
};

#endif