#ifndef PID_H
#define PID_H

class PIDController {
public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0), integral_(0) {}

    double getValue(double current_value, double target_value) {
        double error = target_value - current_value;

        double P_out = kp_ * error;

        integral_ += error;
        double I_out = ki_ * integral_;

        double derivative = error - prev_error_;
        double D_out = kd_ * derivative;

        double output = P_out + I_out + D_out;

        prev_error_ = error;

        return output;
    }

    void reset() {
        prev_error_ = 0;
        integral_ = 0;
    }

    void setCoefficients(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

private:
    double kp_;
    double ki_;
    double kd_;

    double prev_error_; 
    double integral_; 
};

#endif