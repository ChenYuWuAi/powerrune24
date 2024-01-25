// PID.h
#ifndef PID_H
#define PID_H

class PIDController {
public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0), integral_(0) {}

    double getValue(double current_value, double target_value) {
        // Compute the error
        double error = target_value - current_value;

        // Proportional term
        double P_out = kp_ * error;

        // Integral term
        integral_ += error;
        double I_out = ki_ * integral_;

        // Derivative term
        double derivative = error - prev_error_;
        double D_out = kd_ * derivative;

        // Combine all the terms
        double output = P_out + I_out + D_out;

        // Update error
        prev_error_ = error;

        return output;
    }

    // Reset the PID controller, useful for reinitializing in case of new control cycle
    void reset() {
        prev_error_ = 0;
        integral_ = 0;
    }

    // Update PID coefficients, can be useful for tuning the PID on the fly
    void setCoefficients(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

private:
    double kp_; // Proportional gain
    double ki_; // Integral gain
    double kd_; // Derivative gain

    double prev_error_; // Previous error value for the derivative term
    double integral_;   // Integral of error for the integral term
};

#endif // PID_H