#include "pid_control.hpp"

PIDControl::PIDControl() {
    p_gain_ = 1.45;
    i_gain_ = 0.1;
    d_gain_ = 1.2;
    prev_error_ = 0;
    i_control_ = 0;
    control_time_ = 0.02;
}

double PIDControl::pid(double target_vel, double current_vel) {
    double error = target_vel - current_vel;

    double p_control = p_gain_ * error;
    i_control_ += i_gain_ * error * control_time_;
    double d_control = d_gain_ * (error - prev_error_) / control_time_;

    double output = p_control + i_control_ + d_control;
    prev_error_ = error;

    // ROS_INFO("PID output: target_vel=%.2f, current_vel=%.2f, output=%.2f", target_vel, current_vel, output);
    return output;
}

