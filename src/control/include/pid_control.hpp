#ifndef PID_CONTROL_HPP
#define PID_CONTROL_HPP

#include "ros/ros.h"

class PIDControl {
public:
    PIDControl();
    double pid(double target_vel, double current_vel);

private:
    double p_gain_;
    double i_gain_;
    double d_gain_;
    double prev_error_;
    double i_control_;
    double control_time_;
};

#endif // PID_CONTROL_HPP
