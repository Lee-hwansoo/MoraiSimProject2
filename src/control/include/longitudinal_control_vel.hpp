#ifndef LONGITUDINAL_CONTROL_VEL_HPP
#define LONGITUDINAL_CONTROL_VEL_HPP

#include <vector>
#include <nav_msgs/Path.h>

class VelocityPlanning {
public:
    VelocityPlanning(double car_max_speed, double road_friction);
    std::vector<double> curvedBaseVelocity(const nav_msgs::Path& global_path, int point_num);

private:
    double car_max_speed_;
    double road_friction_;
};

#endif // LONGITUDINAL_CONTROL_VEL_HPP
