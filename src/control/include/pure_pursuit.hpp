#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <morai_msgs/CtrlCmd.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <chrono>
#include "pid_control.hpp"
#include "longitudinal_control_vel.hpp"

class PurePursuit {
public:
    PurePursuit();
    void run();

private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void statusCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg);
    void collisionCallback(const std_msgs::Bool::ConstPtr& msg);
    void modeCallback(const std_msgs::String::ConstPtr& msg);
    int getCurrentWaypoint(const morai_msgs::EgoVehicleStatus& ego_status);
    double calcPurePursuit();

    ros::NodeHandle nh_;

    ros::Subscriber global_path_sub_;
    ros::Subscriber local_path_sub_;
    ros::Subscriber mode_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber status_sub_;
    ros::Subscriber collision_sub_;

    ros::Publisher ctrl_cmd_pub_;

    morai_msgs::CtrlCmd ctrl_cmd_msg_;
    nav_msgs::Path global_path_;
    nav_msgs::Path path_;
    morai_msgs::EgoVehicleStatus status_msg_;

    geometry_msgs::Point forward_point_;
    geometry_msgs::Point current_position_;

    double vehicle_length_;
    double parking_time_;
    double lfd_;
    double min_lfd_;
    double max_lfd_;
    double lfd_gain_;
    double target_velocity_;

    bool is_path_;
    bool is_odom_;
    bool is_status_;
    bool is_global_path_;
    bool is_look_forward_point_;
    bool is_stop_;
    bool is_collision_;

    std::string mode_;

    PIDControl pid_;
    VelocityPlanning vel_planning_;
    std::vector<double> velocity_list_;

    std::vector<Eigen::Vector2d> global_path_coords_;

    double vehicle_yaw_;
};

#endif // PURE_PURSUIT_HPP
