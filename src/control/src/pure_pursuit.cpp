#include "pure_pursuit.hpp"

PurePursuit::PurePursuit() : nh_("~"), pid_(), vel_planning_(0, 0.15) {
    global_path_sub_ = nh_.subscribe("/global_path", 1, &PurePursuit::globalPathCallback, this);
    local_path_sub_ = nh_.subscribe("/local_path", 1, &PurePursuit::pathCallback, this);
    mode_sub_ = nh_.subscribe("/mode", 1, &PurePursuit::modeCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 1, &PurePursuit::odomCallback, this);
    status_sub_ = nh_.subscribe("/Ego_topic", 1, &PurePursuit::statusCallback, this);
    collision_sub_ = nh_.subscribe("/pred_collision", 1, &PurePursuit::collisionCallback, this);

    ctrl_cmd_pub_ = nh_.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd_0", 1);

    ctrl_cmd_msg_.longlCmdType = 1;

    vehicle_length_ = 4.470;
    parking_time_ = 1.0;
    lfd_ = 0.0;
    min_lfd_ = 2.3;
    max_lfd_ = 30.0;
    lfd_gain_ = 0.6;
    target_velocity_ = 60.0;

    vel_planning_ = VelocityPlanning(target_velocity_ / 3.6, 0.15);

    is_path_ = false;
    is_odom_ = false;
    is_status_ = false;
    is_global_path_ = false;
    is_look_forward_point_ = false;
    is_stop_ = false;
    is_collision_ = false;

    while (!is_global_path_) {
        ros::spinOnce();
    }

    velocity_list_ = vel_planning_.curvedBaseVelocity(global_path_, 50);
    ROS_INFO("Received global path data");
}

void PurePursuit::run() {
    ros::Rate rate(30);
    while (ros::ok()) {
        if (is_path_ && is_odom_ && is_status_) {
            int current_waypoint = getCurrentWaypoint(status_msg_);
            // std::cout << target_velocity_ << std::endl;
            target_velocity_ = velocity_list_[current_waypoint] * 3;
            double steering = calcPurePursuit();
            // std::cout << steering << std::endl;
            if (is_look_forward_point_) {
                ctrl_cmd_msg_.steering = steering;
            } else {
                ctrl_cmd_msg_.steering = 0.0;
            }

            double output = pid_.pid(target_velocity_, status_msg_.velocity.x * 3.6);

            if (mode_ == "PARKING MODE") {
                if (!is_stop_) {
                    auto start_time = std::chrono::high_resolution_clock::now();
                    while (std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).count() < 5.0) {
                        ctrl_cmd_msg_.accel = 0.0;
                        ctrl_cmd_msg_.brake = 1.0;
                        ros::Duration(0.3).sleep();
                    }
                    is_stop_ = true;
                }
                ctrl_cmd_msg_.accel = 0.1;
                ctrl_cmd_msg_.brake = 0.0;
            } else if (mode_ == "FINISH MODE") {
                ctrl_cmd_msg_.accel = 0.0;
                ctrl_cmd_msg_.brake = 1.0;
            } else {
                if (!is_collision_) {
                    if (output > 0.0) {
                        ctrl_cmd_msg_.accel = output;
                        ctrl_cmd_msg_.brake = 0.0;
                    } else if (output > -8.0) {
                        ctrl_cmd_msg_.accel = 0.0;
                        ctrl_cmd_msg_.brake = 0.0;
                    } else {
                        ctrl_cmd_msg_.accel = 0.0;
                        ctrl_cmd_msg_.brake = 0.35;
                    }
                } else {
                    ctrl_cmd_msg_.accel = 0.0;
                    ctrl_cmd_msg_.brake = 0.3;
                }
            }

            // ROS_INFO("Publishing control command: accel=%.2f, brake=%.2f, steering=%.2f",
            //          ctrl_cmd_msg_.accel, ctrl_cmd_msg_.brake, ctrl_cmd_msg_.steering);

            ctrl_cmd_pub_.publish(ctrl_cmd_msg_);
        // } else {
        //     ROS_WARN("Waiting for required data: is_path_=%d, is_odom_=%d, is_status_=%d",
        //              is_path_, is_odom_, is_status_);
        }
        rate.sleep();
        ros::spinOnce();
    }
}

void PurePursuit::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    is_path_ = true;
    path_ = *msg;
    // ROS_INFO("Path received");
}

void PurePursuit::statusCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
    is_status_ = true;
    status_msg_ = *msg;
    // ROS_INFO("Status received: position=(%.2f, %.2f), velocity=%.2f", 
    //          msg->position.x, msg->position.y, msg->velocity.x);
}

void PurePursuit::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    is_odom_ = true;
    tf::Quaternion odom_quaternion(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    double roll, pitch;
    tf::Matrix3x3(odom_quaternion).getRPY(roll, pitch, vehicle_yaw_);
    current_position_.x = msg->pose.pose.position.x;
    current_position_.y = msg->pose.pose.position.y;
    // ROS_INFO("Odometry received");
}

void PurePursuit::globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    global_path_ = *msg;
    is_global_path_ = true;
    global_path_coords_.clear();
    for (const auto& pose : msg->poses) {
        global_path_coords_.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
    // ROS_INFO("Global path received");
}

void PurePursuit::collisionCallback(const std_msgs::Bool::ConstPtr& msg) {
    is_collision_ = msg->data;
    // ROS_INFO("Collision status received: %d", is_collision_);
}

void PurePursuit::modeCallback(const std_msgs::String::ConstPtr& msg) {
    mode_ = msg->data;
    // ROS_INFO("Mode received: %s", mode_.c_str());
}

int PurePursuit::getCurrentWaypoint(const morai_msgs::EgoVehicleStatus& ego_status) {
    double min_dist = std::numeric_limits<double>::max();
    int closest_waypoint = 0;

    for (size_t i = 0; i < global_path_coords_.size(); ++i) {
        double dx = ego_status.position.x - global_path_coords_[i][0];
        double dy = ego_status.position.y - global_path_coords_[i][1];
        double dist = sqrt(dx * dx + dy * dy);

        if (dist < min_dist) {
            min_dist = dist;
            closest_waypoint = i;
        }
    }
    // ROS_INFO("closest_waypoint: %d", closest_waypoint);
    return closest_waypoint;
}

double PurePursuit::calcPurePursuit() {
    lfd_ = status_msg_.velocity.x * lfd_gain_;
    lfd_ = std::max(min_lfd_, std::min(lfd_, max_lfd_));

    Eigen::Matrix3d trans_matrix;
    trans_matrix << cos(vehicle_yaw_), -sin(vehicle_yaw_), current_position_.x,
                    sin(vehicle_yaw_),  cos(vehicle_yaw_), current_position_.y,
                    0, 0, 1;

    Eigen::Matrix3d det_trans_matrix = trans_matrix.inverse();

    is_look_forward_point_ = false;
    for (const auto& pose : path_.poses) {
        Eigen::Vector3d global_path_point(pose.pose.position.x, pose.pose.position.y, 1);
        Eigen::Vector3d local_path_point = det_trans_matrix * global_path_point;

        if (local_path_point[0] > 0) {
            double dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2));
            if (dis >= lfd_) {
                forward_point_ = pose.pose.position;
                is_look_forward_point_ = true;
                break;
            }
        }
    }

    if (is_look_forward_point_) {
        double theta = atan2(forward_point_.y - current_position_.y, forward_point_.x - current_position_.x) - vehicle_yaw_;
        // ROS_INFO("Calculated steering angle: %.2f", atan2(2 * vehicle_length_ * sin(theta), lfd_));
        return atan2(2 * vehicle_length_ * sin(theta), lfd_);
    } else {
        // ROS_WARN("No look forward point found");
        return 0.0;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit");
    PurePursuit pure_pursuit;
    pure_pursuit.run();
    return 0;
}
