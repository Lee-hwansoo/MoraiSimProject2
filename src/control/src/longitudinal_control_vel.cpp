#include "longitudinal_control_vel.hpp"
#include <cmath>
#include <Eigen/Dense>

VelocityPlanning::VelocityPlanning(double car_max_speed, double road_friction)
    : car_max_speed_(car_max_speed), road_friction_(road_friction) {}

std::vector<double> VelocityPlanning::curvedBaseVelocity(const nav_msgs::Path& global_path, int point_num) {
    std::vector<double> out_vel_plan(global_path.poses.size(), 0);

    for (int i = 0; i < point_num; ++i) {
        out_vel_plan[i] = car_max_speed_ * 0.45;
    }

    for (int i = point_num; i < global_path.poses.size() - point_num; ++i) {
        std::vector<std::vector<double>> x_list;
        std::vector<double> y_list;

        for (int box = -point_num; box < point_num; ++box) {
            double x = global_path.poses[i + box].pose.position.x;
            double y = global_path.poses[i + box].pose.position.y;
            x_list.push_back({-2 * x, -2 * y, 1});
            y_list.push_back((-x * x) - (y * y));
        }

        Eigen::MatrixXd x_matrix(x_list.size(), 3);
        Eigen::VectorXd y_matrix(y_list.size());

        for (int j = 0; j < x_list.size(); ++j) {
            x_matrix(j, 0) = x_list[j][0];
            x_matrix(j, 1) = x_list[j][1];
            x_matrix(j, 2) = x_list[j][2];
            y_matrix(j) = y_list[j];
        }

        Eigen::MatrixXd x_trans = x_matrix.transpose();
        Eigen::VectorXd a_matrix = (x_trans * x_matrix).colPivHouseholderQr().solve(x_trans * y_matrix);

        double a = a_matrix(0);
        double b = a_matrix(1);
        double c = a_matrix(2);
        double r = sqrt(a * a + b * b - c);

        double v_max = sqrt(r * 9.8 * road_friction_);

        if (v_max > car_max_speed_) {
            v_max = car_max_speed_;
        }

        out_vel_plan[i] = v_max;
    }

    for (int i = global_path.poses.size() - point_num; i < global_path.poses.size() - 30; ++i) {
        out_vel_plan[i] = 10;
    }

    for (int i = global_path.poses.size() - 30; i < global_path.poses.size(); ++i) {
        out_vel_plan[i] = 0;
    }

    return out_vel_plan;
}
