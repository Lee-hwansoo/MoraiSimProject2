#include "state_models.hpp"
#include <iostream>
#include <stdexcept>

// CA Class Implementation
CA::CA(double dt) : dt(dt) {}

double CA::get_dt() const {
    return this->dt;
}

Eigen::RowVectorXd CA::step(const Eigen::RowVectorXd& x) const {
    double dt = this->dt;

    double px = x(0);
    double py = x(1);
    double v = x(2);
    double a = x(3);
    double theta = x(4);

    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);

    Eigen::RowVectorXd x_new(5);
    x_new << px + (v + 0.5 * a * dt) * cos_theta * dt,
             py + (v + 0.5 * a * dt) * sin_theta * dt,
             v + a * dt,
             a,
             theta;

    return x_new;
}

Eigen::RowVectorXd CA::H(const Eigen::RowVectorXd& x) const {
    // x = [x, y, v, a, theta]
    Eigen::RowVectorXd z(4);
    z << x(0), x(1), x(2), x(4);
    return z;
}

Eigen::MatrixXd CA::JA(const Eigen::RowVectorXd& x) const {
    double dt = this->dt;

    double px = x(0);
    double py = x(1);
    double v = x(2);
    double a = x(3);
    double theta = x(4);

    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);

    Eigen::MatrixXd JA_(5, 5);
    JA_ << 1, 0, cos_theta * dt, 0.5 * cos_theta * dt * dt, -(v + 0.5 * a * dt) * sin_theta * dt,
          0, 1, sin_theta * dt, 0.5 * sin_theta * dt * dt, (v + 0.5 * a * dt) * cos_theta * dt,
          0, 0, 1, dt, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;

    return JA_;
}


Eigen::MatrixXd CA::JH(const Eigen::RowVectorXd& x) const {
    double dt = this->dt;

    Eigen::MatrixXd JH_(4, 5);
    JH_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 0, 1;

    return JH_;
}

// CTRA Class Implementation
CTRA::CTRA(double dt) : dt(dt) {}

double CTRA::get_dt() const {
    return this->dt;
}

Eigen::RowVectorXd CTRA::step(const Eigen::RowVectorXd& x) const {
    double dt = this->dt;

    double px = x(0);
    double py = x(1);
    double v = x(2);
    double a = x(3);
    double theta = x(4);
    double r = x(5);

    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);

    Eigen::RowVectorXd x_new(6);

    if (std::abs(r) > 0.1) {
        double theta_dt = theta + r * dt;
        double cos_theta_dt = std::cos(theta_dt);
        double sin_theta_dt = std::sin(theta_dt);

        x_new << px + v / r * (sin_theta_dt - sin_theta) + v / (r * r) * (cos_theta_dt + dt * r * sin_theta_dt - cos_theta),
                 py + v / r * (-cos_theta_dt + cos_theta) + v / (r * r) * (sin_theta_dt - dt * r * cos_theta_dt - sin_theta),
                 v + a * dt,
                 a,
                 theta + r * dt,
                 r;
    } else {
        x_new << px + v * cos_theta * dt,
                 py + v * sin_theta * dt,
                 v + a * dt,
                 a,
                 theta,
                 r;
    }

    return x_new;
}

Eigen::RowVectorXd CTRA::H(const Eigen::RowVectorXd& x) const {
    // x = [x, y, v, a, theta, theta_rate]
    Eigen::RowVectorXd z(4);
    z << x(0), x(1), x(2), x(4);
    return z;
}

Eigen::MatrixXd CTRA::JA(const Eigen::RowVectorXd& x) const {
    double dt = this->dt;

    double px = x(0);
    double py = x(1);
    double v = x(2);
    double a = x(3);
    double theta = x(4);
    double r = x(5);

    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);

    Eigen::MatrixXd JA_(6, 6);

    if (std::abs(r) > 0.1) {
        double theta_dt = theta + r * dt;
        double cos_theta_dt = std::cos(theta_dt);
        double sin_theta_dt = std::sin(theta_dt);

        JA_ << 1, 0, (sin_theta_dt - sin_theta) / r, (-cos_theta + cos_theta_dt + r * dt * sin_theta_dt) / (r * r), ((r * v + a * r * dt) * cos_theta_dt - a * sin_theta_dt - v * r * cos_theta + a * sin_theta) / (r * r), -2 / (r * r * r) * ((r * v + a * r * dt) * sin_theta_dt + a * cos_theta_dt - v * r * sin_theta - a * cos_theta) + ((v + a * dt) * sin_theta_dt + dt * (r * v + a * r * dt) * cos_theta_dt - dt * a * sin_theta_dt - v * sin_theta) / (r * r),
              0, 1, (-cos_theta_dt + cos_theta) / r, (-sin_theta + sin_theta_dt - r * dt * cos_theta_dt) / (r * r), ((r * v + a * r * dt) * sin_theta_dt + a * cos_theta_dt - v * r * sin_theta - a * cos_theta) / (r * r), -2 / (r * r * r) * ((-r * v - a * r * dt) * cos_theta_dt + a * sin_theta_dt + v * r * cos_theta - a * sin_theta) + ((-v - a * dt) * cos_theta_dt + dt * (r * v + a * r * dt) * sin_theta_dt + a * dt * cos_theta_dt + v * cos_theta) / (r * r),
              0, 0, 1, dt, 0, 0,
              0, 0, 0, 1, 0, 0,
              0, 0, 0, 0, 1, dt,
              0, 0, 0, 0, 0, 1;
    } else {
        JA_ << 1, 0, cos_theta * dt, 0.5 * cos_theta * dt * dt, -(v + 0.5 * a * dt) * sin_theta * dt, 0,
              0, 1, sin_theta * dt, 0.5 * sin_theta * dt * dt, (v + 0.5 * a * dt) * cos_theta * dt, 0,
              0, 0, 1, dt, 0, 0,
              0, 0, 0, 1, 0, 0,
              0, 0, 0, 0, 1, dt,
              0, 0, 0, 0, 0, 1;
    }

    return JA_;
}

Eigen::MatrixXd CTRA::JH(const Eigen::RowVectorXd& x) const {
    double dt = this->dt;

    double px = x(0);
    double py = x(1);
    double v = x(2);
    double a = x(3);
    double theta = x(4);
    double r = x(5);

    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);

    Eigen::MatrixXd JH_(4, 6);

    if (std::abs(r) > 0.1) {
        double theta_dt = theta + r * dt;
        double cos_theta_dt = std::cos(theta_dt);
        double sin_theta_dt = std::sin(theta_dt);

        JH_ << 1, 0, (sin_theta_dt - sin_theta) / r, (-cos_theta + cos_theta_dt + r * dt * sin_theta_dt) / (r * r), ((r * v + a * r * dt) * cos_theta_dt - a * sin_theta_dt - v * r * cos_theta + a * sin_theta) / (r * r), -2 / (r * r * r) * ((r * v + a * r * dt) * sin_theta_dt + a * cos_theta_dt - v * r * sin_theta - a * cos_theta) + ((v + a * dt) * sin_theta_dt + dt * (r * v + a * r * dt) * cos_theta_dt - dt * a * sin_theta_dt - v * sin_theta) / (r * r),
              0, 1, (-cos_theta_dt + cos_theta) / r, (-sin_theta + sin_theta_dt - r * dt * cos_theta_dt) / (r * r), ((r * v + a * r * dt) * sin_theta_dt + a * cos_theta_dt - v * r * sin_theta - a * cos_theta) / (r * r), -2 / (r * r * r) * ((-r * v - a * r * dt) * cos_theta_dt + a * sin_theta_dt + v * r * cos_theta - a * sin_theta) + ((-v - a * dt) * cos_theta_dt + dt * (r * v + a * r * dt) * sin_theta_dt + a * dt * cos_theta_dt + v * cos_theta) / (r * r),
              0, 0, 1, dt, 0, 0,
              0, 0, 0, 0, 1, dt;
    } else {
        JH_ << 1, 0, cos_theta * dt, 0.5 * cos_theta * dt * dt, -(v + 0.5 * a * dt) * sin_theta * dt, 0,
              0, 1, sin_theta * dt, 0.5 * sin_theta * dt * dt, (v + 0.5 * a * dt) * cos_theta * dt, 0,
              0, 0, 1, dt, 0, 0,
              0, 0, 0, 0, 1, dt;
    }

    return JH_;
}
