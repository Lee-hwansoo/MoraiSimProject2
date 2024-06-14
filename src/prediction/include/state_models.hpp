#ifndef STATE_MODELS_HPP
#define STATE_MODELS_HPP

#include <Eigen/Dense>
#include <cmath>

// Abstract base class for state models
class StateModel {
public:
    virtual ~StateModel() = default;

    virtual double get_dt() const = 0;
    virtual Eigen::RowVectorXd step(const Eigen::RowVectorXd& x) const = 0;
    virtual Eigen::RowVectorXd H(const Eigen::RowVectorXd& x) const = 0;
    virtual Eigen::MatrixXd JA(const Eigen::RowVectorXd& x) const = 0;
    virtual Eigen::MatrixXd JH(const Eigen::RowVectorXd& x) const = 0;
};

// Constant Acceleration (CA) model
class CA : public StateModel {
public:
    explicit CA(double dt = 0.1);
    ~CA() = default;

    double get_dt() const override;
    Eigen::RowVectorXd step(const Eigen::RowVectorXd& x) const override;
    Eigen::RowVectorXd H(const Eigen::RowVectorXd& x) const override;
    Eigen::MatrixXd JA(const Eigen::RowVectorXd& x) const override;
    Eigen::MatrixXd JH(const Eigen::RowVectorXd& x) const override;

private:
    double dt;
};

// Coordinated Turn with Rate and Acceleration (CTRA) model
class CTRA : public StateModel {
public:
    explicit CTRA(double dt = 0.1);
    ~CTRA() = default;

    double get_dt() const override;
    Eigen::RowVectorXd step(const Eigen::RowVectorXd& x) const override;
    Eigen::RowVectorXd H(const Eigen::RowVectorXd& x) const override;
    Eigen::MatrixXd JA(const Eigen::RowVectorXd& x) const override;
    Eigen::MatrixXd JH(const Eigen::RowVectorXd& x) const override;

private:
    double dt;
};

#endif // STATE_MODELS_HPP
