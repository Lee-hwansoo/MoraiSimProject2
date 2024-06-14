#ifndef STATE_FILTERS_HPP
#define STATE_FILTERS_HPP

#include <Eigen/Dense>
#include <vector>
#include <functional>
#include <random>
#include <cmath>
#include <memory>
#include "state_models.hpp"


class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter(int x_dim, int z_dim, std::shared_ptr<StateModel> model);
    ~ExtendedKalmanFilter() = default;

    void prediction(const Eigen::RowVectorXd& u = Eigen::RowVectorXd::Zero(0),
                    std::function<Eigen::MatrixXd(const Eigen::RowVectorXd&)> JA = nullptr,
                    std::function<Eigen::RowVectorXd(const Eigen::RowVectorXd&)> F = nullptr,
                    const Eigen::MatrixXd& Q = Eigen::MatrixXd());

    void correction(const Eigen::RowVectorXd& z,
                    std::function<Eigen::MatrixXd(const Eigen::RowVectorXd&)> JH = nullptr,
                    std::function<Eigen::RowVectorXd(const Eigen::RowVectorXd&)> H = nullptr,
                    const Eigen::MatrixXd& R = Eigen::MatrixXd());

    Eigen::MatrixXd pred(double T);

    int x_dim, z_dim;

    Eigen::MatrixXd Q;   // 프로세스 노이즈 공분산 행렬
    Eigen::MatrixXd R;   // 측정 노이즈 공분산 행렬
    Eigen::MatrixXd B;   // 제어 변수 행렬
    Eigen::MatrixXd P;   // 오차 공분산 행렬

    std::function<Eigen::RowVectorXd(const Eigen::RowVectorXd&)> F;  // 상태 전이 함수
    std::function<Eigen::RowVectorXd(const Eigen::RowVectorXd&)> H;  // 측정 함수
    std::function<Eigen::MatrixXd(const Eigen::RowVectorXd&)> JA; // 상태 전이 함수의 야코비안
    std::function<Eigen::MatrixXd(const Eigen::RowVectorXd&)> JH; // 측정 함수의 야코비안

    Eigen::RowVectorXd x;   // 상태 벡터
    Eigen::RowVectorXd y;   // 측정 벡터

    Eigen::MatrixXd K;   // 칼만 이득
    Eigen::MatrixXd S;   // 측정 오차 공분산 행렬
    Eigen::MatrixXd _I;   // 단위 행렬
    Eigen::MatrixXd SI;  // 측정 오차 공분산 행렬의 역행렬

    Eigen::MatrixXd inv(const Eigen::MatrixXd& mat);

    double likelihood;

    std::shared_ptr<StateModel> model; // 상태 모델
private:

};

class IMMFilter {
public:
    IMMFilter(const std::vector<std::shared_ptr<ExtendedKalmanFilter>>& filters, const Eigen::RowVectorXd& mu, const Eigen::MatrixXd& M);
    ~IMMFilter() = default;

    void mixing();
    void prediction(bool mixing = true);
    void merging(const Eigen::RowVectorXd& z);
    Eigen::MatrixXd pred(double T);

    int N;  // 필터 수

    std::vector<std::shared_ptr<ExtendedKalmanFilter>> filters;
    Eigen::RowVectorXd mu;  // 모드 확률 벡터
    Eigen::MatrixXd M;   // 전이 확률 행렬
    Eigen::MatrixXd mM;
    Eigen::MatrixXd omega;
    Eigen::RowVectorXd likelihood;

    Eigen::RowVectorXd x;
    Eigen::MatrixXd P;

    std::vector<Eigen::RowVectorXd> xs;  // 혼합된 상태 벡터
    std::vector<Eigen::MatrixXd> Ps;  // 혼합된 공분산 행렬
private:

};

#endif // STATE_FILTERS_HPP
