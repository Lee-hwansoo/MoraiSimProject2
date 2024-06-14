#include "state_filters.hpp"
#include <iostream>
#include <stdexcept>

// ExtendedKalmanFilter Class Implementation
ExtendedKalmanFilter::ExtendedKalmanFilter(int x_dim, int z_dim, std::shared_ptr<StateModel> model)
    : x_dim(x_dim),
      z_dim(z_dim),
      model(model),
      Q(Eigen::MatrixXd::Identity(x_dim, x_dim)),
      R(Eigen::MatrixXd::Identity(z_dim, z_dim)),
      B(Eigen::MatrixXd::Zero(x_dim, 0)),
      P(Eigen::MatrixXd::Identity(x_dim, x_dim)),
      F([this](const Eigen::RowVectorXd& x) -> Eigen::RowVectorXd { return this->model->step(x); }),
      H([this](const Eigen::RowVectorXd& x) -> Eigen::RowVectorXd { return this->model->H(x); }),
      JA([this](const Eigen::RowVectorXd& x) -> Eigen::MatrixXd { return this->model->JA(x); }),
      JH([this](const Eigen::RowVectorXd& x) -> Eigen::MatrixXd { return this->model->JH(x); }),
      x(Eigen::RowVectorXd::Zero(x_dim)),
      y(Eigen::RowVectorXd::Zero(z_dim)),
      K(Eigen::MatrixXd::Zero(x_dim, z_dim)),
      S(Eigen::MatrixXd::Zero(z_dim, z_dim)),
      _I(Eigen::MatrixXd::Identity(x_dim, x_dim)),
      SI(Eigen::MatrixXd::Zero(z_dim, z_dim)),
      likelihood(1.0) {}

Eigen::MatrixXd ExtendedKalmanFilter::inv(const Eigen::MatrixXd& mat) {
    return mat.inverse();
}

void ExtendedKalmanFilter::prediction(const Eigen::RowVectorXd& u,
                                      std::function<Eigen::MatrixXd(const Eigen::RowVectorXd&)> JA,
                                      std::function<Eigen::RowVectorXd(const Eigen::RowVectorXd&)> F,
                                      const Eigen::MatrixXd& Q) {

    Eigen::MatrixXd JA_ = JA ? JA(this->x) : (this->JA ? this->JA(this->x) : Eigen::MatrixXd::Identity(this->x_dim, this->x_dim));
    std::function<Eigen::RowVectorXd(const Eigen::RowVectorXd&)> F_ = F ? F : this->F;
    Eigen::MatrixXd Q_ = Q.size() == 0 ? this->Q : Q;

    this->x = F_(this->x);
    this->P = ((JA_ * this->P) * JA_.transpose()) + Q_;
}

void ExtendedKalmanFilter::correction(const Eigen::RowVectorXd& z,
                                      std::function<Eigen::MatrixXd(const Eigen::RowVectorXd&)> JH,
                                      std::function<Eigen::RowVectorXd(const Eigen::RowVectorXd&)> H,
                                      const Eigen::MatrixXd& R) {

    Eigen::MatrixXd JH_ = JH ? JH(this->x) : (this->JH ? this->JH(this->x) : Eigen::MatrixXd::Zero(this->z_dim, this->x_dim));
    std::function<Eigen::RowVectorXd(const Eigen::RowVectorXd&)> H_ = H ? H : this->H;
    Eigen::MatrixXd R_ = R.size() == 0 ? this->R : R;

    Eigen::RowVectorXd z_pred = H_(this->x);
    this->y = z - z_pred;

    Eigen::MatrixXd PHT = this->P * JH_.transpose();

    this->S = JH_ * PHT + R_;
    this->SI = this->inv(this->S);
    this->K = PHT * this->SI;

    this->x = this->x + (this->y * this->K.transpose());
    Eigen::MatrixXd I_KH = this->_I - this->K * JH_;
    this->P = ((I_KH * this->P) * I_KH.transpose()) + ((this->K * R_) * this->K.transpose());

    double detS = this->S.determinant();
    double exponent = -0.5 * (this->y.transpose() * this->y * this->SI ).trace();
    this->likelihood = (1.0 / std::sqrt(std::pow(2 * M_PI, this->y.size()) * detS)) * std::exp(exponent);
}

Eigen::MatrixXd ExtendedKalmanFilter::pred(double T) {
    double dt = this->model->get_dt();
    int num_steps = static_cast<int>(T / dt);

    Eigen::MatrixXd X(num_steps + 1, this->x.size());
    Eigen::RowVectorXd x_ = this->x;
    X.row(0) = x_;

    for (int i = 0; i < num_steps; ++i) {
        x_ = this->F(x_);
        X.row(i + 1) = x_;
    }

    return X;
}


// IMMFilter Class Implementation
IMMFilter::IMMFilter(const std::vector<std::shared_ptr<ExtendedKalmanFilter>>& filters, const Eigen::RowVectorXd& mu, const Eigen::MatrixXd& M)
    : N(filters.size()),
      filters(filters),
      mu(mu),
      M(M),
      mM(mu * M),
      omega(Eigen::MatrixXd::Constant(N, N, 1.0 / N)),
      likelihood(Eigen::RowVectorXd::Zero(N)){
        int max_state_dim = 0;
        int target_filter = 0;

        for (int i = 0; i < this->N; ++i) {
            if (filters[i]->x.size() > max_state_dim) {
                max_state_dim = filters[i]->x.size();
                target_filter = i;
            }
        }

        x = Eigen::RowVectorXd::Zero(max_state_dim);
        P = Eigen::MatrixXd::Zero(filters[target_filter]->P.rows(), filters[target_filter]->P.cols());
}

void IMMFilter::mixing() {
    int N = this->N;
    this->xs.clear();
    this->Ps.clear();

    for (int i = 0; i < N; ++i) {
        Eigen::RowVectorXd x_ = Eigen::RowVectorXd ::Zero(this->filters[i]->x.size());
        for (int j = 0; j < N; ++j) {
            x_.head(5) += this->filters[j]->x.head(5) * this->omega(j, i);
        }
        if (this->filters[i]->x.size() > 5 && x_.size() > 5) {
            x_(5) = this->filters[i]->x(5);
        }
        this->xs.push_back(x_);

        Eigen::MatrixXd P_ = Eigen::MatrixXd::Zero(this->filters[i]->P.rows(), this->filters[i]->P.cols());
        for (int j = 0; j < N; ++j) {
            Eigen::RowVectorXd y_ = this->filters[j]->x.head(5) - x_.head(5);

            // P_ = 5x5, P_.block = 5x5, omega = 2x2, y_ = 1x5, filsers[j]->P.block = 5x5
            // 5x1 * 1x5 = 5x5
            P_.block(0, 0, 5, 5) += this->omega(j, i) * ((y_.transpose() * y_) + this->filters[j]->P.block(0, 0, 5, 5));
        }
        if (this->filters[i]->x.size() > 5 && P_.size() > 25) {
            P_(5, 5) = this->filters[i]->P(5, 5);
        }
        this->Ps.push_back(P_);
    }
}

void IMMFilter::prediction(bool mixing) {
    int N = this->N;
    if (mixing) {
        this->mixing();
    }

    for (int i = 0; i < N; ++i) {
        if (mixing) {
            this->filters[i]->x = this->xs[i];
            this->filters[i]->P = this->Ps[i];
        }
        this->filters[i]->prediction();
    }
}

void IMMFilter::merging(const Eigen::RowVectorXd& z) {
    int N = this->N;

    for (int i = 0; i < N; ++i) {
        this->filters[i]->correction(z);
        this->likelihood(i) = this->filters[i]->likelihood;
    }

    this->mu = this->mM.array() * this->likelihood.array();
    this->mu /= this->mu.sum();
    this->mM = this->mu * this->M;

    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            this->omega(i, j) = (this->M(i, j) * this->mu(i)) / this->mM(j);
        }
    }

    this->x.setZero();
    for (int i = 0; i < N; ++i) {
        this->x.head(5) += this->filters[i]->x.head(5) * this->mu(i);
        if (this->filters[i]->x.size() > 5) {
            this->x(5) = this->filters[i]->x(5);
        }
    }

    this->P.setZero();
    for (int i = 0; i < N; ++i) {
        Eigen::RowVectorXd y_ = this->filters[i]->x.head(5) - this->x.head(5);
        this->P.block(0, 0, 5, 5) += this->mu(i) * ((y_.transpose() * y_) + this->filters[i]->P.block(0, 0, 5, 5));
        if (this->filters[i]->x.size() > 5) {
            this->P(5, 5) = std::pow(this->filters[i]->x(5) - this->x(5), 2) + this->filters[i]->P(5, 5);
        }
    }
}

Eigen::MatrixXd IMMFilter::pred(double T) {
    double dt = this->filters[0]->model->get_dt();
    int num_steps = static_cast<int>(T / dt);
    int N = this->N;

    Eigen::MatrixXd X(num_steps + 1, this->x.size());
    X.row(0) = this->x;

    Eigen::MatrixXd omega_ = this->omega;
    Eigen::RowVectorXd mu_ = this->mu;
    Eigen::MatrixXd mM_ = this->mM;
    Eigen::RowVectorXd likelihood_ = Eigen::RowVectorXd::Ones(N);
    std::vector<std::shared_ptr<ExtendedKalmanFilter>> filters_;
    std::vector<Eigen::RowVectorXd> xs_;
    std::vector<Eigen::MatrixXd> Ps_;

    for (const auto& filter : this->filters) {
        filters_.emplace_back(std::make_unique<ExtendedKalmanFilter>(*filter));
    }

    for (int i = 0; i < num_steps; ++i) {
        xs_.clear();
        Ps_.clear();
        for (int j = 0; j < N; ++j) {
            Eigen::RowVectorXd x_ = Eigen::RowVectorXd::Zero(filters_[j]->x.size());
            for (int k = 0; k < N; ++k) {
                x_.head(5) += filters_[k]->x.head(5) * omega_(k, j);
                if (filters_[k]->x.size() > 5 && x_.size() > 5){
                    x_(5) = filters_[k]->x(5);
                }
            }
            xs_.push_back(x_);

            Eigen::MatrixXd P_ = Eigen::MatrixXd::Zero(filters_[j]->P.rows(), filters_[j]->P.cols());
            for (int k = 0; k < N; ++k) {
                Eigen::RowVectorXd y_ = filters_[k]->x.head(5) - x_.head(5);
                P_.block(0, 0, 5, 5) += omega_(k, j) * ((y_.transpose() * y_) + filters_[k]->P.block(0, 0, 5, 5));
                if (filters_[k]->x.size() > 5 && P_.size() > 25){
                    P_(5, 5) = filters_[k]->P(5, 5);
                }
            }
            Ps_.push_back(P_);
        }

        for (int j = 0; j < N; ++j) {
            filters_[j]->x = xs_[j];
            filters_[j]->P = Ps_[j];
            filters_[j]->prediction();

            likelihood_[j] = filters_[j]->P(0, 0) + filters_[j]->P(1, 1);
        }

        Eigen::RowVectorXd y_ = Eigen::RowVectorXd::Zero(this->x.size());
        mu_ = mM_.array() * likelihood_.array();
        mu_ /= mu_.sum();
        mM_ = mu_ * this->M;

        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                omega_(i, j) = (this->M(i, j) * mu_(i)) / mM_(j);
            }
        }

        for (int i = 0; i < N; ++i) {
            y_.head(5) += filters_[i]->x.head(5) * mu_(i);
            if(filters_[i]->x.size() > 5 && y_.size() > 5){
                y_(5) = filters_[i]->x(5);
            }
        }
        X.row(i + 1) = y_;
    }

    return X;
}
