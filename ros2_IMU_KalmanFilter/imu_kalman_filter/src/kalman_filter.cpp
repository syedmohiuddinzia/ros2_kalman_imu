// kalman_filter.cpp
#include "kalman_filter.hpp"

KalmanFilter::KalmanFilter(int state_dim, int measurement_dim)
    : state_dim_(state_dim),
      measurement_dim_(measurement_dim),
      state_(Eigen::VectorXd::Zero(state_dim)),
      P_(Eigen::MatrixXd::Identity(state_dim, state_dim)),
      Q_(Eigen::MatrixXd::Identity(state_dim, state_dim) * 0.1),
      R_(Eigen::MatrixXd::Identity(measurement_dim, measurement_dim) * 0.1) {}

void KalmanFilter::predict(const Eigen::MatrixXd& F) {
    state_ = F * state_;
    P_ = F * P_ * F.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H) {
    Eigen::VectorXd y = z - H * state_;
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    
    state_ = state_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(state_dim_, state_dim_) - K * H) * P_;
}

Eigen::VectorXd KalmanFilter::getState() const {
    return state_;
}
