// kalman_filter.hpp
#pragma once

#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter(int state_dim, int measurement_dim);

    void predict(const Eigen::MatrixXd& F);
    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H);

    Eigen::VectorXd getState() const;

private:
    int state_dim_;
    int measurement_dim_;
    
    Eigen::VectorXd state_;  // State vector
    Eigen::MatrixXd P_;      // Covariance matrix
    Eigen::MatrixXd Q_;      // Process noise covariance
    Eigen::MatrixXd R_;      // Measurement noise covariance
};
