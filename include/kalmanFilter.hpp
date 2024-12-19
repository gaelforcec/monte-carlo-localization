#pragma once

#include "Eigen/Dense"

class KalmanFilter {
public:
    static const int STATE_SIZE = 6;
    static const int MEASUREMENT_SIZE = 3;

    KalmanFilter();
    
    void predict(double dt);
    void update(const Eigen::Vector3d& measurement);
    Eigen::VectorXd getState() const { return x_; }
    Eigen::MatrixXd getCovariance() const { return P_; }
    void setState(const Eigen::VectorXd& state) { x_ = state; }

private:
    Eigen::VectorXd x_;  // [x, y, theta, v_x, v_y, omega]
    Eigen::MatrixXd P_;  // State covariance
    Eigen::MatrixXd F_;  // State transition matrix
    Eigen::MatrixXd H_;  // Measurement matrix
    Eigen::MatrixXd Q_;  // Process noise
    Eigen::MatrixXd R_;  // Measurement noise
};