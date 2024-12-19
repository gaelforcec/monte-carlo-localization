#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter() {
    // Initialize state
    x_ = Eigen::VectorXd::Zero(STATE_SIZE);
    
    // Initialize covariance matrices
    P_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    P_ *= 1.0;  // Initial uncertainty
    
    H_ = Eigen::MatrixXd::Zero(MEASUREMENT_SIZE, STATE_SIZE);
    H_.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3);
    
    Q_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    Q_.block<3,3>(0,0) *= 0.1;  // Position noise
    Q_.block<3,3>(3,3) *= 0.2;  // Velocity noise
    
    R_ = Eigen::MatrixXd::Identity(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
    R_ *= 0.2;  // Measurement noise
}

void KalmanFilter::predict(double dt) {
    // Update state transition matrix
    F_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    F_.block<3,3>(0,3) = dt * Eigen::MatrixXd::Identity(3,3);
    
    // Predict state and covariance
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::Vector3d& measurement) {
    // Single step update without loops
    Eigen::VectorXd y = measurement - H_ * x_;
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    
    x_ = x_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) - K * H_) * P_;
}