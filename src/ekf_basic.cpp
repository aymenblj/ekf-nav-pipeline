// ekf_basic.cpp
#include "ekf_basic.hpp"

namespace ekf {

BasicEKF::BasicEKF() {
    x_ = Eigen::VectorXd::Zero(6); // [lat, lon, ve, vn, ax, ay]
    P_ = Eigen::MatrixXd::Identity(6, 6);
    F_ = Eigen::MatrixXd::Identity(6, 6);
    Q_ = 0.1 * Eigen::MatrixXd::Identity(6, 6);
    H_ = Eigen::MatrixXd::Zero(2, 6); // GPS measures lat, lon
    H_(0, 0) = 1.0;
    H_(1, 1) = 1.0;
    R_ = 0.5 * Eigen::MatrixXd::Identity(2, 2);
}

void BasicEKF::init(const Eigen::VectorXd& measurement) {
    x_(0) = measurement(0);  // lat
    x_(1) = measurement(1);  // lon
    is_initialized_ = true;
}

void BasicEKF::predict(double dt) {
    if (!is_initialized_) return;

    F_(0, 2) = dt;
    F_(1, 3) = dt;
    F_(2, 4) = dt;
    F_(3, 5) = dt;

    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void BasicEKF::update(const Eigen::VectorXd& measurement) {
    Eigen::Vector2d z = measurement.head<2>();  // [lat, lon]
    Eigen::Vector2d y = z - H_ * x_;
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H_) * P_;
}


Eigen::VectorXd BasicEKF::state() const {
    return x_;
}

} // namespace ekf
