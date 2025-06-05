#pragma once

#include "ekf_interfaces.hpp"

namespace ekf {

class BasicEKF : public IEKF {
public:
    BasicEKF();
    void init(const Eigen::VectorXd&  measurement) override;
    void predict(double dt) override;
    void update(const Eigen::VectorXd&  measurement) override;
    Eigen::VectorXd state() const override;

private:
    Eigen::VectorXd x_;   // state
    Eigen::MatrixXd P_;   // covariance
    Eigen::MatrixXd F_;   // transition model
    Eigen::MatrixXd Q_;   // process noise
    Eigen::MatrixXd H_;   // measurement model
    Eigen::MatrixXd R_;   // measurement noise
    bool is_initialized_ = false;
};

} // namespace ekf
