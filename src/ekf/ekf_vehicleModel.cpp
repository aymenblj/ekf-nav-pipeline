#include "ekf/ekf_vehicleModel.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <utility>

namespace ekf {

namespace {
// ormalization to keep yaw within [-π, π].
inline double wrapAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}
}

VehicleEKF::VehicleEKF() 
    : x_(Eigen::VectorXd::Zero(6)),
      P_(Eigen::MatrixXd::Identity(6, 6)),
      Q_(Eigen::MatrixXd::Zero(6, 6)) 
{
    setNoiseParams(NoiseParams{});
}

void VehicleEKF::setNoiseParams(const NoiseParams& np) {
    noise_ = np;
    Q_.setZero();
    Q_(0,0) = noise_.lat_var;
    Q_(1,1) = noise_.lon_var;
    Q_(2,2) = noise_.v_var;
    Q_(3,3) = noise_.yaw_var;
    Q_(4,4) = noise_.af_var;
    Q_(5,5) = noise_.wz_var;
}

void VehicleEKF::initialize(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0) {
    x_ = x0;
    P_ = P0;
    is_initialized_ = true;
}

Eigen::VectorXd VehicleEKF::processModel(const Eigen::VectorXd& x, double dt) const {
    Eigen::VectorXd x_pred = x;
    const double lat = x(0), lon = x(1), v = x(2), yaw = x(3), af = x(4), wz = x(5);

    const double new_yaw = yaw + wz * dt;
    const double new_v = v + af * dt;

    const double dx = new_v * std::cos(new_yaw) * dt;
    const double dy = new_v * std::sin(new_yaw) * dt;
    const double dlat = dy / kLatToMeters;
    const double dlon = dx / lonToMeters(lon, lat);

    x_pred(0) += dlat;
    x_pred(1) += dlon;
    x_pred(2) = new_v;
    x_pred(3) = wrapAngle(new_yaw);
    // af and wz are random walks (remain same)
    // x_pred(4) = af, x_pred(5) = wz

    return x_pred;
}

Eigen::MatrixXd VehicleEKF::processJacobian(const Eigen::VectorXd& x, double dt) const {
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    const double lat = x(0), lon = x(1), v = x(2), yaw = x(3), af = x(4), wz = x(5);
    const double new_yaw = yaw + wz * dt;
    const double new_v = v + af * dt;
    const double meters_per_deg_lon = lonToMeters(lon, lat);

    F(0, 2) = (std::sin(new_yaw) * dt) / kLatToMeters;
    F(0, 3) = (new_v * std::cos(new_yaw) * dt) / kLatToMeters;
    F(0, 4) = (std::sin(new_yaw) * dt * dt) / kLatToMeters;
    F(0, 5) = (new_v * std::cos(new_yaw) * dt * dt) / kLatToMeters;

    F(1, 2) = (std::cos(new_yaw) * dt) / meters_per_deg_lon;
    F(1, 3) = (-new_v * std::sin(new_yaw) * dt) / meters_per_deg_lon;
    F(1, 4) = (std::cos(new_yaw) * dt * dt) / meters_per_deg_lon;
    F(1, 5) = (-new_v * std::sin(new_yaw) * dt * dt) / meters_per_deg_lon;

    F(2,4) = dt;
    F(3,5) = dt;
    return F;
}

void VehicleEKF::predict(double dt, double /*accel_in*/, double /*gyro_z_in*/) {
    if (!is_initialized_) return;
    // Optionally: treat accel_in, gyro_z_in as control inputs for af and wz process models
    // Here, we let measurement updates correct af and wz
    x_ = processModel(x_, dt);
    const auto F = processJacobian(x_, dt);
    P_ = F * P_ * F.transpose() + Q_;
}

Eigen::Vector2d VehicleEKF::gnssMeasurement(const Eigen::VectorXd& x) const {
    return x.head<2>();
}

Eigen::Matrix<double,2,6> VehicleEKF::gnssJacobian() const {
    Eigen::Matrix<double,2,6> H = Eigen::Matrix<double,2,6>::Zero();
    H(0,0) = 1;
    H(1,1) = 1;
    return H;
}

void VehicleEKF::genericUpdate(const Eigen::Ref<const Eigen::VectorXd>& residual,
                               const Eigen::Ref<const Eigen::MatrixXd>& H,
                               const Eigen::Ref<const Eigen::MatrixXd>& R)
{
    const Eigen::MatrixXd S = H * P_ * H.transpose() + R; // Innovation covariance
    Eigen::MatrixXd K = P_ * H.transpose();               // Kalman gain numerator
    // Solve S * X = K^T, so K = (S.ldlt().solve(K.transpose())).transpose()
    K = S.ldlt().solve(K.transpose()).transpose();

    x_.noalias() += K * residual;
    x_(3) = wrapAngle(x_(3));
    P_.noalias() -= K * H * P_;
}

void VehicleEKF::updateGNSS(const Eigen::Vector2d& gnss_pos, const Eigen::Matrix2d& R) {
    if (!is_initialized_) return;
    const auto z_pred = gnssMeasurement(x_);
    const auto y = gnss_pos - z_pred;
    const auto H = gnssJacobian();
    genericUpdate(y, H, R);
}

void VehicleEKF::updateVelocity(double v_meas, double R_v) {
    if (!is_initialized_) return;
    const double y = v_meas - x_(2);
    Eigen::Matrix<double,1,6> H = Eigen::Matrix<double,1,6>::Zero();
    H(0,2) = 1;
    Eigen::Matrix<double,1,1> R;
    R(0,0) = R_v;
    genericUpdate(Eigen::Matrix<double,1,1>::Constant(y), H, R);
}

void VehicleEKF::updateYaw(double yaw_meas, double R_yaw) {
    if (!is_initialized_) return;
    const double y = std::atan2(std::sin(yaw_meas - x_(3)), std::cos(yaw_meas - x_(3)));
    Eigen::Matrix<double,1,6> H = Eigen::Matrix<double,1,6>::Zero();
    H(0,3) = 1;
    Eigen::Matrix<double,1,1> R;
    R(0,0) = R_yaw;
    genericUpdate(Eigen::Matrix<double,1,1>::Constant(y), H, R);
}

void VehicleEKF::updateAccel(double af_meas, double R_af) {
    if (!is_initialized_) return;
    const double y = af_meas - x_(4);
    Eigen::Matrix<double,1,6> H = Eigen::Matrix<double,1,6>::Zero();
    H(0,4) = 1;
    Eigen::Matrix<double,1,1> R;
    R(0,0) = R_af;
    genericUpdate(Eigen::Matrix<double,1,1>::Constant(y), H, R);
}

void VehicleEKF::updateYawRate(double wz_meas, double R_wz) {
    if (!is_initialized_) return;
    const double y = wz_meas - x_(5);
    Eigen::Matrix<double,1,6> H = Eigen::Matrix<double,1,6>::Zero();
    H(0,5) = 1;
    Eigen::Matrix<double,1,1> R;
    R(0,0) = R_wz;
    genericUpdate(Eigen::Matrix<double,1,1>::Constant(y), H, R);
}

} // namespace ekf