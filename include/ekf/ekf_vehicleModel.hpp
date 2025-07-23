#pragma once

#include "ekf/ekf_interface.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <map>
#include <string>

namespace ekf {

/**
 * @class VehicleEKF
 * @brief Extended Kalman Filter (EKF) for a simple vehicle model.
 * 
 * State vector: [latitude, longitude, velocity, yaw, forward acceleration, yaw rate]
 * 
 * Process noise and measurement noise parameters are set via std::map<std::string, double>.
 * The required keys are:
 *  Process noise: latitude, longitude, velocity, yaw, accel_fwd, yaw_rate
 *  Measurement noise: gnss, velocity, yaw, accel_fwd, yaw_rate
 */
class VehicleEKF : public IEKF {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VehicleEKF();

    /// Set process noise parameters (Q-diagonal) by name.
    void setProcessNoiseParams(const std::map<std::string, double>& q) override;

    /// Set measurement noise parameters (R-diagonal) by name.
    void setMeasurementNoiseParams(const std::map<std::string, double>& r) override;

    // IEKF interface implementation:
    void initialize(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0) override;
    void predict(double dt, double accel_in, double gyro_z_in) override;
    void updateGNSS(const Eigen::Vector2d& gnss_pos, const Eigen::Matrix2d& R) override;
    void updateVelocity(double v_meas, double R_v) override;
    void updateYaw(double yaw_meas, double R_yaw) override;
    void updateAccel(double af_meas, double R_af) override;
    void updateYawRate(double wz_meas, double R_wz) override;

    Eigen::VectorXd getState() const override { return x_; }
    Eigen::MatrixXd getCovariance() const override { return P_; }

    /// Get a copy of current process noise parameter map.
    std::map<std::string, double> getProcessNoiseParamMap() const;
    /// Get a copy of current measurement noise parameter map.
    std::map<std::string, double> getMeasurementNoiseParamMap() const;

private:
    Eigen::VectorXd x_;                  ///< Filter state [lat, lon, v, yaw, af, wz]
    Eigen::MatrixXd P_;                  ///< State covariance
    Eigen::MatrixXd Q_;                  ///< Process noise covariance

    // Noise parameters
    std::map<std::string, double> proc_noise_;
    std::map<std::string, double> meas_noise_;

    static constexpr double kLatToMeters = 111320.0;  ///< Approximate meters per degree latitude

    static double lonToMeters(double lon, double lat) {
        constexpr double kPi = 3.14159265358979323846;
        return 111320.0 * std::cos(lat * kPi / 180.0);
    }

    Eigen::VectorXd processModel(const Eigen::VectorXd& x, double dt) const;
    Eigen::MatrixXd processJacobian(const Eigen::VectorXd& x, double dt) const;
    Eigen::Vector2d gnssMeasurement(const Eigen::VectorXd& x) const;
    Eigen::Matrix<double,2,6> gnssJacobian() const;
    void genericUpdate(const Eigen::Ref<const Eigen::VectorXd>& residual,
                       const Eigen::Ref<const Eigen::MatrixXd>& H,
                       const Eigen::Ref<const Eigen::MatrixXd>& R);

    bool is_initialized_ = false;
};

} // namespace ekf