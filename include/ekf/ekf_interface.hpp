#pragma once
#include <Eigen/Dense>

namespace ekf {

/**
 * @brief Interface for Extended Kalman Filter (EKF) implementations.
 * 
 * Defines the required methods for EKF-based filters to support state initialization,
 * prediction, and update using various measurement types.
 */
class IEKF {
public:
    virtual ~IEKF() = default;

    /**
     * @brief Initialize the filter state and covariance.
     * @param x0 Initial state vector.
     * @param P0 Initial covariance matrix.
     */
    virtual void initialize(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0) = 0;

    /**
     * @brief Predict the next state using a process model.
     * @param dt    Time step in seconds.
     * @param accel Longitudinal acceleration input (optional, may be ignored).
     * @param gyro_z Yaw rate input (optional, may be ignored).
     */
    virtual void predict(double dt, double accel, double gyro_z) = 0;

    /**
     * @brief Update the filter using GNSS (latitude, longitude) position.
     * @param gnss_pos 2D vector with measured latitude and longitude.
     * @param R        2x2 covariance of GNSS measurement.
     */
    virtual void updateGNSS(const Eigen::Vector2d& gnss_pos, const Eigen::Matrix2d& R) = 0;

    /**
     * @brief Update the filter using a velocity measurement.
     * @param v_meas Measured velocity (m/s).
     * @param R_v    Variance of the velocity measurement.
     */
    virtual void updateVelocity(double v_meas, double R_v) = 0;

    /**
     * @brief Update the filter using a yaw measurement.
     * @param yaw_meas Measured yaw (radians).
     * @param R_yaw    Variance of the yaw measurement.
     */
    virtual void updateYaw(double yaw_meas, double R_yaw) = 0;

    /**
     * @brief (Optional) Update the filter using a forward acceleration measurement.
     * @param af_meas Measured forward acceleration (m/s^2).
     * @param R_af    Variance of the acceleration measurement.
     */
    virtual void updateAccel(double af_meas, double R_af) { /* optional, default do nothing */ }

    /**
     * @brief (Optional) Update the filter using a yaw rate measurement.
     * @param wz_meas Measured yaw rate (rad/s).
     * @param R_wz    Variance of the yaw rate measurement.
     */
    virtual void updateYawRate(double wz_meas, double R_wz) { /* optional, default do nothing */ }

    /**
     * @brief Get the current state estimate vector.
     * @return State vector.
     */
    virtual Eigen::VectorXd getState() const = 0;

    /**
     * @brief Get the current state covariance matrix.
     * @return Covariance matrix.
     */
    virtual Eigen::MatrixXd getCovariance() const = 0;
};

} // namespace ekf