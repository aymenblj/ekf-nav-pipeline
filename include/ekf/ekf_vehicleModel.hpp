#pragma once

#include "ekf/ekf_interface.hpp"
#include <Eigen/Dense>
#include <cmath>

namespace ekf {

/**
 * @class VehicleEKF
 * @brief Extended Kalman Filter (EKF) for a simple vehicle model.
 * 
 * State vector: [latitude, longitude, velocity, yaw, forward acceleration, yaw rate]
 */
class VehicleEKF : public IEKF {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @struct NoiseParams
     * @brief Structure holding all tunable noise parameters for process and measurement models.
     */
    struct NoiseParams {
        double lat_var = 1e-6;       ///< Process noise variance for latitude (deg^2)
        double lon_var = 1e-6;       ///< Process noise variance for longitude (deg^2)
        double v_var   = 0.1;        ///< Process noise variance for velocity (m^2/s^2)
        double yaw_var = 0.001;      ///< Process noise variance for yaw (rad^2)
        double af_var  = 0.1;        ///< Process noise variance for acceleration (m^2/s^4)
        double wz_var  = 0.001;      ///< Process noise variance for yaw rate (rad^2/s^2)

        double gnss_var     = 2e-6;   ///< GNSS measurement noise variance (deg^2)
        double vel_var      = 0.04;   ///< Velocity measurement noise variance (m^2/s^2)
        double yaw_meas_var = 0.0025; ///< Yaw measurement noise variance (rad^2)
        double af_meas_var  = 0.1;    ///< Acceleration measurement noise variance (m^2/s^4)
        double wz_meas_var  = 0.001;  ///< Yaw rate measurement noise variance (rad^2/s^2)
    };

    /**
     * @brief Construct a new VehicleEKF.
     */
    VehicleEKF();

    /**
     * @brief Set the filter and measurement noise parameters.
     * @param np Noise parameter struct.
     */
    void setNoiseParams(const NoiseParams& np);

    // IEKF interface implementation:
    /**
     * @brief Initialize the EKF state and covariance.
     * @param x0 Initial state vector.
     * @param P0 Initial covariance matrix.
     */
    void initialize(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0) override;

    /**
     * @brief Predict step of the EKF.
     * @param dt        Time step [s].
     * @param accel_in  Forward acceleration input [m/s^2].
     * @param gyro_z_in Yaw rate input [rad/s].
     */
    void predict(double dt, double accel_in, double gyro_z_in) override;

    /**
     * @brief GNSS position update.
     * @param gnss_pos GNSS position [lat, lon] in degrees.
     * @param R        GNSS measurement noise covariance (2x2).
     */
    void updateGNSS(const Eigen::Vector2d& gnss_pos, const Eigen::Matrix2d& R) override;

    /**
     * @brief Velocity measurement update.
     * @param v_meas Measured velocity [m/s].
     * @param R_v    Velocity measurement noise variance.
     */
    void updateVelocity(double v_meas, double R_v) override;

    /**
     * @brief Yaw measurement update.
     * @param yaw_meas Measured yaw [rad].
     * @param R_yaw    Yaw measurement noise variance.
     */
    void updateYaw(double yaw_meas, double R_yaw) override;

    /**
     * @brief Forward acceleration measurement update.
     * @param af_meas Measured forward acceleration [m/s^2].
     * @param R_af    Forward acceleration measurement noise variance.
     */
    void updateAccel(double af_meas, double R_af) override;

    /**
     * @brief Yaw rate measurement update.
     * @param wz_meas Measured yaw rate [rad/s].
     * @param R_wz    Yaw rate measurement noise variance.
     */
    void updateYawRate(double wz_meas, double R_wz) override;

    /**
     * @brief Get the filter state vector.
     * @return State vector.
     */
    Eigen::VectorXd getState() const override { return x_; }

    /**
     * @brief Get the state covariance matrix.
     * @return Covariance matrix.
     */
    Eigen::MatrixXd getCovariance() const override { return P_; }

private:
    Eigen::VectorXd x_;    ///< Filter state [lat, lon, v, yaw, af, wz]
    Eigen::MatrixXd P_;    ///< State covariance
    Eigen::MatrixXd Q_;    ///< Process noise covariance
    NoiseParams noise_;    ///< Noise parameter struct

    static constexpr double kLatToMeters = 111320.0;  ///< Approximate meters per degree latitude

    /**
     * @brief Compute meters per degree longitude at a given latitude.
     * @param lon Longitude (unused).
     * @param lat Latitude in degrees.
     * @return Meters per degree longitude.
     */
    static double lonToMeters(double lon, double lat) {
        constexpr double kPi = 3.14159265358979323846;
        return 111320.0 * std::cos(lat * kPi / 180.0);
    }

    /**
     * @brief EKF process model for vehicle state propagation.
     * @param x  State vector.
     * @param dt Time step.
     * @return Predicted state vector.
     */
    Eigen::VectorXd processModel(const Eigen::VectorXd& x, double dt) const;

    /**
     * @brief Jacobian of the process model.
     * @param x  State vector.
     * @param dt Time step.
     * @return Process model Jacobian matrix.
     */
    Eigen::MatrixXd processJacobian(const Eigen::VectorXd& x, double dt) const;

    /**
     * @brief GNSS measurement model.
     * @param x State vector.
     * @return GNSS position measurement [lat, lon].
     */
    Eigen::Vector2d gnssMeasurement(const Eigen::VectorXd& x) const;

    /**
     * @brief Jacobian of the GNSS measurement model.
     * @return GNSS measurement Jacobian (2x6).
     */
    Eigen::Matrix<double,2,6> gnssJacobian() const;

    /**
     * @brief Generic measurement update step.
     * @param residual Measurement residual vector.
     * @param H        Measurement Jacobian.
     * @param R        Measurement noise covariance.
     */
    void genericUpdate(const Eigen::Ref<const Eigen::VectorXd>& residual,
                       const Eigen::Ref<const Eigen::MatrixXd>& H,
                       const Eigen::Ref<const Eigen::MatrixXd>& R);

    bool is_initialized_ = false;
};

} // namespace ekf