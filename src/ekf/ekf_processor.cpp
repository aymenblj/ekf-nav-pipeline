#include "ekf/ekf_processor.hpp"
#include <stdexcept>
#include <cmath>
#include <limits>

namespace ekf {

namespace {
// Utility: Convert OXTSData to a 6D measurement vector for EKF update
Eigen::VectorXd convertToMeasurement(const utils::OXTSData& data) {
    Eigen::VectorXd z(6);
    z << data.lat, data.lon, std::hypot(data.ve, data.vn), data.yaw, data.af, data.wz;
    return z;
}
} // namespace

EKFProcessor::EKFProcessor(std::unique_ptr<IEKF> ekf, const std::string& logCSV)
    : ekf_(std::move(ekf)), resultCSVPath_(logCSV) 
{
    if (!resultCSVPath_.empty()) {
        logger_.emplace(resultCSVPath_, std::vector<std::string>{
            "timestamp", "lat", "lon", "v", "yaw", "af", "wz", "gnss_available"
        });
    }
}

Eigen::VectorXd EKFProcessor::process(const utils::OXTSData& data) {
    if (!ekf_) throw std::runtime_error("EKF pointer is null");
    if (!data.hasVelocity()) throw std::invalid_argument("Invalid OXTSData: missing velocity information.");

    auto measurement = convertToMeasurement(data);
    if (measurement.size() != 6) throw std::runtime_error("Measurement vector size mismatch");

    if (!last_) {
        Eigen::VectorXd x0(6);
        x0 << data.lat, data.lon, std::hypot(data.ve, data.vn), data.yaw, data.af, data.wz;
        Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(6, 6) * 1.0;
        ekf_->initialize(x0, P0);
    } else {
        double dt = data.timelapse - last_->timelapse;

        ekf_->predict(dt, data.af, data.wz);

        // EKF update steps: only update with available (non-NaN) measurements
        if (data.hasGPS()) {
            Eigen::Vector2d gnss_pos(data.lat, data.lon);
            Eigen::Matrix2d R_gnss = Eigen::Matrix2d::Identity() * 2.0e-6;
            ekf_->updateGNSS(gnss_pos, R_gnss);
        }

        double v_meas = std::hypot(data.ve, data.vn);
        if (!std::isnan(v_meas)) {
            double R_v = 0.04; // (m/s)^2
            ekf_->updateVelocity(v_meas, R_v);
        }

        if (!std::isnan(data.yaw)) {
            double R_yaw = 0.0025; // rad^2
            ekf_->updateYaw(data.yaw, R_yaw);
        }

        if (!std::isnan(data.af)) {
            double R_af = 0.1; // (m/s^2)^2
            ekf_->updateAccel(data.af, R_af);
        }

        if (!std::isnan(data.wz)) {
            double R_wz = 0.001; // (rad/s)^2
            ekf_->updateYawRate(data.wz, R_wz);
        }
    }

    const auto& state = ekf_->getState();
    trajectory_.push_back(state);

    if (logger_) {
        bool gnss_available = data.hasGPS();
        logger_->logRow(
            data.timestamp, state(0), state(1), state(2), state(3), state(4), state(5),
            gnss_available ? 1 : 0
        );
    }

    last_ = data;
    return state;
}

const std::vector<Eigen::VectorXd>& EKFProcessor::results() const {
    return trajectory_;
}

} // namespace ekf