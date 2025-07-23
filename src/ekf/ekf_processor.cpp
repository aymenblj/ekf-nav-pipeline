#include "ekf/ekf_processor.hpp"
#include "ekf/ekf_vehicleModel.hpp"
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
        // If the implementation supports, dynamic_cast to access known keys
        std::map<std::string, double> meas_noise;
        if (auto* veh_ekf = dynamic_cast<VehicleEKF*>(ekf_.get())) {
            meas_noise = veh_ekf->getMeasurementNoiseParamMap();
        }

        if (data.hasGPS()) {
            Eigen::Vector2d gnss_pos(data.lat, data.lon);
            double r_val = meas_noise.count("gnss") ? meas_noise["gnss"] : 2e-6;
            Eigen::Matrix2d R_gnss = Eigen::Matrix2d::Identity() * r_val;
            ekf_->updateGNSS(gnss_pos, R_gnss);
        }

        double v_meas = std::hypot(data.ve, data.vn);
        if (!std::isnan(v_meas)) {
            double r_val = meas_noise.count("velocity") ? meas_noise["velocity"] : 0.04;
            ekf_->updateVelocity(v_meas, r_val);
        }

        if (!std::isnan(data.yaw)) {
            double r_val = meas_noise.count("yaw") ? meas_noise["yaw"] : 0.0025;
            ekf_->updateYaw(data.yaw, r_val);
        }

        if (!std::isnan(data.af)) {
            double r_val = meas_noise.count("accel_fwd") ? meas_noise["accel_fwd"] : 0.1;
            ekf_->updateAccel(data.af, r_val);
        }

        if (!std::isnan(data.wz)) {
            double r_val = meas_noise.count("yaw_rate") ? meas_noise["yaw_rate"] : 0.001;
            ekf_->updateYawRate(data.wz, r_val);
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