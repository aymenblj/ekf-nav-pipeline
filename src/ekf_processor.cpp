#include "ekf_processor.hpp"
#include "oxts_types.hpp"
#include <stdexcept>

namespace ekf {

EKFProcessor::EKFProcessor(IEKFPtr ekf, const std::string& logCSV)
    : ekf_(std::move(ekf)), resultCSVPath_(logCSV) {
    if (!logCSV.empty()) {
        logger_.emplace(logCSV, std::vector<std::string>{
            "timestamp", "lat", "lon", "ve", "vn", "ax", "ay"
        });
    }
}

Eigen::VectorXd convertToMeasurement(const utils::OXTSData& data) {
    Eigen::VectorXd v(6);
    v << data.lat, data.lon, data.ve, data.vn, data.ax, data.ay;
    return v;
}

Eigen::VectorXd EKFProcessor::process(const utils::OXTSData& data) {
    if (!ekf_) {
        throw std::runtime_error("EKF pointer is null");
    }

    if (!data.hasGPS()) {
        throw std::invalid_argument("Invalid OXTSData: missing GPS information.");
    }

    if (!data.hasVelocity()) {
        throw std::invalid_argument("Invalid OXTSData: missing velocity information.");
    }

    auto measurement = convertToMeasurement(data);
    if (measurement.size() != 6) {
        throw std::runtime_error("Measurement vector size mismatch");
    }

    if (!last_) {
        ekf_->init(measurement);
    } else {
        double dt = 0.01; // use the given frequency: 10 HZ. TODO: parse and update later
        ekf_->predict(dt);
    }

    ekf_->update(measurement);
    const Eigen::VectorXd& state = ekf_->state();
    trajectory_.push_back(state);

    if (logger_) {
        logger_->logRow(data.timestamp, state(0), state(1), state(2), state(3), state(4), state(5));
    }

    last_ = data;

    return state;
}

const std::vector<Eigen::VectorXd>& EKFProcessor::results() const {
    return trajectory_;
}

} // namespace ekf
