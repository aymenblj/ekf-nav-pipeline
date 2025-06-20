#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <optional>
#include <memory>
#include "utils/oxts_types.hpp"
#include "utils/csv_logger.hpp"
#include "ekf/ekf_interface.hpp"

namespace ekf {

/**
 * @brief High-level EKF processor that handles measurement ingestion, filtering, and logging.
 * 
 * Wraps an EKF implementation and manages the filter lifecycle and logging of results.
 */
class EKFProcessor {
public:
    /**
     * @brief Construct a new EKFProcessor.
     * @param ekf    Unique pointer to an EKF implementation.
     * @param logCSV Optional path to a CSV file for logging state estimates.
     */
    EKFProcessor(std::unique_ptr<IEKF> ekf, const std::string& logCSV);

    /**
     * @brief Process a new sensor data measurement.
     * 
     * Runs the EKF predict and update steps using the provided data.
     * @param data A single OXTSData measurement.
     * @return The updated state vector after processing.
     */
    Eigen::VectorXd process(const utils::OXTSData& data);

    /**
     * @brief Get the full trajectory of state estimates processed so far.
     * @return Vector of state estimate vectors for all processed steps.
     */
    const std::vector<Eigen::VectorXd>& results() const;

private:
    std::unique_ptr<IEKF> ekf_;                       ///< EKF filter implementation
    std::optional<utils::OXTSData> last_;             ///< Last processed data for dt calculation
    std::vector<Eigen::VectorXd> trajectory_;         ///< Trajectory of EKF state estimates
    std::string resultCSVPath_;                       ///< Path to logged CSV file
    std::optional<utils::Logger> logger_;             ///< Optional CSV logger
};

} // namespace ekf