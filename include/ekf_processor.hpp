#pragma once

#include "ekf_interfaces.hpp"
#include "oxts_types.hpp"
#include "csv_logger.hpp"
#include <vector>
#include <optional>
#include <memory>
#include <Eigen/Dense>

namespace ekf {

class EKFProcessor {
public:
    explicit EKFProcessor(IEKFPtr ekf, const std::string& resultCSVPath = "");

    Eigen::VectorXd process(const utils::OXTSData& data);
    const std::vector<Eigen::VectorXd>& results() const;

private:
    IEKFPtr ekf_;
    std::vector<Eigen::VectorXd> trajectory_;
    std::optional<utils::OXTSData> last_;
    std::string resultCSVPath_;
    std::optional<utils::Logger> logger_;
};

} // namespace ekf
