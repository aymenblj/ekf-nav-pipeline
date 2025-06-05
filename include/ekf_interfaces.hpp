// ekf_interfaces.hpp
#pragma once

#include <memory>
#include <Eigen/Dense>
#include <optional>

namespace ekf {

class IEKF {
public:
    virtual ~IEKF() = default;
    virtual void init(const Eigen::VectorXd&  data) = 0;
    virtual void predict(double dt) = 0;
    virtual void update(const Eigen::VectorXd&  data) = 0;
    virtual Eigen::VectorXd state() const = 0;
};

using IEKFPtr = std::unique_ptr<IEKF>;

} // namespace ekf
