#pragma once

#include <Eigen/Core>

namespace NexDynIPC::Physics {

// Interface for Signed Distance Field
class SDF {
public:
    virtual ~SDF() = default;
    
    virtual double evaluate(const Eigen::Vector3d& p) const = 0;
    virtual Eigen::Vector3d gradient(const Eigen::Vector3d& p) const = 0;
};

} // namespace NexDynIPC::Physics
