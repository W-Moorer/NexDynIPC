#pragma once

#include <Eigen/Core>
#include <vector>

namespace NexDynIPC::Physics {

class Shape {
public:
    virtual ~Shape() = default;

    // Compute Signed Distance Function (SDF) at point p
    virtual double sdf(const Eigen::Vector3d& p) const = 0;

    // Compute SDF gradient (normal) at point p
    virtual Eigen::Vector3d gradient(const Eigen::Vector3d& p) const = 0;

    // Compute Axis-Aligned Bounding Box (min, max)
    virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> computeAABB() const = 0;
};

} // namespace NexDynIPC::Physics
