#include "NexDynIPC/Physics/Geometry/MeshShape.hpp"
#include <limits>

namespace NexDynIPC::Physics {

MeshShape::MeshShape(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces)
    : V_(vertices), F_(faces) {
    // Compute AABB
    aabb_min_ = V_.colwise().minCoeff();
    aabb_max_ = V_.colwise().maxCoeff();
}

double MeshShape::sdf(const Eigen::Vector3d& p) const {
    // TOD0: Implement actual mesh SDF query (e.g. using AABB tree or grid)
    // For now, return distance to AABB center as a placeholder
    Eigen::Vector3d center = (aabb_min_ + aabb_max_) * 0.5;
    return (p - center).norm() - 0.5 * (aabb_max_ - aabb_min_).norm();
}

Eigen::Vector3d MeshShape::gradient(const Eigen::Vector3d& p) const {
    // Placeholder gradient
    Eigen::Vector3d center = (aabb_min_ + aabb_max_) * 0.5;
    return (p - center).normalized();
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> MeshShape::computeAABB() const {
    return {aabb_min_, aabb_max_};
}

} // namespace NexDynIPC::Physics
