#include "NexDynIPC/Physics/Geometry/AABB.h"

namespace NexDynIPC::Physics::Geometry {

AABB buildAABB(const Eigen::MatrixXd& vertices)
{
    if (vertices.rows() == 0) {
        return AABB();
    }

    Eigen::Vector3d min_v = vertices.row(0);
    Eigen::Vector3d max_v = vertices.row(0);

    for (int i = 1; i < vertices.rows(); ++i) {
        min_v = min_v.cwiseMin(vertices.row(i).transpose());
        max_v = max_v.cwiseMax(vertices.row(i).transpose());
    }

    return AABB(min_v, max_v);
}

AABB buildAABB(const Eigen::MatrixXd& vertices, double inflationRadius)
{
    return buildAABB(vertices).inflated(inflationRadius);
}

AABB buildAABB(const Eigen::MatrixXd& vertices, const std::vector<int>& indices)
{
    if (indices.empty()) {
        return AABB();
    }

    Eigen::Vector3d min_v = vertices.row(indices[0]);
    Eigen::Vector3d max_v = vertices.row(indices[0]);

    for (size_t i = 1; i < indices.size(); ++i) {
        min_v = min_v.cwiseMin(vertices.row(indices[i]).transpose());
        max_v = max_v.cwiseMax(vertices.row(indices[i]).transpose());
    }

    return AABB(min_v, max_v);
}

} // namespace NexDynIPC::Physics::Geometry
