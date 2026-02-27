#pragma once

#include "NexDynIPC/Physics/Geometry/Shape.h"
#include <vector>

namespace NexDynIPC::Physics {

class MeshShape : public Shape {
public:
    MeshShape(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces);

    double sdf(const Eigen::Vector3d& p) const override;
    Eigen::Vector3d gradient(const Eigen::Vector3d& p) const override;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> computeAABB() const override;

    const Eigen::MatrixXd& vertices() const { return V_; }
    const Eigen::MatrixXi& faces() const { return F_; }

private:
    Eigen::MatrixXd V_;
    Eigen::MatrixXi F_;
    Eigen::Vector3d aabb_min_;
    Eigen::Vector3d aabb_max_;
};

} // namespace NexDynIPC::Physics
