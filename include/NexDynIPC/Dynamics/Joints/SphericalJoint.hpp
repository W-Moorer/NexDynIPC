#pragma once

#include "NexDynIPC/Dynamics/Joints/Joint.hpp"
#include <Eigen/Geometry>

namespace NexDynIPC::Dynamics {

class SphericalJoint : public Joint {
public:
    SphericalJoint(int bodyA, int bodyB, 
                   const Eigen::Vector3d& anchorA, const Eigen::Vector3d& anchorB);

    int dim() const override { return 3; } // 3 pos
    
    int getBodyAId() const { return bodyA_id_; }
    int getBodyBId() const { return bodyB_id_; }

    void computeC(const Eigen::VectorXd& x, Eigen::VectorXd& C) const override;
    void computeJ(const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& J) const override;

    // Must be called before optimization step to map body IDs to x indices and set reference orientation
    void updateState(int idxA, int idxB, 
                     const Eigen::Vector3d& pA, const Eigen::Quaterniond& qA, 
                     const Eigen::Vector3d& pB, const Eigen::Quaterniond& qB);

private:
    int bodyA_id_;
    int bodyB_id_;
    
    // Global indices in x vector (-1 if static)
    int global_idx_A_ = -1;
    int global_idx_B_ = -1;

    // Reference/Static State
    Eigen::Vector3d pA_ref_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d pB_ref_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond qA_ref_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond qB_ref_ = Eigen::Quaterniond::Identity();

    Eigen::Vector3d anchorA_; // Local
    Eigen::Vector3d anchorB_; // Local
};

} // namespace NexDynIPC::Dynamics
