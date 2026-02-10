#include "NexDynIPC/Dynamics/Joints/FixedJoint.hpp"
#include <iostream>

namespace NexDynIPC::Dynamics {

FixedJoint::FixedJoint(int bodyId, 
                       const Eigen::Vector3d& target_position,
                       const Eigen::Quaterniond& target_orientation)
    : body_id_(bodyId), 
      target_position_(target_position), 
      target_orientation_(target_orientation)
{
    lambda_ = Eigen::VectorXd::Zero(dim());
}

void FixedJoint::updateState(int globalIdx, 
                             const Eigen::Vector3d& position, 
                             const Eigen::Quaterniond& orientation) {
    global_idx_ = globalIdx;
    q_ref_ = orientation;
}

void FixedJoint::computeC(const Eigen::VectorXd& x, Eigen::VectorXd& C) const {
    if (C.size() != dim()) C.resize(dim());

    if (global_idx_ < 0) {
        C.setZero();
        return;
    }

    // Position constraint: p - p_target = 0
    Eigen::Vector3d p = x.segment<3>(global_idx_);
    C.segment<3>(0) = p - target_position_;

    // Rotation constraint: theta = 0 (no rotation allowed from target orientation)
    // Since x stores incremental rotation theta relative to q_ref_,
    // and we want q_final == target_orientation,
    // the constraint is simply theta = 0 (if q_ref_ == target_orientation at each step).
    // More precisely: we want the total rotation to bring body to target.
    // Since q_ref_ is the current orientation at start of step, 
    // and target is target_orientation_, the required incremental rotation is:
    // dq = target * q_ref_.inverse()
    // If target == q_ref_, then theta_target = 0.
    // General case: theta_target = rotation_vector(target * q_ref_.inverse())
    Eigen::Quaterniond dq_target = target_orientation_ * q_ref_.conjugate();
    Eigen::AngleAxisd aa(dq_target);
    Eigen::Vector3d theta_target = aa.angle() * aa.axis();
    
    Eigen::Vector3d theta = x.segment<3>(global_idx_ + 3);
    C.segment<3>(3) = theta - theta_target;
}

void FixedJoint::computeJ(const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& J) const {
    std::vector<Eigen::Triplet<double>> triplets;

    if (global_idx_ >= 0) {
        // dC_pos / dp = I (3x3)
        triplets.emplace_back(0, global_idx_ + 0, 1.0);
        triplets.emplace_back(1, global_idx_ + 1, 1.0);
        triplets.emplace_back(2, global_idx_ + 2, 1.0);

        // dC_rot / dtheta = I (3x3)
        triplets.emplace_back(3, global_idx_ + 3, 1.0);
        triplets.emplace_back(4, global_idx_ + 4, 1.0);
        triplets.emplace_back(5, global_idx_ + 5, 1.0);
    }

    J.setFromTriplets(triplets.begin(), triplets.end());
}

} // namespace NexDynIPC::Dynamics
