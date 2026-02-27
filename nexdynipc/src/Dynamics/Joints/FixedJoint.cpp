#include "NexDynIPC/Dynamics/Joints/FixedJoint.h"
#include "NexDynIPC/Dynamics/Joints/JointPrimitives.h"

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
    if (C.size() != dim()) {
        C.resize(dim());
    }

    if (global_idx_ < 0) {
        C.setZero();
        return;
    }

    JointPrimitives::BodyState state{global_idx_, target_position_, q_ref_};
    const auto eval = JointPrimitives::evaluateBody(state, x);

    const JointPrimitives::FixedPositionPrimitive fixed_pos{target_position_};
    const JointPrimitives::FixedOrientationPrimitive fixed_ori{target_orientation_};

    C.segment<3>(0) = fixed_pos.value(eval);
    C.segment<3>(3) = fixed_ori.value(state, x);
}

void FixedJoint::computeJ(const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& J) const {
    std::vector<Eigen::Triplet<double>> triplets;

    JointPrimitives::BodyState state{global_idx_, target_position_, q_ref_};
    const JointPrimitives::FixedPositionPrimitive fixed_pos{target_position_};
    const JointPrimitives::FixedOrientationPrimitive fixed_ori{target_orientation_};

    fixed_pos.addJacobianRows(0, state, triplets);
    fixed_ori.addJacobianRows(3, state, triplets);

    J.resize(dim(), x.size());
    J.setFromTriplets(triplets.begin(), triplets.end());
}

} // namespace NexDynIPC::Dynamics
