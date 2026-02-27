#include "NexDynIPC/Dynamics/Joints/SphericalJoint.h"
#include "NexDynIPC/Dynamics/Joints/JointPrimitives.h"

namespace NexDynIPC::Dynamics {

SphericalJoint::SphericalJoint(int bodyA, int bodyB, 
                       const Eigen::Vector3d& anchorA, const Eigen::Vector3d& anchorB)
    : bodyA_id_(bodyA), bodyB_id_(bodyB), 
      anchorA_(anchorA), anchorB_(anchorB) 
{
    lambda_ = Eigen::VectorXd::Zero(dim());
}

void SphericalJoint::updateState(int idxA, int idxB, 
                             const Eigen::Vector3d& pA, const Eigen::Quaterniond& qA, 
                             const Eigen::Vector3d& pB, const Eigen::Quaterniond& qB) {
    global_idx_A_ = idxA;
    global_idx_B_ = idxB;
    
    // Store reference state
    pA_ref_ = pA;
    qA_ref_ = qA;
    pB_ref_ = pB;
    qB_ref_ = qB;
}

void SphericalJoint::computeC(const Eigen::VectorXd& x, Eigen::VectorXd& C) const {
    if (C.size() != dim()) {
        C.resize(dim());
    }

    JointPrimitives::BodyState stateA{global_idx_A_, pA_ref_, qA_ref_};
    JointPrimitives::BodyState stateB{global_idx_B_, pB_ref_, qB_ref_};
    const auto evalA = JointPrimitives::evaluateBody(stateA, x);
    const auto evalB = JointPrimitives::evaluateBody(stateB, x);

    const JointPrimitives::PointPrimitive point{anchorA_, anchorB_};
    C.segment<3>(0) = point.value(evalA, evalB);
}

void SphericalJoint::computeJ(const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& J) const {
    std::vector<Eigen::Triplet<double>> triplets;

    JointPrimitives::BodyState stateA{global_idx_A_, pA_ref_, qA_ref_};
    JointPrimitives::BodyState stateB{global_idx_B_, pB_ref_, qB_ref_};
    const auto evalA = JointPrimitives::evaluateBody(stateA, x);
    const auto evalB = JointPrimitives::evaluateBody(stateB, x);

    const JointPrimitives::PointPrimitive point{anchorA_, anchorB_};
    point.addJacobianRows(0, stateA, stateB, evalA, evalB, triplets);

    J.resize(dim(), x.size());
    J.setFromTriplets(triplets.begin(), triplets.end());
}

} // namespace NexDynIPC::Dynamics
