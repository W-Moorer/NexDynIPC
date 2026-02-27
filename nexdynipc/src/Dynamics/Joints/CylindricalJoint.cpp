#include "NexDynIPC/Dynamics/Joints/CylindricalJoint.h"
#include "NexDynIPC/Dynamics/Joints/JointPrimitives.h"

namespace NexDynIPC::Dynamics {

CylindricalJoint::CylindricalJoint(int bodyA, int bodyB, 
                       const Eigen::Vector3d& anchorA, const Eigen::Vector3d& anchorB,
                       const Eigen::Vector3d& axisA, const Eigen::Vector3d& axisB)
    : bodyA_id_(bodyA), bodyB_id_(bodyB), 
      anchorA_(anchorA), anchorB_(anchorB), 
      axisA_(axisA.normalized()), axisB_(axisB.normalized()) 
{
    lambda_ = Eigen::VectorXd::Zero(dim());
    JointPrimitives::computeBasis(axisA_, uA_, vA_);
    JointPrimitives::computeBasis(axisB_, uB_, vB_);
}

void CylindricalJoint::updateState(int idxA, int idxB, 
                             const Eigen::Vector3d& pA, const Eigen::Quaterniond& qA, 
                             const Eigen::Vector3d& pB, const Eigen::Quaterniond& qB) {
    global_idx_A_ = idxA;
    global_idx_B_ = idxB;
    
    pA_ref_ = pA;
    qA_ref_ = qA;
    pB_ref_ = pB;
    qB_ref_ = qB;
}

void CylindricalJoint::computeC(const Eigen::VectorXd& x, Eigen::VectorXd& C) const {
    if (C.size() != dim()) {
        C.resize(dim());
    }

    JointPrimitives::BodyState stateA{global_idx_A_, pA_ref_, qA_ref_};
    JointPrimitives::BodyState stateB{global_idx_B_, pB_ref_, qB_ref_};
    const auto evalA = JointPrimitives::evaluateBody(stateA, x);
    const auto evalB = JointPrimitives::evaluateBody(stateB, x);

    const JointPrimitives::PointOnAxisPrimitive point_on_axis{anchorA_, anchorB_, uA_, vA_};
    const JointPrimitives::DirectionPrimitive direction{axisA_, uB_, vB_};

    C.segment<2>(0) = point_on_axis.value(evalA, evalB);
    C.segment<2>(2) = direction.value(evalA, evalB);
}

void CylindricalJoint::computeJ(const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& J) const {
    std::vector<Eigen::Triplet<double>> triplets;

    JointPrimitives::BodyState stateA{global_idx_A_, pA_ref_, qA_ref_};
    JointPrimitives::BodyState stateB{global_idx_B_, pB_ref_, qB_ref_};
    const auto evalA = JointPrimitives::evaluateBody(stateA, x);
    const auto evalB = JointPrimitives::evaluateBody(stateB, x);

    const JointPrimitives::PointOnAxisPrimitive point_on_axis{anchorA_, anchorB_, uA_, vA_};
    const JointPrimitives::DirectionPrimitive direction{axisA_, uB_, vB_};

    point_on_axis.addJacobianRows(0, stateA, stateB, evalA, evalB, triplets);
    direction.addJacobianRows(2, stateA, stateB, evalA, evalB, triplets);

    J.resize(dim(), x.size());
    J.setFromTriplets(triplets.begin(), triplets.end());
}

} // namespace NexDynIPC::Dynamics
