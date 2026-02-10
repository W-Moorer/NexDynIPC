#include "NexDynIPC/Dynamics/Joints/RevoluteJoint.hpp"
#include <iostream>

namespace NexDynIPC::Dynamics {

RevoluteJoint::RevoluteJoint(int bodyA, int bodyB, 
                       const Eigen::Vector3d& anchorA, const Eigen::Vector3d& anchorB, 
                       const Eigen::Vector3d& axisA, const Eigen::Vector3d& axisB)
    : bodyA_id_(bodyA), bodyB_id_(bodyB), 
      anchorA_(anchorA), anchorB_(anchorB), 
      axisA_(axisA), axisB_(axisB) 
{
    lambda_ = Eigen::VectorXd::Zero(dim());
}

void RevoluteJoint::updateState(int idxA, int idxB, 
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

// Helper to compute q_new from theta increment
static Eigen::Quaterniond updateRotation(const Eigen::Quaterniond& q_ref, const Eigen::Vector3d& theta) {
    double angle = theta.norm();
    if (angle < 1e-10) return q_ref;
    Eigen::Vector3d axis = theta / angle;
    Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, axis));
    return (dq * q_ref).normalized();
}

static Eigen::Matrix3d crossMat(const Eigen::Vector3d& v) {
    Eigen::Matrix3d M;
    M << 0, -v.z(), v.y(),
         v.z(), 0, -v.x(),
         -v.y(), v.x(), 0;
    return M;
}

void RevoluteJoint::computeC(const Eigen::VectorXd& x, Eigen::VectorXd& C) const {
    if (C.size() != dim()) C.resize(dim());

    Eigen::Vector3d pA, pB;
    Eigen::Quaterniond qA, qB;

    // Body A State
    if (global_idx_A_ >= 0) {
        pA = x.segment<3>(global_idx_A_);
        Eigen::Vector3d thetaA = x.segment<3>(global_idx_A_ + 3);
        qA = updateRotation(qA_ref_, thetaA);
    } else {
        pA = pA_ref_;
        qA = qA_ref_;
    }

    // Body B State
    if (global_idx_B_ >= 0) {
        pB = x.segment<3>(global_idx_B_);
        Eigen::Vector3d thetaB = x.segment<3>(global_idx_B_ + 3);
        qB = updateRotation(qB_ref_, thetaB);
    } else {
        pB = pB_ref_;
        qB = qB_ref_;
    }

    // 1. Position Constraint: pA + RA * rA - (pB + RB * rB) = 0
    Eigen::Vector3d rA_world = qA * anchorA_;
    Eigen::Vector3d rB_world = qB * anchorB_;
    C.segment<3>(0) = (pA + rA_world) - (pB + rB_world);

    // 2. Alignment Constraint: (RA * nA) x (RB * nB) = 0
    Eigen::Vector3d nA_world = qA * axisA_;
    Eigen::Vector3d nB_world = qB * axisB_;
    C.segment<3>(3) = nA_world.cross(nB_world);
}

void RevoluteJoint::computeJ(const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& J) const {
    std::vector<Eigen::Triplet<double>> triplets;

    Eigen::Quaterniond qA, qB;
    
    // Body A State for Jacobian
    if (global_idx_A_ >= 0) {
        Eigen::Vector3d thetaA = x.segment<3>(global_idx_A_ + 3);
        qA = updateRotation(qA_ref_, thetaA);
    } else {
        qA = qA_ref_;
    }

    // Body B State for Jacobian
    if (global_idx_B_ >= 0) {
        Eigen::Vector3d thetaB = x.segment<3>(global_idx_B_ + 3);
        qB = updateRotation(qB_ref_, thetaB);
    } else {
        qB = qB_ref_;
    }

    Eigen::Vector3d rA_world = qA * anchorA_;
    Eigen::Vector3d rB_world = qB * anchorB_;
    Eigen::Vector3d nA_world = qA * axisA_;
    Eigen::Vector3d nB_world = qB * axisB_;

    // --- Position Constraint Derivatives ---
    
    // Body A Derivatives
    if (global_idx_A_ >= 0) {
        // dC_pos / dpA = I
        triplets.emplace_back(0, global_idx_A_ + 0, 1);
        triplets.emplace_back(1, global_idx_A_ + 1, 1);
        triplets.emplace_back(2, global_idx_A_ + 2, 1);

        // dC_pos / dthetaA = - [rA_world]_x
        Eigen::Matrix3d drA_dtheta = -crossMat(rA_world);
        for(int i=0; i<3; ++i) for(int j=0; j<3; ++j) triplets.emplace_back(i, global_idx_A_ + 3 + j, drA_dtheta(i,j));
    }

    // Body B Derivatives
    if (global_idx_B_ >= 0) {
        // dC_pos / dpB = -I
        triplets.emplace_back(0, global_idx_B_ + 0, -1);
        triplets.emplace_back(1, global_idx_B_ + 1, -1);
        triplets.emplace_back(2, global_idx_B_ + 2, -1);

        // dC_pos / dthetaB = [rB_world]_x
        Eigen::Matrix3d drB_dtheta = crossMat(rB_world); 
        for(int i=0; i<3; ++i) for(int j=0; j<3; ++j) triplets.emplace_back(i, global_idx_B_ + 3 + j, drB_dtheta(i,j));
    }

    // --- Alignment Constraint Derivatives ---
    
    // Body A Derivatives
    if (global_idx_A_ >= 0) {
        // dC_align / dthetaA = [nB_world]_x * [nA_world]_x
        Eigen::Matrix3d dalign_dthetaA = crossMat(nB_world) * crossMat(nA_world);
        for(int i=0; i<3; ++i) for(int j=0; j<3; ++j) triplets.emplace_back(3 + i, global_idx_A_ + 3 + j, dalign_dthetaA(i,j));
    }

    // Body B Derivatives
    if (global_idx_B_ >= 0) {
        // dC_align / dthetaB = - [nA_world]_x * [nB_world]_x
        Eigen::Matrix3d dalign_dthetaB = -crossMat(nA_world) * crossMat(nB_world);
        for(int i=0; i<3; ++i) for(int j=0; j<3; ++j) triplets.emplace_back(3 + i, global_idx_B_ + 3 + j, dalign_dthetaB(i,j));
    }

    J.setFromTriplets(triplets.begin(), triplets.end());
}

} // namespace NexDynIPC::Dynamics
