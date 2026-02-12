#include "NexDynIPC/Dynamics/Joints/CylindricalJoint.h"
#include <iostream>

namespace NexDynIPC::Dynamics {

static void computeBasis(const Eigen::Vector3d& axis, Eigen::Vector3d& u, Eigen::Vector3d& v) {
    Eigen::Vector3d a = axis.normalized();
    if (std::abs(a.x()) < 0.8) {
        u = a.cross(Eigen::Vector3d::UnitX()).normalized();
    } else {
        u = a.cross(Eigen::Vector3d::UnitY()).normalized();
    }
    v = a.cross(u).normalized();
}

CylindricalJoint::CylindricalJoint(int bodyA, int bodyB, 
                       const Eigen::Vector3d& anchorA, const Eigen::Vector3d& anchorB,
                       const Eigen::Vector3d& axisA, const Eigen::Vector3d& axisB)
    : bodyA_id_(bodyA), bodyB_id_(bodyB), 
      anchorA_(anchorA), anchorB_(anchorB), 
      axisA_(axisA.normalized()), axisB_(axisB.normalized()) 
{
    lambda_ = Eigen::VectorXd::Zero(dim());
    computeBasis(axisA_, uA_, vA_);
    computeBasis(axisB_, uB_, vB_);
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

void CylindricalJoint::computeC(const Eigen::VectorXd& x, Eigen::VectorXd& C) const {
    if (C.size() != dim()) C.resize(dim());

    Eigen::Vector3d pA, pB;
    Eigen::Quaterniond qA, qB;

    if (global_idx_A_ >= 0) {
        pA = x.segment<3>(global_idx_A_);
        qA = updateRotation(qA_ref_, x.segment<3>(global_idx_A_ + 3));
    } else {
        pA = pA_ref_;
        qA = qA_ref_;
    }

    if (global_idx_B_ >= 0) {
        pB = x.segment<3>(global_idx_B_);
        qB = updateRotation(qB_ref_, x.segment<3>(global_idx_B_ + 3));
    } else {
        pB = pB_ref_;
        qB = qB_ref_;
    }

    Eigen::Vector3d rA_world = qA * anchorA_;
    Eigen::Vector3d rB_world = qB * anchorB_;
    Eigen::Vector3d d_pos = (pB + rB_world) - (pA + rA_world);
    
    Eigen::Vector3d nA_world = qA * axisA_;
    Eigen::Vector3d uA_world = qA * uA_;
    Eigen::Vector3d vA_world = qA * vA_;
    
    // 1. Position constraints (2): No translation perp to nA
    C(0) = d_pos.dot(uA_world);
    C(1) = d_pos.dot(vA_world);

    // 2. Rotation constraints (2): Align nA and nB
    Eigen::Vector3d nB_world = qB * axisB_;
    Eigen::Vector3d uB_world = qB * uB_; // Used to project?
    // Actually we can just use nA_world cross nB_world = 0 (2 DOFs).
    // Or dots like before:
    // nA . uB = 0
    // nA . vB = 0
    // This constrains nA to be perp to uB and vB => nA || nB.
    
    Eigen::Vector3d vB_world = qB * vB_;
    
    C(2) = nA_world.dot(uB_world);
    C(3) = nA_world.dot(vB_world);
}

void CylindricalJoint::computeJ(const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& J) const {
    std::vector<Eigen::Triplet<double>> triplets;

    Eigen::Quaterniond qA, qB;
    if (global_idx_A_ >= 0) qA = updateRotation(qA_ref_, x.segment<3>(global_idx_A_ + 3));
    else qA = qA_ref_;
    if (global_idx_B_ >= 0) qB = updateRotation(qB_ref_, x.segment<3>(global_idx_B_ + 3));
    else qB = qB_ref_;

    Eigen::Vector3d rA_world = qA * anchorA_;
    Eigen::Vector3d rB_world = qB * anchorB_;
    Eigen::Vector3d d_pos = (global_idx_B_ >= 0 ? x.segment<3>(global_idx_B_) : pB_ref_) + rB_world 
                          - ((global_idx_A_ >= 0 ? x.segment<3>(global_idx_A_) : pA_ref_) + rA_world);
    
    Eigen::Vector3d nA_world = qA * axisA_;
    Eigen::Vector3d uA_world = qA * uA_;
    Eigen::Vector3d vA_world = qA * vA_;
    
    Eigen::Vector3d nB_world = qB * axisB_;
    Eigen::Vector3d uB_world = qB * uB_;
    Eigen::Vector3d vB_world = qB * vB_;

    // Reuse logic from Prismatic (Position part and Alignment part)
    
    // C(0) = d_pos . uA
    if (global_idx_A_ >= 0) {
        // dpA
        triplets.emplace_back(0, global_idx_A_ + 0, -uA_world.x());
        triplets.emplace_back(0, global_idx_A_ + 1, -uA_world.y());
        triplets.emplace_back(0, global_idx_A_ + 2, -uA_world.z());
        
        // dthetaA
        Eigen::Vector3d g = uA_world.cross(rA_world) + uA_world.cross(d_pos);
        for(int i=0; i<3; ++i) triplets.emplace_back(0, global_idx_A_ + 3 + i, g(i));
    }
    if (global_idx_B_ >= 0) {
        // dpB
        triplets.emplace_back(0, global_idx_B_ + 0, uA_world.x());
        triplets.emplace_back(0, global_idx_B_ + 1, uA_world.y());
        triplets.emplace_back(0, global_idx_B_ + 2, uA_world.z());

        // dthetaB
        Eigen::Vector3d g = rB_world.cross(uA_world);
        for(int i=0; i<3; ++i) triplets.emplace_back(0, global_idx_B_ + 3 + i, g(i));
    }

    // C(1) = d_pos . vA
    if (global_idx_A_ >= 0) {
        triplets.emplace_back(1, global_idx_A_ + 0, -vA_world.x());
        triplets.emplace_back(1, global_idx_A_ + 1, -vA_world.y());
        triplets.emplace_back(1, global_idx_A_ + 2, -vA_world.z());
        
        Eigen::Vector3d g = vA_world.cross(rA_world) + vA_world.cross(d_pos);
        for(int i=0; i<3; ++i) triplets.emplace_back(1, global_idx_A_ + 3 + i, g(i));
    }
    if (global_idx_B_ >= 0) {
        triplets.emplace_back(1, global_idx_B_ + 0, vA_world.x());
        triplets.emplace_back(1, global_idx_B_ + 1, vA_world.y());
        triplets.emplace_back(1, global_idx_B_ + 2, vA_world.z());

        Eigen::Vector3d g = rB_world.cross(vA_world);
        for(int i=0; i<3; ++i) triplets.emplace_back(1, global_idx_B_ + 3 + i, g(i));
    }

    // C(2) = nA . uB
    if (global_idx_A_ >= 0) {
        Eigen::Vector3d g = nA_world.cross(uB_world);
        for(int i=0; i<3; ++i) triplets.emplace_back(2, global_idx_A_ + 3 + i, g(i));
    }
    if (global_idx_B_ >= 0) {
        Eigen::Vector3d g = uB_world.cross(nA_world);
        for(int i=0; i<3; ++i) triplets.emplace_back(2, global_idx_B_ + 3 + i, g(i));
    }

    // C(3) = nA . vB
    if (global_idx_A_ >= 0) {
        Eigen::Vector3d g = nA_world.cross(vB_world);
        for(int i=0; i<3; ++i) triplets.emplace_back(3, global_idx_A_ + 3 + i, g(i));
    }
    if (global_idx_B_ >= 0) {
        Eigen::Vector3d g = vB_world.cross(nA_world);
        for(int i=0; i<3; ++i) triplets.emplace_back(3, global_idx_B_ + 3 + i, g(i));
    }

    J.setFromTriplets(triplets.begin(), triplets.end());
}

} // namespace NexDynIPC::Dynamics
