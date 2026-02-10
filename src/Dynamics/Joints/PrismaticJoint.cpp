#include "NexDynIPC/Dynamics/Joints/PrismaticJoint.hpp"
#include <iostream>

namespace NexDynIPC::Dynamics {

// Helper to construct orthonormal basis
static void computeBasis(const Eigen::Vector3d& axis, Eigen::Vector3d& u, Eigen::Vector3d& v) {
    Eigen::Vector3d a = axis.normalized();
    if (std::abs(a.x()) < 0.8) {
        u = a.cross(Eigen::Vector3d::UnitX()).normalized();
    } else {
        u = a.cross(Eigen::Vector3d::UnitY()).normalized();
    }
    v = a.cross(u).normalized();
}

PrismaticJoint::PrismaticJoint(int bodyA, int bodyB, 
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

void PrismaticJoint::updateState(int idxA, int idxB, 
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

void PrismaticJoint::computeC(const Eigen::VectorXd& x, Eigen::VectorXd& C) const {
    if (C.size() != dim()) C.resize(dim());

    Eigen::Vector3d pA, pB;
    Eigen::Quaterniond qA, qB;

    if (global_idx_A_ >= 0) {
        pA = x.segment<3>(global_idx_A_);
        Eigen::Vector3d thetaA = x.segment<3>(global_idx_A_ + 3);
        qA = updateRotation(qA_ref_, thetaA);
    } else {
        pA = pA_ref_;
        qA = qA_ref_;
    }

    if (global_idx_B_ >= 0) {
        pB = x.segment<3>(global_idx_B_);
        Eigen::Vector3d thetaB = x.segment<3>(global_idx_B_ + 3);
        qB = updateRotation(qB_ref_, thetaB);
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

    // 2. Rotation constraints (3): Relative rotation fixed.
    // Ideally qB * qA^-1 = qB0 * qA0^-1 (constant relative orientation).
    // Or simply align frames completely if set up that way.
    // Let's implement:
    // a) nA aligned with nB (2 constraints)
    // b) uA aligned with uB (1 constraint, no relative roll)
    
    Eigen::Vector3d nB_world = qB * axisB_;
    Eigen::Vector3d uB_world = qB * uB_;
    Eigen::Vector3d vB_world = qB * vB_;

    // We use cross product for alignment
    // nA x nB = 0 (2 constraints). But we need to be careful with projection.
    // Easier: dot products.
    // nA . uB = 0
    // nA . vB = 0
    // uA . vB = 0 (restrict roll) or uA . uB = 1
    
    // Let's stick to the plan formulation:
    // nA = nB => nA x nB = 0 (2 constraints)
    // Relative rotation fixed => uA . uB = 1 (or uA x uB = 0)
    
    // To match dim=3 rotation constraints, we essentially say Frame A and Frame B are locked in rotation.
    // Standard Prismatic only allows translation. So R_A = R_B (if initially aligned).
    // If not initially aligned, R_B = R_A * R_rel_const.
    // C_rot = logs(R_B * R_rel_const^T * R_A^T) = 0?
    
    // Simplified Implementation:
    // 2 constraints: nA x nB = 0
    // 1 constraint: uA . uB = 1 (or uA . vB = 0 if initially aligned that way)
    
    // Let's use 3 constraints from: nA x nB = 0 (2 DOFs) and uA x uB = 0 (1 DOF overlapping).
    // Actually, simply:
    // C(2) = nA_world . uB_world (should be 0)
    // C(3) = nA_world . vB_world (should be 0)
    // C(4) = uA_world . vB_world (should be 0, prevents roll)
    
    // NOTE: This assumes initially nA || nB and uA || uB. 
    // If user setup has them rotated, we should compute relative rotation at start.
    // But for now, let's assume standard definitions where frames are aligned or we use dot products.
    
    C(2) = nA_world.dot(uB_world);
    C(3) = nA_world.dot(vB_world);
    C(4) = uA_world.dot(vB_world);
}

void PrismaticJoint::computeJ(const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& J) const {
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

    // --- Derivatives ---
    
    // Helper lambda for dot product derivative
    // d(a . b)/dx = da/dx . b + a . db/dx
    auto add_dot_grad = [&](int row, Eigen::Vector3d a, Eigen::Vector3d b, 
                            bool a_is_pos, bool b_is_pos, // true if vector is position diff, false if vector
                            bool a_is_pA, bool a_is_pB, // if a is pos diff
                            const Eigen::Matrix3d& da_dthetaA, const Eigen::Matrix3d& da_dthetaB,
                            const Eigen::Matrix3d& db_dthetaA, const Eigen::Matrix3d& db_dthetaB) {
        
        // dp
        if (a_is_pos) {
             if (global_idx_A_ >= 0) {
                 triplets.emplace_back(row, global_idx_A_ + 0, -1.0 * b.x());
                 triplets.emplace_back(row, global_idx_A_ + 1, -1.0 * b.y());
                 triplets.emplace_back(row, global_idx_A_ + 2, -1.0 * b.z());
             }
             if (global_idx_B_ >= 0) {
                 triplets.emplace_back(row, global_idx_B_ + 0, 1.0 * b.x());
                 triplets.emplace_back(row, global_idx_B_ + 1, 1.0 * b.y());
                 triplets.emplace_back(row, global_idx_B_ + 2, 1.0 * b.z());
             }
        }
        
        // dthetaA
        if (global_idx_A_ >= 0) {
            Eigen::Vector3d grad = Eigen::Vector3d::Zero();
            if (a_is_pos) grad += da_dthetaA.transpose() * b; // da/dthetaA is actually for rA part
            else grad += da_dthetaA.transpose() * b;
            
            if (!a_is_pos) grad += db_dthetaA.transpose() * a; // if b depends on A (unlikely unless b is also on A)
            
            for(int i=0; i<3; ++i) triplets.emplace_back(row, global_idx_A_ + 3 + i, grad(i));
        }

        // dthetaB
        if (global_idx_B_ >= 0) {
            Eigen::Vector3d grad = Eigen::Vector3d::Zero();
            if (a_is_pos) grad += da_dthetaB.transpose() * b; 
            
            if (!a_is_pos) grad += db_dthetaB.transpose() * a;
            else grad += db_dthetaB.transpose() * a; // b depends on B

             for(int i=0; i<3; ++i) triplets.emplace_back(row, global_idx_B_ + 3 + i, grad(i));
        }
    };
    
    // Matrices
    Eigen::Matrix3d drA = -crossMat(rA_world);
    Eigen::Matrix3d drB = crossMat(rB_world); // for d_pos part: pB+rB - ... so +drB
    
    Eigen::Matrix3d d_dpos_dthetaA = -drA; // correction: d(-rA)/dthetaA = -(-[rA]x) = [rA]x? No. 
    // d(pA+rA)/dthetaA = [nA]x? No. d(Rq v)/dtheta = -[Rq v]x.
    // d_pos = ... - (pA + qA*anchorA)
    // d(d_pos)/dthetaA = - (-[rA_world]x) = [rA_world]x = -drA implies drA definition was -[...]
    // Let's stick to standard: d(R v)/dtheta = -[Rv]x.
    // So d(-rA)/dthetaA = - (-[rA]x) = [rA]x.
    // d(rB)/dthetaB = -[rB]x.
    
    Eigen::Matrix3d dnA = -crossMat(nA_world);
    Eigen::Matrix3d duA = -crossMat(uA_world);
    Eigen::Matrix3d dvA = -crossMat(vA_world);
    
    Eigen::Matrix3d dnB = -crossMat(nB_world);
    Eigen::Matrix3d duB = -crossMat(uB_world);
    Eigen::Matrix3d dvB = -crossMat(vB_world);
    
    // C(0) = d_pos . uA
    // grad A: d(d_pos)/dA . uA + d_pos . d(uA)/dA
    //   d(d_pos)/dpA = -I
    //   d(d_pos)/dthetaA = [rA]x
    //   d(uA)/dthetaA = -[uA]x
    if (global_idx_A_ >= 0) {
        // dpA
        triplets.emplace_back(0, global_idx_A_ + 0, -uA_world.x());
        triplets.emplace_back(0, global_idx_A_ + 1, -uA_world.y());
        triplets.emplace_back(0, global_idx_A_ + 2, -uA_world.z());
        
        // dthetaA: ([rA]x)^T * uA + (-[uA]x)^T * d_pos
        // = (uA x rA) + (uA x d_pos)
        Eigen::Vector3d g = uA_world.cross(rA_world) + uA_world.cross(d_pos);
        for(int i=0; i<3; ++i) triplets.emplace_back(0, global_idx_A_ + 3 + i, g(i));
    }
    if (global_idx_B_ >= 0) {
        // dpB
        triplets.emplace_back(0, global_idx_B_ + 0, uA_world.x());
        triplets.emplace_back(0, global_idx_B_ + 1, uA_world.y());
        triplets.emplace_back(0, global_idx_B_ + 2, uA_world.z());

        // dthetaB: (-[rB]x)^T * uA = uA x (-rB) = rB x uA
        Eigen::Vector3d g = rB_world.cross(uA_world); // wait. d(d_pos)/dthetaB = -[rB]x. Transpose is [rB]x. [rB]x * uA = rB x uA.
        // Correction: d(Rq v)/dq * dq = (-[Rv]x) * theta.
        // So Jacobian block is -[Rv]x.
        // Transpose logic in previous dot helper was: grad = J^T * b. Here we want J directly.
        // C = a.b. dC/dx = (da/dx)^T b + (db/dx)^T a.
        // da/dx is matrix.
        // term 1: ( -[rB]x )^T uA = [rB]x uA = rB x uA. Correct.
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
        // d(nA)/dthetaA . uB = (-[nA]x nA_world)^T uB = (nA x uB) ??? 
        // No. d(nA)/dthetaA = -[nA]x.
        // (-[nA]x)^T uB = [nA]x uB = nA x uB.
        Eigen::Vector3d g = nA_world.cross(uB_world);
        for(int i=0; i<3; ++i) triplets.emplace_back(2, global_idx_A_ + 3 + i, g(i));
    }
    if (global_idx_B_ >= 0) {
        // nA . d(uB)/dthetaB
        // d(uB)/dthetaB = -[uB]x
        // (-[uB]x)^T nA = [uB]x nA = uB x nA.
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

    // C(4) = uA . vB
    if (global_idx_A_ >= 0) {
        Eigen::Vector3d g = uA_world.cross(vB_world);
        for(int i=0; i<3; ++i) triplets.emplace_back(4, global_idx_A_ + 3 + i, g(i));
    }
    if (global_idx_B_ >= 0) {
        Eigen::Vector3d g = vB_world.cross(uA_world);
        for(int i=0; i<3; ++i) triplets.emplace_back(4, global_idx_B_ + 3 + i, g(i));
    }

    J.setFromTriplets(triplets.begin(), triplets.end());
}

} // namespace NexDynIPC::Dynamics
