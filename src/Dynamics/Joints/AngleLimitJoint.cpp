#include "NexDynIPC/Dynamics/Joints/AngleLimitJoint.h"
#include <algorithm>

namespace NexDynIPC::Dynamics {

namespace {
Eigen::Quaterniond updateRotation(const Eigen::Quaterniond& q_ref, const Eigen::Vector3d& theta) {
    const double angle = theta.norm();
    if (angle < 1e-10) {
        return q_ref;
    }
    const Eigen::Vector3d axis = theta / angle;
    const Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, axis));
    return (dq * q_ref).normalized();
}
} // namespace

AngleLimitJoint::AngleLimitJoint(int bodyA,
                                 int bodyB,
                                 const Eigen::Vector3d& axis,
                                 double min_angle_rad,
                                 double max_angle_rad)
    : bodyA_id_(bodyA)
    , bodyB_id_(bodyB)
    , axis_(axis.normalized())
    , min_angle_rad_(min_angle_rad)
    , max_angle_rad_(max_angle_rad) {
    lambda_ = Eigen::VectorXd::Zero(dim());
}

void AngleLimitJoint::updateState(int idxA,
                                  int idxB,
                                  const Eigen::Vector3d& pA,
                                  const Eigen::Quaterniond& qA,
                                  const Eigen::Vector3d& pB,
                                  const Eigen::Quaterniond& qB) {
    (void)pA;
    (void)pB;
    global_idx_A_ = idxA;
    global_idx_B_ = idxB;
    qA_ref_ = qA;
    qB_ref_ = qB;
}

double AngleLimitJoint::currentRelativeAngle(const Eigen::VectorXd& x) const {
    Eigen::Quaterniond qA = qA_ref_;
    Eigen::Quaterniond qB = qB_ref_;

    if (global_idx_A_ >= 0 && global_idx_A_ + 5 < x.size()) {
        const Eigen::Vector3d thetaA = x.segment<3>(global_idx_A_ + 3);
        qA = updateRotation(qA_ref_, thetaA);
    }
    if (global_idx_B_ >= 0 && global_idx_B_ + 5 < x.size()) {
        const Eigen::Vector3d thetaB = x.segment<3>(global_idx_B_ + 3);
        qB = updateRotation(qB_ref_, thetaB);
    }

    const Eigen::Quaterniond qRel = qA.conjugate() * qB;
    const Eigen::AngleAxisd aa(qRel);
    return aa.angle() * aa.axis().dot(axis_);
}

void AngleLimitJoint::computeC(const Eigen::VectorXd& x, Eigen::VectorXd& C) const {
    if (C.size() != dim()) {
        C.resize(dim());
    }
    const double angle = currentRelativeAngle(x);
    C(0) = min_angle_rad_ - angle;
    C(1) = angle - max_angle_rad_;
}

void AngleLimitJoint::computeJ(const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& J) const {
    std::vector<Eigen::Triplet<double>> triplets;
    J.resize(dim(), x.size());
    (void)x;

    if (global_idx_A_ >= 0) {
        for (int k = 0; k < 3; ++k) {
            triplets.emplace_back(0, global_idx_A_ + 3 + k, axis_[k]);
            triplets.emplace_back(1, global_idx_A_ + 3 + k, -axis_[k]);
        }
    }
    if (global_idx_B_ >= 0) {
        for (int k = 0; k < 3; ++k) {
            triplets.emplace_back(0, global_idx_B_ + 3 + k, -axis_[k]);
            triplets.emplace_back(1, global_idx_B_ + 3 + k, axis_[k]);
        }
    }

    J.setFromTriplets(triplets.begin(), triplets.end());
}

double AngleLimitJoint::value(const Eigen::VectorXd& x) const {
    Eigen::VectorXd g(dim());
    computeC(x, g);

    double energy = 0.0;
    for (int i = 0; i < dim(); ++i) {
        const double gp = std::max(g(i), 0.0);
        energy += lambda_(i) * gp + 0.5 * mu_ * gp * gp;
    }
    return energy;
}

void AngleLimitJoint::gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const {
    Eigen::VectorXd g(dim());
    computeC(x, g);

    Eigen::SparseMatrix<double> J(dim(), x.size());
    computeJ(x, J);

    Eigen::VectorXd eff = Eigen::VectorXd::Zero(dim());
    for (int i = 0; i < dim(); ++i) {
        if (g(i) > 0.0) {
            eff(i) = lambda_(i) + mu_ * g(i);
        }
    }

    grad += J.transpose() * eff;
}

void AngleLimitJoint::hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) const {
    Eigen::VectorXd g(dim());
    computeC(x, g);

    const bool active_min = g(0) > 0.0;
    const bool active_max = g(1) > 0.0;
    if (!active_min && !active_max) {
        return;
    }

    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    if (active_min) {
        H += mu_ * (axis_ * axis_.transpose());
    }
    if (active_max) {
        H += mu_ * (axis_ * axis_.transpose());
    }

    if (global_idx_A_ >= 0) {
        const int rA = global_idx_A_ + 3;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                triplets.emplace_back(rA + i, rA + j, H(i, j));
            }
        }
    }

    if (global_idx_B_ >= 0) {
        const int rB = global_idx_B_ + 3;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                triplets.emplace_back(rB + i, rB + j, H(i, j));
            }
        }
    }

    if (global_idx_A_ >= 0 && global_idx_B_ >= 0) {
        const int rA = global_idx_A_ + 3;
        const int rB = global_idx_B_ + 3;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                triplets.emplace_back(rA + i, rB + j, -H(i, j));
                triplets.emplace_back(rB + i, rA + j, -H(i, j));
            }
        }
    }
}

void AngleLimitJoint::updateLambda(const Eigen::VectorXd& x) {
    Eigen::VectorXd g(dim());
    computeC(x, g);
    if (lambda_.size() != dim()) {
        lambda_ = Eigen::VectorXd::Zero(dim());
    }
    for (int i = 0; i < dim(); ++i) {
        lambda_(i) = std::max(0.0, lambda_(i) + mu_ * g(i));
    }
}

} // namespace NexDynIPC::Dynamics
