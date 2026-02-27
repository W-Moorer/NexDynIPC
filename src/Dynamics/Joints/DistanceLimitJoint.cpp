#include "NexDynIPC/Dynamics/Joints/DistanceLimitJoint.h"
#include <algorithm>

namespace NexDynIPC::Dynamics {

DistanceLimitJoint::DistanceLimitJoint(int bodyA,
                                       int bodyB,
                                       double min_distance,
                                       double max_distance)
    : bodyA_id_(bodyA)
    , bodyB_id_(bodyB)
    , min_distance_(min_distance)
    , max_distance_(std::max(max_distance, min_distance)) {
    lambda_ = Eigen::VectorXd::Zero(dim());
}

void DistanceLimitJoint::updateState(int idxA,
                                     int idxB,
                                     const Eigen::Vector3d& pA,
                                     const Eigen::Quaterniond& qA,
                                     const Eigen::Vector3d& pB,
                                     const Eigen::Quaterniond& qB) {
    (void)qA;
    (void)qB;
    global_idx_A_ = idxA;
    global_idx_B_ = idxB;
    pA_ref_ = pA;
    pB_ref_ = pB;
}

Eigen::Vector3d DistanceLimitJoint::getPosition(const Eigen::VectorXd& x,
                                                int global_idx,
                                                const Eigen::Vector3d& p_ref) const {
    if (global_idx >= 0 && global_idx + 2 < x.size()) {
        return x.segment<3>(global_idx);
    }
    return p_ref;
}

Eigen::Vector3d DistanceLimitJoint::direction(const Eigen::VectorXd& x, double& distance) const {
    const Eigen::Vector3d pA = getPosition(x, global_idx_A_, pA_ref_);
    const Eigen::Vector3d pB = getPosition(x, global_idx_B_, pB_ref_);
    const Eigen::Vector3d d = pB - pA;
    distance = d.norm();
    if (distance < 1e-12) {
        return Eigen::Vector3d::UnitX();
    }
    return d / distance;
}

void DistanceLimitJoint::computeC(const Eigen::VectorXd& x, Eigen::VectorXd& C) const {
    if (C.size() != dim()) {
        C.resize(dim());
    }

    double dist = 0.0;
    (void)direction(x, dist);

    C(0) = min_distance_ - dist;
    C(1) = dist - max_distance_;
}

void DistanceLimitJoint::computeJ(const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& J) const {
    std::vector<Eigen::Triplet<double>> triplets;
    J.resize(dim(), x.size());

    double dist = 0.0;
    const Eigen::Vector3d dir = direction(x, dist);

    if (global_idx_A_ >= 0) {
        for (int k = 0; k < 3; ++k) {
            triplets.emplace_back(0, global_idx_A_ + k, dir[k]);
            triplets.emplace_back(1, global_idx_A_ + k, -dir[k]);
        }
    }
    if (global_idx_B_ >= 0) {
        for (int k = 0; k < 3; ++k) {
            triplets.emplace_back(0, global_idx_B_ + k, -dir[k]);
            triplets.emplace_back(1, global_idx_B_ + k, dir[k]);
        }
    }

    J.setFromTriplets(triplets.begin(), triplets.end());
}

double DistanceLimitJoint::value(const Eigen::VectorXd& x) const {
    Eigen::VectorXd g(dim());
    computeC(x, g);

    double energy = 0.0;
    for (int i = 0; i < dim(); ++i) {
        const double gp = std::max(g(i), 0.0);
        energy += lambda_(i) * gp + 0.5 * mu_ * gp * gp;
    }
    return energy;
}

void DistanceLimitJoint::gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const {
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

void DistanceLimitJoint::hessian(const Eigen::VectorXd& x,
                                 std::vector<Eigen::Triplet<double>>& triplets) const {
    Eigen::VectorXd g(dim());
    computeC(x, g);

    const bool active_min = g(0) > 0.0;
    const bool active_max = g(1) > 0.0;
    if (!active_min && !active_max) {
        return;
    }

    double dist = 0.0;
    const Eigen::Vector3d dir = direction(x, dist);
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    if (active_min) {
        H += mu_ * (dir * dir.transpose());
    }
    if (active_max) {
        H += mu_ * (dir * dir.transpose());
    }

    if (global_idx_A_ >= 0) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                triplets.emplace_back(global_idx_A_ + i, global_idx_A_ + j, H(i, j));
            }
        }
    }

    if (global_idx_B_ >= 0) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                triplets.emplace_back(global_idx_B_ + i, global_idx_B_ + j, H(i, j));
            }
        }
    }

    if (global_idx_A_ >= 0 && global_idx_B_ >= 0) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                triplets.emplace_back(global_idx_A_ + i, global_idx_B_ + j, -H(i, j));
                triplets.emplace_back(global_idx_B_ + i, global_idx_A_ + j, -H(i, j));
            }
        }
    }
}

void DistanceLimitJoint::updateLambda(const Eigen::VectorXd& x) {
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
