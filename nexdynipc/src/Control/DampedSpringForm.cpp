#include "NexDynIPC/Control/DampedSpringForm.h"

namespace NexDynIPC::Control {

DampedSpringForm::DampedSpringForm(std::shared_ptr<Dynamics::RigidBody> bodyA,
                                   std::shared_ptr<Dynamics::RigidBody> bodyB,
                                   double rest_length,
                                   double stiffness,
                                   double damping)
    : bodyA_(std::move(bodyA))
    , bodyB_(std::move(bodyB))
    , rest_length_(rest_length)
    , stiffness_(stiffness)
    , damping_(damping) {}

int DampedSpringForm::findBodyIndex(const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies,
                                    const std::shared_ptr<Dynamics::RigidBody>& body) const {
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i] == body) {
            return static_cast<int>(i * 6);
        }
    }
    return -1;
}

void DampedSpringForm::updateGlobalIndices(const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies) {
    global_idx_A_ = findBodyIndex(bodies, bodyA_);
    global_idx_B_ = findBodyIndex(bodies, bodyB_);
}

Eigen::Vector3d DampedSpringForm::extractPosition(const Eigen::VectorXd& x, int global_idx) const {
    if (global_idx < 0 || global_idx + 2 >= x.size()) {
        return Eigen::Vector3d::Zero();
    }
    return x.segment<3>(global_idx);
}

Eigen::Vector3d DampedSpringForm::springDirection(const Eigen::VectorXd& x, double& length) const {
    const Eigen::Vector3d pA = extractPosition(x, global_idx_A_);
    const Eigen::Vector3d pB = extractPosition(x, global_idx_B_);
    const Eigen::Vector3d d = pB - pA;
    length = d.norm();
    if (length < 1e-12) {
        return Eigen::Vector3d::UnitX();
    }
    return d / length;
}

double DampedSpringForm::relativeVelocityAlongSpring(const Eigen::Vector3d& dir) const {
    Eigen::Vector3d vA = Eigen::Vector3d::Zero();
    Eigen::Vector3d vB = Eigen::Vector3d::Zero();
    if (bodyA_) {
        vA = bodyA_->velocity;
    }
    if (bodyB_) {
        vB = bodyB_->velocity;
    }
    return (vB - vA).dot(dir);
}

double DampedSpringForm::value(const Eigen::VectorXd& x) const {
    double length = 0.0;
    (void)springDirection(x, length);
    const double e = length - rest_length_;
    return 0.5 * stiffness_ * e * e;
}

void DampedSpringForm::gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const {
    double length = 0.0;
    const Eigen::Vector3d dir = springDirection(x, length);
    const double e = length - rest_length_;
    const double v_rel = relativeVelocityAlongSpring(dir);

    const Eigen::Vector3d traction = (stiffness_ * e + damping_ * v_rel) * dir;

    if (global_idx_B_ >= 0) {
        grad.segment<3>(global_idx_B_) += traction;
    }
    if (global_idx_A_ >= 0) {
        grad.segment<3>(global_idx_A_) -= traction;
    }
}

void DampedSpringForm::hessian(const Eigen::VectorXd& x,
                               std::vector<Eigen::Triplet<double>>& triplets) const {
    double length = 0.0;
    const Eigen::Vector3d dir = springDirection(x, length);
    const Eigen::Matrix3d H = stiffness_ * (dir * dir.transpose());

    if (global_idx_B_ >= 0) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                triplets.emplace_back(global_idx_B_ + i, global_idx_B_ + j, H(i, j));
            }
        }
    }
    if (global_idx_A_ >= 0) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                triplets.emplace_back(global_idx_A_ + i, global_idx_A_ + j, H(i, j));
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

} // namespace NexDynIPC::Control
