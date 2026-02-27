#include "NexDynIPC/Dynamics/Forms/AngleLimitForm.h"

namespace NexDynIPC::Dynamics {

AngleLimitForm::AngleLimitForm(std::shared_ptr<RigidBody> bodyA,
                               std::shared_ptr<RigidBody> bodyB,
                               const Eigen::Vector3d& axis,
                               double min_angle_rad,
                               double max_angle_rad,
                               double stiffness)
    : bodyA_(std::move(bodyA))
    , bodyB_(std::move(bodyB))
    , axis_(axis.normalized())
    , min_angle_rad_(min_angle_rad)
    , max_angle_rad_(max_angle_rad)
    , stiffness_(stiffness) {}

void AngleLimitForm::updateGlobalIndices(const std::vector<std::shared_ptr<RigidBody>>& bodies) {
    global_idx_A_ = findBodyIndex(bodies, bodyA_);
    global_idx_B_ = findBodyIndex(bodies, bodyB_);
}

int AngleLimitForm::findBodyIndex(const std::vector<std::shared_ptr<RigidBody>>& bodies,
                                  const std::shared_ptr<RigidBody>& body) const {
    if (!body) {
        return -1;
    }
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i] == body) {
            return static_cast<int>(i * 6);
        }
    }
    return -1;
}

double AngleLimitForm::baseRelativeAngle() const {
    const Eigen::Quaterniond qA = bodyA_ ? bodyA_->orientation : Eigen::Quaterniond::Identity();
    const Eigen::Quaterniond qB = bodyB_ ? bodyB_->orientation : Eigen::Quaterniond::Identity();
    const Eigen::Quaterniond qRel = qA.conjugate() * qB;

    const Eigen::AngleAxisd aa(qRel);
    return aa.angle() * aa.axis().dot(axis_);
}

double AngleLimitForm::projectedDeltaAngle(const Eigen::VectorXd& x) const {
    Eigen::Vector3d thetaA = Eigen::Vector3d::Zero();
    Eigen::Vector3d thetaB = Eigen::Vector3d::Zero();

    if (global_idx_A_ >= 0 && global_idx_A_ + 5 < x.size()) {
        thetaA = x.segment<3>(global_idx_A_ + 3);
    }
    if (global_idx_B_ >= 0 && global_idx_B_ + 5 < x.size()) {
        thetaB = x.segment<3>(global_idx_B_ + 3);
    }

    return (thetaB - thetaA).dot(axis_);
}

double AngleLimitForm::currentRelativeAngle(const Eigen::VectorXd& x) const {
    return baseRelativeAngle() + projectedDeltaAngle(x);
}

double AngleLimitForm::value(const Eigen::VectorXd& x) const {
    const double angle = currentRelativeAngle(x);
    if (angle < min_angle_rad_) {
        const double violation = min_angle_rad_ - angle;
        return 0.5 * stiffness_ * violation * violation;
    }
    if (angle > max_angle_rad_) {
        const double violation = angle - max_angle_rad_;
        return 0.5 * stiffness_ * violation * violation;
    }
    return 0.0;
}

void AngleLimitForm::gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const {
    const double angle = currentRelativeAngle(x);

    double dE_dangle = 0.0;
    if (angle < min_angle_rad_) {
        dE_dangle = stiffness_ * (angle - min_angle_rad_);
    } else if (angle > max_angle_rad_) {
        dE_dangle = stiffness_ * (angle - max_angle_rad_);
    } else {
        return;
    }

    if (global_idx_B_ >= 0 && global_idx_B_ + 5 < grad.size()) {
        grad.segment<3>(global_idx_B_ + 3) += dE_dangle * axis_;
    }
    if (global_idx_A_ >= 0 && global_idx_A_ + 5 < grad.size()) {
        grad.segment<3>(global_idx_A_ + 3) -= dE_dangle * axis_;
    }
}

void AngleLimitForm::hessian(const Eigen::VectorXd& x,
                             std::vector<Eigen::Triplet<double>>& triplets) const {
    const double angle = currentRelativeAngle(x);
    if (angle >= min_angle_rad_ && angle <= max_angle_rad_) {
        return;
    }

    const Eigen::Matrix3d H = stiffness_ * (axis_ * axis_.transpose());

    if (global_idx_B_ >= 0) {
        const int rB = global_idx_B_ + 3;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                triplets.emplace_back(rB + i, rB + j, H(i, j));
            }
        }
    }
    if (global_idx_A_ >= 0) {
        const int rA = global_idx_A_ + 3;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                triplets.emplace_back(rA + i, rA + j, H(i, j));
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

} // namespace NexDynIPC::Dynamics
