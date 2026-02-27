#include "NexDynIPC/Control/PositionDriveForm.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace NexDynIPC::Control {

PositionDriveForm::PositionDriveForm(std::shared_ptr<Dynamics::RigidBody> bodyA,
                                     std::shared_ptr<Dynamics::RigidBody> bodyB,
                                     const Eigen::Vector3d& axis,
                                     double kp,
                                     double target_position)
    : bodyA_(std::move(bodyA))
    , bodyB_(std::move(bodyB))
    , axis_(axis.normalized())
    , kp_(kp)
    , target_position_(target_position)
    , max_force_(std::numeric_limits<double>::infinity())
    , deadzone_m_(0.0)
    , dt_(0.01) {
    if (!bodyB_) {
        throw std::invalid_argument("PositionDriveForm: bodyB cannot be null");
    }
}

void PositionDriveForm::setTargetPosition(double target_position) {
    target_position_ = target_position;
}

void PositionDriveForm::setPositionGain(double kp) {
    kp_ = kp;
}

void PositionDriveForm::setMaxForce(double max_force) {
    if (max_force <= 0.0) {
        max_force_ = std::numeric_limits<double>::infinity();
        return;
    }
    max_force_ = max_force;
}

void PositionDriveForm::setDeadzone(double deadzone_m) {
    deadzone_m_ = std::max(0.0, deadzone_m);
}

void PositionDriveForm::updateGlobalIndices(const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies) {
    global_idx_A_ = -1;
    global_idx_B_ = -1;
    if (bodyA_) {
        global_idx_A_ = findBodyIndex(bodies, bodyA_);
    }
    global_idx_B_ = findBodyIndex(bodies, bodyB_);
}

int PositionDriveForm::findBodyIndex(const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies,
                                     const std::shared_ptr<Dynamics::RigidBody>& body) const {
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i] == body) {
            return static_cast<int>(i * 6);
        }
    }
    return -1;
}

double PositionDriveForm::getCurrentPosition(const Eigen::VectorXd& x) const {
    Eigen::Vector3d pA = Eigen::Vector3d::Zero();
    Eigen::Vector3d pB = Eigen::Vector3d::Zero();
    if (global_idx_A_ >= 0 && global_idx_A_ + 2 < x.size()) {
        pA = x.segment<3>(global_idx_A_);
    }
    if (global_idx_B_ >= 0 && global_idx_B_ + 2 < x.size()) {
        pB = x.segment<3>(global_idx_B_);
    }
    return (pB - pA).dot(axis_);
}

double PositionDriveForm::getPositionError(const Eigen::VectorXd& x) const {
    return target_position_ - getCurrentPosition(x);
}

double PositionDriveForm::applyDeadzone(double position_error) const {
    if (deadzone_m_ <= 0.0) {
        return position_error;
    }
    if (position_error > deadzone_m_) {
        return position_error - deadzone_m_;
    }
    if (position_error < -deadzone_m_) {
        return position_error + deadzone_m_;
    }
    return 0.0;
}

double PositionDriveForm::computeSaturatedForce(double effective_position_error) const {
    const double raw_force = kp_ * effective_position_error;
    if (!std::isfinite(max_force_) || max_force_ <= 0.0) {
        return raw_force;
    }
    const double s = raw_force / max_force_;
    return max_force_ * std::tanh(s);
}

double PositionDriveForm::getDriveForce(const Eigen::VectorXd& x) const {
    return computeSaturatedForce(applyDeadzone(getPositionError(x)));
}

double PositionDriveForm::value(const Eigen::VectorXd& x) const {
    const double e = applyDeadzone(getPositionError(x));
    if (!std::isfinite(max_force_)) {
        return 0.5 * kp_ * e * e;
    }
    if (kp_ <= 0.0 || max_force_ <= 0.0) {
        return 0.0;
    }
    const double s = (kp_ * e) / max_force_;
    return (max_force_ * max_force_ / kp_) * std::log(std::cosh(s));
}

void PositionDriveForm::gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const {
    const double force = getDriveForce(x);
    if (global_idx_B_ >= 0) {
        grad.segment<3>(global_idx_B_) -= force * axis_;
    }
    if (global_idx_A_ >= 0) {
        grad.segment<3>(global_idx_A_) += force * axis_;
    }
}

void PositionDriveForm::hessian(const Eigen::VectorXd& x,
                                std::vector<Eigen::Triplet<double>>& triplets) const {
    double h = kp_;
    const double err = getPositionError(x);
    if (std::isfinite(max_force_) && kp_ > 0.0 && max_force_ > 0.0) {
        if (std::abs(err) <= deadzone_m_) {
            h = 0.0;
        } else {
            const double s = (kp_ * applyDeadzone(err)) / max_force_;
            const double t = std::tanh(s);
            const double sech2 = 1.0 - t * t;
            h = kp_ * sech2;
        }
    }
    if (h == 0.0) {
        return;
    }

    const Eigen::Matrix3d H = h * (axis_ * axis_.transpose());
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
