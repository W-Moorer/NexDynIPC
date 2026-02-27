#include "NexDynIPC/Control/LinearVelocityDriveForm.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace NexDynIPC::Control {

LinearVelocityDriveForm::LinearVelocityDriveForm(std::shared_ptr<Dynamics::RigidBody> bodyA,
                                                 std::shared_ptr<Dynamics::RigidBody> bodyB,
                                                 const Eigen::Vector3d& axis,
                                                 double kv,
                                                 double v_target)
    : bodyA_(std::move(bodyA))
    , bodyB_(std::move(bodyB))
    , axis_(axis.normalized())
    , kv_(kv)
    , v_target_(v_target)
    , dt_(0.01)
    , max_force_(std::numeric_limits<double>::infinity())
    , delay_mps_(0.0)
    , delay_tau_seconds_(0.0)
    , filtered_target_velocity_(v_target) {
    if (!bodyB_) {
        throw std::invalid_argument("LinearVelocityDriveForm: bodyB cannot be null");
    }
}

void LinearVelocityDriveForm::setTargetVelocity(double v_target) {
    v_target_ = v_target;
}

void LinearVelocityDriveForm::setVelocityGain(double kv) {
    kv_ = kv;
}

void LinearVelocityDriveForm::setMaxForce(double max_force) {
    if (max_force <= 0.0) {
        max_force_ = std::numeric_limits<double>::infinity();
        return;
    }
    max_force_ = max_force;
}

void LinearVelocityDriveForm::setDelay(double delay_mps) {
    delay_mps_ = std::max(0.0, delay_mps);
}

void LinearVelocityDriveForm::setDelayTau(double delay_tau_seconds) {
    delay_tau_seconds_ = std::max(0.0, delay_tau_seconds);
    if (delay_tau_seconds_ <= 0.0) {
        filter_initialized_ = false;
        filtered_target_velocity_ = v_target_;
    }
}

void LinearVelocityDriveForm::advanceControlState() {
    if (delay_tau_seconds_ <= 0.0) {
        filtered_target_velocity_ = v_target_;
        filter_initialized_ = false;
        return;
    }

    if (!filter_initialized_) {
        filtered_target_velocity_ = v_target_;
        filter_initialized_ = true;
        return;
    }

    if (dt_ <= 0.0) {
        return;
    }

    const double alpha = dt_ / (delay_tau_seconds_ + dt_);
    filtered_target_velocity_ += alpha * (v_target_ - filtered_target_velocity_);
}

double LinearVelocityDriveForm::getEffectiveTargetVelocity() const {
    return effectiveTargetVelocity();
}

void LinearVelocityDriveForm::updateGlobalIndices(const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies) {
    global_idx_A_ = -1;
    global_idx_B_ = -1;

    if (bodyA_) {
        global_idx_A_ = findBodyIndex(bodies, bodyA_);
    }
    global_idx_B_ = findBodyIndex(bodies, bodyB_);
    has_prev_state_ = false;
}

int LinearVelocityDriveForm::findBodyIndex(const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies,
                                           const std::shared_ptr<Dynamics::RigidBody>& body) const {
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i] == body) {
            return static_cast<int>(i * 6);
        }
    }
    return -1;
}

Eigen::Vector3d LinearVelocityDriveForm::extractPosition(const Eigen::VectorXd& x, int global_idx) const {
    if (global_idx < 0 || global_idx + 2 >= x.size()) {
        return Eigen::Vector3d::Zero();
    }
    return x.segment<3>(global_idx);
}

double LinearVelocityDriveForm::computeJointVelocity(const Eigen::VectorXd& x) const {
    const Eigen::Vector3d pos_A = extractPosition(x, global_idx_A_);
    const Eigen::Vector3d pos_B = extractPosition(x, global_idx_B_);

    Eigen::Vector3d vel_A = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_B = Eigen::Vector3d::Zero();
    if (has_prev_state_ && dt_ > 0.0) {
        vel_A = (pos_A - prev_pos_A_) / dt_;
        vel_B = (pos_B - prev_pos_B_) / dt_;
    }

    prev_pos_A_ = pos_A;
    prev_pos_B_ = pos_B;
    has_prev_state_ = true;

    return (vel_B - vel_A).dot(axis_);
}

double LinearVelocityDriveForm::getCurrentVelocity(const Eigen::VectorXd& x) const {
    return computeJointVelocity(x);
}

double LinearVelocityDriveForm::getVelocityError(const Eigen::VectorXd& x) const {
    return effectiveTargetVelocity() - getCurrentVelocity(x);
}

double LinearVelocityDriveForm::getDriveForce(const Eigen::VectorXd& x) const {
    const double v_error = getVelocityError(x);
    const double v_error_eff = applyDelayDeadzone(v_error);
    return computeSaturatedForce(v_error_eff);
}

double LinearVelocityDriveForm::getCurrentVelocityFromBodies() const {
    Eigen::Vector3d vA = Eigen::Vector3d::Zero();
    Eigen::Vector3d vB = Eigen::Vector3d::Zero();
    if (bodyA_) {
        vA = bodyA_->velocity;
    }
    if (bodyB_) {
        vB = bodyB_->velocity;
    }
    return (vB - vA).dot(axis_);
}

double LinearVelocityDriveForm::getVelocityErrorFromBodies() const {
    return effectiveTargetVelocity() - getCurrentVelocityFromBodies();
}

double LinearVelocityDriveForm::getDriveForceFromBodies() const {
    const double v_error = getVelocityErrorFromBodies();
    const double v_error_eff = applyDelayDeadzone(v_error);
    return computeSaturatedForce(v_error_eff);
}

bool LinearVelocityDriveForm::isForceSaturatedFromBodies() const {
    if (!std::isfinite(max_force_)) {
        return false;
    }
    return std::abs(getDriveForceFromBodies()) >= 0.98 * max_force_;
}

double LinearVelocityDriveForm::applyDelayDeadzone(double velocity_error) const {
    if (delay_mps_ <= 0.0) {
        return velocity_error;
    }
    if (velocity_error > delay_mps_) {
        return velocity_error - delay_mps_;
    }
    if (velocity_error < -delay_mps_) {
        return velocity_error + delay_mps_;
    }
    return 0.0;
}

double LinearVelocityDriveForm::computeSaturatedForce(double effective_velocity_error) const {
    const double raw_force = kv_ * effective_velocity_error;
    if (!std::isfinite(max_force_) || max_force_ <= 0.0) {
        return raw_force;
    }
    const double s = raw_force / max_force_;
    return max_force_ * std::tanh(s);
}

double LinearVelocityDriveForm::value(const Eigen::VectorXd& x) const {
    const double v = computeJointVelocity(x);
    const double v_error = effectiveTargetVelocity() - v;
    const double v_error_eff = applyDelayDeadzone(v_error);

    if (!std::isfinite(max_force_)) {
        return 0.5 * kv_ * v_error_eff * v_error_eff;
    }
    if (kv_ <= 0.0 || max_force_ <= 0.0) {
        return 0.0;
    }

    const double s = (kv_ * v_error_eff) / max_force_;
    return (max_force_ * max_force_ / kv_) * std::log(std::cosh(s));
}

void LinearVelocityDriveForm::gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const {
    if (dt_ <= 0.0) {
        return;
    }
    const double v = computeJointVelocity(x);
    const double v_error = effectiveTargetVelocity() - v;
    const double force = computeSaturatedForce(applyDelayDeadzone(v_error));
    const double coeff = force / dt_;

    if (global_idx_B_ >= 0) {
        grad.segment<3>(global_idx_B_) -= coeff * axis_;
    }
    if (global_idx_A_ >= 0) {
        grad.segment<3>(global_idx_A_) += coeff * axis_;
    }
}

void LinearVelocityDriveForm::hessian(const Eigen::VectorXd& x,
                                      std::vector<Eigen::Triplet<double>>& triplets) const {
    if (dt_ <= 0.0) {
        return;
    }

    double h = kv_ / (dt_ * dt_);
    if (std::isfinite(max_force_) && kv_ > 0.0 && max_force_ > 0.0) {
        const double v = computeJointVelocity(x);
        const double v_error = effectiveTargetVelocity() - v;
        if (std::abs(v_error) <= delay_mps_) {
            h = 0.0;
        } else {
            const double s = (kv_ * applyDelayDeadzone(v_error)) / max_force_;
            const double t = std::tanh(s);
            const double sech2 = 1.0 - t * t;
            h = (kv_ * sech2) / (dt_ * dt_);
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

double LinearVelocityDriveForm::effectiveTargetVelocity() const {
    if (delay_tau_seconds_ <= 0.0 || !filter_initialized_) {
        return v_target_;
    }
    return filtered_target_velocity_;
}

} // namespace NexDynIPC::Control
