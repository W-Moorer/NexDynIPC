#pragma once

#include "NexDynIPC/Dynamics/Forms/Form.h"
#include "NexDynIPC/Dynamics/RigidBody.h"
#include <limits>
#include <memory>
#include <vector>

namespace NexDynIPC::Control {

class LinearVelocityDriveForm : public Dynamics::Form {
public:
    LinearVelocityDriveForm(std::shared_ptr<Dynamics::RigidBody> bodyA,
                            std::shared_ptr<Dynamics::RigidBody> bodyB,
                            const Eigen::Vector3d& axis,
                            double kv,
                            double v_target);

    ~LinearVelocityDriveForm() override = default;

    void setTargetVelocity(double v_target);
    double getTargetVelocity() const { return v_target_; }

    void setVelocityGain(double kv);
    double getVelocityGain() const { return kv_; }

    void setMaxForce(double max_force);
    double getMaxForce() const { return max_force_; }

    void setDelay(double delay_mps);
    double getDelay() const { return delay_mps_; }

    void setDelayTau(double delay_tau_seconds);
    double getDelayTau() const { return delay_tau_seconds_; }

    void advanceControlState();
    double getEffectiveTargetVelocity() const;

    void setTimeStep(double dt) { dt_ = dt; }
    double getTimeStep() const { return dt_; }

    void updateGlobalIndices(const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies);

    double getCurrentVelocity(const Eigen::VectorXd& x) const;
    double getVelocityError(const Eigen::VectorXd& x) const;
    double getDriveForce(const Eigen::VectorXd& x) const;

    double getCurrentVelocityFromBodies() const;
    double getVelocityErrorFromBodies() const;
    double getDriveForceFromBodies() const;
    bool isForceSaturatedFromBodies() const;

    double value(const Eigen::VectorXd& x) const override;
    void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const override;
    void hessian(const Eigen::VectorXd& x,
                 std::vector<Eigen::Triplet<double>>& triplets) const override;

private:
    std::shared_ptr<Dynamics::RigidBody> bodyA_;
    std::shared_ptr<Dynamics::RigidBody> bodyB_;
    Eigen::Vector3d axis_;

    double kv_;
    double v_target_;
    double dt_;
    double max_force_;
    double delay_mps_;
    double delay_tau_seconds_;

    int global_idx_A_ = -1;
    int global_idx_B_ = -1;

    mutable Eigen::Vector3d prev_pos_A_ = Eigen::Vector3d::Zero();
    mutable Eigen::Vector3d prev_pos_B_ = Eigen::Vector3d::Zero();
    mutable bool has_prev_state_ = false;
    mutable bool filter_initialized_ = false;
    mutable double filtered_target_velocity_ = 0.0;

    int findBodyIndex(const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies,
                      const std::shared_ptr<Dynamics::RigidBody>& body) const;
    Eigen::Vector3d extractPosition(const Eigen::VectorXd& x, int global_idx) const;
    double computeJointVelocity(const Eigen::VectorXd& x) const;
    double applyDelayDeadzone(double velocity_error) const;
    double computeSaturatedForce(double effective_velocity_error) const;
    double effectiveTargetVelocity() const;
};

} // namespace NexDynIPC::Control
