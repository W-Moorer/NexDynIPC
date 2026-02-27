#pragma once

#include "NexDynIPC/Dynamics/Forms/Form.h"
#include "NexDynIPC/Dynamics/RigidBody.h"
#include <limits>
#include <memory>
#include <vector>

namespace NexDynIPC::Control {

class PositionDriveForm : public Dynamics::Form {
public:
    PositionDriveForm(std::shared_ptr<Dynamics::RigidBody> bodyA,
                      std::shared_ptr<Dynamics::RigidBody> bodyB,
                      const Eigen::Vector3d& axis,
                      double kp,
                      double target_position);

    ~PositionDriveForm() override = default;

    void setTargetPosition(double target_position);
    double getTargetPosition() const { return target_position_; }

    void setPositionGain(double kp);
    double getPositionGain() const { return kp_; }

    void setMaxForce(double max_force);
    double getMaxForce() const { return max_force_; }

    void setDeadzone(double deadzone_m);
    double getDeadzone() const { return deadzone_m_; }

    void setTimeStep(double dt) { dt_ = dt; }
    double getTimeStep() const { return dt_; }

    void updateGlobalIndices(const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies);

    double getCurrentPosition(const Eigen::VectorXd& x) const;
    double getPositionError(const Eigen::VectorXd& x) const;
    double getDriveForce(const Eigen::VectorXd& x) const;

    double value(const Eigen::VectorXd& x) const override;
    void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const override;
    void hessian(const Eigen::VectorXd& x,
                 std::vector<Eigen::Triplet<double>>& triplets) const override;

private:
    std::shared_ptr<Dynamics::RigidBody> bodyA_;
    std::shared_ptr<Dynamics::RigidBody> bodyB_;
    Eigen::Vector3d axis_;
    double kp_;
    double target_position_;
    double max_force_;
    double deadzone_m_;
    double dt_;

    int global_idx_A_ = -1;
    int global_idx_B_ = -1;

    int findBodyIndex(const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies,
                      const std::shared_ptr<Dynamics::RigidBody>& body) const;
    double applyDeadzone(double position_error) const;
    double computeSaturatedForce(double effective_position_error) const;
};

} // namespace NexDynIPC::Control
