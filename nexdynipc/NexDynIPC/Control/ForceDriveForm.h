#pragma once

#include "NexDynIPC/Dynamics/Forms/Form.h"
#include "NexDynIPC/Dynamics/RigidBody.h"
#include <memory>
#include <vector>

namespace NexDynIPC::Control {

class ForceDriveForm : public Dynamics::Form {
public:
    ForceDriveForm(std::shared_ptr<Dynamics::RigidBody> body,
                   const Eigen::Vector3d& force_world,
                   const Eigen::Vector3d& torque_world = Eigen::Vector3d::Zero());

    ~ForceDriveForm() override = default;

    void setForce(const Eigen::Vector3d& force_world) { force_world_ = force_world; }
    const Eigen::Vector3d& getForce() const { return force_world_; }

    void setTorque(const Eigen::Vector3d& torque_world) { torque_world_ = torque_world; }
    const Eigen::Vector3d& getTorque() const { return torque_world_; }

    void setTimeStep(double dt) { dt_ = dt; }
    double getTimeStep() const { return dt_; }

    void updateGlobalIndices(const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies);

    double value(const Eigen::VectorXd& x) const override;
    void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const override;
    void hessian(const Eigen::VectorXd& x,
                 std::vector<Eigen::Triplet<double>>& triplets) const override;

private:
    std::shared_ptr<Dynamics::RigidBody> body_;
    Eigen::Vector3d force_world_;
    Eigen::Vector3d torque_world_;
    double dt_ = 0.01;
    int global_idx_ = -1;
};

} // namespace NexDynIPC::Control
