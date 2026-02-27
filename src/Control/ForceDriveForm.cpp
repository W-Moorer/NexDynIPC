#include "NexDynIPC/Control/ForceDriveForm.h"

namespace NexDynIPC::Control {

ForceDriveForm::ForceDriveForm(std::shared_ptr<Dynamics::RigidBody> body,
                               const Eigen::Vector3d& force_world,
                               const Eigen::Vector3d& torque_world)
    : body_(std::move(body))
    , force_world_(force_world)
    , torque_world_(torque_world) {}

void ForceDriveForm::updateGlobalIndices(const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies) {
    global_idx_ = -1;
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i] == body_) {
            global_idx_ = static_cast<int>(i * 6);
            break;
        }
    }
}

double ForceDriveForm::value(const Eigen::VectorXd& x) const {
    (void)x;
    return 0.0;
}

void ForceDriveForm::gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const {
    (void)x;
    if (global_idx_ < 0) {
        return;
    }
    grad.segment<3>(global_idx_) -= force_world_;
    grad.segment<3>(global_idx_ + 3) -= torque_world_;
}

void ForceDriveForm::hessian(const Eigen::VectorXd& x,
                             std::vector<Eigen::Triplet<double>>& triplets) const {
    (void)x;
    (void)triplets;
}

} // namespace NexDynIPC::Control
