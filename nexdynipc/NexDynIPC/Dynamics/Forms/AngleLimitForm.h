#pragma once

#include "NexDynIPC/Dynamics/Forms/Form.h"
#include "NexDynIPC/Dynamics/RigidBody.h"
#include <memory>

namespace NexDynIPC::Dynamics {

class AngleLimitForm : public Form {
public:
    AngleLimitForm(std::shared_ptr<RigidBody> bodyA,
                   std::shared_ptr<RigidBody> bodyB,
                   const Eigen::Vector3d& axis,
                   double min_angle_rad,
                   double max_angle_rad,
                   double stiffness);

    void updateGlobalIndices(const std::vector<std::shared_ptr<RigidBody>>& bodies);

    double value(const Eigen::VectorXd& x) const override;
    void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const override;
    void hessian(const Eigen::VectorXd& x,
                 std::vector<Eigen::Triplet<double>>& triplets) const override;

private:
    std::shared_ptr<RigidBody> bodyA_;
    std::shared_ptr<RigidBody> bodyB_;
    Eigen::Vector3d axis_;
    double min_angle_rad_;
    double max_angle_rad_;
    double stiffness_;

    int global_idx_A_ = -1;
    int global_idx_B_ = -1;

    int findBodyIndex(const std::vector<std::shared_ptr<RigidBody>>& bodies,
                      const std::shared_ptr<RigidBody>& body) const;
    double baseRelativeAngle() const;
    double projectedDeltaAngle(const Eigen::VectorXd& x) const;
    double currentRelativeAngle(const Eigen::VectorXd& x) const;
};

} // namespace NexDynIPC::Dynamics
