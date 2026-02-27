#pragma once

#include "NexDynIPC/Dynamics/Forms/Form.h"
#include "NexDynIPC/Dynamics/RigidBody.h"
#include <memory>
#include <vector>

namespace NexDynIPC::Control {

class DampedSpringForm : public Dynamics::Form {
public:
    DampedSpringForm(std::shared_ptr<Dynamics::RigidBody> bodyA,
                     std::shared_ptr<Dynamics::RigidBody> bodyB,
                     double rest_length,
                     double stiffness,
                     double damping);

    ~DampedSpringForm() override = default;

    void setRestLength(double rest_length) { rest_length_ = rest_length; }
    double getRestLength() const { return rest_length_; }

    void setStiffness(double stiffness) { stiffness_ = stiffness; }
    double getStiffness() const { return stiffness_; }

    void setDamping(double damping) { damping_ = damping; }
    double getDamping() const { return damping_; }

    void updateGlobalIndices(const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies);

    double value(const Eigen::VectorXd& x) const override;
    void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const override;
    void hessian(const Eigen::VectorXd& x,
                 std::vector<Eigen::Triplet<double>>& triplets) const override;

private:
    std::shared_ptr<Dynamics::RigidBody> bodyA_;
    std::shared_ptr<Dynamics::RigidBody> bodyB_;

    int global_idx_A_ = -1;
    int global_idx_B_ = -1;

    double rest_length_;
    double stiffness_;
    double damping_;

    int findBodyIndex(const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies,
                      const std::shared_ptr<Dynamics::RigidBody>& body) const;
    Eigen::Vector3d extractPosition(const Eigen::VectorXd& x, int global_idx) const;
    Eigen::Vector3d springDirection(const Eigen::VectorXd& x, double& length) const;
    double relativeVelocityAlongSpring(const Eigen::Vector3d& dir) const;
};

} // namespace NexDynIPC::Control
