#pragma once

#include "NexDynIPC/Dynamics/Forms/Form.h"
#include "NexDynIPC/Dynamics/Joints/Joint.h"
#include <vector>
#include <memory>

namespace NexDynIPC::Dynamics {

class ConstraintForm : public Form {
public:
    ConstraintForm();
    ~ConstraintForm() override = default;

    void addJoint(std::shared_ptr<Joint> joint);
    
    // Form interface
    double value(const Eigen::VectorXd& x) const override;
    void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const override;
    void hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) const override;

    // Helper to update all joints (lambda += mu * C)
    void updateLambdas(const Eigen::VectorXd& x);

    const std::vector<std::shared_ptr<Joint>>& getJoints() const { return joints_; }

private:
    std::vector<std::shared_ptr<Joint>> joints_;
};

} // namespace NexDynIPC::Dynamics
