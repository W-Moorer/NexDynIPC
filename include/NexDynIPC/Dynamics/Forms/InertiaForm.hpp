#pragma once

#include "NexDynIPC/Dynamics/Forms/Form.hpp"
#include "NexDynIPC/Dynamics/World.hpp"

namespace NexDynIPC::Dynamics {

class InertiaForm : public Form {
public:
    InertiaForm(const World& world, double dt);

    // Update the predicted state (x_hat) based on previous velocity
    void setPredictiveState(const Eigen::VectorXd& x_hat);

    double value(const Eigen::VectorXd& x) const override;
    void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const override;
    void hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) const override;

private:
    const World& world_;
    double dt_;
    Eigen::VectorXd x_hat_; // Target position/orientation if no forces applied
    // We assume Mass matrix M is constant block diagonal
};

} // namespace NexDynIPC::Dynamics
