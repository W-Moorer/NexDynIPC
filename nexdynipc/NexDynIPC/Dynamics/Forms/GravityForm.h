#pragma once

#include "NexDynIPC/Dynamics/Forms/Form.h"
#include "NexDynIPC/Dynamics/World.h"

namespace NexDynIPC::Dynamics {

class GravityForm : public Form {
public:
    GravityForm(const World& world, const Eigen::Vector3d& g);

    double value(const Eigen::VectorXd& x) const override;
    void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const override;
    void hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) const override;

private:
    const World& world_;
    Eigen::Vector3d g_;
};

} // namespace NexDynIPC::Dynamics
