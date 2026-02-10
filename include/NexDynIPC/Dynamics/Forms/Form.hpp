#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>

namespace NexDynIPC::Dynamics {

class Form {
public:
    virtual ~Form() = default;

    // Compute potential energy value
    virtual double value(const Eigen::VectorXd& x) const = 0;

    // Compute gradient (force)
    virtual void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const = 0;

    // Compute Hessian (stiffness)
    // Add triplets to the list
    virtual void hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) const = 0;
};

} // namespace NexDynIPC::Dynamics
