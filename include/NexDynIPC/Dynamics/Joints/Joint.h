#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>

namespace NexDynIPC::Dynamics {

class Joint {
public:
    virtual ~Joint() = default;

    // Dimension of the constraint C(x)
    virtual int dim() const = 0;

    // Compute constraint vector C(x)
    virtual void computeC(const Eigen::VectorXd& x, Eigen::VectorXd& C) const = 0;

    // Compute constraint Jacobian J = dC/dx
    // J should be (dim() x x.size()) sparse matrix
    virtual void computeJ(const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& J) const = 0;

    // ALM Energy: E = lambda^T * C + 0.5 * mu * C^T * C
    virtual double value(const Eigen::VectorXd& x) const;

    // Gradient: dE/dx = J^T * (lambda + mu * C)
    virtual void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const;

    // Hessian: d^2E/dx^2 approx J^T * mu * J (Gauss-Newton)
    virtual void hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) const;

    // Update Lagrange Multiplier: lambda += mu * C
    void updateLambda(const Eigen::VectorXd& x);

    // Reset Multiplier
    void resetLambda();

    // Set/Get Stiffness (Penalty Parameter)
    void setStiffness(double mu) { mu_ = mu; }
    double getStiffness() const { return mu_; }

protected:
    Eigen::VectorXd lambda_;
    double mu_ = 1e8; // Default stiffness
};

} // namespace NexDynIPC::Dynamics
