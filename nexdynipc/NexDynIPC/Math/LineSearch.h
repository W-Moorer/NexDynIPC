#pragma once

#include <Eigen/Core>
#include <functional>

namespace NexDynIPC::Math {

// Interface for line search strategies
class LineSearchStrategy {
public:
    virtual ~LineSearchStrategy() = default;

    // Run line search to find a suitable step size alpha
    // f: Function to evaluate objective value (and optionally gradient)
    // x: Current position
    // dir: Search direction
    // grad: Gradient at x (for checking descent condition)
    // initial_alpha: Starting step size
    // Returns: The accepted step size alpha, or 0.0 if failed
    virtual double search(
        const std::function<double(const Eigen::VectorXd&)>& f,
        const Eigen::VectorXd& x,
        const Eigen::VectorXd& dir,
        const Eigen::VectorXd& grad,
        double initial_alpha = 1.0) = 0;
};

// Standard Armijo backtracking line search
class BacktrackingLineSearch : public LineSearchStrategy {
public:
    double c1 = 1e-4; // Armijo condition parameter
    double rho = 0.5; // Backtracking reduction factor
    int max_iter = 50;

    double search(
        const std::function<double(const Eigen::VectorXd&)>& f,
        const Eigen::VectorXd& x,
        const Eigen::VectorXd& dir,
        const Eigen::VectorXd& grad,
        double initial_alpha = 1.0) override;
};

} // namespace NexDynIPC::Math
