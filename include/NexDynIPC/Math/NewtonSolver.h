#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>
#include <memory> 

namespace NexDynIPC::Math {

// Abstract base class for defining an optimization problem
class OptimizationProblem {
public:
    virtual ~OptimizationProblem() = default;

    // Compute objective function value
    virtual double computeValue(const Eigen::VectorXd& x) = 0;

    // Compute gradient
    virtual void computeGradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) = 0;

    // Compute Hessian
    virtual void computeHessian(const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& hess) = 0;

    // Optional: Pre-iteration callback
    virtual void preIteration(const Eigen::VectorXd& x) {}
    
    // Optional: Post-iteration callback
    virtual void postIteration(const Eigen::VectorXd& x) {}
    
    // Optional: Check if constraints are satisfied
    virtual bool isValid(const Eigen::VectorXd& x) { return true; }
    
    // Optional: Max step size allowed (e.g. for CCD)
    virtual double maxStep(const Eigen::VectorXd& x, const Eigen::VectorXd& dx) { return 1.0; }
};

// Newton-Raphson Solver
class NewtonSolver {
public:
    struct Options {
        int max_iterations = 20;
        double tolerance = 1e-6;
        double line_search_rho = 0.5;
        double line_search_c1 = 1e-4;
        bool use_line_search = true;
    };

    NewtonSolver(const Options& opts = Options());
    ~NewtonSolver();

    // Minimize the objective function defined by problem
    // Returns true if converged
    bool minimize(OptimizationProblem& problem, Eigen::VectorXd& x);

    // Get number of iterations performed
    int getIterations() const { return iterations_; }

private:
    Options opts_;
    int iterations_ = 0;
};

} // namespace NexDynIPC::Math
