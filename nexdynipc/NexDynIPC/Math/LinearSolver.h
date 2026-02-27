#pragma once

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <memory>

namespace NexDynIPC::Math {

class LinearSolver {
public:
    virtual ~LinearSolver() = default;

    // Analyze the sparsity pattern of matrix A
    virtual void analyzePattern(const Eigen::SparseMatrix<double>& A) = 0;

    // Factorize the matrix A
    virtual void factorize(const Eigen::SparseMatrix<double>& A) = 0;

    // Solve Ax = b
    virtual void solve(const Eigen::VectorXd& b, Eigen::VectorXd& x) = 0;
    
    // Check if the solver was successful
    virtual bool info() const = 0;
};

// Factory method to create a default solver (currently Eigen::SimplicialLDLT)
std::unique_ptr<LinearSolver> createCholeskySolver();

} // namespace NexDynIPC::Math
