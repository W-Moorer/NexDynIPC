#include "NexDynIPC/Math/NewtonSolver.h"
#include "NexDynIPC/Math/LineSearch.h"
#include "NexDynIPC/Math/LinearSolver.h"

#include <iostream>

namespace NexDynIPC::Math {

NewtonSolver::NewtonSolver(const Options& opts) : opts_(opts) {}
NewtonSolver::~NewtonSolver() = default;

bool NewtonSolver::minimize(OptimizationProblem& problem, Eigen::VectorXd& x) {
    iterations_ = 0;
    
    // Create solvers
    auto linear_solver = createCholeskySolver();
    BacktrackingLineSearch line_search;
    line_search.c1 = opts_.line_search_c1;
    line_search.rho = opts_.line_search_rho;

    Eigen::VectorXd grad;
    Eigen::SparseMatrix<double> hess;
    Eigen::VectorXd dx;

    for (int iter = 0; iter < opts_.max_iterations; ++iter) {
        iterations_ = iter;

        // 1. Compute Gradient and Hessian
        problem.computeGradient(x, grad);
        
        // Check convergence based on gradient norm
        if (grad.template lpNorm<Eigen::Infinity>() < opts_.tolerance) {
            return true;
        }

        problem.computeHessian(x, hess);

        // 2. Solve H * dx = -g
        linear_solver->analyzePattern(hess);
        linear_solver->factorize(hess);
        
        if (!linear_solver->info()) {
           // std::cerr << "NewtonSolver: Hessian factorization failed." << std::endl;
            return false;
        }
        
        dx.resize(grad.size());
        linear_solver->solve(-grad, dx); // dx = -H^-1 g

        // 3. Line Search
        // Limit max step size (e.g. for CCD)
        double max_step = problem.maxStep(x, dx);
        double initial_alpha = std::min(1.0, max_step);

        double alpha = line_search.search(
            [&](const Eigen::VectorXd& xt) { return problem.computeValue(xt); },
            x, dx, grad, initial_alpha
        );

        if (alpha == 0.0) {
            // Line search failed
            // std::cerr << "NewtonSolver: Line search failed." << std::endl;
            return false;
        }

        // 4. Update
        x += alpha * dx;
        
        problem.postIteration(x);
    }

    return false; // Reached max iterations
}

} // namespace NexDynIPC::Math
