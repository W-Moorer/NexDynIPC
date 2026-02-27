#include "NexDynIPC/Math/NewtonSolver.h"
#include "NexDynIPC/Math/LineSearch.h"
#include "NexDynIPC/Math/LinearSolver.h"

namespace NexDynIPC::Math {

NewtonSolver::NewtonSolver(const Options& opts) : opts_(opts) {}
NewtonSolver::~NewtonSolver() = default;

bool NewtonSolver::minimize(OptimizationProblem& problem, Eigen::VectorXd& x) {
    iterations_ = 0;
    last_iteration_diagnostics_.clear();
    
    // Create solvers
    auto linear_solver = createCholeskySolver();
    BacktrackingLineSearch line_search;
    line_search.c1 = opts_.line_search_c1;
    line_search.rho = opts_.line_search_rho;

    Eigen::VectorXd grad;
    Eigen::VectorXd contact_grad;
    Eigen::SparseMatrix<double> hess;
    Eigen::VectorXd dx;

    for (int iter = 0; iter < opts_.max_iterations; ++iter) {
        iterations_ = iter;

        // 1. Compute Gradient and Hessian
        problem.computeGradient(x, grad);
        const double grad_inf = grad.template lpNorm<Eigen::Infinity>();
        
        // Check convergence based on gradient norm
        if (grad_inf < opts_.tolerance) {
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

        contact_grad.setZero(x.size());
        problem.computeContactGradient(x, contact_grad);
        const double contact_residual = contact_grad.template lpNorm<Eigen::Infinity>();
        const double contact_value_before = problem.computeContactValue(x);
        const std::function<double(const Eigen::VectorXd&)> contact_merit = [&](const Eigen::VectorXd& xt) {
            return problem.computeContactValue(xt);
        };

        double alpha = line_search.search(
            [&](const Eigen::VectorXd& xt) { return problem.computeValue(xt); },
            x, dx, grad,
            &contact_merit,
            &contact_grad,
            initial_alpha
        );

        if (alpha == 0.0) {
            // Line search failed
            // std::cerr << "NewtonSolver: Line search failed." << std::endl;
            return false;
        }

        const Eigen::VectorXd x_trial = x + alpha * dx;
        const double contact_value_after = problem.computeContactValue(x_trial);
        IterationDiagnostics diag;
        diag.iteration = iter;
        diag.line_search_alpha = alpha;
        diag.contact_residual = contact_residual;
        diag.contact_value_before = contact_value_before;
        diag.contact_value_after = contact_value_after;
        last_iteration_diagnostics_.push_back(diag);

        // 4. Update
        x = x_trial;
        
        problem.postIteration(x);
    }

    return false; // Reached max iterations
}

} // namespace NexDynIPC::Math
