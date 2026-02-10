#include "NexDynIPC/Math/LineSearch.hpp"
#include <iostream>

namespace NexDynIPC::Math {

double BacktrackingLineSearch::search(
    const std::function<double(const Eigen::VectorXd&)>& f,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& dir,
    const Eigen::VectorXd& grad,
    double initial_alpha) 
{
    double alpha = initial_alpha;
    double fx = f(x);
    double descent = grad.dot(dir); // slope

    // Ensure descent direction
    if (descent > 0) {
        // std::cerr << "Warning: Not a descent direction during line search." << std::endl;
        return 0.0;
    }

    for (int i = 0; i < max_iter; ++i) {
        Eigen::VectorXd x_new = x + alpha * dir;
        double fx_new = f(x_new);

        // Armijo condition: f(x + alpha*p) <= f(x) + c1 * alpha * grad(x)^T * p
        if (fx_new <= fx + c1 * alpha * descent) {
            return alpha; // Accepted
        }

        alpha *= rho;
    }

    return 0.0; // Failed to find step
}

} // namespace NexDynIPC::Math
