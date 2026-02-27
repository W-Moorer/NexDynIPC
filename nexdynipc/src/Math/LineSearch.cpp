#include "NexDynIPC/Math/LineSearch.h"
#include <iostream>

namespace NexDynIPC::Math {

double BacktrackingLineSearch::search(
    const std::function<double(const Eigen::VectorXd&)>& f,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& dir,
    const Eigen::VectorXd& grad,
    const std::function<double(const Eigen::VectorXd&)>* contact_f,
    const Eigen::VectorXd* contact_grad,
    double initial_alpha) 
{
    double alpha = initial_alpha;
    double fx = f(x);
    double descent = grad.dot(dir); // slope

    const bool has_contact_merit = (contact_f != nullptr);
    double f_contact = 0.0;
    double contact_descent = 0.0;
    if (has_contact_merit) {
        f_contact = (*contact_f)(x);
        if (contact_grad != nullptr && contact_grad->size() == dir.size()) {
            contact_descent = contact_grad->dot(dir);
        }
    }

    // Ensure descent direction
    if (descent > 0) {
        // std::cerr << "Warning: Not a descent direction during line search." << std::endl;
        return 0.0;
    }

    for (int i = 0; i < max_iter; ++i) {
        Eigen::VectorXd x_new = x + alpha * dir;
        double fx_new = f(x_new);

        // Armijo condition: f(x + alpha*p) <= f(x) + c1 * alpha * grad(x)^T * p
        bool contact_ok = true;
        if (has_contact_merit) {
            const double f_contact_new = (*contact_f)(x_new);
            if (contact_descent <= 0.0) {
                contact_ok = (f_contact_new <= f_contact + c1 * alpha * contact_descent);
            } else {
                contact_ok = (f_contact_new <= f_contact);
            }
        }

        if (fx_new <= fx + c1 * alpha * descent && contact_ok) {
            return alpha; // Accepted
        }

        alpha *= rho;
    }

    return 0.0; // Failed to find step
}

} // namespace NexDynIPC::Math
