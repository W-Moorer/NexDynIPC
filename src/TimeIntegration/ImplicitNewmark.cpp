#include "NexDynIPC/TimeIntegration/ImplicitNewmark.hpp"

namespace NexDynIPC::TimeIntegration {

    ImplicitNewmark::ImplicitNewmark(double beta, double gamma)
        : beta_(beta), gamma_(gamma) {}

    Eigen::VectorXd ImplicitNewmark::x_tilde() const {
        return x_prev_ + dt_ * v_prev_ + dt_ * dt_ * (0.5 - beta_) * a_prev_;
    }

    double ImplicitNewmark::acceleration_scaling() const {
        return beta_ * dt_ * dt_;
    }

    void ImplicitNewmark::update_quantities(const Eigen::VectorXd& x_new) {
        // a_{n+1} = (x_{n+1} - x_n - h*v_n - h^2*(0.5-beta)*a_n) / (beta * h^2)
        // Note: x_tilde = x_n + h*v_n + h^2*(0.5-beta)*a_n
        // So a_{n+1} = (x_{n+1} - x_tilde) / (beta * h^2)
        
        Eigen::VectorXd xt = x_tilde();
        Eigen::VectorXd a_new = (x_new - xt) / (beta_ * dt_ * dt_);

        // v_{n+1} = v_n + h * ((1-gamma)*a_n + gamma*a_{n+1})
        Eigen::VectorXd v_new = v_prev_ + dt_ * ((1.0 - gamma_) * a_prev_ + gamma_ * a_new);

        x_prev_ = x_new;
        v_prev_ = v_new;
        a_prev_ = a_new;
    }

} // namespace NexDynIPC::TimeIntegration
