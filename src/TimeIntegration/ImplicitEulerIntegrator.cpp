#include "NexDynIPC/TimeIntegration/ImplicitEulerIntegrator.h"

namespace NexDynIPC::TimeIntegration {

    Eigen::VectorXd ImplicitEuler::x_tilde() const {
        return x_prev_ + dt_ * v_prev_; // First order prediction
    }

    double ImplicitEuler::acceleration_scaling() const {
        return dt_ * dt_; // Standard backward Euler scaling for inertia term optimization
    }

    void ImplicitEuler::update_quantities(const Eigen::VectorXd& x_new) {
        // v_{n+1} = (x_{n+1} - x_n) / h
        Eigen::VectorXd v_new = (x_new - x_prev_) / dt_;
        
        // Acceleration is not explicitly tracked in Euler for the update, 
        // but we update it for consistency if needed by other systems.
        // a_{n+1} = (v_{n+1} - v_n) / h
        Eigen::VectorXd a_new = (v_new - v_prev_) / dt_;

        x_prev_ = x_new;
        v_prev_ = v_new;
        a_prev_ = a_new;
    }

} // namespace NexDynIPC::TimeIntegration
