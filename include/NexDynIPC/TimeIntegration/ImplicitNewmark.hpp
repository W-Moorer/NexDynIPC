#pragma once

#include "ImplicitTimeIntegrator.hpp"

namespace NexDynIPC::TimeIntegration {

    /**
     * @brief Implicit Newmark-beta time integration scheme.
     * 
     * Update formulas:
     * x_{n+1} = x_n + h*v_n + h^2/2 * ((1-2beta)*a_n + 2beta*a_{n+1})
     * v_{n+1} = v_n + h * ((1-gamma)*a_n + gamma*a_{n+1})
     * 
     * Default parameters (Average Acceleration / Trapezoidal Rule):
     * beta = 0.25, gamma = 0.5
     * This configuration is unconditional stable, 2nd order accurate, and symplectic (no numerical damping).
     */
    class ImplicitNewmark : public ImplicitTimeIntegrator {
    public:
        ImplicitNewmark(double beta = 0.25, double gamma = 0.5);

        Eigen::VectorXd x_tilde() const override;
        double acceleration_scaling() const override;
        void update_quantities(const Eigen::VectorXd& x_new) override;

        double beta() const { return beta_; }
        double gamma() const { return gamma_; }

    private:
        double beta_;
        double gamma_;
    };

} // namespace NexDynIPC::TimeIntegration
