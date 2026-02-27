#pragma once

#include "ImplicitTimeIntegrator.h"

namespace NexDynIPC::TimeIntegration {

    /**
     * @brief Implicit Backward Euler time integration scheme.
     * 
     * Update formulas:
     * x_{n+1} = x_n + h * v_{n+1}
     * v_{n+1} = (x_{n+1} - x_n) / h
     * 
     * This is 1st order accurate and has significant numerical damping (A-stable).
     * It corresponds to Newmark with beta=1, gamma=1 (incorrect, actually specific case).
     * 
     * In our framework:
     * x_tilde = x_n + h * v_n
     * acceleration_scaling = h^2
     */
    class ImplicitEuler : public ImplicitTimeIntegrator {
    public:
        ImplicitEuler() = default;

        Eigen::VectorXd x_tilde() const override;
        double acceleration_scaling() const override;
        void update_quantities(const Eigen::VectorXd& x_new) override;
    };

} // namespace NexDynIPC::TimeIntegration
