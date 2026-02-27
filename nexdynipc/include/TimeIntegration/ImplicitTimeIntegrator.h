#pragma once

#include <Eigen/Core>
#include <memory>
#include <nlohmann/json.hpp>

namespace NexDynIPC::TimeIntegration {

    /**
     * @brief Abstract base class for implicit time integration schemes.
     * Designed to support IPC (Incremental Potential Contact) framework.
     */
    class ImplicitTimeIntegrator {
    public:
        virtual ~ImplicitTimeIntegrator() = default;

        /**
         * @brief Initialize the integrator with the starting state.
         * @param x Initial position
         * @param v Initial velocity
         * @param a Initial acceleration
         * @param dt Time step size
         */
        virtual void init(const Eigen::VectorXd& x, const Eigen::VectorXd& v, const Eigen::VectorXd& a, double dt);

        /**
         * @brief Compute the predicted position x_tilde.
         * Used in the inertia term: 0.5 * (x - x_tilde)^T * M * (x - x_tilde)
         * For Implicit Euler: x_tilde = x_n + h * v_n
         * For Newmark: x_tilde = x_n + h * v_n + h^2 * (0.5 - beta) * a_n
         */
        virtual Eigen::VectorXd x_tilde() const = 0;

        /**
         * @brief Compute the scaling factor for the acceleration term (and thus potential energy).
         * Used to scale forces/potentials: scaling * V(x)
         * For Implicit Euler: h^2
         * For Newmark: beta * h^2
         */
        virtual double acceleration_scaling() const = 0;

        /**
         * @brief Update internal state after a successful time step.
         * @param x_new The converged solution for the next time step (x_{n+1})
         */
        virtual void update_quantities(const Eigen::VectorXd& x_new) = 0;

        /**
         * @brief Get the current time step size.
         */
        double dt() const { return dt_; }

        // Accessors for current state (previous step's result)
        const Eigen::VectorXd& x_prev() const { return x_prev_; }
        const Eigen::VectorXd& v_prev() const { return v_prev_; }
        const Eigen::VectorXd& a_prev() const { return a_prev_; }

        /**
         * @brief Factory method to create an integrator from JSON configuration.
         */
        static std::shared_ptr<ImplicitTimeIntegrator> create(const nlohmann::json& config);

    protected:
        double dt_ = 0.0;
        Eigen::VectorXd x_prev_;
        Eigen::VectorXd v_prev_;
        Eigen::VectorXd a_prev_;
    };

} // namespace NexDynIPC::TimeIntegration
