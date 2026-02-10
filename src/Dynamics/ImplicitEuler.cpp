#include "NexDynIPC/Dynamics/ImplicitEuler.hpp"
#include "NexDynIPC/Dynamics/Forms/InertiaForm.hpp"
#include "NexDynIPC/Dynamics/Forms/GravityForm.hpp"
#include <iostream>

namespace NexDynIPC::Dynamics {

class SimulationProblem : public Math::OptimizationProblem {
public:
    SimulationProblem(World& world, double dt) : world_(world), dt_(dt) {
        // Create inertia form
        inertia_form_ = std::make_unique<InertiaForm>(world, dt);
        // Create gravity form
        gravity_form_ = std::make_unique<GravityForm>(world, Eigen::Vector3d(0, -9.81, 0));
    }

    void setPredictiveState(const Eigen::VectorXd& x_hat) {
        inertia_form_->setPredictiveState(x_hat);
    }

    double computeValue(const Eigen::VectorXd& x) override {
        double val = inertia_form_->value(x);
        double dt2 = dt_ * dt_;
        val += dt2 * gravity_form_->value(x); // Add gravity scaled by dt^2
        for (const auto& form : world_.forms) {
            val += dt2 * form->value(x); // Potential energy scaled by dt^2
        }
        return val;
    }

    void computeGradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) override {
        // Initialize gradient
        grad.setZero(x.size());
        
        Eigen::VectorXd tmp_grad(x.size());
        double dt2 = dt_ * dt_;
        
        // Inertia gradient
        tmp_grad.setZero();
        inertia_form_->gradient(x, tmp_grad);
        grad += tmp_grad;

        // Gravity gradient
        tmp_grad.setZero();
        gravity_form_->gradient(x, tmp_grad);
        grad += dt2 * tmp_grad;

        // Other forms
        for (const auto& form : world_.forms) {
            tmp_grad.setZero();
            form->gradient(x, tmp_grad);
            grad += dt2 * tmp_grad;
        }
    }

    void computeHessian(const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& hess) override {
        std::vector<Eigen::Triplet<double>> triplets;
        double dt2 = dt_ * dt_;
        
        // Inertia Hessian
        inertia_form_->hessian(x, triplets);
        
        // Gravity Hessian (Zero, but keeps consistency)
        // We need to scale triplets?
        // GravityForm adds triplets. We can't easily scale them in-place if they are just pushed.
        // We should probably scale them after or wrap the form.
        // But Form::hessian takes vector& triplets.
        
        // Workaround: Create a temp vector, then scale and add.
        std::vector<Eigen::Triplet<double>> tmp_triplets;
        
        gravity_form_->hessian(x, tmp_triplets);
        for(auto& t : tmp_triplets) {
            triplets.emplace_back(t.row(), t.col(), t.value() * dt2);
        }

        // Other forms
        for (const auto& form : world_.forms) {
            tmp_triplets.clear();
            form->hessian(x, tmp_triplets);
            for(auto& t : tmp_triplets) {
                triplets.emplace_back(t.row(), t.col(), t.value() * dt2);
            }
        }

        hess.resize(x.size(), x.size());
        hess.setFromTriplets(triplets.begin(), triplets.end());
    }

private:
    World& world_;
    double dt_;
    std::unique_ptr<InertiaForm> inertia_form_;
    std::unique_ptr<GravityForm> gravity_form_;
};

ImplicitEuler::ImplicitEuler() {
    // Configure solver options if needed
    // solver_.setTolerance(1e-4);
}

void ImplicitEuler::step(World& world, double dt) {
    int n = 0;
    for (const auto& body : world.bodies) {
        if (!body->is_static) n += 6;
    }
    
    Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(n);

    // 1. Gather state
    int idx = 0;
    for (const auto& body : world.bodies) {
        if (body->is_static) continue;

        x.segment<3>(idx) = body->position;
        x.segment<3>(idx+3) = Eigen::Vector3d::Zero(); // Incremental rotation from current step

        v.segment<3>(idx) = body->velocity;
        v.segment<3>(idx+3) = body->angular_velocity;
        
        idx += 6;
    }

    // 2. Predict x_hat = x_n + h * v_n
    Eigen::VectorXd x_hat = x + dt * v;
    
    // 3. Construct optimization problem
    SimulationProblem problem(world, dt);
    problem.setPredictiveState(x_hat);

    // 4. Solve for x_{n+1}
    // Initial guess: x_hat
    Eigen::VectorXd x_new = x_hat;
    
    bool converged = solver_.minimize(problem, x_new);
    if (!converged) {
        std::cerr << "Solver did not converge!" << std::endl;
    }

    // 5. Update World State
    idx = 0;
    for (auto& body : world.bodies) {
        if (body->is_static) {
            continue;
        }

        Eigen::Vector3d p_new = x_new.segment<3>(idx);
        Eigen::Vector3d theta_new = x_new.segment<3>(idx+3); // Incremental rotation

        // Update velocity: v = (x_new - x_old) / dt
        // x_old was x (where theta was 0)
        Eigen::Vector3d p_old = x.segment<3>(idx);
        Eigen::Vector3d theta_old = x.segment<3>(idx+3); // 0

        // Update acceleration
        // a = (v_new - v_old) / dt
        body->linear_acceleration = (p_new - p_old) / dt - body->velocity; // Wait, correct formula is (v_new - v_old) / dt
        // Previous velocity is body->velocity.
        // New velocity is (p_new - p_old) / dt (Approx)
        Eigen::Vector3d v_new = (p_new - p_old) / dt;
        Eigen::Vector3d w_new = (theta_new - theta_old) / dt;

        body->linear_acceleration = (v_new - body->velocity) / dt;
        body->angular_acceleration = (w_new - body->angular_velocity) / dt;

        body->velocity = v_new;
        body->angular_velocity = w_new;

        // Update Position
        body->position = p_new;

        // Update Orientation
        // dTheta is rotation vector
        double angle = theta_new.norm();
        if (angle > 1e-10) {
            Eigen::Vector3d axis = theta_new / angle;
            Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, axis));
            body->orientation = (dq * body->orientation).normalized();
        }

        idx += 6;
    }
}

} // namespace NexDynIPC::Dynamics
