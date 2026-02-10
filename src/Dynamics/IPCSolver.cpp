#include "NexDynIPC/Dynamics/IPCSolver.hpp"
#include "NexDynIPC/Dynamics/Forms/InertiaForm.hpp"
#include "NexDynIPC/Dynamics/Forms/GravityForm.hpp"
#include "NexDynIPC/Dynamics/Forms/ConstraintForm.hpp"
#include "NexDynIPC/Dynamics/Joints/RevoluteJoint.hpp"
#include "NexDynIPC/Dynamics/Joints/FixedJoint.hpp"
#include "NexDynIPC/TimeIntegration/ImplicitEulerIntegrator.hpp"
#include <iostream>
#include <unordered_map>

namespace NexDynIPC::Dynamics {

class SimulationProblem : public Math::OptimizationProblem {
public:
    SimulationProblem(World& world, std::shared_ptr<TimeIntegration::ImplicitTimeIntegrator> integrator) 
        : world_(world), integrator_(integrator) {
        // Create inertia form with integrator's dt
        inertia_form_ = std::make_unique<InertiaForm>(world, integrator_->dt());
        // Create gravity form (potential energy)
        gravity_form_ = std::make_unique<GravityForm>(world, Eigen::Vector3d(0, -9.81, 0));
        
        // Create Constraint Form
        constraint_form_ = std::make_unique<ConstraintForm>();
        for(auto& joint : world.joints) {
            constraint_form_->addJoint(joint);
        }
    }

    void setPredictiveState(const Eigen::VectorXd& x_hat) {
        inertia_form_->setPredictiveState(x_hat);
    }

    double computeValue(const Eigen::VectorXd& x) override {
        // Interia Term: 0.5 * (x - x_tilde)^T * M * (x - x_tilde)
        double val = inertia_form_->value(x);
        
        // Scaling for potential energy terms: beta * h^2 (or h^2 for Euler)
        double scaling = integrator_->acceleration_scaling();
        
        // Potential Energies
        val += scaling * gravity_form_->value(x);
        
        // Constraints
        // ALM constraints should NOT be scaled by dt^2 if they are meant to be hard constraints
        // competing with the inertia term (M * dx) directly.
        val += constraint_form_->value(x);

        for (const auto& form : world_.forms) {
            val += scaling * form->value(x); 
        }
        return val;
    }

    void computeGradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) override {
        grad.setZero(x.size());
        
        Eigen::VectorXd tmp_grad(x.size());
        double scaling = integrator_->acceleration_scaling(); // beta * h^2
        
        // Inertia gradient
        tmp_grad.setZero();
        inertia_form_->gradient(x, tmp_grad);
        grad += tmp_grad;

        // Gravity gradient
        tmp_grad.setZero();
        gravity_form_->gradient(x, tmp_grad);
        grad += scaling * tmp_grad;

        // Constraint gradient
        tmp_grad.setZero();
        constraint_form_->gradient(x, tmp_grad);
        grad += tmp_grad; // UN-SCALED!

        // Other forms
        for (const auto& form : world_.forms) {
            tmp_grad.setZero();
            form->gradient(x, tmp_grad);
            grad += scaling * tmp_grad;
        }
    }

    void computeHessian(const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& hess) override {
        std::vector<Eigen::Triplet<double>> triplets;
        double scaling = integrator_->acceleration_scaling(); // beta * h^2
        
        // Inertia Hessian
        inertia_form_->hessian(x, triplets);
        
        // Temporary storage for scaling
        std::vector<Eigen::Triplet<double>> tmp_triplets;
        
        // Gravity Hessian
        gravity_form_->hessian(x, tmp_triplets);
        for(auto& t : tmp_triplets) {
            triplets.emplace_back(t.row(), t.col(), t.value() * scaling);
        }

        // Constraint Hessian
        tmp_triplets.clear();
        constraint_form_->hessian(x, tmp_triplets);
        for(auto& t : tmp_triplets) {
            triplets.emplace_back(t.row(), t.col(), t.value()); // UN-SCALED!
        }

        // Other forms
        for (const auto& form : world_.forms) {
            tmp_triplets.clear();
            form->hessian(x, tmp_triplets);
            for(auto& t : tmp_triplets) {
                triplets.emplace_back(t.row(), t.col(), t.value() * scaling);
            }
        }

        hess.resize(x.size(), x.size());
        hess.setFromTriplets(triplets.begin(), triplets.end());
    }

    ConstraintForm* getConstraintForm() { return constraint_form_.get(); }

private:
    World& world_;
    std::shared_ptr<TimeIntegration::ImplicitTimeIntegrator> integrator_;
    std::unique_ptr<InertiaForm> inertia_form_;
    std::unique_ptr<GravityForm> gravity_form_;
    std::unique_ptr<ConstraintForm> constraint_form_;
};

IPCSolver::IPCSolver() {
    // Default integrator? Or require setting?
    // Let's create a default Euler to be safe, but usually setIntegrator is called.
    integrator_ = std::make_shared<TimeIntegration::ImplicitEuler>();
}

void IPCSolver::setIntegrator(std::shared_ptr<TimeIntegration::ImplicitTimeIntegrator> integrator) {
    integrator_ = integrator;
}

void IPCSolver::step(World& world, double dt) {
    if (!integrator_) {
        std::cerr << "Reference Error: Integrator not set in IPCSolver!" << std::endl;
        return;
    }

    // --- 1. Gather State from World ---
    // All bodies participate in optimization (no is_static skip).
    // Fixed bodies are constrained via FixedJoint.
    int n = static_cast<int>(world.bodies.size()) * 6;
    
    Eigen::VectorXd x_curr = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd v_curr = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd a_curr = Eigen::VectorXd::Zero(n);

    int idx = 0;
    for (const auto& body : world.bodies) {
        x_curr.segment<3>(idx) = body->position;
        x_curr.segment<3>(idx+3) = Eigen::Vector3d::Zero(); // incremental rotation

        v_curr.segment<3>(idx) = body->velocity;
        v_curr.segment<3>(idx+3) = body->angular_velocity;
        
        a_curr.segment<3>(idx) = body->linear_acceleration;
        a_curr.segment<3>(idx+3) = body->angular_acceleration;
        
        idx += 6;
    }

    // --- 2. Initialize Integrator (if needed) ---
    // Simple check: if dt changed or first step?
    // Let's just call init for now if we want to sync world state -> integrator. 
    // BUT integrator has internal state (x_prev, v_prev, a_prev).
    // If we call init every step, we are essentially doing "restart" every step.
    // For proper Newmark, we should use the accelerator's internal state if continuous.
    // However, World is the source of truth.
    // Let's trust World state is correct (updated by previous step).
    integrator_->init(x_curr, v_curr, a_curr, dt);

    // --- 3. Predict ---
    Eigen::VectorXd x_hat = integrator_->x_tilde();
    
    // --- 4. Optimization Loop (Newton) ---
    
    // Initial guess x_new = x_hat? Or x_curr?
    // Newmark solves for x_{n+1}. x_hat is just the predictor for inertia term.
    // A good initial guess for the solver is usually x_hat (predictor).
    Eigen::VectorXd x_new = x_hat; 

    // Helper maps: all bodies get an index now
    std::unordered_map<int, int> body_id_to_idx;
    std::unordered_map<int, std::shared_ptr<RigidBody>> body_id_to_ptr;
    idx = 0;
    for (const auto& body : world.bodies) {
        body_id_to_idx[body->id] = idx;
        body_id_to_ptr[body->id] = body;
        idx += 6;
    }

    const int max_alm_iters = 10;
    
    for(int alm_iter = 0; alm_iter < max_alm_iters; ++alm_iter) {
        // Update Joint State
        for (auto& joint : world.joints) {
            // RevoluteJoint
            auto revolute = std::dynamic_pointer_cast<NexDynIPC::Dynamics::RevoluteJoint>(joint);
            if (revolute) {
                int idA = revolute->getBodyAId();
                int idB = revolute->getBodyBId();
                
                int idxA = body_id_to_idx[idA];
                int idxB = body_id_to_idx[idB];
                Eigen::Vector3d pA = body_id_to_ptr[idA]->position;
                Eigen::Quaterniond qA = body_id_to_ptr[idA]->orientation;
                Eigen::Vector3d pB = body_id_to_ptr[idB]->position;
                Eigen::Quaterniond qB = body_id_to_ptr[idB]->orientation;
                revolute->updateState(idxA, idxB, pA, qA, pB, qB);
                continue;
            }

            // FixedJoint
            auto fixed = std::dynamic_pointer_cast<NexDynIPC::Dynamics::FixedJoint>(joint);
            if (fixed) {
                int bodyId = fixed->getBodyId();
                int bodyIdx = body_id_to_idx[bodyId];
                Eigen::Vector3d p = body_id_to_ptr[bodyId]->position;
                Eigen::Quaterniond q = body_id_to_ptr[bodyId]->orientation;
                fixed->updateState(bodyIdx, p, q);
                continue;
            }
        }

        SimulationProblem problem(world, integrator_);
        problem.setPredictiveState(x_hat);

        bool converged = solver_.minimize(problem, x_new);
        
        // Update Multipliers
        // Note: Lambdas need to be scaled? 
        // ALM update typically: lambda += mu * C(x).
        // Since we scaled C(x) in energy by `beta h^2`, the "effective" constraint force is scaled.
        // But lambda itself is physical force.
        // ConstraintForm::updateLambdas depends on implementation.
        problem.getConstraintForm()->updateLambdas(x_new);
    }

    // --- 5. Update Integrator State ---
    integrator_->update_quantities(x_new);

    // --- 6. Update World Bodies ---
    const Eigen::VectorXd& x_final = integrator_->x_prev(); // x_{n+1}
    const Eigen::VectorXd& v_final = integrator_->v_prev(); // v_{n+1}
    const Eigen::VectorXd& a_final = integrator_->a_prev(); // a_{n+1}

    idx = 0;
    for (auto& body : world.bodies) {
        Eigen::Vector3d p_new = x_final.segment<3>(idx);
        Eigen::Vector3d theta_new = x_final.segment<3>(idx+3);

        body->linear_acceleration = a_final.segment<3>(idx);
        body->angular_acceleration = a_final.segment<3>(idx+3);

        body->velocity = v_final.segment<3>(idx);
        body->angular_velocity = v_final.segment<3>(idx+3);

        body->position = p_new;

        // Update Orientation
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
