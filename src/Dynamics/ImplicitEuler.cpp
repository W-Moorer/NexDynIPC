#include "NexDynIPC/Dynamics/ImplicitEuler.hpp"
#include "NexDynIPC/Dynamics/Forms/InertiaForm.hpp"
#include "NexDynIPC/Dynamics/Forms/GravityForm.hpp"
#include "NexDynIPC/Dynamics/Forms/ConstraintForm.hpp"
#include "NexDynIPC/Dynamics/Joints/HingeJoint.hpp"
#include <iostream>
#include <unordered_map>

namespace NexDynIPC::Dynamics {

class SimulationProblem : public Math::OptimizationProblem {
public:
    SimulationProblem(World& world, double dt) : world_(world), dt_(dt) {
        // Create inertia form
        inertia_form_ = std::make_unique<InertiaForm>(world, dt);
        // Create gravity form
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
        double val = inertia_form_->value(x);
        double dt2 = dt_ * dt_;
        val += dt2 * gravity_form_->value(x); // Add gravity scaled by dt^2
        
        // ALM constraint energy is NOT scaled by dt² — it's a constraint
        // enforcement mechanism, not a potential energy. It must compete
        // directly with the inertia term (weight ~m) to properly enforce joints.
        val += constraint_form_->value(x);

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

        // Constraint gradient (NOT scaled by dt² — see computeValue comment)
        tmp_grad.setZero();
        constraint_form_->gradient(x, tmp_grad);
        grad += tmp_grad;

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

        // Constraint Hessian (NOT scaled by dt² — see computeValue comment)
        tmp_triplets.clear();
        constraint_form_->hessian(x, tmp_triplets);
        for(auto& t : tmp_triplets) {
            triplets.emplace_back(t.row(), t.col(), t.value());
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

    ConstraintForm* getConstraintForm() { return constraint_form_.get(); }

private:
    World& world_;
    double dt_;
    std::unique_ptr<InertiaForm> inertia_form_;
    std::unique_ptr<GravityForm> gravity_form_;
    std::unique_ptr<ConstraintForm> constraint_form_;
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
    
    // --- Map Body IDs to State ---
    std::unordered_map<int, int> body_id_to_idx;
    std::unordered_map<int, std::shared_ptr<RigidBody>> body_id_to_ptr;
    idx = 0;
    for (const auto& body : world.bodies) {
        if (!body->is_static) {
            body_id_to_idx[body->id] = idx;
            idx += 6;
        }
        body_id_to_ptr[body->id] = body;
    }

    // ALM Loop
    Eigen::VectorXd x_new = x_hat;
    const int max_alm_iters = 10;
    
    for(int alm_iter = 0; alm_iter < max_alm_iters; ++alm_iter) {
        // Update Joint State (indices and reference orientation)
        for (auto& joint : world.joints) {
            auto hinge = std::dynamic_pointer_cast<NexDynIPC::Dynamics::HingeJoint>(joint);
            if (hinge) {
                int idA = hinge->getBodyAId();
                int idB = hinge->getBodyBId();
                
                int idxA = -1;
                int idxB = -1;
                Eigen::Vector3d pA = Eigen::Vector3d::Zero();
                Eigen::Vector3d pB = Eigen::Vector3d::Zero();
                Eigen::Quaterniond qA = Eigen::Quaterniond::Identity();
                Eigen::Quaterniond qB = Eigen::Quaterniond::Identity();

                if (body_id_to_idx.count(idA)) {
                    idxA = body_id_to_idx[idA];
                    pA = body_id_to_ptr[idA]->position; // Not strictly needed if dynamic, but good for consistency
                    qA = body_id_to_ptr[idA]->orientation;
                } else if (body_id_to_ptr.count(idA)) {
                    // Static body
                    pA = body_id_to_ptr[idA]->position;
                    qA = body_id_to_ptr[idA]->orientation;
                }

                if (body_id_to_idx.count(idB)) {
                    idxB = body_id_to_idx[idB];
                    pB = body_id_to_ptr[idB]->position;
                    qB = body_id_to_ptr[idB]->orientation;
                } else if (body_id_to_ptr.count(idB)) {
                    // Static body
                    pB = body_id_to_ptr[idB]->position;
                    qB = body_id_to_ptr[idB]->orientation;
                }
                
                hinge->updateState(idxA, idxB, pA, qA, pB, qB);
            }
        }

        // Construct optimization problem
        SimulationProblem problem(world, dt);
        problem.setPredictiveState(x_hat);

        // Solve Inner Loop (Newton)
        bool converged = solver_.minimize(problem, x_new);
        
        // Update Multipliers
        problem.getConstraintForm()->updateLambdas(x_new);
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
