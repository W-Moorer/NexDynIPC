#include "NexDynIPC/Dynamics/IPCSolver.h"
#include "NexDynIPC/Dynamics/Forms/InertiaForm.h"
#include "NexDynIPC/Dynamics/Forms/GravityForm.h"
#include "NexDynIPC/Dynamics/Forms/ConstraintForm.h"
#include "NexDynIPC/Dynamics/Joints/RevoluteJoint.h"
#include "NexDynIPC/Dynamics/Joints/FixedJoint.h"
#include "NexDynIPC/TimeIntegration/ImplicitEulerIntegrator.h"
#include "NexDynIPC/Physics/Geometry/MeshShape.h"
#include <iostream>
#include <unordered_map>
#include <algorithm>

namespace NexDynIPC::Dynamics {

class SimulationProblem : public Math::OptimizationProblem {
public:
    SimulationProblem(World& world, std::shared_ptr<TimeIntegration::ImplicitTimeIntegrator> integrator) 
        : world_(world), integrator_(integrator) {
        inertia_form_ = std::make_unique<InertiaForm>(world, integrator_->dt());
        gravity_form_ = std::make_unique<GravityForm>(world, Eigen::Vector3d(0, -9.81, 0));
        
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
        double scaling = integrator_->acceleration_scaling();
        
        val += scaling * gravity_form_->value(x);
        val += constraint_form_->value(x);

        for (const auto& form : world_.forms) {
            val += scaling * form->value(x); 
        }
        return val;
    }

    void computeGradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) override {
        grad.setZero(x.size());
        
        Eigen::VectorXd tmp_grad(x.size());
        double scaling = integrator_->acceleration_scaling();
        
        tmp_grad.setZero();
        inertia_form_->gradient(x, tmp_grad);
        grad += tmp_grad;

        tmp_grad.setZero();
        gravity_form_->gradient(x, tmp_grad);
        grad += scaling * tmp_grad;

        tmp_grad.setZero();
        constraint_form_->gradient(x, tmp_grad);
        grad += tmp_grad;

        for (const auto& form : world_.forms) {
            tmp_grad.setZero();
            form->gradient(x, tmp_grad);
            grad += scaling * tmp_grad;
        }
    }

    void computeHessian(const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& hess) override {
        std::vector<Eigen::Triplet<double>> triplets;
        double scaling = integrator_->acceleration_scaling();
        
        inertia_form_->hessian(x, triplets);
        
        std::vector<Eigen::Triplet<double>> tmp_triplets;
        
        gravity_form_->hessian(x, tmp_triplets);
        for(auto& t : tmp_triplets) {
            triplets.emplace_back(t.row(), t.col(), t.value() * scaling);
        }

        tmp_triplets.clear();
        constraint_form_->hessian(x, tmp_triplets);
        for(auto& t : tmp_triplets) {
            triplets.emplace_back(t.row(), t.col(), t.value());
        }

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
    integrator_ = std::make_shared<TimeIntegration::ImplicitEuler>();
}

void IPCSolver::setIntegrator(std::shared_ptr<TimeIntegration::ImplicitTimeIntegrator> integrator) {
    integrator_ = integrator;
}

void IPCSolver::initializeAdaptiveBarrier(World& world) {
    if (world.bodies.empty()) return;
    
    double total_mass = 0.0;
    Eigen::Vector3d bbox_min(std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::infinity());
    Eigen::Vector3d bbox_max(-std::numeric_limits<double>::infinity(),
                              -std::numeric_limits<double>::infinity(),
                              -std::numeric_limits<double>::infinity());
    
    for (const auto& body : world.bodies) {
        total_mass += body->mass;
        
        bbox_min = bbox_min.cwiseMin(body->position);
        bbox_max = bbox_max.cwiseMax(body->position);
    }
    
    double avg_mass = total_mass / world.bodies.size();
    double bbox_diagonal = (bbox_max - bbox_min).norm();
    
    if (bbox_diagonal < 1e-6) {
        bbox_diagonal = 1.0;
    }
    
    adaptive_barrier_.initialize(avg_mass, bbox_diagonal);
}

double IPCSolver::computeMaxStep(World& world, double dt) {
    if (!enable_ccd_ || world.bodies.empty()) {
        return dt;
    }
    
    double toi = ccd_system_.computeEarliestTOI(world.bodies, dt);
    
    if (toi < 1.0) {
        return toi * ccd_safety_factor_ * dt;
    }
    
    return dt;
}

std::vector<double> IPCSolver::computeNormalForces(
    World& world,
    const Eigen::VectorXd& x) {
    
    std::vector<double> normal_forces;
    
    // Compute normal forces from barrier potential gradient
    // This is a simplified version - in practice, you'd compute this from
    // the barrier potential gradient at each contact point
    
    for (size_t i = 0; i < world.bodies.size(); ++i) {
        for (size_t j = i + 1; j < world.bodies.size(); ++j) {
            auto meshA = std::dynamic_pointer_cast<Physics::MeshShape>(world.bodies[i]->shape);
            auto meshB = std::dynamic_pointer_cast<Physics::MeshShape>(world.bodies[j]->shape);
            
            if (!meshA || !meshB) continue;
            
            // Compute approximate normal force from barrier
            // F_n = -grad_barrier
            double dhat = adaptive_barrier_.currentDhat();
            double kappa = adaptive_barrier_.currentStiffness();
            
            // For each potential contact, compute normal force
            // This is simplified - real implementation would check actual contacts
            Eigen::Vector3d posA = world.bodies[i]->position;
            Eigen::Vector3d posB = world.bodies[j]->position;
            double dist = (posA - posB).norm();
            
            if (dist < dhat && dist > 0) {
                double barrier_grad = Physics::Contact::BarrierPotential::gradient(dist, dhat, kappa);
                double normal_force = std::abs(barrier_grad);
                normal_forces.push_back(normal_force);
            }
        }
    }
    
    return normal_forces;
}

std::vector<ContactPair> IPCSolver::detectContacts(World& world) {
    std::vector<ContactPair> contacts;
    
    for (size_t i = 0; i < world.bodies.size(); ++i) {
        for (size_t j = i + 1; j < world.bodies.size(); ++j) {
            auto meshA = std::dynamic_pointer_cast<Physics::MeshShape>(world.bodies[i]->shape);
            auto meshB = std::dynamic_pointer_cast<Physics::MeshShape>(world.bodies[j]->shape);
            
            if (!meshA || !meshB) continue;
            
            // Simplified contact detection
            // In practice, you'd use the actual collision detection results
            Eigen::Vector3d posA = world.bodies[i]->position;
            Eigen::Vector3d posB = world.bodies[j]->position;
            double dist = (posA - posB).norm();
            
            double dhat = adaptive_barrier_.currentDhat();
            
            if (dist < dhat && dist > 0) {
                ContactPair contact;
                contact.bodyA_idx = i;
                contact.bodyB_idx = j;
                contact.primitive_type = 0; // vertex-face approximation
                contact.primitiveA_idx = 0;
                contact.primitiveB_idx = 0;
                contact.contact_point = 0.5 * (posA + posB);
                contact.normal = (posB - posA).normalized();
                contact.distance = dist;
                
                contacts.push_back(contact);
            }
        }
    }
    
    return contacts;
}

void IPCSolver::step(World& world, double dt) {
    if (!integrator_) {
        std::cerr << "Reference Error: Integrator not set in IPCSolver!" << std::endl;
        return;
    }

    if (enable_adaptive_barrier_) {
        static bool barrier_initialized = false;
        if (!barrier_initialized) {
            initializeAdaptiveBarrier(world);
            barrier_initialized = true;
        }
        
        double min_dist = Physics::Contact::computeSceneMinDistance(world.bodies);
        if (min_dist < std::numeric_limits<double>::infinity()) {
            adaptive_barrier_.updateStiffness(min_dist);
        }
    }

    double actual_dt = dt;
    if (enable_ccd_) {
        actual_dt = computeMaxStep(world, dt);
    }

    int n = static_cast<int>(world.bodies.size()) * 6;
    
    Eigen::VectorXd x_curr = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd v_curr = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd a_curr = Eigen::VectorXd::Zero(n);

    int idx = 0;
    for (const auto& body : world.bodies) {
        x_curr.segment<3>(idx) = body->position;
        x_curr.segment<3>(idx+3) = Eigen::Vector3d::Zero();

        v_curr.segment<3>(idx) = body->velocity;
        v_curr.segment<3>(idx+3) = body->angular_velocity;
        
        a_curr.segment<3>(idx) = body->linear_acceleration;
        a_curr.segment<3>(idx+3) = body->angular_acceleration;
        
        idx += 6;
    }

    integrator_->init(x_curr, v_curr, a_curr, actual_dt);

    Eigen::VectorXd x_hat = integrator_->x_tilde();
    Eigen::VectorXd x_new = x_hat; 

    std::unordered_map<int, int> body_id_to_idx;
    std::unordered_map<int, std::shared_ptr<RigidBody>> body_id_to_ptr;
    idx = 0;
    for (const auto& body : world.bodies) {
        body_id_to_idx[body->id] = idx;
        body_id_to_ptr[body->id] = body;
        idx += 6;
    }

    // Setup friction form if enabled
    if (enable_friction_ && friction_coeff_ > 0) {
        friction_form_ = std::make_shared<FrictionForm>(world, friction_coeff_);
        auto contacts = detectContacts(world);
        friction_form_->updateContactPoints(contacts);
        world.forms.push_back(friction_form_);
    }

    const int max_alm_iters = 10;
    
    for(int alm_iter = 0; alm_iter < max_alm_iters; ++alm_iter) {
        for (auto& joint : world.joints) {
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
        
        problem.getConstraintForm()->updateLambdas(x_new);
        
        // Update normal forces for friction
        if (enable_friction_ && friction_form_) {
            auto normal_forces = computeNormalForces(world, x_new);
            friction_form_->updateNormalForces(normal_forces);
        }
    }

    integrator_->update_quantities(x_new);

    const Eigen::VectorXd& x_final = integrator_->x_prev();
    const Eigen::VectorXd& v_final = integrator_->v_prev();
    const Eigen::VectorXd& a_final = integrator_->a_prev();

    idx = 0;
    for (auto& body : world.bodies) {
        Eigen::Vector3d p_new = x_final.segment<3>(idx);
        Eigen::Vector3d theta_new = x_final.segment<3>(idx+3);

        body->linear_acceleration = a_final.segment<3>(idx);
        body->angular_acceleration = a_final.segment<3>(idx+3);

        body->velocity = v_final.segment<3>(idx);
        body->angular_velocity = v_final.segment<3>(idx+3);

        body->position = p_new;

        double angle = theta_new.norm();
        if (angle > 1e-10) {
            Eigen::Vector3d axis = theta_new / angle;
            Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, axis));
            body->orientation = (dq * body->orientation).normalized();
        }

        idx += 6;
    }
    
    // Remove friction form from world forms after step
    if (enable_friction_ && friction_form_) {
        world.forms.erase(
            std::remove(world.forms.begin(), world.forms.end(), friction_form_),
            world.forms.end());
    }
}

} // namespace NexDynIPC::Dynamics
