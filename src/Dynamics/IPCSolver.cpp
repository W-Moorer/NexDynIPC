#include "NexDynIPC/Dynamics/IPCSolver.h"
#include "NexDynIPC/Dynamics/Forms/InertiaForm.h"
#include "NexDynIPC/Dynamics/Forms/GravityForm.h"
#include "NexDynIPC/Dynamics/Forms/ConstraintForm.h"
#include "NexDynIPC/Dynamics/Joints/HingeJoint.h"
#include "NexDynIPC/Dynamics/Joints/FixedJoint.h"
#include "NexDynIPC/Dynamics/Joints/SphericalJoint.h"
#include "NexDynIPC/Dynamics/Joints/PrismaticJoint.h"
#include "NexDynIPC/Dynamics/Joints/CylindricalJoint.h"
#include "NexDynIPC/Dynamics/Joints/AngleLimitJoint.h"
#include "NexDynIPC/Dynamics/Joints/DistanceLimitJoint.h"
#include "NexDynIPC/Control/VelocityDriveForm.h"
#include "NexDynIPC/Control/LinearVelocityDriveForm.h"
#include "NexDynIPC/Control/PositionDriveForm.h"
#include "NexDynIPC/Control/ForceDriveForm.h"
#include "NexDynIPC/Control/DampedSpringForm.h"
#include "NexDynIPC/TimeIntegration/ImplicitEulerIntegrator.h"
#include "NexDynIPC/Physics/Geometry/MeshShape.h"
#include "NexDynIPC/Physics/Contact/BroadPhase.h"
#include "NexDynIPC/Physics/Contact/CollisionCandidates.h"
#include "NexDynIPC/Physics/Contact/Distance.h"
#include <iostream>
#include <unordered_map>
#include <algorithm>
#include <cmath>

namespace NexDynIPC::Dynamics {

namespace {
constexpr double kMaxALMStiffness = 1e12;
}

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
    (void)world;
    (void)x;
    std::vector<double> normal_forces;

    normal_forces.reserve(cached_contacts_.size());
    const double dhat = adaptive_barrier_.currentDhat();
    const double kappa = adaptive_barrier_.currentStiffness();
    for (const auto& contact : cached_contacts_) {
        if (contact.distance > 0.0 && contact.distance < dhat) {
            const double barrier_grad = Physics::BarrierPotential::gradient(contact.distance, dhat, kappa);
            normal_forces.push_back(std::abs(barrier_grad));
        } else {
            normal_forces.push_back(0.0);
        }
    }

    cached_normal_forces_ = normal_forces;
    return normal_forces;
}

std::vector<ContactPair> IPCSolver::detectContacts(World& world) {
    std::vector<ContactPair> contacts;

    std::vector<Physics::Geometry::AABB> aabbs;
    aabbs.reserve(world.bodies.size());

    for (const auto& body : world.bodies) {
        const auto mesh = std::dynamic_pointer_cast<Physics::MeshShape>(body->shape);
        if (!mesh) {
            aabbs.emplace_back(body->position, body->position);
            continue;
        }

        const auto [local_min, local_max] = mesh->computeAABB();
        Eigen::Vector3d world_min = body->toWorld(local_min);
        Eigen::Vector3d world_max = body->toWorld(local_max);
        aabbs.emplace_back(world_min.cwiseMin(world_max), world_min.cwiseMax(world_max));
    }

    Physics::BVHBroadPhase broad_phase(adaptive_barrier_.currentDhat());
    const auto broad_pairs = broad_phase.detect(aabbs);

    Physics::Contact::CollisionCandidatesBuilder candidate_builder;
    const auto candidates = candidate_builder.build(world.bodies, broad_pairs);

    const auto distances = Physics::Contact::computeNarrowPhaseDistances(
        candidates,
        world.bodies,
        adaptive_barrier_.currentDhat());

    contacts.reserve(distances.size());
    for (const auto& d : distances) {
        ContactPair contact;
        contact.bodyA_idx = d.bodyA_idx;
        contact.bodyB_idx = d.bodyB_idx;
        contact.primitive_type = static_cast<int>(d.type);
        contact.primitiveA_idx = d.primitiveA_idx;
        contact.primitiveB_idx = d.primitiveB_idx;
        contact.contact_point = 0.5 * (d.pointA + d.pointB);
        contact.normal = d.normal;
        contact.distance = d.distance;
        contacts.push_back(contact);
    }

    cached_contacts_ = contacts;
    return contacts;
}

void IPCSolver::stepSingle(World& world, double dt) {
    if (!integrator_) {
        std::cerr << "Reference Error: Integrator not set in IPCSolver!" << std::endl;
        return;
    }

    cached_contacts_.clear();
    cached_normal_forces_.clear();

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
        friction_form_->setTimeStep(actual_dt);
        auto contacts = detectContacts(world);
        friction_form_->updateContactPoints(contacts);
        world.forms.push_back(friction_form_);
    }

    for (const auto& form : world.forms) {
        auto drive = std::dynamic_pointer_cast<NexDynIPC::Control::VelocityDriveForm>(form);
        if (drive) {
            drive->setTimeStep(actual_dt);
            drive->advanceControlState();
            drive->updateGlobalIndices(world.bodies);
            continue;
        }

        auto linear_drive = std::dynamic_pointer_cast<NexDynIPC::Control::LinearVelocityDriveForm>(form);
        if (linear_drive) {
            linear_drive->setTimeStep(actual_dt);
            linear_drive->advanceControlState();
            linear_drive->updateGlobalIndices(world.bodies);
            continue;
        }

        auto position_drive = std::dynamic_pointer_cast<NexDynIPC::Control::PositionDriveForm>(form);
        if (position_drive) {
            position_drive->setTimeStep(actual_dt);
            position_drive->updateGlobalIndices(world.bodies);
            continue;
        }

        auto force_drive = std::dynamic_pointer_cast<NexDynIPC::Control::ForceDriveForm>(form);
        if (force_drive) {
            force_drive->setTimeStep(actual_dt);
            force_drive->updateGlobalIndices(world.bodies);
            continue;
        }

        auto damped_spring = std::dynamic_pointer_cast<NexDynIPC::Control::DampedSpringForm>(form);
        if (damped_spring) {
            damped_spring->updateGlobalIndices(world.bodies);
        }
    }

    last_max_constraint_violation_ = 0.0;
    last_dual_residual_ = 0.0;

    for(int alm_iter = 0; alm_iter < alm_max_iters_; ++alm_iter) {
        for (auto& joint : world.joints) {
            auto hinge = std::dynamic_pointer_cast<NexDynIPC::Dynamics::HingeJoint>(joint);
            if (hinge) {
                int idA = hinge->getBodyAId();
                int idB = hinge->getBodyBId();
                
                int idxA = body_id_to_idx[idA];
                int idxB = body_id_to_idx[idB];
                Eigen::Vector3d pA = body_id_to_ptr[idA]->position;
                Eigen::Quaterniond qA = body_id_to_ptr[idA]->orientation;
                Eigen::Vector3d pB = body_id_to_ptr[idB]->position;
                Eigen::Quaterniond qB = body_id_to_ptr[idB]->orientation;
                hinge->updateState(idxA, idxB, pA, qA, pB, qB);
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

            auto spherical = std::dynamic_pointer_cast<NexDynIPC::Dynamics::SphericalJoint>(joint);
            if (spherical) {
                int idA = spherical->getBodyAId();
                int idB = spherical->getBodyBId();

                int idxA = body_id_to_idx[idA];
                int idxB = body_id_to_idx[idB];
                Eigen::Vector3d pA = body_id_to_ptr[idA]->position;
                Eigen::Quaterniond qA = body_id_to_ptr[idA]->orientation;
                Eigen::Vector3d pB = body_id_to_ptr[idB]->position;
                Eigen::Quaterniond qB = body_id_to_ptr[idB]->orientation;
                spherical->updateState(idxA, idxB, pA, qA, pB, qB);
                continue;
            }

            auto prismatic = std::dynamic_pointer_cast<NexDynIPC::Dynamics::PrismaticJoint>(joint);
            if (prismatic) {
                int idA = prismatic->getBodyAId();
                int idB = prismatic->getBodyBId();

                int idxA = body_id_to_idx[idA];
                int idxB = body_id_to_idx[idB];
                Eigen::Vector3d pA = body_id_to_ptr[idA]->position;
                Eigen::Quaterniond qA = body_id_to_ptr[idA]->orientation;
                Eigen::Vector3d pB = body_id_to_ptr[idB]->position;
                Eigen::Quaterniond qB = body_id_to_ptr[idB]->orientation;
                prismatic->updateState(idxA, idxB, pA, qA, pB, qB);
                continue;
            }

            auto cylindrical = std::dynamic_pointer_cast<NexDynIPC::Dynamics::CylindricalJoint>(joint);
            if (cylindrical) {
                int idA = cylindrical->getBodyAId();
                int idB = cylindrical->getBodyBId();

                int idxA = body_id_to_idx[idA];
                int idxB = body_id_to_idx[idB];
                Eigen::Vector3d pA = body_id_to_ptr[idA]->position;
                Eigen::Quaterniond qA = body_id_to_ptr[idA]->orientation;
                Eigen::Vector3d pB = body_id_to_ptr[idB]->position;
                Eigen::Quaterniond qB = body_id_to_ptr[idB]->orientation;
                cylindrical->updateState(idxA, idxB, pA, qA, pB, qB);
                continue;
            }

            auto angle_limit = std::dynamic_pointer_cast<NexDynIPC::Dynamics::AngleLimitJoint>(joint);
            if (angle_limit) {
                int idA = angle_limit->getBodyAId();
                int idB = angle_limit->getBodyBId();

                int idxA = body_id_to_idx[idA];
                int idxB = body_id_to_idx[idB];
                Eigen::Vector3d pA = body_id_to_ptr[idA]->position;
                Eigen::Quaterniond qA = body_id_to_ptr[idA]->orientation;
                Eigen::Vector3d pB = body_id_to_ptr[idB]->position;
                Eigen::Quaterniond qB = body_id_to_ptr[idB]->orientation;
                angle_limit->updateState(idxA, idxB, pA, qA, pB, qB);
                continue;
            }

            auto distance_limit = std::dynamic_pointer_cast<NexDynIPC::Dynamics::DistanceLimitJoint>(joint);
            if (distance_limit) {
                int idA = distance_limit->getBodyAId();
                int idB = distance_limit->getBodyBId();

                int idxA = body_id_to_idx[idA];
                int idxB = body_id_to_idx[idB];
                Eigen::Vector3d pA = body_id_to_ptr[idA]->position;
                Eigen::Quaterniond qA = body_id_to_ptr[idA]->orientation;
                Eigen::Vector3d pB = body_id_to_ptr[idB]->position;
                Eigen::Quaterniond qB = body_id_to_ptr[idB]->orientation;
                distance_limit->updateState(idxA, idxB, pA, qA, pB, qB);
                continue;
            }
        }

        SimulationProblem problem(world, integrator_);
        problem.setPredictiveState(x_hat);

        bool converged = solver_.minimize(problem, x_new);
        if (!converged && enable_newton_fallback_) {
            for (int retry = 0; retry < newton_fallback_retries_ && !converged; ++retry) {
                x_new = newton_fallback_damping_ * x_hat + (1.0 - newton_fallback_damping_) * x_new;
                converged = solver_.minimize(problem, x_new);
                ++last_newton_fallbacks_;
            }
        }
        if (!converged) {
            last_solver_converged_ = false;
        }
        
        double max_constraint_violation = 0.0;
        double dual_residual = 0.0;
        for (const auto& joint : problem.getConstraintForm()->getJoints()) {
            Eigen::VectorXd C;
            joint->computeC(x_new, C);
            if (C.size() > 0) {
                const double inf_norm = C.cwiseAbs().maxCoeff();
                max_constraint_violation = std::max(max_constraint_violation, inf_norm);
                const double dual_candidate = joint->getStiffness() * inf_norm;
                if (std::isfinite(dual_candidate)) {
                    dual_residual = std::max(dual_residual, std::min(dual_candidate, kMaxALMStiffness));
                } else {
                    dual_residual = std::max(dual_residual, kMaxALMStiffness);
                }

                if (inf_norm > alm_hardening_trigger_ * alm_constraint_tolerance_) {
                    const double next_stiffness = joint->getStiffness() * alm_hardening_ratio_;
                    if (std::isfinite(next_stiffness)) {
                        joint->setStiffness(std::min(next_stiffness, kMaxALMStiffness));
                    } else {
                        joint->setStiffness(kMaxALMStiffness);
                    }
                }
            }
        }

        problem.getConstraintForm()->updateLambdas(x_new);
        
        // Update normal forces for friction
        if (enable_friction_ && friction_form_) {
            auto normal_forces = computeNormalForces(world, x_new);
            friction_form_->updateNormalForces(normal_forces);
        }

        last_max_constraint_violation_ = max_constraint_violation;
        last_dual_residual_ = dual_residual;

        if (converged
            && max_constraint_violation <= alm_constraint_tolerance_
            && dual_residual <= alm_dual_tolerance_) {
            break;
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

    double max_vw = 0.0;
    int drive_count = 0;
    int saturated_count = 0;
    for (const auto& form : world.forms) {
        auto drive = std::dynamic_pointer_cast<NexDynIPC::Control::VelocityDriveForm>(form);
        if (!drive) {
            continue;
        }
        ++drive_count;
        max_vw = std::max(max_vw, std::abs(drive->getVelocityErrorFromBodies()));
        if (drive->isTorqueSaturatedFromBodies()) {
            ++saturated_count;
        }
    }

    last_angular_velocity_error_ = max_vw;
    last_torque_saturation_ratio_ = (drive_count > 0)
        ? static_cast<double>(saturated_count) / static_cast<double>(drive_count)
        : 0.0;
    
    // Remove friction form from world forms after step
    if (enable_friction_ && friction_form_) {
        world.forms.erase(
            std::remove(world.forms.begin(), world.forms.end(), friction_form_),
            world.forms.end());
    }
}

void IPCSolver::step(World& world, double dt) {
    if (!enable_ccd_) {
        last_ccd_toi_ratio_ = 1.0;
        last_ccd_substeps_ = 1;
        last_newton_fallbacks_ = 0;
        last_solver_converged_ = true;
        stepSingle(world, dt);
        return;
    }

    double remaining_dt = dt;
    int substeps = 0;
    const double min_substep_dt = std::max(1e-8, ccd_min_step_ratio_ * dt);

    last_newton_fallbacks_ = 0;
    last_solver_converged_ = true;
    last_ccd_toi_ratio_ = 1.0;

    while (remaining_dt > 1e-12 && substeps < ccd_max_substeps_) {
        double step_dt = computeMaxStep(world, remaining_dt);
        if (remaining_dt > 1e-12) {
            last_ccd_toi_ratio_ = std::max(0.0, std::min(1.0, step_dt / remaining_dt));
        }

        if (step_dt < min_substep_dt) {
            step_dt = std::min(min_substep_dt, remaining_dt);
        }

        if (step_dt <= 1e-12) {
            break;
        }

        stepSingle(world, step_dt);
        remaining_dt -= step_dt;
        ++substeps;
    }

    if (remaining_dt > 1e-10) {
        stepSingle(world, remaining_dt);
        ++substeps;
    }

    last_ccd_substeps_ = std::max(1, substeps);
}

} // namespace NexDynIPC::Dynamics
