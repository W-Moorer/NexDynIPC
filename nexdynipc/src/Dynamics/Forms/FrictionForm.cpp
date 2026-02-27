#include "NexDynIPC/Dynamics/Forms/FrictionForm.h"
#include "NexDynIPC/Dynamics/RigidBody.h"

namespace NexDynIPC::Dynamics {

FrictionForm::FrictionForm(World& world, double friction_coeff, double eps)
    : world_(world), friction_coeff_(friction_coeff), eps_(eps) {}

double FrictionForm::value(const Eigen::VectorXd& x) const {
    double total_friction_energy = 0.0;
    
    for (size_t i = 0; i < contacts_.size(); ++i) {
        if (i >= normal_forces_.size()) continue;
        
        const auto& contact = contacts_[i];
        double normal_force = normal_forces_[i];
        
        if (normal_force <= 0) continue;
        
        Eigen::Vector3d tangent_disp = computeTangentDisplacement(contact, x);
        
        double friction_energy = Physics::Contact::FrictionPotential::value(
            tangent_disp, normal_force, friction_coeff_, eps_);
        
        total_friction_energy += friction_energy;
    }
    
    return total_friction_energy;
}

void FrictionForm::gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const {
    grad.setZero(x.size());
    
    for (size_t i = 0; i < contacts_.size(); ++i) {
        if (i >= normal_forces_.size()) continue;
        
        const auto& contact = contacts_[i];
        double normal_force = normal_forces_[i];
        
        if (normal_force <= 0) continue;
        
        Eigen::Vector3d tangent_disp = computeTangentDisplacement(contact, x);
        
        Eigen::Vector3d friction_grad = Physics::Contact::FrictionPotential::gradient(
            tangent_disp, normal_force, friction_coeff_, eps_);
        
        // Compute gradient w.r.t body DOFs
        // This requires the Jacobian of tangent displacement w.r.t body state
        // For simplicity, we approximate using finite differences or
        // use the contact normal to project the friction force
        
        int idxA = contact.bodyA_idx * 6;
        int idxB = contact.bodyB_idx * 6;
        
        // Project friction gradient onto body DOFs
        // Friction force opposes tangent displacement
        Eigen::Vector3d friction_force = -friction_grad;
        
        // Apply to body A (opposite direction)
        grad.segment<3>(idxA) += friction_force;
        
        // Apply to body B
        grad.segment<3>(idxB) -= friction_force;
    }
}

void FrictionForm::hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) const {
    for (size_t i = 0; i < contacts_.size(); ++i) {
        if (i >= normal_forces_.size()) continue;
        
        const auto& contact = contacts_[i];
        double normal_force = normal_forces_[i];
        
        if (normal_force <= 0) continue;
        
        Eigen::Vector3d tangent_disp = computeTangentDisplacement(contact, x);
        
        Eigen::Matrix3d friction_hess = Physics::Contact::FrictionPotential::hessian(
            tangent_disp, normal_force, friction_coeff_, eps_);
        
        int idxA = contact.bodyA_idx * 6;
        int idxB = contact.bodyB_idx * 6;
        
        // Add Hessian blocks for body A
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                triplets.emplace_back(idxA + row, idxA + col, friction_hess(row, col));
            }
        }
        
        // Add Hessian blocks for body B
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                triplets.emplace_back(idxB + row, idxB + col, friction_hess(row, col));
            }
        }
        
        // Add cross terms (negative)
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                triplets.emplace_back(idxA + row, idxB + col, -friction_hess(row, col));
                triplets.emplace_back(idxB + row, idxA + col, -friction_hess(row, col));
            }
        }
    }
}

void FrictionForm::updateNormalForces(const std::vector<double>& normal_forces) {
    normal_forces_ = normal_forces;
}

void FrictionForm::updateContactPoints(const std::vector<ContactPair>& contacts) {
    contacts_ = contacts;
}

Eigen::Vector3d FrictionForm::computeTangentDisplacement(
    const ContactPair& contact,
    const Eigen::VectorXd& x) const {
    
    // Get current body states
    Eigen::Vector3d posA, posB;
    Eigen::Quaterniond rotA, rotB;
    
    getBodyState(x, contact.bodyA_idx, posA, rotA);
    getBodyState(x, contact.bodyB_idx, posB, rotB);
    
    // Compute relative tangential velocity/displacement
    // This is a simplified version - in practice, you'd track the contact point
    // over time and compute the tangential displacement
    
    Eigen::Vector3d rel_velocity = 
        world_.bodies[contact.bodyA_idx]->velocity - 
        world_.bodies[contact.bodyB_idx]->velocity;
    
    // Project onto tangent plane (perpendicular to normal)
    Eigen::Vector3d normal = contact.normal;
    Eigen::Vector3d tangent_velocity = rel_velocity - rel_velocity.dot(normal) * normal;
    
    // Approximate displacement as velocity * dt
    // In a real implementation, you'd integrate this over the timestep
    double dt = 0.01; // Should be passed from solver
    return tangent_velocity * dt;
}

void FrictionForm::getBodyState(
    const Eigen::VectorXd& x,
    int body_idx,
    Eigen::Vector3d& position,
    Eigen::Quaterniond& orientation) const {
    
    int idx = body_idx * 6;
    position = x.segment<3>(idx);
    
    // Convert rotation vector to quaternion
    Eigen::Vector3d theta = x.segment<3>(idx + 3);
    double angle = theta.norm();
    if (angle > 1e-10) {
        Eigen::Vector3d axis = theta / angle;
        orientation = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
    } else {
        orientation = Eigen::Quaterniond::Identity();
    }
}

} // namespace NexDynIPC::Dynamics
