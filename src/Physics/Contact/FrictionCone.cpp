#include "NexDynIPC/Physics/Contact/FrictionCone.h"

namespace NexDynIPC::Physics::Contact {

Eigen::Vector3d FrictionCone::projectToCone(
    const Eigen::Vector3d& friction_force,
    double normal_force,
    double mu) {
    
    if (normal_force <= 0) {
        // No normal force means no friction force
        return Eigen::Vector3d::Zero();
    }
    
    double max_friction = mu * normal_force;
    double friction_norm = friction_force.norm();
    
    if (friction_norm <= max_friction) {
        // Already inside the cone
        return friction_force;
    }
    
    // Project onto the cone boundary
    return (max_friction / friction_norm) * friction_force;
}

bool FrictionCone::isInsideCone(
    const Eigen::Vector3d& friction_force,
    double normal_force,
    double mu) {
    
    if (normal_force <= 0) {
        return friction_force.norm() < 1e-12;
    }
    
    double max_friction = mu * normal_force;
    return friction_force.norm() <= max_friction + 1e-12;
}

double FrictionCone::distanceToCone(
    const Eigen::Vector3d& friction_force,
    double normal_force,
    double mu) {
    
    if (normal_force <= 0) {
        return friction_force.norm();
    }
    
    double max_friction = mu * normal_force;
    double friction_norm = friction_force.norm();
    
    if (friction_norm <= max_friction) {
        // Inside the cone - distance is negative (or zero on boundary)
        return friction_norm - max_friction;
    }
    
    // Outside the cone - positive distance
    return friction_norm - max_friction;
}

} // namespace NexDynIPC::Physics::Contact
