#pragma once

#include <Eigen/Core>
#include <cmath>

namespace NexDynIPC::Physics::Contact {

class FrictionCone {
public:
    // Project friction force onto the friction cone
    // The friction cone constraint is: ||f_t|| <= mu * f_n
    // where f_t is the tangential friction force and f_n is the normal force
    
    static Eigen::Vector3d projectToCone(
        const Eigen::Vector3d& friction_force,
        double normal_force,
        double mu);
    
    static bool isInsideCone(
        const Eigen::Vector3d& friction_force,
        double normal_force,
        double mu);
    
    // Compute the distance from the friction force to the cone boundary
    static double distanceToCone(
        const Eigen::Vector3d& friction_force,
        double normal_force,
        double mu);
    
    // Get the cone radius at a given normal force
    static double coneRadius(double normal_force, double mu) {
        return mu * std::max(0.0, normal_force);
    }
};

} // namespace NexDynIPC::Physics::Contact
