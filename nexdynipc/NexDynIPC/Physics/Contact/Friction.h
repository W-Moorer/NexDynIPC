#pragma once

#include <Eigen/Core>
#include <cmath>

namespace NexDynIPC::Physics::Contact {

class FrictionPotential {
public:
    // Smooth approximation of Coulomb friction using Huber loss
    // D(u) = mu * N * (sqrt(||u||^2 + eps^2) - eps)
    // This is a smoothed version of D(u) = mu * N * ||u||
    
    static double value(
        const Eigen::Vector3d& tangent_displacement,
        double normal_force,
        double friction_coeff,
        double eps = 1e-8);
    
    static Eigen::Vector3d gradient(
        const Eigen::Vector3d& tangent_displacement,
        double normal_force,
        double friction_coeff,
        double eps = 1e-8);
    
    static Eigen::Matrix3d hessian(
        const Eigen::Vector3d& tangent_displacement,
        double normal_force,
        double friction_coeff,
        double eps = 1e-8);
    
    // Alternative: smooth friction using polynomial approximation
    // For small displacements: D(u) ≈ 0.5 * mu * N / eps * ||u||^2
    // For large displacements: D(u) ≈ mu * N * (||u|| - 0.5 * eps)
    static double valueSmooth(
        const Eigen::Vector3d& tangent_displacement,
        double normal_force,
        double friction_coeff,
        double eps = 1e-8);
    
    static Eigen::Vector3d gradientSmooth(
        const Eigen::Vector3d& tangent_displacement,
        double normal_force,
        double friction_coeff,
        double eps = 1e-8);
    
    static Eigen::Matrix3d hessianSmooth(
        const Eigen::Vector3d& tangent_displacement,
        double normal_force,
        double friction_coeff,
        double eps = 1e-8);
};

} // namespace NexDynIPC::Physics::Contact
