#include "NexDynIPC/Physics/Contact/Friction.h"

namespace NexDynIPC::Physics::Contact {

double FrictionPotential::value(
    const Eigen::Vector3d& tangent_displacement,
    double normal_force,
    double friction_coeff,
    double eps) {
    
    if (normal_force <= 0) return 0.0;
    
    double u_norm = tangent_displacement.norm();
    double smoothed_norm = std::sqrt(u_norm * u_norm + eps * eps);
    
    return friction_coeff * normal_force * (smoothed_norm - eps);
}

Eigen::Vector3d FrictionPotential::gradient(
    const Eigen::Vector3d& tangent_displacement,
    double normal_force,
    double friction_coeff,
    double eps) {
    
    if (normal_force <= 0) return Eigen::Vector3d::Zero();
    
    double u_norm = tangent_displacement.norm();
    if (u_norm < 1e-12) {
        // Handle zero displacement - gradient is zero
        return Eigen::Vector3d::Zero();
    }
    
    double smoothed_norm = std::sqrt(u_norm * u_norm + eps * eps);
    double scale = friction_coeff * normal_force / smoothed_norm;
    
    return scale * tangent_displacement;
}

Eigen::Matrix3d FrictionPotential::hessian(
    const Eigen::Vector3d& tangent_displacement,
    double normal_force,
    double friction_coeff,
    double eps) {
    
    if (normal_force <= 0) return Eigen::Matrix3d::Zero();
    
    double u_norm = tangent_displacement.norm();
    if (u_norm < 1e-12) {
        // Handle zero displacement - return regularized Hessian
        return (friction_coeff * normal_force / eps) * Eigen::Matrix3d::Identity();
    }
    
    double smoothed_norm = std::sqrt(u_norm * u_norm + eps * eps);
    double scale = friction_coeff * normal_force / smoothed_norm;
    
    // Hessian = scale * (I - u * u^T / (u_norm^2 + eps^2))
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Vector3d u = tangent_displacement;
    
    return scale * (I - u * u.transpose() / (smoothed_norm * smoothed_norm));
}

// Smooth polynomial friction (piecewise quadratic/linear)
double FrictionPotential::valueSmooth(
    const Eigen::Vector3d& tangent_displacement,
    double normal_force,
    double friction_coeff,
    double eps) {
    
    if (normal_force <= 0) return 0.0;
    
    double u_norm = tangent_displacement.norm();
    
    if (u_norm <= eps) {
        // Quadratic region: D(u) = 0.5 * mu * N / eps * ||u||^2
        return 0.5 * friction_coeff * normal_force / eps * u_norm * u_norm;
    } else {
        // Linear region: D(u) = mu * N * (||u|| - 0.5 * eps)
        return friction_coeff * normal_force * (u_norm - 0.5 * eps);
    }
}

Eigen::Vector3d FrictionPotential::gradientSmooth(
    const Eigen::Vector3d& tangent_displacement,
    double normal_force,
    double friction_coeff,
    double eps) {
    
    if (normal_force <= 0) return Eigen::Vector3d::Zero();
    
    double u_norm = tangent_displacement.norm();
    if (u_norm < 1e-12) {
        return Eigen::Vector3d::Zero();
    }
    
    Eigen::Vector3d u_hat = tangent_displacement / u_norm;
    
    if (u_norm <= eps) {
        // Quadratic region: grad = mu * N / eps * u
        return friction_coeff * normal_force / eps * tangent_displacement;
    } else {
        // Linear region: grad = mu * N * u_hat
        return friction_coeff * normal_force * u_hat;
    }
}

Eigen::Matrix3d FrictionPotential::hessianSmooth(
    const Eigen::Vector3d& tangent_displacement,
    double normal_force,
    double friction_coeff,
    double eps) {
    
    if (normal_force <= 0) return Eigen::Matrix3d::Zero();
    
    double u_norm = tangent_displacement.norm();
    
    if (u_norm < 1e-12) {
        // Return regularized Hessian
        return (friction_coeff * normal_force / eps) * Eigen::Matrix3d::Identity();
    }
    
    if (u_norm <= eps) {
        // Quadratic region: hess = mu * N / eps * I
        return (friction_coeff * normal_force / eps) * Eigen::Matrix3d::Identity();
    } else {
        // Linear region: hess = mu * N / ||u|| * (I - u_hat * u_hat^T)
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Vector3d u_hat = tangent_displacement / u_norm;
        
        return (friction_coeff * normal_force / u_norm) * (I - u_hat * u_hat.transpose());
    }
}

} // namespace NexDynIPC::Physics::Contact
