#include "NexDynIPC/Dynamics/Forms/InertiaForm.hpp"

namespace NexDynIPC::Dynamics {

InertiaForm::InertiaForm(const World& world, double dt) : world_(world), dt_(dt) {
    int n = 0;
    for (const auto& body : world.bodies) {
        if (!body->is_static) n += 6;
    }
    x_hat_ = Eigen::VectorXd::Zero(n);
}

void InertiaForm::setPredictiveState(const Eigen::VectorXd& x_hat) {
    x_hat_ = x_hat;
}

double InertiaForm::value(const Eigen::VectorXd& x) const {
    double energy = 0.0;
    int idx = 0;
    for (const auto& body : world_.bodies) {
        if (body->is_static) { 
            continue; 
        }

        // Translation
        Eigen::Vector3d p = x.segment<3>(idx);
        Eigen::Vector3d p_hat = x_hat_.segment<3>(idx);
        double m = body->mass;
        energy += 0.5 * m * (p - p_hat).squaredNorm();

        // Rotation
        // Assume scaled axis angle difference for small steps approximates Riemannian metric
        Eigen::Vector3d theta = x.segment<3>(idx + 3);
        Eigen::Vector3d theta_hat = x_hat_.segment<3>(idx + 3);
        
        Eigen::Matrix3d I = body->inertia_body; 
        energy += 0.5 * (theta - theta_hat).transpose() * I * (theta - theta_hat);

        idx += 6;
    }
    return energy;
}

void InertiaForm::gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const {
    // grad = M * (x - x_hat)
    int idx = 0;
    for (const auto& body : world_.bodies) {
        if (body->is_static) {
            continue;
        }

        // Translation
        Eigen::Vector3d p = x.segment<3>(idx);
        Eigen::Vector3d p_hat = x_hat_.segment<3>(idx);
        grad.segment<3>(idx) = body->mass * (p - p_hat);

        // Rotation
        Eigen::Vector3d theta = x.segment<3>(idx + 3);
        Eigen::Vector3d theta_hat = x_hat_.segment<3>(idx + 3);
        grad.segment<3>(idx + 3) = body->inertia_body * (theta - theta_hat);

        idx += 6;
    }
}

void InertiaForm::hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) const {
    // Hessian = M
    int idx = 0;
    for (const auto& body : world_.bodies) {
        if (body->is_static) {
            continue;
        }

        // Translation Mass
        double m = body->mass;
        triplets.emplace_back(idx + 0, idx + 0, m);
        triplets.emplace_back(idx + 1, idx + 1, m);
        triplets.emplace_back(idx + 2, idx + 2, m);

        // Rotation Inertia
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                 triplets.emplace_back(idx + 3 + i, idx + 3 + j, body->inertia_body(i, j));
            }
        }

        idx += 6;
    }
}

} // namespace NexDynIPC::Dynamics
