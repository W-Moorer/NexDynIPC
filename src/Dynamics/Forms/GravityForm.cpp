#include "NexDynIPC/Dynamics/Forms/GravityForm.hpp"

namespace NexDynIPC::Dynamics {

GravityForm::GravityForm(const World& world, const Eigen::Vector3d& g) 
    : world_(world), g_(g) {}

double GravityForm::value(const Eigen::VectorXd& x) const {
    double val = 0.0;
    int idx = 0;
    for (const auto& body : world_.bodies) {
        Eigen::Vector3d p = x.segment<3>(idx);
        // PE = - m * g^T * p
        val -= body->mass * g_.dot(p);
        idx += 6;
    }
    return val;
}

void GravityForm::gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const {
    int idx = 0;
    for (const auto& body : world_.bodies) {
        // Force = m * g
        // Potential gradient = - Force = -m * g
        grad.segment<3>(idx) = -body->mass * g_;
        grad.segment<3>(idx + 3).setZero(); // No torque from gravity at CM
        idx += 6;
    }
}

void GravityForm::hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) const {
    // Gravity is linear, Hessian is zero
}

} // namespace NexDynIPC::Dynamics
