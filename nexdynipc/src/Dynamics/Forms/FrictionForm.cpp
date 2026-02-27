#include "NexDynIPC/Dynamics/Forms/FrictionForm.h"
#include "NexDynIPC/Dynamics/RigidBody.h"

namespace NexDynIPC::Dynamics {

namespace {
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d s = Eigen::Matrix3d::Zero();
    s(0, 1) = -v.z();
    s(0, 2) = v.y();
    s(1, 0) = v.z();
    s(1, 2) = -v.x();
    s(2, 0) = -v.y();
    s(2, 1) = v.x();
    return s;
}
}

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
        
        int idxA = contact.bodyA_idx * 6;
        int idxB = contact.bodyB_idx * 6;

        Eigen::Vector3d posA, posB;
        Eigen::Quaterniond rotA, rotB;
        getBodyState(x, contact.bodyA_idx, posA, rotA);
        getBodyState(x, contact.bodyB_idx, posB, rotB);

        const Eigen::Vector3d n = (contact.normal.squaredNorm() > 1e-16)
            ? contact.normal.normalized()
            : Eigen::Vector3d::UnitY();
        const Eigen::Matrix3d tangent_projector =
            Eigen::Matrix3d::Identity() - n * n.transpose();

        const auto& bodyA = world_.bodies[contact.bodyA_idx];
        const auto& bodyB = world_.bodies[contact.bodyB_idx];
        const Eigen::Vector3d localA = bodyA->orientation.conjugate() * (contact.contact_point - bodyA->position);
        const Eigen::Vector3d localB = bodyB->orientation.conjugate() * (contact.contact_point - bodyB->position);
        const Eigen::Vector3d rA = rotA * localA;
        const Eigen::Vector3d rB = rotB * localB;

        Eigen::Matrix<double, 3, 6> JA = Eigen::Matrix<double, 3, 6>::Zero();
        Eigen::Matrix<double, 3, 6> JB = Eigen::Matrix<double, 3, 6>::Zero();

        JA.block<3, 3>(0, 0) = -tangent_projector;
        JB.block<3, 3>(0, 0) = tangent_projector;
        JA.block<3, 3>(0, 3) = tangent_projector * skewSymmetric(rA);
        JB.block<3, 3>(0, 3) = -tangent_projector * skewSymmetric(rB);

        grad.segment<6>(idxA) += JA.transpose() * friction_grad;
        grad.segment<6>(idxB) += JB.transpose() * friction_grad;
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

        Eigen::Vector3d posA, posB;
        Eigen::Quaterniond rotA, rotB;
        getBodyState(x, contact.bodyA_idx, posA, rotA);
        getBodyState(x, contact.bodyB_idx, posB, rotB);

        const Eigen::Vector3d n = (contact.normal.squaredNorm() > 1e-16)
            ? contact.normal.normalized()
            : Eigen::Vector3d::UnitY();
        const Eigen::Matrix3d tangent_projector =
            Eigen::Matrix3d::Identity() - n * n.transpose();

        const auto& bodyA = world_.bodies[contact.bodyA_idx];
        const auto& bodyB = world_.bodies[contact.bodyB_idx];
        const Eigen::Vector3d localA = bodyA->orientation.conjugate() * (contact.contact_point - bodyA->position);
        const Eigen::Vector3d localB = bodyB->orientation.conjugate() * (contact.contact_point - bodyB->position);
        const Eigen::Vector3d rA = rotA * localA;
        const Eigen::Vector3d rB = rotB * localB;

        Eigen::Matrix<double, 3, 6> JA = Eigen::Matrix<double, 3, 6>::Zero();
        Eigen::Matrix<double, 3, 6> JB = Eigen::Matrix<double, 3, 6>::Zero();

        JA.block<3, 3>(0, 0) = -tangent_projector;
        JB.block<3, 3>(0, 0) = tangent_projector;
        JA.block<3, 3>(0, 3) = tangent_projector * skewSymmetric(rA);
        JB.block<3, 3>(0, 3) = -tangent_projector * skewSymmetric(rB);

        const Eigen::Matrix<double, 6, 6> HAA = JA.transpose() * friction_hess * JA;
        const Eigen::Matrix<double, 6, 6> HBB = JB.transpose() * friction_hess * JB;
        const Eigen::Matrix<double, 6, 6> HAB = JA.transpose() * friction_hess * JB;
        const Eigen::Matrix<double, 6, 6> HBA = JB.transpose() * friction_hess * JA;

        for (int row = 0; row < 6; ++row) {
            for (int col = 0; col < 6; ++col) {
                triplets.emplace_back(idxA + row, idxA + col, HAA(row, col));
                triplets.emplace_back(idxB + row, idxB + col, HBB(row, col));
                triplets.emplace_back(idxA + row, idxB + col, HAB(row, col));
                triplets.emplace_back(idxB + row, idxA + col, HBA(row, col));
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
    
    const Eigen::Vector3d prev_posA = world_.bodies[contact.bodyA_idx]->position;
    const Eigen::Vector3d prev_posB = world_.bodies[contact.bodyB_idx]->position;

    const Eigen::Vector3d dispA = posA - prev_posA;
    const Eigen::Vector3d dispB = posB - prev_posB;
    const auto& bodyA = world_.bodies[contact.bodyA_idx];
    const auto& bodyB = world_.bodies[contact.bodyB_idx];
    const Eigen::Vector3d localA = bodyA->orientation.conjugate() * (contact.contact_point - bodyA->position);
    const Eigen::Vector3d localB = bodyB->orientation.conjugate() * (contact.contact_point - bodyB->position);
    const Eigen::Vector3d rA = rotA * localA;
    const Eigen::Vector3d rB = rotB * localB;

    Eigen::Vector3d omegaA = Eigen::Vector3d::Zero();
    Eigen::Vector3d omegaB = Eigen::Vector3d::Zero();
    Eigen::Vector3d point_velA = Eigen::Vector3d::Zero();
    Eigen::Vector3d point_velB = Eigen::Vector3d::Zero();
    if (dt_ > 1e-12) {
        omegaA = x.segment<3>(contact.bodyA_idx * 6 + 3) / dt_;
        omegaB = x.segment<3>(contact.bodyB_idx * 6 + 3) / dt_;
        point_velA = dispA / dt_;
        point_velB = dispB / dt_;
    }
    point_velA += omegaA.cross(rA);
    point_velB += omegaB.cross(rB);

    const Eigen::Vector3d rel_velocity = point_velB - point_velA;

    const Eigen::Vector3d normal = (contact.normal.squaredNorm() > 1e-16)
        ? contact.normal.normalized()
        : Eigen::Vector3d::UnitY();
    const Eigen::Vector3d tangent_velocity = rel_velocity - rel_velocity.dot(normal) * normal;
    return tangent_velocity * dt_;
}

void FrictionForm::getBodyState(
    const Eigen::VectorXd& x,
    int body_idx,
    Eigen::Vector3d& position,
    Eigen::Quaterniond& orientation) const {
    
    int idx = body_idx * 6;
    position = x.segment<3>(idx);
    
    const Eigen::Vector3d theta = x.segment<3>(idx + 3);
    const double angle = theta.norm();
    if (angle > 1e-10) {
        const Eigen::Vector3d axis = theta / angle;
        const Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, axis));
        orientation = (dq * world_.bodies[body_idx]->orientation).normalized();
    } else {
        orientation = world_.bodies[body_idx]->orientation;
    }
}

} // namespace NexDynIPC::Dynamics
