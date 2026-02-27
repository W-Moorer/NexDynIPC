#include "NexDynIPC/Dynamics/Forms/DistanceBarrierForm.h"
#include "NexDynIPC/Physics/Contact/Barrier.h"
#include <algorithm>
#include <cmath>

namespace NexDynIPC::Dynamics {

DistanceBarrierForm::DistanceBarrierForm(World& world, double dhat, double stiffness)
    : world_(world), dhat_(std::max(1e-9, dhat)), stiffness_(std::max(0.0, stiffness)) {}

double DistanceBarrierForm::value(const Eigen::VectorXd& x) const {
    double total = 0.0;
    for (const auto& contact : contacts_) {
        const double dist = contactDistance(contact, x);
        total += Physics::BarrierPotential::value(dist, dhat_, stiffness_);
    }
    return total;
}

void DistanceBarrierForm::gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const {
    grad.setZero(x.size());

    for (const auto& contact : contacts_) {
        const double dist = contactDistance(contact, x);
        if (dist >= dhat_) {
            continue;
        }

        const int idxA = contact.bodyA_idx * 6;
        const int idxB = contact.bodyB_idx * 6;
        const Eigen::Vector3d n = contact.normal;
        const double dphi_dd = Physics::BarrierPotential::gradient(dist, dhat_, stiffness_);

        grad.segment<3>(idxA) += -dphi_dd * n;
        grad.segment<3>(idxB) += dphi_dd * n;
    }
}

void DistanceBarrierForm::hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) const {
    for (const auto& contact : contacts_) {
        const double dist = contactDistance(contact, x);
        if (dist >= dhat_) {
            continue;
        }

        const int idxA = contact.bodyA_idx * 6;
        const int idxB = contact.bodyB_idx * 6;
        const Eigen::Vector3d n = contact.normal;
        const double d2phi_dd2 = Physics::BarrierPotential::hessian(dist, dhat_, stiffness_);
        const Eigen::Matrix3d block = d2phi_dd2 * (n * n.transpose());

        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                triplets.emplace_back(idxA + r, idxA + c, block(r, c));
                triplets.emplace_back(idxB + r, idxB + c, block(r, c));
                triplets.emplace_back(idxA + r, idxB + c, -block(r, c));
                triplets.emplace_back(idxB + r, idxA + c, -block(r, c));
            }
        }
    }
}

void DistanceBarrierForm::updateContacts(const std::vector<ContactPair>& contacts) {
    contacts_ = contacts;
}

void DistanceBarrierForm::setParameters(double dhat, double stiffness) {
    dhat_ = std::max(1e-9, dhat);
    stiffness_ = std::max(0.0, stiffness);
}

std::vector<double> DistanceBarrierForm::computeNormalForces(const Eigen::VectorXd& x) const {
    std::vector<double> normal_forces;
    normal_forces.reserve(contacts_.size());

    for (const auto& contact : contacts_) {
        const double dist = contactDistance(contact, x);
        if (dist < dhat_ && dist > 0.0) {
            const double barrier_grad = Physics::BarrierPotential::gradient(dist, dhat_, stiffness_);
            normal_forces.push_back(std::abs(barrier_grad));
        } else {
            normal_forces.push_back(0.0);
        }
    }

    return normal_forces;
}

double DistanceBarrierForm::contactDistance(const ContactPair& contact, const Eigen::VectorXd& x) const {
    const int idxA = contact.bodyA_idx * 6;
    const int idxB = contact.bodyB_idx * 6;

    const Eigen::Vector3d pA = x.segment<3>(idxA);
    const Eigen::Vector3d pB = x.segment<3>(idxB);

    const Eigen::Vector3d n = contact.normal;
    const double projected = (pB - pA).dot(n);
    return std::max(1e-9, projected);
}

} // namespace NexDynIPC::Dynamics
