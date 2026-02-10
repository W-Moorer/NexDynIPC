#include "NexDynIPC/Dynamics/Forms/ConstraintForm.hpp"

namespace NexDynIPC::Dynamics {

ConstraintForm::ConstraintForm() {}

void ConstraintForm::addJoint(std::shared_ptr<Joint> joint) {
    joints_.push_back(joint);
}

double ConstraintForm::value(const Eigen::VectorXd& x) const {
    double total_energy = 0;
    for (const auto& joint : joints_) {
        total_energy += joint->value(x);
    }
    return total_energy;
}

void ConstraintForm::gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const {
    for (const auto& joint : joints_) {
        joint->gradient(x, grad);
    }
}

void ConstraintForm::hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) const {
    for (const auto& joint : joints_) {
        joint->hessian(x, triplets);
    }
}

void ConstraintForm::updateLambdas(const Eigen::VectorXd& x) {
    for (auto& joint : joints_) {
        joint->updateLambda(x);
    }
}

} // namespace NexDynIPC::Dynamics
