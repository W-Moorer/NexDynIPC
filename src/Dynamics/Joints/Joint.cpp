#include "NexDynIPC/Dynamics/Joints/Joint.h"

namespace NexDynIPC::Dynamics {

double Joint::value(const Eigen::VectorXd& x) const {
    Eigen::VectorXd C(dim());
    computeC(x, C);

    // E = lambda^T * C + 0.5 * mu * C^T * C
    double term1 = 0;
    if (lambda_.size() == dim()) {
        term1 = lambda_.dot(C);
    }
    double term2 = 0.5 * mu_ * C.squaredNorm();

    return term1 + term2;
}

void Joint::gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const {
    Eigen::VectorXd C(dim());
    computeC(x, C);

    Eigen::SparseMatrix<double> J(dim(), x.size());
    computeJ(x, J);

    // grad = J^T * (lambda + mu * C)
    Eigen::VectorXd effective_force = mu_ * C;
    if (lambda_.size() == dim()) {
        effective_force += lambda_;
    }

    grad += J.transpose() * effective_force;
}

void Joint::hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) const {
    Eigen::SparseMatrix<double> J(dim(), x.size());
    computeJ(x, J);

    // H approx J^T * mu * J
    // This is dense if we multiply directly. But J is sparse.
    // However, J^T * J can be dense-ish depending on connectivity.
    // For joints, J usually involves 2 bodies (12 dofs). So J^T * J is block diagonal-ish.
    
    // Explicit multiplication for SparseMatrix
    Eigen::SparseMatrix<double> JT = J.transpose();
    Eigen::SparseMatrix<double> H = JT * J * mu_;

    // Extract triplets from H
    for (int k = 0; k < H.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(H, k); it; ++it) {
            triplets.emplace_back(it.row(), it.col(), it.value());
        }
    }
}

void Joint::updateLambda(const Eigen::VectorXd& x) {
    Eigen::VectorXd C(dim());
    computeC(x, C);

    if (lambda_.size() != dim()) {
        lambda_ = Eigen::VectorXd::Zero(dim());
    }

    lambda_ += mu_ * C;
}

void Joint::resetLambda() {
    lambda_ = Eigen::VectorXd::Zero(dim());
}

} // namespace NexDynIPC::Dynamics
