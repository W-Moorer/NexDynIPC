#include "NexDynIPC/Math/LinearSolver.h"

// Use standard Eigen Cholesky solver for now
// We can switch to CHOLMOD later if performance is an issue
#include <Eigen/SparseCholesky>

namespace NexDynIPC::Math {

namespace {

class EigenSimplicialLDLTSolver : public LinearSolver {
public:
    void analyzePattern(const Eigen::SparseMatrix<double>& A) override {
        solver_.analyzePattern(A);
    }

    void factorize(const Eigen::SparseMatrix<double>& A) override {
        solver_.factorize(A);
    }

    void solve(const Eigen::VectorXd& b, Eigen::VectorXd& x) override {
        x = solver_.solve(b);
    }
    
    bool info() const override {
        return solver_.info() == Eigen::Success;
    }

private:
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver_;
    // Alternatively: Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double>> solver_;
};

} // namespace

std::unique_ptr<LinearSolver> createCholeskySolver() {
    return std::make_unique<EigenSimplicialLDLTSolver>();
}

} // namespace NexDynIPC::Math
