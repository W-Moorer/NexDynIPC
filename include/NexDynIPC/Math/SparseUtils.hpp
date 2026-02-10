#pragma once

#include <Eigen/Sparse>
#include <vector>

namespace NexDynIPC::Math {

// Helper to construct sparse matrices from triplets
inline void setFromTriplets(Eigen::SparseMatrix<double>& A, 
                            const std::vector<Eigen::Triplet<double>>& triplets) {
    A.setFromTriplets(triplets.begin(), triplets.end());
}

} // namespace NexDynIPC::Math
