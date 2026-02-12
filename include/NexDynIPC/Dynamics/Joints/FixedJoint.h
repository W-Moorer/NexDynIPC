#pragma once

#include "NexDynIPC/Dynamics/Joints/Joint.h"
#include <Eigen/Geometry>

namespace NexDynIPC::Dynamics {

/// FixedJoint constrains a single body to a fixed position and orientation.
/// It has 6 DOFs: 3 position + 3 rotation.
class FixedJoint : public Joint {
public:
    FixedJoint(int bodyId, 
               const Eigen::Vector3d& target_position,
               const Eigen::Quaterniond& target_orientation = Eigen::Quaterniond::Identity());

    int dim() const override { return 6; }

    int getBodyId() const { return body_id_; }

    void computeC(const Eigen::VectorXd& x, Eigen::VectorXd& C) const override;
    void computeJ(const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& J) const override;

    /// Must be called before optimization step.
    void updateState(int globalIdx, 
                     const Eigen::Vector3d& position, 
                     const Eigen::Quaterniond& orientation);

private:
    int body_id_;
    int global_idx_ = -1;

    Eigen::Vector3d target_position_;
    Eigen::Quaterniond target_orientation_;

    // Reference orientation at start of step (for incremental rotation)
    Eigen::Quaterniond q_ref_ = Eigen::Quaterniond::Identity();
};

} // namespace NexDynIPC::Dynamics
