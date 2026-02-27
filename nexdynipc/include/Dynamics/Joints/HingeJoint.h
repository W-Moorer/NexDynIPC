#pragma once

#include "NexDynIPC/Dynamics/Joints/Joint.h"
#include <Eigen/Geometry>

namespace NexDynIPC::Dynamics {

class HingeJoint : public Joint {
public:
    HingeJoint(int bodyA,
               int bodyB,
               const Eigen::Vector3d& anchorA,
               const Eigen::Vector3d& anchorB,
               const Eigen::Vector3d& axisA,
               const Eigen::Vector3d& axisB);

    int dim() const override { return 6; }

    int getBodyAId() const { return bodyA_id_; }
    int getBodyBId() const { return bodyB_id_; }

    void computeC(const Eigen::VectorXd& x, Eigen::VectorXd& C) const override;
    void computeJ(const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& J) const override;

    void updateState(int idxA,
                     int idxB,
                     const Eigen::Vector3d& pA,
                     const Eigen::Quaterniond& qA,
                     const Eigen::Vector3d& pB,
                     const Eigen::Quaterniond& qB);

private:
    int bodyA_id_;
    int bodyB_id_;

    int global_idx_A_ = -1;
    int global_idx_B_ = -1;

    Eigen::Vector3d pA_ref_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d pB_ref_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond qA_ref_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond qB_ref_ = Eigen::Quaterniond::Identity();

    Eigen::Vector3d anchorA_;
    Eigen::Vector3d anchorB_;
    Eigen::Vector3d axisA_;
    Eigen::Vector3d axisB_;
};

} // namespace NexDynIPC::Dynamics
