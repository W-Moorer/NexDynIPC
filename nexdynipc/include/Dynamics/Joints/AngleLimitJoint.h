#pragma once

#include "NexDynIPC/Dynamics/Joints/Joint.h"
#include <Eigen/Geometry>

namespace NexDynIPC::Dynamics {

class AngleLimitJoint : public Joint {
public:
    AngleLimitJoint(int bodyA,
                    int bodyB,
                    const Eigen::Vector3d& axis,
                    double min_angle_rad,
                    double max_angle_rad);

    int dim() const override { return 2; }

    int getBodyAId() const { return bodyA_id_; }
    int getBodyBId() const { return bodyB_id_; }

    void computeC(const Eigen::VectorXd& x, Eigen::VectorXd& C) const override;
    void computeJ(const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& J) const override;

    double value(const Eigen::VectorXd& x) const override;
    void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const override;
    void hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) const override;
    void updateLambda(const Eigen::VectorXd& x) override;

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

    Eigen::Quaterniond qA_ref_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond qB_ref_ = Eigen::Quaterniond::Identity();

    Eigen::Vector3d axis_;
    double min_angle_rad_;
    double max_angle_rad_;

    double currentRelativeAngle(const Eigen::VectorXd& x) const;
};

} // namespace NexDynIPC::Dynamics
