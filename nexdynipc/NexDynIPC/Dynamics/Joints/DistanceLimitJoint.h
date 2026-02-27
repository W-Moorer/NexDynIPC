#pragma once

#include "NexDynIPC/Dynamics/Joints/Joint.h"
#include <Eigen/Geometry>

namespace NexDynIPC::Dynamics {

class DistanceLimitJoint : public Joint {
public:
    DistanceLimitJoint(int bodyA,
                       int bodyB,
                       double min_distance,
                       double max_distance);

    int dim() const override { return 2; }

    int getBodyAId() const { return bodyA_id_; }
    int getBodyBId() const { return bodyB_id_; }

    double getMinDistance() const { return min_distance_; }
    double getMaxDistance() const { return max_distance_; }

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

    Eigen::Vector3d pA_ref_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d pB_ref_ = Eigen::Vector3d::Zero();

    double min_distance_;
    double max_distance_;

    Eigen::Vector3d getPosition(const Eigen::VectorXd& x, int global_idx, const Eigen::Vector3d& p_ref) const;
    Eigen::Vector3d direction(const Eigen::VectorXd& x, double& distance) const;
};

} // namespace NexDynIPC::Dynamics
