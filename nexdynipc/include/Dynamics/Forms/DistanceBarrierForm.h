#pragma once

#include "NexDynIPC/Dynamics/Forms/Form.h"
#include "NexDynIPC/Dynamics/Forms/FrictionForm.h"
#include "NexDynIPC/Dynamics/World.h"
#include <vector>

namespace NexDynIPC::Dynamics {

class DistanceBarrierForm : public Form {
public:
    DistanceBarrierForm(World& world, double dhat, double stiffness);

    double value(const Eigen::VectorXd& x) const override;
    void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const override;
    void hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) const override;

    void updateContacts(const std::vector<ContactPair>& contacts);
    void setParameters(double dhat, double stiffness);
    std::vector<double> computeNormalForces(const Eigen::VectorXd& x) const;

private:
    World& world_;
    double dhat_;
    double stiffness_;
    std::vector<ContactPair> contacts_;

    double contactDistance(const ContactPair& contact, const Eigen::VectorXd& x) const;
};

} // namespace NexDynIPC::Dynamics
