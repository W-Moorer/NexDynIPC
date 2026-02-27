#pragma once

#include "NexDynIPC/Dynamics/Forms/Form.h"
#include "NexDynIPC/Dynamics/World.h"
#include "NexDynIPC/Physics/Contact/Friction.h"
#include "NexDynIPC/Physics/Geometry/MeshShape.h"
#include <vector>
#include <memory>

namespace NexDynIPC::Dynamics {

struct ContactPair {
    int bodyA_idx;
    int bodyB_idx;
    int primitive_type; // 0: vertex-face, 1: edge-edge
    int primitiveA_idx;
    int primitiveB_idx;
    Eigen::Vector3d contact_point;
    Eigen::Vector3d normal;
    double distance;
};

class FrictionForm : public Form {
public:
    FrictionForm(World& world, double friction_coeff, double eps = 1e-8);
    
    // Form interface
    double value(const Eigen::VectorXd& x) const override;
    void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const override;
    void hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) const override;
    
    // Friction specific interface
    void updateNormalForces(const std::vector<double>& normal_forces);
    void updateContactPoints(const std::vector<ContactPair>& contacts);
    
    void setFrictionCoefficient(double mu) { friction_coeff_ = mu; }
    double frictionCoefficient() const { return friction_coeff_; }
    
    void setSmoothingParameter(double eps) { eps_ = eps; }
    double smoothingParameter() const { return eps_; }
    
private:
    World& world_;
    double friction_coeff_;
    double eps_;
    
    std::vector<ContactPair> contacts_;
    std::vector<double> normal_forces_;
    
    // Compute tangent displacement for a contact pair
    Eigen::Vector3d computeTangentDisplacement(
        const ContactPair& contact,
        const Eigen::VectorXd& x) const;
    
    // Get body position and orientation from state vector
    void getBodyState(
        const Eigen::VectorXd& x,
        int body_idx,
        Eigen::Vector3d& position,
        Eigen::Quaterniond& orientation) const;
};

} // namespace NexDynIPC::Dynamics
