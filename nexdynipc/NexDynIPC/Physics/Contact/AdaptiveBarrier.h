#pragma once

#include "NexDynIPC/Physics/Contact/Barrier.h"
#include "NexDynIPC/Dynamics/RigidBody.h"
#include "NexDynIPC/Physics/Geometry/MeshShape.h"
#include <cmath>
#include <limits>

namespace NexDynIPC::Physics::Contact {

using namespace NexDynIPC::Dynamics;
using namespace NexDynIPC::Physics::Geometry;

struct AdaptiveBarrierParams {
    double initial_dhat = 1e-3;
    double dhat_epsilon = 1e-9;
    double min_stiffness_scale = 1e11;
    double max_stiffness = 1e20;
    double stiffness_growth_rate = 2.0;
};

class AdaptiveBarrier : public BarrierPotential {
public:
    AdaptiveBarrier(const AdaptiveBarrierParams& params = AdaptiveBarrierParams())
        : params_(params), kappa_(0), dhat_(params.initial_dhat), kappa_min_(0) {}
    
    void initialize(double average_mass, double bbox_diagonal) {
        kappa_min_ = params_.min_stiffness_scale * average_mass / (bbox_diagonal * bbox_diagonal);
        kappa_ = kappa_min_;
        dhat_ = params_.initial_dhat;
    }
    
    void updateStiffness(double min_distance) {
        if (min_distance < params_.dhat_epsilon && kappa_ < params_.max_stiffness) {
            kappa_ = std::min(kappa_ * params_.stiffness_growth_rate, params_.max_stiffness);
        }
    }
    
    void setStiffness(double kappa) { kappa_ = kappa; }
    double currentStiffness() const { return kappa_; }
    
    void setDhat(double dhat) { dhat_ = dhat; }
    double currentDhat() const { return dhat_; }
    
    double minStiffness() const { return kappa_min_; }
    double maxStiffness() const { return params_.max_stiffness; }
    
    static double value(double d, double dhat, double kappa) {
        return BarrierPotential::value(d, dhat, kappa);
    }
    
    static double gradient(double d, double dhat, double kappa) {
        return BarrierPotential::gradient(d, dhat, kappa);
    }
    
    static double hessian(double d, double dhat, double kappa) {
        return BarrierPotential::hessian(d, dhat, kappa);
    }
    
    double value(double d) const {
        return BarrierPotential::value(d, dhat_, kappa_);
    }
    
    double gradient(double d) const {
        return BarrierPotential::gradient(d, dhat_, kappa_);
    }
    
    double hessian(double d) const {
        return BarrierPotential::hessian(d, dhat_, kappa_);
    }
    
    bool isActive(double d) const {
        return d < dhat_ && d > 0;
    }
    
    double barrierClamp(double d) const {
        if (d <= 0) return std::numeric_limits<double>::infinity();
        if (d >= dhat_) return 0;
        return value(d);
    }
    
private:
    AdaptiveBarrierParams params_;
    double kappa_;
    double dhat_;
    double kappa_min_;
};

inline double computeMinDistance(
    const Eigen::Vector3d& p,
    const Eigen::MatrixXd& vertices,
    const Eigen::MatrixXi& faces) {
    
    double min_dist = std::numeric_limits<double>::infinity();
    
    for (int i = 0; i < faces.rows(); ++i) {
        Eigen::Vector3d v0 = vertices.row(faces(i, 0));
        Eigen::Vector3d v1 = vertices.row(faces(i, 1));
        Eigen::Vector3d v2 = vertices.row(faces(i, 2));
        
        Eigen::Vector3d e1 = v1 - v0;
        Eigen::Vector3d e2 = v2 - v0;
        Eigen::Vector3d n = e1.cross(e2);
        double area = n.norm();
        
        if (area < 1e-12) continue;
        
        n.normalize();
        double dist = std::abs((p - v0).dot(n));
        
        Eigen::Vector3d bary;
        Eigen::Matrix3d M;
        M.col(0) = v1 - v0;
        M.col(1) = v2 - v0;
        M.col(2) = n;
        
        Eigen::Vector3d coeffs = M.inverse() * (p - v0);
        bary(0) = 1.0 - coeffs(0) - coeffs(1);
        bary(1) = coeffs(0);
        bary(2) = coeffs(1);
        
        if (bary.minCoeff() >= -1e-6 && bary.maxCoeff() <= 1.0 + 1e-6) {
            min_dist = std::min(min_dist, dist);
        }
    }
    
    return min_dist;
}

inline double computeSceneMinDistance(
    const std::vector<std::shared_ptr<RigidBody>>& bodies) {
    
    double min_dist = std::numeric_limits<double>::infinity();
    
    for (size_t i = 0; i < bodies.size(); ++i) {
        auto meshA = std::dynamic_pointer_cast<MeshShape>(bodies[i]->shape);
        if (!meshA) continue;
        
        const auto& VA = meshA->vertices();
        
        for (int vi = 0; vi < VA.rows(); ++vi) {
            Eigen::Vector3d world_vertex = bodies[i]->position + 
                bodies[i]->orientation * Eigen::Vector3d(VA.row(vi));
            
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                auto meshB = std::dynamic_pointer_cast<MeshShape>(bodies[j]->shape);
                if (!meshB) continue;
                
                Eigen::MatrixXd VB_world(meshB->vertices().rows(), 3);
                for (int k = 0; k < meshB->vertices().rows(); ++k) {
                    VB_world.row(k) = bodies[j]->position + 
                        bodies[j]->orientation * Eigen::Vector3d(meshB->vertices().row(k));
                }
                
                double dist = computeMinDistance(world_vertex, VB_world, meshB->faces());
                min_dist = std::min(min_dist, dist);
            }
        }
    }
    
    return min_dist;
}

} // namespace NexDynIPC::Physics::Contact
