#pragma once

#include "Impact.h"
#include "TimeOfImpact.h"
#include "NexDynIPC/Physics/Contact/BroadPhase.h"
#include "NexDynIPC/Physics/Geometry/BVH.h"
#include "NexDynIPC/Physics/Geometry/MeshShape.h"
#include "NexDynIPC/Dynamics/RigidBody.h"
#include <vector>
#include <memory>
#include <limits>
#include <algorithm>

namespace NexDynIPC::Physics::CCD {

using namespace NexDynIPC::Dynamics;
using namespace NexDynIPC::Physics::Geometry;

enum class DetectionMethod {
    BRUTE_FORCE,
    BVH
};

struct CollisionCandidate {
    int bodyA_idx;
    int bodyB_idx;
    int type; // 0: vertex-face, 1: edge-edge
    int primitiveA_idx;
    int primitiveB_idx;
};

class CCDSystem {
public:
    CCDSystem() : toi_calculator_(), method_(DetectionMethod::BVH) {}
    
    void setDetectionMethod(DetectionMethod method) { method_ = method; }
    void setTOITolerance(double tol) { toi_calculator_.setTOITolerance(tol); }
    
    double computeEarliestTOI(
        const std::vector<std::shared_ptr<RigidBody>>& bodies,
        double dt) const {
        
        std::vector<CollisionCandidate> candidates;
        detectCandidates(bodies, candidates);
        
        double earliest_toi = 1.0;
        
        for (const auto& candidate : candidates) {
            double toi = std::numeric_limits<double>::infinity();
            bool collision = false;
            
            const auto& bodyA = bodies[candidate.bodyA_idx];
            const auto& bodyB = bodies[candidate.bodyB_idx];
            
            Eigen::Vector3d posA_t0 = bodyA->position;
            Eigen::Quaterniond rotA_t0 = bodyA->orientation;
            Eigen::Vector3d posA_t1 = posA_t0 + bodyA->velocity * dt;
            Eigen::Quaterniond rotA_t1 = rotA_t0; // TODO: integrate angular velocity
            
            Eigen::Vector3d posB_t0 = bodyB->position;
            Eigen::Quaterniond rotB_t0 = bodyB->orientation;
            Eigen::Vector3d posB_t1 = posB_t0 + bodyB->velocity * dt;
            Eigen::Quaterniond rotB_t1 = rotB_t0; // TODO: integrate angular velocity
            
            if (candidate.type == 0) {
                collision = toi_calculator_.computeFaceVertex(
                    *bodyA, posA_t0, rotA_t0, posA_t1, rotA_t1, candidate.primitiveA_idx,
                    *bodyB, posB_t0, rotB_t0, posB_t1, rotB_t1, candidate.primitiveB_idx,
                    toi, earliest_toi);
            } else if (candidate.type == 1) {
                collision = toi_calculator_.computeEdgeEdge(
                    *bodyA, posA_t0, rotA_t0, posA_t1, rotA_t1, candidate.primitiveA_idx,
                    *bodyB, posB_t0, rotB_t0, posB_t1, rotB_t1, candidate.primitiveB_idx,
                    toi, earliest_toi);
            }
            
            if (collision && toi < earliest_toi) {
                earliest_toi = toi;
            }
        }
        
        return earliest_toi;
    }
    
    Impacts detectCollisions(
        const std::vector<std::shared_ptr<RigidBody>>& bodies,
        double dt) const {
        
        Impacts impacts;
        
        std::vector<CollisionCandidate> candidates;
        detectCandidates(bodies, candidates);
        
        double earliest_toi = 1.0;
        
        for (const auto& candidate : candidates) {
            double toi = std::numeric_limits<double>::infinity();
            bool collision = false;
            
            const auto& bodyA = bodies[candidate.bodyA_idx];
            const auto& bodyB = bodies[candidate.bodyB_idx];
            
            Eigen::Vector3d posA_t0 = bodyA->position;
            Eigen::Quaterniond rotA_t0 = bodyA->orientation;
            Eigen::Vector3d posA_t1 = posA_t0 + bodyA->velocity * dt;
            Eigen::Quaterniond rotA_t1 = rotA_t0;
            
            Eigen::Vector3d posB_t0 = bodyB->position;
            Eigen::Quaterniond rotB_t0 = bodyB->orientation;
            Eigen::Vector3d posB_t1 = posB_t0 + bodyB->velocity * dt;
            Eigen::Quaterniond rotB_t1 = rotB_t0;
            
            if (candidate.type == 0) {
                collision = toi_calculator_.computeFaceVertex(
                    *bodyA, posA_t0, rotA_t0, posA_t1, rotA_t1, candidate.primitiveA_idx,
                    *bodyB, posB_t0, rotB_t0, posB_t1, rotB_t1, candidate.primitiveB_idx,
                    toi, earliest_toi);
                    
                if (collision) {
                    impacts.fv_impacts.emplace_back(
                        toi, candidate.primitiveB_idx, 0.0, 0.0, candidate.primitiveA_idx);
                }
            } else if (candidate.type == 1) {
                collision = toi_calculator_.computeEdgeEdge(
                    *bodyA, posA_t0, rotA_t0, posA_t1, rotA_t1, candidate.primitiveA_idx,
                    *bodyB, posB_t0, rotB_t0, posB_t1, rotB_t1, candidate.primitiveB_idx,
                    toi, earliest_toi);
                    
                if (collision) {
                    impacts.ee_impacts.emplace_back(
                        toi, candidate.primitiveA_idx, 0.5,
                        candidate.primitiveB_idx, 0.5);
                }
            }
        }
        
        return impacts;
    }
    
private:
    TimeOfImpactCalculator toi_calculator_;
    DetectionMethod method_;
    
    void detectCandidates(
        const std::vector<std::shared_ptr<RigidBody>>& bodies,
        std::vector<CollisionCandidate>& candidates) const {
        
        candidates.clear();
        
        if (method_ == DetectionMethod::BRUTE_FORCE) {
            detectCandidatesBruteForce(bodies, candidates);
        } else {
            detectCandidatesBVH(bodies, candidates);
        }
    }
    
    void detectCandidatesBruteForce(
        const std::vector<std::shared_ptr<RigidBody>>& bodies,
        std::vector<CollisionCandidate>& candidates) const {
        
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                generateBodyPairCandidates(bodies[i], bodies[j], i, j, candidates);
            }
        }
    }
    
    void detectCandidatesBVH(
        const std::vector<std::shared_ptr<RigidBody>>& bodies,
        std::vector<CollisionCandidate>& candidates) const {
        
        std::vector<AABB> aabbs;
        aabbs.reserve(bodies.size());
        
        for (const auto& body : bodies) {
            auto mesh = std::dynamic_pointer_cast<MeshShape>(body->shape);
            if (!mesh) {
                aabbs.emplace_back(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
                continue;
            }
            
            auto aabb_pair = mesh->computeAABB();
            
            Eigen::Vector3d world_min = body->position + body->orientation * aabb_pair.first;
            Eigen::Vector3d world_max = body->position + body->orientation * aabb_pair.second;
            
            Eigen::Vector3d inflated_min = world_min.cwiseMin(world_max);
            Eigen::Vector3d inflated_max = world_max.cwiseMax(world_min);
            
            aabbs.emplace_back(inflated_min, inflated_max);
        }
        
        BVH bvh;
        bvh.init(aabbs);
        
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                const auto& nodeA = bvh.nodes()[i];
                const auto& nodeB = bvh.nodes()[j];
                
                if (nodeA.bbox.intersects(nodeB.bbox)) {
                    generateBodyPairCandidates(bodies[i], bodies[j], i, j, candidates);
                }
            }
        }
    }
    
    void generateBodyPairCandidates(
        const std::shared_ptr<RigidBody>& bodyA,
        const std::shared_ptr<RigidBody>& bodyB,
        int bodyA_idx, int bodyB_idx,
        std::vector<CollisionCandidate>& candidates) const {
        
        auto meshA = std::dynamic_pointer_cast<MeshShape>(bodyA->shape);
        auto meshB = std::dynamic_pointer_cast<MeshShape>(bodyB->shape);
        
        if (!meshA || !meshB) return;
        
        const auto& VA = meshA->vertices();
        const auto& FA = meshA->faces();
        const auto& FB = meshB->faces();
        
        for (int vi = 0; vi < VA.rows(); ++vi) {
            for (int fi = 0; fi < FB.rows(); ++fi) {
                candidates.push_back({bodyA_idx, bodyB_idx, 0, vi, fi});
            }
        }
        
        for (int eiA = 0; eiA < FA.rows(); ++eiA) {
            for (int eiB = 0; eiB < FB.rows(); ++eiB) {
                candidates.push_back({bodyA_idx, bodyB_idx, 1, eiA, eiB});
            }
        }
    }
};

} // namespace NexDynIPC::Physics::CCD
