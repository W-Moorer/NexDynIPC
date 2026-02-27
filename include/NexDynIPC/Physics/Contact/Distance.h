#pragma once

#include "NexDynIPC/Dynamics/RigidBody.h"
#include <Eigen/Core>
#include <vector>
#include <memory>

namespace NexDynIPC::Physics::Contact {

enum class ContactPrimitiveType {
    VertexFace = 0,
    EdgeEdge = 1
};

struct ContactDistanceResult {
    int bodyA_idx = -1;
    int bodyB_idx = -1;
    ContactPrimitiveType type = ContactPrimitiveType::VertexFace;
    int primitiveA_idx = -1;
    int primitiveB_idx = -1;

    double distance = 0.0;
    Eigen::Vector3d pointA = Eigen::Vector3d::Zero();
    Eigen::Vector3d pointB = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitX();
};

double pointPointDistance(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);

double pointEdgeDistance(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& e0,
    const Eigen::Vector3d& e1,
    Eigen::Vector3d& closest_edge_point);

double pointTriangleDistance(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& t0,
    const Eigen::Vector3d& t1,
    const Eigen::Vector3d& t2,
    Eigen::Vector3d& closest_triangle_point);

double edgeEdgeDistance(
    const Eigen::Vector3d& a0,
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& b0,
    const Eigen::Vector3d& b1,
    Eigen::Vector3d& closest_a,
    Eigen::Vector3d& closest_b);

struct CollisionCandidate {
    int bodyA_idx = -1;
    int bodyB_idx = -1;
    ContactPrimitiveType type = ContactPrimitiveType::VertexFace;
    int primitiveA_idx = -1;
    int primitiveB_idx = -1;
};

std::vector<ContactDistanceResult> computeNarrowPhaseDistances(
    const std::vector<CollisionCandidate>& candidates,
    const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies,
    double max_distance);

} // namespace NexDynIPC::Physics::Contact
