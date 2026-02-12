#pragma once

#include "NexDynIPC/Math/Interval.h"
#include "NexDynIPC/Dynamics/RigidBody.h"
#include <Eigen/Geometry>

namespace NexDynIPC::Physics::CCD {

using namespace NexDynIPC::Math;
using namespace NexDynIPC::Dynamics;

struct PoseInterval {
    Vector3I position;
    Eigen::Quaterniond rotation_t0;
    Eigen::Quaterniond rotation_t1;

    PoseInterval() = default;

    PoseInterval(const Eigen::Vector3d& pos_t0, const Eigen::Vector3d& pos_t1,
                 const Eigen::Quaterniond& rot_t0, const Eigen::Quaterniond& rot_t1)
        : rotation_t0(rot_t0), rotation_t1(rot_t1) {
        position = Vector3I(
            Interval(pos_t0.x(), pos_t1.x()),
            Interval(pos_t0.y(), pos_t1.y()),
            Interval(pos_t0.z(), pos_t1.z())
        );
    }

    static PoseInterval fromPoses(
        const Eigen::Vector3d& pos_t0, const Eigen::Quaterniond& rot_t0,
        const Eigen::Vector3d& pos_t1, const Eigen::Quaterniond& rot_t1) {
        return PoseInterval(pos_t0, pos_t1, rot_t0, rot_t1);
    }
};

inline Vector3I interpolateRotation(
    const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1,
    const Interval& t, const Eigen::Vector3d& local_point) {
    
    double t_lo = lower(t);
    double t_hi = upper(t);
    
    Eigen::Quaterniond q_lo = q0.slerp(t_lo, q1);
    Eigen::Quaterniond q_hi = q0.slerp(t_hi, q1);
    
    Eigen::Vector3d p_lo = q_lo * local_point;
    Eigen::Vector3d p_hi = q_hi * local_point;
    
    return Vector3I(
        Interval(std::min(p_lo.x(), p_hi.x()), std::max(p_lo.x(), p_hi.x())),
        Interval(std::min(p_lo.y(), p_hi.y()), std::max(p_lo.y(), p_hi.y())),
        Interval(std::min(p_lo.z(), p_hi.z()), std::max(p_lo.z(), p_hi.z()))
    );
}

inline Vector3I vertexTrajectoryAABB(
    const RigidBody& body,
    const PoseInterval& pose,
    const Eigen::Vector3d& local_vertex,
    const Interval& t = Interval(0, 1)) {
    
    Vector3I rotated = interpolateRotation(
        pose.rotation_t0, pose.rotation_t1, t, local_vertex);
    
    Vector3I world_pos(
        pose.position.x() + rotated.x(),
        pose.position.y() + rotated.y(),
        pose.position.z() + rotated.z()
    );
    
    return world_pos;
}

inline Vector3I edgeTrajectoryAABB(
    const RigidBody& body,
    const PoseInterval& pose,
    const Eigen::Vector3d& local_v0,
    const Eigen::Vector3d& local_v1,
    const Interval& t = Interval(0, 1),
    const Interval& alpha = Interval(0, 1)) {
    
    Vector3I v0_world = vertexTrajectoryAABB(body, pose, local_v0, t);
    Vector3I v1_world = vertexTrajectoryAABB(body, pose, local_v1, t);
    
    double a_lo = lower(alpha);
    double a_hi = upper(alpha);
    
    Interval x = v0_world.x() * (1.0 - Interval(a_lo, a_hi)) + v1_world.x() * Interval(a_lo, a_hi);
    Interval y = v0_world.y() * (1.0 - Interval(a_lo, a_hi)) + v1_world.y() * Interval(a_lo, a_hi);
    Interval z = v0_world.z() * (1.0 - Interval(a_lo, a_hi)) + v1_world.z() * Interval(a_lo, a_hi);
    
    return Vector3I(x, y, z);
}

inline Vector3I faceTrajectoryAABB(
    const RigidBody& body,
    const PoseInterval& pose,
    const Eigen::Vector3d& local_v0,
    const Eigen::Vector3d& local_v1,
    const Eigen::Vector3d& local_v2,
    const Interval& t = Interval(0, 1),
    const Interval& u = Interval(0, 1),
    const Interval& v = Interval(0, 1)) {
    
    Vector3I v0_world = vertexTrajectoryAABB(body, pose, local_v0, t);
    Vector3I v1_world = vertexTrajectoryAABB(body, pose, local_v1, t);
    Vector3I v2_world = vertexTrajectoryAABB(body, pose, local_v2, t);
    
    Interval one(1.0);
    Interval u_int(lower(u), upper(u));
    Interval v_int(lower(v), upper(v));
    Interval w_int = one - u_int - v_int;
    
    Interval x = w_int * v0_world.x() + u_int * v1_world.x() + v_int * v2_world.x();
    Interval y = w_int * v0_world.y() + u_int * v1_world.y() + v_int * v2_world.y();
    Interval z = w_int * v0_world.z() + u_int * v1_world.z() + v_int * v2_world.z();
    
    return Vector3I(x, y, z);
}

inline Vector3I edgeVertexDistance(
    const RigidBody& bodyA, const PoseInterval& poseA, const Eigen::Vector3d& local_vertex,
    const RigidBody& bodyB, const PoseInterval& poseB,
    const Eigen::Vector3d& edge_v0, const Eigen::Vector3d& edge_v1,
    const Interval& t = Interval(0, 1),
    const Interval& alpha = Interval(0, 1)) {
    
    Vector3I vertex_world = vertexTrajectoryAABB(bodyA, poseA, local_vertex, t);
    Vector3I edge_point = edgeTrajectoryAABB(bodyB, poseB, edge_v0, edge_v1, t, alpha);
    
    return Vector3I(
        vertex_world.x() - edge_point.x(),
        vertex_world.y() - edge_point.y(),
        vertex_world.z() - edge_point.z()
    );
}

inline Vector3I edgeEdgeDistance(
    const RigidBody& bodyA, const PoseInterval& poseA,
    const Eigen::Vector3d& edgeA_v0, const Eigen::Vector3d& edgeA_v1,
    const RigidBody& bodyB, const PoseInterval& poseB,
    const Eigen::Vector3d& edgeB_v0, const Eigen::Vector3d& edgeB_v1,
    const Interval& t = Interval(0, 1),
    const Interval& alpha = Interval(0, 1),
    const Interval& beta = Interval(0, 1)) {
    
    Vector3I edgeA_point = edgeTrajectoryAABB(bodyA, poseA, edgeA_v0, edgeA_v1, t, alpha);
    Vector3I edgeB_point = edgeTrajectoryAABB(bodyB, poseB, edgeB_v0, edgeB_v1, t, beta);
    
    return Vector3I(
        edgeA_point.x() - edgeB_point.x(),
        edgeA_point.y() - edgeB_point.y(),
        edgeA_point.z() - edgeB_point.z()
    );
}

inline Vector3I faceVertexDistance(
    const RigidBody& bodyA, const PoseInterval& poseA, const Eigen::Vector3d& local_vertex,
    const RigidBody& bodyB, const PoseInterval& poseB,
    const Eigen::Vector3d& face_v0, const Eigen::Vector3d& face_v1, const Eigen::Vector3d& face_v2,
    const Interval& t = Interval(0, 1),
    const Interval& u = Interval(0, 1),
    const Interval& v = Interval(0, 1)) {
    
    Vector3I vertex_world = vertexTrajectoryAABB(bodyA, poseA, local_vertex, t);
    Vector3I face_point = faceTrajectoryAABB(bodyB, poseB, face_v0, face_v1, face_v2, t, u, v);
    
    return Vector3I(
        vertex_world.x() - face_point.x(),
        vertex_world.y() - face_point.y(),
        vertex_world.z() - face_point.z()
    );
}

} // namespace NexDynIPC::Physics::CCD
