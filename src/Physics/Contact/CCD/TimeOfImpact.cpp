#include "NexDynIPC/Physics/Contact/CCD/TimeOfImpact.h"

namespace NexDynIPC::Physics::CCD {

using namespace NexDynIPC::Math;

bool computeEdgeVertexTOIImpl(
    const RigidBody& bodyA,
    const Eigen::Vector3d& posA_t0, const Eigen::Quaterniond& rotA_t0,
    const Eigen::Vector3d& posA_t1, const Eigen::Quaterniond& rotA_t1,
    const Eigen::Vector3d& local_vertex,
    const RigidBody& bodyB,
    const Eigen::Vector3d& posB_t0, const Eigen::Quaterniond& rotB_t0,
    const Eigen::Vector3d& posB_t1, const Eigen::Quaterniond& rotB_t1,
    const Eigen::Vector3d& edge_v0, const Eigen::Vector3d& edge_v1,
    double& toi,
    double earliest_toi,
    double toi_tolerance) {
    
    PoseInterval poseA = PoseInterval::fromPoses(posA_t0, rotA_t0, posA_t1, rotA_t1);
    PoseInterval poseB = PoseInterval::fromPoses(posB_t0, rotB_t0, posB_t1, rotB_t1);
    
    auto distance = [&](const VectorXI& params) -> VectorXI {
        Interval t = params(0);
        Interval alpha = params(1);
        
        Vector3I dist_vec = edgeVertexDistance(
            bodyA, poseA, local_vertex,
            bodyB, poseB, edge_v0, edge_v1,
            t, alpha);
        
        VectorXI result(3);
        result(0) = dist_vec.x();
        result(1) = dist_vec.y();
        result(2) = dist_vec.z();
        return result;
    };
    
    double edge_len = (edge_v1 - edge_v0).norm();
    Eigen::Vector2d tol(toi_tolerance, DEFAULT_LENGTH_TOL / edge_len);
    
    VectorXI x0 = Vector2I(Interval(0, earliest_toi), Interval(0, 1));
    VectorXI toi_interval;
    
    bool is_impacting = intervalRootFinder(distance, x0, tol, toi_interval);
    
    toi = is_impacting ? lower(toi_interval(0)) : std::numeric_limits<double>::infinity();
    
    return is_impacting;
}

bool computeEdgeEdgeTOIImpl(
    const RigidBody& bodyA,
    const Eigen::Vector3d& posA_t0, const Eigen::Quaterniond& rotA_t0,
    const Eigen::Vector3d& posA_t1, const Eigen::Quaterniond& rotA_t1,
    const Eigen::Vector3d& edgeA_v0, const Eigen::Vector3d& edgeA_v1,
    const RigidBody& bodyB,
    const Eigen::Vector3d& posB_t0, const Eigen::Quaterniond& rotB_t0,
    const Eigen::Vector3d& posB_t1, const Eigen::Quaterniond& rotB_t1,
    const Eigen::Vector3d& edgeB_v0, const Eigen::Vector3d& edgeB_v1,
    double& toi,
    double earliest_toi,
    double toi_tolerance) {
    
    PoseInterval poseA = PoseInterval::fromPoses(posA_t0, rotA_t0, posA_t1, rotA_t1);
    PoseInterval poseB = PoseInterval::fromPoses(posB_t0, rotB_t0, posB_t1, rotB_t1);
    
    auto distance = [&](const VectorXI& params) -> VectorXI {
        Interval t = params(0);
        Interval alpha = params(1);
        Interval beta = params(2);
        
        Vector3I dist_vec = edgeEdgeDistance(
            bodyA, poseA, edgeA_v0, edgeA_v1,
            bodyB, poseB, edgeB_v0, edgeB_v1,
            t, alpha, beta);
        
        VectorXI result(3);
        result(0) = dist_vec.x();
        result(1) = dist_vec.y();
        result(2) = dist_vec.z();
        return result;
    };
    
    double edgeA_len = (edgeA_v1 - edgeA_v0).norm();
    double edgeB_len = (edgeB_v1 - edgeB_v0).norm();
    
    Eigen::Vector3d tol(toi_tolerance, 
                        DEFAULT_LENGTH_TOL / edgeA_len,
                        DEFAULT_LENGTH_TOL / edgeB_len);
    
    VectorXI x0 = Vector3I(Interval(0, earliest_toi), Interval(0, 1), Interval(0, 1));
    VectorXI toi_interval;
    
    bool is_impacting = intervalRootFinder(distance, x0, tol, toi_interval);
    
    toi = is_impacting ? lower(toi_interval(0)) : std::numeric_limits<double>::infinity();
    
    return is_impacting;
}

bool computeFaceVertexTOIImpl(
    const RigidBody& bodyA,
    const Eigen::Vector3d& posA_t0, const Eigen::Quaterniond& rotA_t0,
    const Eigen::Vector3d& posA_t1, const Eigen::Quaterniond& rotA_t1,
    const Eigen::Vector3d& local_vertex,
    const RigidBody& bodyB,
    const Eigen::Vector3d& posB_t0, const Eigen::Quaterniond& rotB_t0,
    const Eigen::Vector3d& posB_t1, const Eigen::Quaterniond& rotB_t1,
    const Eigen::Vector3d& face_v0, const Eigen::Vector3d& face_v1, const Eigen::Vector3d& face_v2,
    double& toi,
    double earliest_toi,
    double toi_tolerance) {
    
    PoseInterval poseA = PoseInterval::fromPoses(posA_t0, rotA_t0, posA_t1, rotA_t1);
    PoseInterval poseB = PoseInterval::fromPoses(posB_t0, rotB_t0, posB_t1, rotB_t1);
    
    auto distance = [&](const VectorXI& params) -> VectorXI {
        Interval t = params(0);
        Interval u = params(1);
        Interval v = params(2);
        
        Vector3I dist_vec = faceVertexDistance(
            bodyA, poseA, local_vertex,
            bodyB, poseB, face_v0, face_v1, face_v2,
            t, u, v);
        
        VectorXI result(3);
        result(0) = dist_vec.x();
        result(1) = dist_vec.y();
        result(2) = dist_vec.z();
        return result;
    };
    
    auto is_valid = [&](const VectorXI& params) -> bool {
        Interval u = params(1);
        Interval v = params(2);
        Interval w = Interval(1.0) - u - v;
        
        return lower(u) >= 0 && upper(u) <= 1 &&
               lower(v) >= 0 && upper(v) <= 1 &&
               lower(w) >= 0 && upper(w) <= 1;
    };
    
    Eigen::Vector3d tol(toi_tolerance, DEFAULT_LENGTH_TOL, DEFAULT_LENGTH_TOL);
    
    VectorXI x0 = Vector3I(Interval(0, earliest_toi), Interval(0, 1), Interval(0, 1));
    VectorXI toi_interval;
    
    bool is_impacting = intervalRootFinder(distance, is_valid, x0, tol, toi_interval);
    
    toi = is_impacting ? lower(toi_interval(0)) : std::numeric_limits<double>::infinity();
    
    return is_impacting;
}

} // namespace NexDynIPC::Physics::CCD
