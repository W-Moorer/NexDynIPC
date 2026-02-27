#pragma once

#include "Impact.h"
#include "IntervalRootFinder.h"
#include "RigidTrajectoryAABB.h"
#include "NexDynIPC/Dynamics/RigidBody.h"
#include "NexDynIPC/Physics/Geometry/MeshShape.h"
#include <Eigen/Geometry>
#include <limits>

namespace NexDynIPC::Physics::CCD {

using namespace NexDynIPC::Dynamics;
using namespace NexDynIPC::Math;

constexpr double DEFAULT_TOI_TOL = 1e-8;
constexpr double DEFAULT_LENGTH_TOL = 1e-12;

// Helper to convert Vector3I to VectorXI
inline VectorXI toVectorXI(const Vector3I& v) {
    VectorXI result(3);
    result(0) = v.x();
    result(1) = v.y();
    result(2) = v.z();
    return result;
}

// Forward declarations
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
    double toi_tolerance);

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
    double toi_tolerance);

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
    double toi_tolerance);

inline bool computeEdgeVertexTOI(
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
    return computeEdgeVertexTOIImpl(bodyA, posA_t0, rotA_t0, posA_t1, rotA_t1, local_vertex,
                                    bodyB, posB_t0, rotB_t0, posB_t1, rotB_t1, edge_v0, edge_v1,
                                    toi, earliest_toi, toi_tolerance);
}

inline bool computeEdgeEdgeTOI(
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
    return computeEdgeEdgeTOIImpl(bodyA, posA_t0, rotA_t0, posA_t1, rotA_t1, edgeA_v0, edgeA_v1,
                                  bodyB, posB_t0, rotB_t0, posB_t1, rotB_t1, edgeB_v0, edgeB_v1,
                                  toi, earliest_toi, toi_tolerance);
}

inline bool computeFaceVertexTOI(
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
    return computeFaceVertexTOIImpl(bodyA, posA_t0, rotA_t0, posA_t1, rotA_t1, local_vertex,
                                    bodyB, posB_t0, rotB_t0, posB_t1, rotB_t1, face_v0, face_v1, face_v2,
                                    toi, earliest_toi, toi_tolerance);
}

class TimeOfImpactCalculator {
public:
    TimeOfImpactCalculator() : toi_tolerance_(DEFAULT_TOI_TOL) {}
    
    void setTOITolerance(double tol) { toi_tolerance_ = tol; }
    double toiTolerance() const { return toi_tolerance_; }
    
    bool computeEdgeVertex(
        const RigidBody& bodyA,
        const Eigen::Vector3d& posA_t0, const Eigen::Quaterniond& rotA_t0,
        const Eigen::Vector3d& posA_t1, const Eigen::Quaterniond& rotA_t1,
        int vertex_idx,
        const RigidBody& bodyB,
        const Eigen::Vector3d& posB_t0, const Eigen::Quaterniond& rotB_t0,
        const Eigen::Vector3d& posB_t1, const Eigen::Quaterniond& rotB_t1,
        int edge_idx,
        double& toi,
        double earliest_toi = 1.0) const {
        
        auto meshA = std::dynamic_pointer_cast<Physics::MeshShape>(bodyA.shape);
        auto meshB = std::dynamic_pointer_cast<Physics::MeshShape>(bodyB.shape);
        
        if (!meshA || !meshB) return false;
        
        Eigen::Vector3d vertex = meshA->vertices().row(vertex_idx);
        Eigen::Vector3d edge_v0 = meshB->vertices().row(meshB->faces()(edge_idx, 0));
        Eigen::Vector3d edge_v1 = meshB->vertices().row(meshB->faces()(edge_idx, 1));
        
        return computeEdgeVertexTOI(
            bodyA, posA_t0, rotA_t0, posA_t1, rotA_t1, vertex,
            bodyB, posB_t0, rotB_t0, posB_t1, rotB_t1, edge_v0, edge_v1,
            toi, earliest_toi, toi_tolerance_);
    }
    
    bool computeEdgeEdge(
        const RigidBody& bodyA,
        const Eigen::Vector3d& posA_t0, const Eigen::Quaterniond& rotA_t0,
        const Eigen::Vector3d& posA_t1, const Eigen::Quaterniond& rotA_t1,
        int edgeA_idx,
        const RigidBody& bodyB,
        const Eigen::Vector3d& posB_t0, const Eigen::Quaterniond& rotB_t0,
        const Eigen::Vector3d& posB_t1, const Eigen::Quaterniond& rotB_t1,
        int edgeB_idx,
        double& toi,
        double earliest_toi = 1.0) const {
        
        auto meshA = std::dynamic_pointer_cast<Physics::MeshShape>(bodyA.shape);
        auto meshB = std::dynamic_pointer_cast<Physics::MeshShape>(bodyB.shape);
        
        if (!meshA || !meshB) return false;
        
        Eigen::Vector3d edgeA_v0 = meshA->vertices().row(meshA->faces()(edgeA_idx, 0));
        Eigen::Vector3d edgeA_v1 = meshA->vertices().row(meshA->faces()(edgeA_idx, 1));
        Eigen::Vector3d edgeB_v0 = meshB->vertices().row(meshB->faces()(edgeB_idx, 0));
        Eigen::Vector3d edgeB_v1 = meshB->vertices().row(meshB->faces()(edgeB_idx, 1));
        
        return computeEdgeEdgeTOI(
            bodyA, posA_t0, rotA_t0, posA_t1, rotA_t1, edgeA_v0, edgeA_v1,
            bodyB, posB_t0, rotB_t0, posB_t1, rotB_t1, edgeB_v0, edgeB_v1,
            toi, earliest_toi, toi_tolerance_);
    }
    
    bool computeFaceVertex(
        const RigidBody& bodyA,
        const Eigen::Vector3d& posA_t0, const Eigen::Quaterniond& rotA_t0,
        const Eigen::Vector3d& posA_t1, const Eigen::Quaterniond& rotA_t1,
        int vertex_idx,
        const RigidBody& bodyB,
        const Eigen::Vector3d& posB_t0, const Eigen::Quaterniond& rotB_t0,
        const Eigen::Vector3d& posB_t1, const Eigen::Quaterniond& rotB_t1,
        int face_idx,
        double& toi,
        double earliest_toi = 1.0) const {
        
        auto meshA = std::dynamic_pointer_cast<Physics::MeshShape>(bodyA.shape);
        auto meshB = std::dynamic_pointer_cast<Physics::MeshShape>(bodyB.shape);
        
        if (!meshA || !meshB) return false;
        
        Eigen::Vector3d vertex = meshA->vertices().row(vertex_idx);
        Eigen::Vector3d face_v0 = meshB->vertices().row(meshB->faces()(face_idx, 0));
        Eigen::Vector3d face_v1 = meshB->vertices().row(meshB->faces()(face_idx, 1));
        Eigen::Vector3d face_v2 = meshB->vertices().row(meshB->faces()(face_idx, 2));
        
        return computeFaceVertexTOI(
            bodyA, posA_t0, rotA_t0, posA_t1, rotA_t1, vertex,
            bodyB, posB_t0, rotB_t0, posB_t1, rotB_t1, face_v0, face_v1, face_v2,
            toi, earliest_toi, toi_tolerance_);
    }
    
private:
    double toi_tolerance_;
};

} // namespace NexDynIPC::Physics::CCD
