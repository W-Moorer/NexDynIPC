#pragma once

#include "NexDynIPC/Dynamics/World.h"
#include "NexDynIPC/Physics/Contact/BroadPhase.h"
#include "NexDynIPC/Physics/Geometry/AABB.h"
#include <Eigen/Core>
#include <unordered_map>
#include <vector>

namespace NexDynIPC::Physics::Contact {

struct VertexVertexCandidate {
    int body0_id = -1;
    int body1_id = -1;
    int vertex0_local_id = -1;
    int vertex1_local_id = -1;
};

struct EdgeVertexCandidate {
    int edge_body_id = -1;
    int vertex_body_id = -1;
    int edge_vertex0_local_id = -1;
    int edge_vertex1_local_id = -1;
    int vertex_local_id = -1;
};

struct EdgeEdgeCandidate {
    int body0_id = -1;
    int body1_id = -1;
    int edge0_vertex0_local_id = -1;
    int edge0_vertex1_local_id = -1;
    int edge1_vertex0_local_id = -1;
    int edge1_vertex1_local_id = -1;
};

struct FaceVertexCandidate {
    int face_body_id = -1;
    int vertex_body_id = -1;
    int face_local_id = -1;
    int vertex_local_id = -1;
};

struct ContactCandidateSet {
    std::vector<VertexVertexCandidate> vv;
    std::vector<EdgeVertexCandidate> ev;
    std::vector<EdgeEdgeCandidate> ee;
    std::vector<FaceVertexCandidate> fv;

    void clear() {
        vv.clear();
        ev.clear();
        ee.clear();
        fv.clear();
    }
};

struct BodyContactMesh {
    int body_id = -1;
    int world_index = -1;
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi edges;
    Eigen::MatrixXi faces;
    Eigen::Vector3d world_position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond world_orientation = Eigen::Quaterniond::Identity();
    Geometry::AABB world_aabb;
};

class ContactAssembler {
public:
    void build(const Dynamics::World& world);

    std::vector<std::pair<int, int>> detectBodyPairs(double inflation_radius) const;
    double estimateMinBodyPairDistance(double inflation_radius) const;

    void buildCandidateSet(double activation_distance, ContactCandidateSet& out_candidates) const;

    const std::vector<BodyContactMesh>& bodyMeshes() const { return body_meshes_; }

private:
    std::vector<BodyContactMesh> body_meshes_;
    std::unordered_map<int, int> body_id_to_mesh_index_;
    mutable IncrementalBVHBroadPhase broad_phase_;
};

} // namespace NexDynIPC::Physics::Contact
