#pragma once

#include <vector>
#include <Eigen/Core>

namespace NexDynIPC::Physics::CCD {

struct EdgeVertexImpact {
    double time;
    int edge_index;
    double alpha;
    int vertex_index;

    EdgeVertexImpact(double t, int edge_idx, double a, int vert_idx)
        : time(t), edge_index(edge_idx), alpha(a), vertex_index(vert_idx) {}

    bool operator==(const EdgeVertexImpact& other) const {
        return time == other.time && edge_index == other.edge_index &&
               alpha == other.alpha && vertex_index == other.vertex_index;
    }
};

struct EdgeEdgeImpact {
    double time;
    int impacted_edge_index;
    double impacted_alpha;
    int impacting_edge_index;
    double impacting_alpha;

    EdgeEdgeImpact(double t, int imp_edge, double imp_alpha, int ing_edge, double ing_alpha)
        : time(t), impacted_edge_index(imp_edge), impacted_alpha(imp_alpha),
          impacting_edge_index(ing_edge), impacting_alpha(ing_alpha) {}

    bool operator==(const EdgeEdgeImpact& other) const {
        return time == other.time && 
               impacted_edge_index == other.impacted_edge_index &&
               impacted_alpha == other.impacted_alpha &&
               impacting_edge_index == other.impacting_edge_index &&
               impacting_alpha == other.impacting_alpha;
    }
};

struct FaceVertexImpact {
    double time;
    int face_index;
    double u;
    double v;
    int vertex_index;

    FaceVertexImpact(double t, int face_idx, double u_, double v_, int vert_idx)
        : time(t), face_index(face_idx), u(u_), v(v_), vertex_index(vert_idx) {}

    bool operator==(const FaceVertexImpact& other) const {
        return time == other.time && face_index == other.face_index &&
               u == other.u && v == other.v && vertex_index == other.vertex_index;
    }
};

struct Impacts {
    std::vector<EdgeVertexImpact> ev_impacts;
    std::vector<EdgeEdgeImpact> ee_impacts;
    std::vector<FaceVertexImpact> fv_impacts;

    size_t size() const {
        return ev_impacts.size() + ee_impacts.size() + fv_impacts.size();
    }

    void clear() {
        ev_impacts.clear();
        ee_impacts.clear();
        fv_impacts.clear();
    }

    bool empty() const {
        return ev_impacts.empty() && ee_impacts.empty() && fv_impacts.empty();
    }
};

template <typename Impact>
bool compareImpactsByTime(const Impact& i1, const Impact& i2) {
    return i1.time < i2.time;
}

} // namespace NexDynIPC::Physics::CCD
