#include "NexDynIPC/Physics/Contact/ContactAssembler.h"
#include "NexDynIPC/Physics/Geometry/MeshShape.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <limits>
#include <numeric>
#include <unordered_set>

namespace NexDynIPC::Physics::Contact {

namespace {

std::uint64_t edge_key(int a, int b) {
    const int lo = std::min(a, b);
    const int hi = std::max(a, b);
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(lo)) << 32)
        | static_cast<std::uint32_t>(hi);
}

Geometry::AABB transform_local_aabb(
    const Geometry::AABB& local_aabb,
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation)
{
    const Eigen::Vector3d mn = local_aabb.min();
    const Eigen::Vector3d mx = local_aabb.max();

    Geometry::AABB world_box;
    for (int ix = 0; ix < 2; ++ix) {
        for (int iy = 0; iy < 2; ++iy) {
            for (int iz = 0; iz < 2; ++iz) {
                const Eigen::Vector3d c(
                    ix ? mx.x() : mn.x(),
                    iy ? mx.y() : mn.y(),
                    iz ? mx.z() : mn.z());
                world_box.expand(position + orientation * c);
            }
        }
    }
    return world_box;
}

double aabb_distance(const Geometry::AABB& a, const Geometry::AABB& b) {
    const auto amin = a.min();
    const auto amax = a.max();
    const auto bmin = b.min();
    const auto bmax = b.max();

    const double dx = std::max({0.0, amin.x() - bmax.x(), bmin.x() - amax.x()});
    const double dy = std::max({0.0, amin.y() - bmax.y(), bmin.y() - amax.y()});
    const double dz = std::max({0.0, amin.z() - bmax.z(), bmin.z() - amax.z()});
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double point_aabb_distance(const Eigen::Vector3d& p, const Geometry::AABB& box) {
    const auto mn = box.min();
    const auto mx = box.max();

    const double dx = std::max({mn.x() - p.x(), 0.0, p.x() - mx.x()});
    const double dy = std::max({mn.y() - p.y(), 0.0, p.y() - mx.y()});
    const double dz = std::max({mn.z() - p.z(), 0.0, p.z() - mx.z()});
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<int> select_top_k_indices(
    int total_count,
    int k,
    const std::function<double(int)>& score_fn)
{
    if (total_count <= 0 || k <= 0) {
        return {};
    }

    std::vector<int> indices(total_count);
    std::iota(indices.begin(), indices.end(), 0);

    if (k < total_count) {
        std::nth_element(indices.begin(), indices.begin() + k, indices.end(),
            [&](int a, int b) { return score_fn(a) < score_fn(b); });
        indices.resize(k);
    }

    std::sort(indices.begin(), indices.end(),
        [&](int a, int b) { return score_fn(a) < score_fn(b); });
    return indices;
}

Eigen::MatrixXi build_unique_edges(const Eigen::MatrixXi& faces) {
    std::unordered_set<std::uint64_t> seen;
    std::vector<Eigen::Vector2i> edges;
    edges.reserve(faces.rows() * 3);

    for (int i = 0; i < faces.rows(); ++i) {
        const int v0 = faces(i, 0);
        const int v1 = faces(i, 1);
        const int v2 = faces(i, 2);

        const std::array<std::pair<int, int>, 3> tri_edges = {
            std::make_pair(v0, v1),
            std::make_pair(v1, v2),
            std::make_pair(v2, v0)
        };

        for (const auto& [a, b] : tri_edges) {
            const auto key = edge_key(a, b);
            if (seen.insert(key).second) {
                edges.emplace_back(std::min(a, b), std::max(a, b));
            }
        }
    }

    Eigen::MatrixXi E(static_cast<int>(edges.size()), 2);
    for (int i = 0; i < static_cast<int>(edges.size()); ++i) {
        E(i, 0) = edges[i](0);
        E(i, 1) = edges[i](1);
    }
    return E;
}

} // namespace

void ContactAssembler::build(const Dynamics::World& world) {
    body_meshes_.clear();
    body_id_to_mesh_index_.clear();

    body_meshes_.reserve(world.bodies.size());

    for (size_t i = 0; i < world.bodies.size(); ++i) {
        auto mesh = std::dynamic_pointer_cast<Physics::MeshShape>(world.bodies[i]->shape);
        if (!mesh) {
            continue;
        }

        BodyContactMesh bcm;
        bcm.body_id = world.bodies[i]->id;
        bcm.world_index = static_cast<int>(i);
        bcm.vertices = mesh->vertices();
        bcm.faces = mesh->faces();
        bcm.edges = build_unique_edges(bcm.faces);
        bcm.world_position = world.bodies[i]->position;
        bcm.world_orientation = world.bodies[i]->orientation;

        const auto [local_min, local_max] = mesh->computeAABB();
        bcm.world_aabb = transform_local_aabb(
            Geometry::AABB(local_min, local_max),
            world.bodies[i]->position,
            world.bodies[i]->orientation);

        body_id_to_mesh_index_[bcm.body_id] = static_cast<int>(body_meshes_.size());
        body_meshes_.push_back(std::move(bcm));
    }
}

std::vector<std::pair<int, int>> ContactAssembler::detectBodyPairs(double inflation_radius) const {
    std::vector<std::pair<int, int>> world_pairs;
    if (body_meshes_.size() < 2) {
        return world_pairs;
    }

    std::vector<Geometry::AABB> aabbs;
    aabbs.reserve(body_meshes_.size());
    for (const auto& mesh : body_meshes_) {
        aabbs.push_back(mesh.world_aabb.inflated(std::max(0.0, inflation_radius)));
    }

    broad_phase_.markNeedsRebuild();
    auto mesh_pairs = broad_phase_.detect(aabbs);

    world_pairs.reserve(mesh_pairs.size());
    for (const auto& p : mesh_pairs) {
        world_pairs.emplace_back(
            body_meshes_[p.first].world_index,
            body_meshes_[p.second].world_index);
    }
    return world_pairs;
}

double ContactAssembler::estimateMinBodyPairDistance(double inflation_radius) const {
    if (body_meshes_.size() < 2) {
        return std::numeric_limits<double>::infinity();
    }

    const auto pairs = detectBodyPairs(inflation_radius);
    if (pairs.empty()) {
        return std::numeric_limits<double>::infinity();
    }

    double min_dist = std::numeric_limits<double>::infinity();
    for (const auto& wp : pairs) {
        const int body_id_a = wp.first;
        const int body_id_b = wp.second;

        int ia = -1;
        int ib = -1;
        for (int k = 0; k < static_cast<int>(body_meshes_.size()); ++k) {
            if (body_meshes_[k].world_index == body_id_a) {
                ia = k;
            }
            if (body_meshes_[k].world_index == body_id_b) {
                ib = k;
            }
        }
        if (ia < 0 || ib < 0) {
            continue;
        }

        min_dist = std::min(min_dist, aabb_distance(body_meshes_[ia].world_aabb, body_meshes_[ib].world_aabb));
    }
    return min_dist;
}

void ContactAssembler::buildCandidateSet(double activation_distance, ContactCandidateSet& out_candidates) const {
    out_candidates.clear();
    if (body_meshes_.size() < 2) {
        return;
    }

    const auto pairs = detectBodyPairs(activation_distance);
    constexpr int kMaxVerticesPerBody = 32;
    constexpr int kMaxEdgesPerBody = 32;
    constexpr int kMaxFacesPerBody = 32;

    auto find_mesh_by_world_index = [&](int world_index) -> const BodyContactMesh* {
        for (const auto& m : body_meshes_) {
            if (m.world_index == world_index) {
                return &m;
            }
        }
        return nullptr;
    };

    for (const auto& p : pairs) {
        const BodyContactMesh* A = find_mesh_by_world_index(p.first);
        const BodyContactMesh* B = find_mesh_by_world_index(p.second);
        if (!A || !B) {
            continue;
        }

        if (aabb_distance(A->world_aabb, B->world_aabb) > activation_distance) {
            continue;
        }

        auto world_vertex = [](const BodyContactMesh* mesh, int local_vertex_id) {
            return mesh->world_position + mesh->world_orientation * mesh->vertices.row(local_vertex_id).transpose();
        };

        const auto vertex_indices_A = select_top_k_indices(
            A->vertices.rows(),
            std::min<int>(kMaxVerticesPerBody, A->vertices.rows()),
            [&](int vid) {
                return point_aabb_distance(world_vertex(A, vid), B->world_aabb);
            });
        const auto vertex_indices_B = select_top_k_indices(
            B->vertices.rows(),
            std::min<int>(kMaxVerticesPerBody, B->vertices.rows()),
            [&](int vid) {
                return point_aabb_distance(world_vertex(B, vid), A->world_aabb);
            });

        const auto edge_indices_A = select_top_k_indices(
            A->edges.rows(),
            std::min<int>(kMaxEdgesPerBody, A->edges.rows()),
            [&](int eid) {
                const Eigen::Vector3d ev0 = world_vertex(A, A->edges(eid, 0));
                const Eigen::Vector3d ev1 = world_vertex(A, A->edges(eid, 1));
                return std::min(point_aabb_distance(ev0, B->world_aabb),
                                point_aabb_distance(ev1, B->world_aabb));
            });
        const auto edge_indices_B = select_top_k_indices(
            B->edges.rows(),
            std::min<int>(kMaxEdgesPerBody, B->edges.rows()),
            [&](int eid) {
                const Eigen::Vector3d ev0 = world_vertex(B, B->edges(eid, 0));
                const Eigen::Vector3d ev1 = world_vertex(B, B->edges(eid, 1));
                return std::min(point_aabb_distance(ev0, A->world_aabb),
                                point_aabb_distance(ev1, A->world_aabb));
            });

        const auto face_indices_A = select_top_k_indices(
            A->faces.rows(),
            std::min<int>(kMaxFacesPerBody, A->faces.rows()),
            [&](int fid) {
                const Eigen::Vector3d fv0 = world_vertex(A, A->faces(fid, 0));
                const Eigen::Vector3d fv1 = world_vertex(A, A->faces(fid, 1));
                const Eigen::Vector3d fv2 = world_vertex(A, A->faces(fid, 2));
                return std::min({point_aabb_distance(fv0, B->world_aabb),
                                 point_aabb_distance(fv1, B->world_aabb),
                                 point_aabb_distance(fv2, B->world_aabb)});
            });
        const auto face_indices_B = select_top_k_indices(
            B->faces.rows(),
            std::min<int>(kMaxFacesPerBody, B->faces.rows()),
            [&](int fid) {
                const Eigen::Vector3d fv0 = world_vertex(B, B->faces(fid, 0));
                const Eigen::Vector3d fv1 = world_vertex(B, B->faces(fid, 1));
                const Eigen::Vector3d fv2 = world_vertex(B, B->faces(fid, 2));
                return std::min({point_aabb_distance(fv0, A->world_aabb),
                                 point_aabb_distance(fv1, A->world_aabb),
                                 point_aabb_distance(fv2, A->world_aabb)});
            });

        for (const int i : vertex_indices_A) {
            for (const int j : vertex_indices_B) {
                out_candidates.vv.push_back({A->body_id, B->body_id, i, j});
            }
        }

        for (const int i : edge_indices_A) {
            for (const int j : vertex_indices_B) {
                out_candidates.ev.push_back(
                    {A->body_id, B->body_id, A->edges(i, 0), A->edges(i, 1), j});
            }
        }
        for (const int i : edge_indices_B) {
            for (const int j : vertex_indices_A) {
                out_candidates.ev.push_back(
                    {B->body_id, A->body_id, B->edges(i, 0), B->edges(i, 1), j});
            }
        }

        for (const int i : edge_indices_A) {
            for (const int j : edge_indices_B) {
                out_candidates.ee.push_back(
                    {A->body_id, B->body_id,
                     A->edges(i, 0), A->edges(i, 1),
                     B->edges(j, 0), B->edges(j, 1)});
            }
        }

        for (const int i : face_indices_A) {
            for (const int j : vertex_indices_B) {
                out_candidates.fv.push_back({A->body_id, B->body_id, i, j});
            }
        }
        for (const int i : face_indices_B) {
            for (const int j : vertex_indices_A) {
                out_candidates.fv.push_back({B->body_id, A->body_id, i, j});
            }
        }
    }
}

} // namespace NexDynIPC::Physics::Contact
