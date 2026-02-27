#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "NexDynIPC/Physics/Contact/Distance.h"
#include "NexDynIPC/Physics/Contact/CollisionCandidates.h"
#include "NexDynIPC/Physics/Contact/BroadPhase.h"
#include "NexDynIPC/Physics/Geometry/MeshShape.h"
#include "NexDynIPC/Dynamics/RigidBody.h"

#include <fstream>
#include <sstream>
#include <filesystem>
#include <limits>
#include <algorithm>

using namespace Catch::Matchers;

namespace {

std::shared_ptr<NexDynIPC::Physics::MeshShape> loadObjAsMeshShape(
    const std::filesystem::path& path,
    int max_vertices = std::numeric_limits<int>::max(),
    int max_faces = std::numeric_limits<int>::max())
{
    std::ifstream in(path);
    if (!in.is_open()) {
        return nullptr;
    }

    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> faces;

    std::string line;
    while (std::getline(in, line)) {
        if (line.size() < 2) {
            continue;
        }

        std::istringstream ls(line);
        std::string head;
        ls >> head;

        if (head == "v") {
            if (static_cast<int>(vertices.size()) >= max_vertices) {
                continue;
            }
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            ls >> x >> y >> z;
            vertices.emplace_back(x, y, z);
            continue;
        }

        if (head == "f") {
            if (static_cast<int>(faces.size()) >= max_faces) {
                continue;
            }
            std::vector<int> face_indices;
            std::string token;
            while (ls >> token) {
                const auto slash = token.find('/');
                const std::string idx_str = slash == std::string::npos
                    ? token
                    : token.substr(0, slash);
                if (idx_str.empty()) {
                    continue;
                }
                const int idx = std::stoi(idx_str);
                if (idx > 0 && idx <= static_cast<int>(vertices.size())) {
                    face_indices.push_back(idx - 1);
                }
            }

            if (face_indices.size() >= 3) {
                for (size_t i = 1; i + 1 < face_indices.size(); ++i) {
                    if (static_cast<int>(faces.size()) >= max_faces) {
                        break;
                    }
                    faces.emplace_back(face_indices[0], face_indices[i], face_indices[i + 1]);
                }
            }
        }
    }

    if (vertices.empty() || faces.empty()) {
        return nullptr;
    }

    Eigen::MatrixXd V(static_cast<int>(vertices.size()), 3);
    for (int i = 0; i < static_cast<int>(vertices.size()); ++i) {
        V.row(i) = vertices[static_cast<size_t>(i)];
    }

    Eigen::MatrixXi F(static_cast<int>(faces.size()), 3);
    for (int i = 0; i < static_cast<int>(faces.size()); ++i) {
        F.row(i) = faces[static_cast<size_t>(i)];
    }

    return std::make_shared<NexDynIPC::Physics::MeshShape>(V, F);
}

} // namespace

TEST_CASE("Contact distance primitives compute expected values", "[contact][distance]")
{
    using namespace NexDynIPC::Physics::Contact;

    SECTION("Point-triangle distance") {
        const Eigen::Vector3d p(0.25, 0.25, 1.0);
        const Eigen::Vector3d t0(0.0, 0.0, 0.0);
        const Eigen::Vector3d t1(1.0, 0.0, 0.0);
        const Eigen::Vector3d t2(0.0, 1.0, 0.0);
        Eigen::Vector3d q = Eigen::Vector3d::Zero();

        const double d = pointTriangleDistance(p, t0, t1, t2, q);

        REQUIRE_THAT(d, WithinAbs(1.0, 1e-8));
        REQUIRE_THAT(q.x(), WithinAbs(0.25, 1e-8));
        REQUIRE_THAT(q.y(), WithinAbs(0.25, 1e-8));
        REQUIRE_THAT(q.z(), WithinAbs(0.0, 1e-8));
    }

    SECTION("Edge-edge distance") {
        const Eigen::Vector3d a0(0.0, 0.0, 0.0);
        const Eigen::Vector3d a1(1.0, 0.0, 0.0);
        const Eigen::Vector3d b0(0.5, -1.0, 1.0);
        const Eigen::Vector3d b1(0.5, 1.0, 1.0);
        Eigen::Vector3d ca = Eigen::Vector3d::Zero();
        Eigen::Vector3d cb = Eigen::Vector3d::Zero();

        const double d = edgeEdgeDistance(a0, a1, b0, b1, ca, cb);
        REQUIRE_THAT(d, WithinAbs(1.0, 1e-8));
        REQUIRE_THAT(ca.x(), WithinAbs(0.5, 1e-8));
        REQUIRE_THAT(ca.z(), WithinAbs(0.0, 1e-8));
        REQUIRE_THAT(cb.x(), WithinAbs(0.5, 1e-8));
        REQUIRE_THAT(cb.z(), WithinAbs(1.0, 1e-8));
    }
}

TEST_CASE("OBJ meshes can build a valid near-contact scene", "[contact][pipeline][obj]")
{
    using namespace NexDynIPC;
    using namespace NexDynIPC::Physics;
    using namespace NexDynIPC::Physics::Contact;
    using namespace NexDynIPC::Dynamics;

    const std::filesystem::path repo_root =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const auto cube_path = repo_root / "models" / "obj" / "test" / "cube.obj";
    const auto sphere_path = repo_root / "models" / "obj" / "test" / "sphere.obj";

    REQUIRE(std::filesystem::exists(cube_path));
    REQUIRE(std::filesystem::exists(sphere_path));

    const auto cube_mesh = loadObjAsMeshShape(cube_path, 64, 128);
    const auto sphere_mesh = loadObjAsMeshShape(sphere_path, 64, 128);
    REQUIRE(cube_mesh != nullptr);
    REQUIRE(sphere_mesh != nullptr);

    auto cube = std::make_shared<RigidBody>();
    cube->id = 0;
    cube->shape = cube_mesh;
    cube->position = Eigen::Vector3d::Zero();
    cube->orientation = Eigen::Quaterniond::Identity();

    auto sphere = std::make_shared<RigidBody>();
    sphere->id = 1;
    sphere->shape = sphere_mesh;
    sphere->orientation = Eigen::Quaterniond::Identity();

    const auto [cube_min, cube_max] = cube_mesh->computeAABB();
    const auto [sphere_min, sphere_max] = sphere_mesh->computeAABB();
    const double scene_gap = -0.01;
    sphere->position = Eigen::Vector3d(
        0.0,
        (cube_max.y() - sphere_min.y()) + scene_gap,
        0.0);

    std::vector<std::shared_ptr<RigidBody>> bodies{cube, sphere};

    std::vector<Physics::Geometry::AABB> aabbs;
    aabbs.push_back(Physics::Geometry::AABB(cube->toWorld(cube_min), cube->toWorld(cube_max)).inflated(0.1));
    aabbs.push_back(Physics::Geometry::AABB(sphere->toWorld(sphere_min), sphere->toWorld(sphere_max)).inflated(0.1));

    Physics::BruteForceBroadPhase broad_phase;
    const auto broad_pairs = broad_phase.detect(aabbs);
    REQUIRE(!broad_pairs.empty());

    CollisionCandidatesBuilder builder;
    const auto candidates = builder.build(bodies, broad_pairs);
    REQUIRE(!candidates.empty());

    const auto distances = computeNarrowPhaseDistances(candidates, bodies, 2.0);
    REQUIRE(!distances.empty());

    double min_distance = std::numeric_limits<double>::infinity();
    for (const auto& d : distances) {
        min_distance = std::min(min_distance, d.distance);
    }

    REQUIRE(min_distance >= 0.0);
    REQUIRE(min_distance < 2.0);
    (void)sphere_max;
}
