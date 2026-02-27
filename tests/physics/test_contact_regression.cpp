#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "NexDynIPC/Physics/Contact/BroadPhase.h"
#include "NexDynIPC/Physics/Contact/AdaptiveBarrier.h"
#include "NexDynIPC/Physics/Geometry/MeshShape.h"
#include "NexDynIPC/Dynamics/IPCSolver.h"

#include <filesystem>
#include <fstream>
#include <sstream>

using Catch::Approx;

namespace {

std::shared_ptr<NexDynIPC::Physics::MeshShape> load_obj_mesh(
    const std::filesystem::path& path,
    int max_vertices = 128,
    int max_faces = 256)
{
    std::ifstream in(path);
    if (!in.is_open()) {
        return nullptr;
    }

    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> faces;
    std::string line;

    while (std::getline(in, line)) {
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
                const auto slash_pos = token.find('/');
                const std::string idx_text = slash_pos == std::string::npos
                    ? token
                    : token.substr(0, slash_pos);
                if (idx_text.empty()) {
                    continue;
                }
                const int idx = std::stoi(idx_text);
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

TEST_CASE("Contact broadphase detects overlap", "[contact][regression]")
{
    using namespace NexDynIPC::Physics;
    std::vector<Geometry::AABB> boxes;
    boxes.emplace_back(Eigen::Vector3d(-0.5, -0.5, -0.5), Eigen::Vector3d(0.5, 0.5, 0.5));
    boxes.emplace_back(Eigen::Vector3d(0.25, -0.25, -0.25), Eigen::Vector3d(1.0, 0.25, 0.25));

    BruteForceBroadPhase broad_phase;
    const auto pairs = broad_phase.detect(boxes);

    REQUIRE(pairs.size() == 1);
    REQUIRE(pairs[0].first == 0);
    REQUIRE(pairs[0].second == 1);
}

TEST_CASE("Contact broadphase excludes separated boxes", "[contact][regression]")
{
    using namespace NexDynIPC::Physics;
    std::vector<Geometry::AABB> boxes;
    boxes.emplace_back(Eigen::Vector3d(-0.5, -0.5, -0.5), Eigen::Vector3d(0.5, 0.5, 0.5));
    boxes.emplace_back(Eigen::Vector3d(2.0, 2.0, 2.0), Eigen::Vector3d(3.0, 3.0, 3.0));

    BVHBroadPhase broad_phase;
    const auto pairs = broad_phase.detect(boxes);
    REQUIRE(pairs.empty());
}

TEST_CASE("Adaptive barrier dhat and activity", "[contact][regression]")
{
    using namespace NexDynIPC::Physics::Contact;
    AdaptiveBarrier barrier;
    barrier.initialize(1.0, 10.0);
    barrier.setDhat(0.05);

    REQUIRE(barrier.currentDhat() == Approx(0.05));
    REQUIRE(barrier.isActive(0.01));
    REQUIRE(!barrier.isActive(0.1));
}

TEST_CASE("IPCSolver contact controls setter/getter", "[contact][regression]")
{
    using namespace NexDynIPC::Dynamics;
    IPCSolver solver;

    solver.enableCCD(false);
    solver.setCCDSafetyFactor(0.6);
    solver.enableFriction(true);
    solver.setFrictionCoefficient(0.35);
    solver.enableAdaptiveBarrier(true);
    solver.adaptiveBarrier().setDhat(0.02);
    solver.setALMContactTolerance(5e-4);

    REQUIRE(!solver.isCCDEnabled());
    REQUIRE(solver.ccdSafetyFactor() == Approx(0.6));
    REQUIRE(solver.isFrictionEnabled());
    REQUIRE(solver.frictionCoefficient() == Approx(0.35));
    REQUIRE(solver.isAdaptiveBarrierEnabled());
    REQUIRE(solver.adaptiveBarrier().currentDhat() == Approx(0.02));
    REQUIRE(solver.lastContactResidual() == Approx(0.0));
    REQUIRE(solver.lastContactResidualSeries().empty());

    const auto export_dir = std::filesystem::temp_directory_path() / "nexdynipc_contact_diag_tests";
    std::filesystem::create_directories(export_dir);
    const auto export_path = export_dir / "contact_residual_series.csv";

    REQUIRE(solver.writeLastContactResidualSeriesCSV(export_path.string()));
    REQUIRE(std::filesystem::exists(export_path));

    std::ifstream in(export_path);
    REQUIRE(in.is_open());
    std::string header;
    REQUIRE(std::getline(in, header));
    REQUIRE(header == "alm_iteration,newton_iteration,contact_residual,line_search_alpha,contact_value_before,contact_value_after");
}

TEST_CASE("OBJ mesh AABB is valid", "[contact][regression]")
{
    const auto repo_root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const auto cube_path = repo_root / "models" / "obj" / "test" / "cube.obj";
    const auto sphere_path = repo_root / "models" / "obj" / "test" / "sphere.obj";

    REQUIRE(std::filesystem::exists(cube_path));
    REQUIRE(std::filesystem::exists(sphere_path));

    const auto cube_mesh = load_obj_mesh(cube_path);
    const auto sphere_mesh = load_obj_mesh(sphere_path);

    REQUIRE(cube_mesh != nullptr);
    REQUIRE(sphere_mesh != nullptr);

    const auto [cube_min, cube_max] = cube_mesh->computeAABB();
    const auto [sphere_min, sphere_max] = sphere_mesh->computeAABB();

    REQUIRE((cube_max.array() >= cube_min.array()).all());
    REQUIRE((sphere_max.array() >= sphere_min.array()).all());
}

TEST_CASE("OBJ contact broadphase smoke", "[contact][regression]")
{
    using namespace NexDynIPC::Physics;

    const auto repo_root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const auto cube_path = repo_root / "models" / "obj" / "test" / "cube.obj";
    const auto sphere_path = repo_root / "models" / "obj" / "test" / "sphere.obj";

    const auto cube_mesh = load_obj_mesh(cube_path);
    const auto sphere_mesh = load_obj_mesh(sphere_path);
    REQUIRE(cube_mesh != nullptr);
    REQUIRE(sphere_mesh != nullptr);

    const auto [cube_min, cube_max] = cube_mesh->computeAABB();
    const auto [sphere_min, sphere_max] = sphere_mesh->computeAABB();

    Geometry::AABB cube_box(cube_min, cube_max);

    const Eigen::Vector3d sphere_extent = sphere_max - sphere_min;
    const Eigen::Vector3d desired_sphere_center = cube_box.center() + Eigen::Vector3d(0.0, 0.25 * cube_box.extent().y(), 0.0);
    Geometry::AABB sphere_box(
        desired_sphere_center - 0.5 * sphere_extent,
        desired_sphere_center + 0.5 * sphere_extent);

    std::vector<Geometry::AABB> boxes;
    boxes.push_back(cube_box.inflated(0.05));
    boxes.push_back(sphere_box.inflated(0.05));

    BruteForceBroadPhase broad_phase;
    const auto pairs = broad_phase.detect(boxes);
    REQUIRE(!pairs.empty());
}
