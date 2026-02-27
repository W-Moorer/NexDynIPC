#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "NexDynIPC/App/SceneLoader.h"
#include "NexDynIPC/App/Simulation.h"
#include "NexDynIPC/Dynamics/World.h"

#include <filesystem>
#include <fstream>

using namespace NexDynIPC::App;
using namespace NexDynIPC::Dynamics;
using Catch::Approx;

namespace {

std::filesystem::path write_temp_scene(const std::string& name, const std::string& content) {
    auto dir = std::filesystem::temp_directory_path() / "nexdynipc_scene_loader_tests";
    std::filesystem::create_directories(dir);
    auto file = dir / (name + ".json");

    std::ofstream out(file);
    out << content;
    out.close();

    return file;
}

} // namespace

TEST_CASE("SceneLoader supports joints and constraints coexistence", "[app][scene_loader]") {
    const std::string scene_json = R"JSON(
{
  "case_name": "coexist_case",
  "bodies": [
    {"id": 0, "name": "base", "mass": 1.0, "position": [0.0, 0.0, 0.0]},
    {"id": 1, "name": "link", "mass": 1.0, "position": [1.0, 0.0, 0.0]}
  ],
  "joints": [
    {
      "type": "fixed",
      "body": 0,
      "position": [0.0, 0.0, 0.0]
    },
    {
      "type": "revolute",
      "body_a": 0,
      "body_b": 1,
      "anchor_a": [0.0, 0.0, 0.0],
      "anchor_b": [-1.0, 0.0, 0.0],
      "axis_a": [0.0, 0.0, 1.0],
      "axis_b": [0.0, 0.0, 1.0]
    }
  ],
  "constraints": [
    {
      "type": "revolute",
      "body_a": 0,
      "body_b": 1,
      "anchor_a": [0.0, 0.0, 0.0],
      "anchor_b": [-1.0, 0.0, 0.0],
      "axis_a": [0.0, 0.0, 1.0],
      "axis_b": [0.0, 0.0, 1.0]
    }
  ]
}
)JSON";

    const auto file = write_temp_scene("coexist_scene", scene_json);

    World world;
    SceneLoader::load(file.string(), world);

    REQUIRE(world.bodies.size() == 2);
    REQUIRE(world.joints.size() == 2); // duplicated revolute should be deduped
}

TEST_CASE("SceneLoader applies ALM and output defaults", "[app][scene_loader]") {
    const std::string scene_json = R"JSON(
{
  "case_name": "alm_case",
  "settings": {
    "dt": 0.002,
    "max_time": 1.2,
    "joint_stiffness": 2000.0,
    "alm_max_iters": 7,
    "alm_constraint_tolerance": 1e-5,
    "alm_dual_tolerance": 1e-4,
    "alm_hardening_trigger": 1.5,
    "alm_hardening_ratio": 1.8
  },
  "bodies": [
    {"id": 0, "name": "base", "mass": 1.0, "position": [0.0, 0.0, 0.0]}
  ]
}
)JSON";

    const auto file = write_temp_scene("alm_scene", scene_json);

    World world;
    SimulationConfig config;
    SceneLoader::load(file.string(), world, config);

    REQUIRE(config.dt == Approx(0.002));
    REQUIRE(config.max_time == Approx(1.2));
    REQUIRE(config.joint_stiffness == Approx(2000.0));
    REQUIRE(config.alm_max_iters == 7);
    REQUIRE(config.alm_constraint_tolerance == Approx(1e-5));
    REQUIRE(config.alm_dual_tolerance == Approx(1e-4));
    REQUIRE(config.alm_hardening_trigger == Approx(1.5));
    REQUIRE(config.alm_hardening_ratio == Approx(1.8));

    REQUIRE(config.output_name == "alm_case");
    REQUIRE(config.output_dir == "output/alm_case");
}
