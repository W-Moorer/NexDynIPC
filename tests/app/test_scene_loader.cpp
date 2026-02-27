#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "NexDynIPC/App/SceneLoader.h"
#include "NexDynIPC/App/Simulation.h"
#include "NexDynIPC/Dynamics/World.h"
#include "NexDynIPC/Control/VelocityDriveForm.h"
#include "NexDynIPC/Control/LinearVelocityDriveForm.h"
#include "NexDynIPC/Control/PositionDriveForm.h"
#include "NexDynIPC/Control/ForceDriveForm.h"
#include "NexDynIPC/Control/DampedSpringForm.h"
#include "NexDynIPC/Dynamics/Joints/AngleLimitJoint.h"
#include "NexDynIPC/Dynamics/Joints/DistanceLimitJoint.h"
#include "NexDynIPC/Dynamics/Joints/HingeJoint.h"

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
      "type": "hinge",
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
      "type": "hinge",
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
    REQUIRE(world.joints.size() == 2); // duplicated hinge should be deduped
}

TEST_CASE("SceneLoader maps deprecated revolute to hinge", "[app][scene_loader]") {
    const std::string scene_json = R"JSON(
{
  "case_name": "revolute_compat_case",
  "bodies": [
    {"id": 0, "name": "base", "mass": 1.0, "position": [0.0, 0.0, 0.0]},
    {"id": 1, "name": "link", "mass": 1.0, "position": [1.0, 0.0, 0.0]}
  ],
  "joints": [
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

    const auto file = write_temp_scene("revolute_compat_scene", scene_json);

    World world;
    SceneLoader::load(file.string(), world);

    REQUIRE(world.joints.size() == 1);
    auto hinge = std::dynamic_pointer_cast<NexDynIPC::Dynamics::HingeJoint>(world.joints[0]);
    REQUIRE(hinge != nullptr);
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
    "alm_hardening_ratio": 1.8,
    "kpi_gate_enabled": true,
    "kpi_v_w_max": 0.3,
    "kpi_t_sat_max": 0.05,
    "kpi_r_dual_max": 2e-3
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
    REQUIRE(config.kpi_gate_enabled == true);
    REQUIRE(config.kpi_v_w_max == Approx(0.3));
    REQUIRE(config.kpi_t_sat_max == Approx(0.05));
    REQUIRE(config.kpi_r_dual_max == Approx(2e-3));

    REQUIRE(config.output_name == "alm_case");
    REQUIRE(config.output_dir == "output/alm_case");
}

TEST_CASE("SceneLoader applies contact settings block", "[app][scene_loader]") {
    const std::string scene_json = R"JSON(
{
  "case_name": "contact_settings_case",
  "settings": {
    "contact": {
      "enabled": true,
      "dhat": 0.02,
      "friction": {
        "enabled": true,
        "mu": 0.45
      },
      "ccd": {
        "enabled": true,
        "safety_factor": 0.7,
        "max_substeps": 6,
        "min_step_ratio": 0.15
      }
    },
    "solver": {
      "newton_fallback_enabled": true,
      "newton_fallback_retries": 3,
      "newton_fallback_damping": 0.4
    }
  },
  "bodies": [
    {"id": 0, "name": "base", "mass": 1.0, "position": [0.0, 0.0, 0.0]}
  ]
}
)JSON";

    const auto file = write_temp_scene("contact_settings_scene", scene_json);

    World world;
    SimulationConfig config;
    SceneLoader::load(file.string(), world, config);

    REQUIRE(config.contact_enabled == true);
    REQUIRE(config.contact_dhat == Approx(0.02));
    REQUIRE(config.contact_friction_enabled == true);
    REQUIRE(config.contact_friction_mu == Approx(0.45));
    REQUIRE(config.contact_ccd_enabled == true);
    REQUIRE(config.contact_ccd_safety_factor == Approx(0.7));
    REQUIRE(config.contact_ccd_max_substeps == 6);
    REQUIRE(config.contact_ccd_min_step_ratio == Approx(0.15));

    REQUIRE(config.solver_newton_fallback_enabled == true);
    REQUIRE(config.solver_newton_fallback_retries == 3);
    REQUIRE(config.solver_newton_fallback_damping == Approx(0.4));
}

TEST_CASE("SceneLoader loads angular velocity drive constraint into forms", "[app][scene_loader]") {
    const std::string scene_json = R"JSON(
{
  "case_name": "drive_case",
  "bodies": [
    {"id": 0, "name": "base", "mass": 1.0, "position": [0.0, 0.0, 0.0]},
    {"id": 1, "name": "link", "mass": 1.0, "position": [1.0, 0.0, 0.0]}
  ],
  "constraints": [
    {
      "type": "angular_velocity_drive",
      "body_a": 0,
      "body_b": 1,
      "axis": [0.0, 0.0, 1.0],
      "target_velocity_radps": 5.0,
      "kv": 150.0,
      "max_torque_nm": 12.0,
      "delay_radps": 0.2,
      "delay_tau": 0.05
    }
  ]
}
)JSON";

    const auto file = write_temp_scene("drive_scene", scene_json);

    World world;
    SceneLoader::load(file.string(), world);

    REQUIRE(world.forms.size() == 1);
    auto drive = std::dynamic_pointer_cast<NexDynIPC::Control::VelocityDriveForm>(world.forms[0]);
    REQUIRE(drive != nullptr);
    REQUIRE(drive->getTargetVelocity() == Approx(5.0));
    REQUIRE(drive->getVelocityGain() == Approx(150.0));
    REQUIRE(drive->getMaxTorque() == Approx(12.0));
    REQUIRE(drive->getDelay() == Approx(0.2));
    REQUIRE(drive->getDelayTau() == Approx(0.05));
}

TEST_CASE("SceneLoader loads linear velocity drive constraint into forms", "[app][scene_loader]") {
    const std::string scene_json = R"JSON(
{
  "case_name": "linear_drive_case",
  "bodies": [
    {"id": 0, "name": "base", "mass": 1.0, "position": [0.0, 0.0, 0.0]},
    {"id": 1, "name": "slider", "mass": 1.0, "position": [1.0, 0.0, 0.0]}
  ],
  "constraints": [
    {
      "type": "linear_velocity_drive",
      "body_a": 0,
      "body_b": 1,
      "axis": [1.0, 0.0, 0.0],
      "target_velocity_mps": 2.0,
      "kv": 80.0,
      "max_force_n": 25.0,
      "delay_mps": 0.1,
      "delay_tau": 0.04
    }
  ]
}
)JSON";

    const auto file = write_temp_scene("linear_drive_scene", scene_json);
    World world;
    SceneLoader::load(file.string(), world);

    REQUIRE(world.forms.size() == 1);
    auto drive = std::dynamic_pointer_cast<NexDynIPC::Control::LinearVelocityDriveForm>(world.forms[0]);
    REQUIRE(drive != nullptr);
    REQUIRE(drive->getTargetVelocity() == Approx(2.0));
    REQUIRE(drive->getVelocityGain() == Approx(80.0));
    REQUIRE(drive->getMaxForce() == Approx(25.0));
    REQUIRE(drive->getDelay() == Approx(0.1));
    REQUIRE(drive->getDelayTau() == Approx(0.04));
}

TEST_CASE("SceneLoader loads position drive constraint into forms", "[app][scene_loader]") {
    const std::string scene_json = R"JSON(
{
  "case_name": "position_drive_case",
  "bodies": [
    {"id": 0, "name": "base", "mass": 1.0, "position": [0.0, 0.0, 0.0]},
    {"id": 1, "name": "slider", "mass": 1.0, "position": [1.0, 0.0, 0.0]}
  ],
  "constraints": [
    {
      "type": "position_drive",
      "body_a": 0,
      "body_b": 1,
      "axis": [1.0, 0.0, 0.0],
      "target_position_m": 0.5,
      "kp": 120.0,
      "max_force_n": 30.0,
      "deadzone_m": 0.01
    }
  ]
}
)JSON";

    const auto file = write_temp_scene("position_drive_scene", scene_json);
    World world;
    SceneLoader::load(file.string(), world);

    REQUIRE(world.forms.size() == 1);
    auto drive = std::dynamic_pointer_cast<NexDynIPC::Control::PositionDriveForm>(world.forms[0]);
    REQUIRE(drive != nullptr);
    REQUIRE(drive->getTargetPosition() == Approx(0.5));
    REQUIRE(drive->getPositionGain() == Approx(120.0));
    REQUIRE(drive->getMaxForce() == Approx(30.0));
    REQUIRE(drive->getDeadzone() == Approx(0.01));
}

TEST_CASE("SceneLoader loads force drive constraint into forms", "[app][scene_loader]") {
    const std::string scene_json = R"JSON(
{
  "case_name": "force_drive_case",
  "bodies": [
    {"id": 0, "name": "body", "mass": 1.0, "position": [0.0, 0.0, 0.0]}
  ],
  "constraints": [
    {
      "type": "force_drive",
      "body": 0,
      "axis": [0.0, 1.0, 0.0],
      "force_n": 10.0,
      "torque": [0.0, 0.0, 1.0]
    }
  ]
}
)JSON";

    const auto file = write_temp_scene("force_drive_scene", scene_json);
    World world;
    SceneLoader::load(file.string(), world);

    REQUIRE(world.forms.size() == 1);
    auto drive = std::dynamic_pointer_cast<NexDynIPC::Control::ForceDriveForm>(world.forms[0]);
    REQUIRE(drive != nullptr);
    REQUIRE(drive->getForce().y() == Approx(10.0));
    REQUIRE(drive->getTorque().z() == Approx(1.0));
}

TEST_CASE("SceneLoader loads angle limit constraint into joints", "[app][scene_loader]") {
    const std::string scene_json = R"JSON(
{
  "case_name": "limit_case",
  "bodies": [
    {"id": 0, "name": "base", "mass": 1.0, "position": [0.0, 0.0, 0.0]},
    {"id": 1, "name": "link", "mass": 1.0, "position": [1.0, 0.0, 0.0]}
  ],
  "constraints": [
    {
      "type": "angle_limit",
      "body_a": 0,
      "body_b": 1,
      "axis": [0.0, 0.0, 1.0],
      "min_angle_deg": -5.0,
      "max_angle_deg": 5.0,
      "stiffness": 100.0
    }
  ]
}
)JSON";

    const auto file = write_temp_scene("limit_scene", scene_json);

    World world;
    SceneLoader::load(file.string(), world);

    REQUIRE(world.joints.size() == 1);
    auto limit = std::dynamic_pointer_cast<NexDynIPC::Dynamics::AngleLimitJoint>(world.joints[0]);
    REQUIRE(limit != nullptr);
}

TEST_CASE("SceneLoader loads distance limit constraint into joints", "[app][scene_loader]") {
    const std::string scene_json = R"JSON(
{
  "case_name": "distance_limit_case",
  "bodies": [
    {"id": 0, "name": "a", "mass": 1.0, "position": [0.0, 0.0, 0.0]},
    {"id": 1, "name": "b", "mass": 1.0, "position": [1.0, 0.0, 0.0]}
  ],
  "constraints": [
    {
      "type": "distance_limit",
      "body_a": 0,
      "body_b": 1,
      "min_distance_m": 0.5,
      "max_distance_m": 1.5,
      "stiffness": 200.0
    }
  ]
}
)JSON";

    const auto file = write_temp_scene("distance_limit_scene", scene_json);

    World world;
    SceneLoader::load(file.string(), world);

    REQUIRE(world.joints.size() == 1);
    auto limit = std::dynamic_pointer_cast<NexDynIPC::Dynamics::DistanceLimitJoint>(world.joints[0]);
    REQUIRE(limit != nullptr);
    REQUIRE(limit->getMinDistance() == Approx(0.5));
    REQUIRE(limit->getMaxDistance() == Approx(1.5));
}

TEST_CASE("SceneLoader loads damped spring constraint into forms", "[app][scene_loader]") {
    const std::string scene_json = R"JSON(
{
  "case_name": "damped_spring_case",
  "bodies": [
    {"id": 0, "name": "a", "mass": 1.0, "position": [0.0, 0.0, 0.0]},
    {"id": 1, "name": "b", "mass": 1.0, "position": [1.0, 0.0, 0.0]}
  ],
  "constraints": [
    {
      "type": "damped_spring",
      "body_a": 0,
      "body_b": 1,
      "rest_length_m": 0.8,
      "stiffness_npm": 120.0,
      "damping_nspm": 6.0
    }
  ]
}
)JSON";

    const auto file = write_temp_scene("damped_spring_scene", scene_json);

    World world;
    SceneLoader::load(file.string(), world);

    REQUIRE(world.forms.size() == 1);
    auto spring = std::dynamic_pointer_cast<NexDynIPC::Control::DampedSpringForm>(world.forms[0]);
    REQUIRE(spring != nullptr);
    REQUIRE(spring->getRestLength() == Approx(0.8));
    REQUIRE(spring->getStiffness() == Approx(120.0));
    REQUIRE(spring->getDamping() == Approx(6.0));
}
