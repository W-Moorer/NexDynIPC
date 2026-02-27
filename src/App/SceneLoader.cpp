#include "NexDynIPC/App/SceneLoader.h"
#include "NexDynIPC/App/Simulation.h"
#include "NexDynIPC/Dynamics/RigidBody.h"
#include "NexDynIPC/Dynamics/Joints/HingeJoint.h"
#include "NexDynIPC/Dynamics/Joints/FixedJoint.h"
#include "NexDynIPC/Dynamics/Joints/SphericalJoint.h"
#include "NexDynIPC/Dynamics/Joints/PrismaticJoint.h"
#include "NexDynIPC/Dynamics/Joints/CylindricalJoint.h"
#include "NexDynIPC/Dynamics/Joints/AngleLimitJoint.h"
#include "NexDynIPC/Dynamics/Joints/DistanceLimitJoint.h"
#include "NexDynIPC/Control/VelocityDriveForm.h"
#include "NexDynIPC/Control/LinearVelocityDriveForm.h"
#include "NexDynIPC/Control/PositionDriveForm.h"
#include "NexDynIPC/Control/ForceDriveForm.h"
#include "NexDynIPC/Control/DampedSpringForm.h"
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <unordered_set>
#include <numbers>

namespace NexDynIPC::App {

namespace {

std::string make_joint_signature(const nlohmann::json& entry) {
    if (!entry.contains("type")) {
        return "";
    }
    nlohmann::json normalized = entry;
    if (normalized.contains("type") && normalized["type"].is_string()) {
        const std::string raw_type = normalized["type"].get<std::string>();
        if (raw_type == "revolute") {
            normalized["type"] = "hinge";
        }
    }
    if (normalized.contains("stiffness")) {
        normalized.erase("stiffness");
    }
    return normalized.dump();
}

bool add_joint_from_entry(const nlohmann::json& j,
                          Dynamics::World& world,
                          std::unordered_set<std::string>& signatures) {
    if (!j.contains("type")) {
        return false;
    }

    const std::string signature = make_joint_signature(j);
    if (!signature.empty() && signatures.find(signature) != signatures.end()) {
        std::cout << "Skipping duplicated joint/constraint entry: " << j.value("type", "unknown") << std::endl;
        return false;
    }

    std::string type = j.value("type", "");
    if (type == "revolute") {
        static bool revolute_deprecated_warned = false;
        if (!revolute_deprecated_warned) {
            std::cout << "[SceneLoader] Deprecated joint type 'revolute' detected. "
                         "Please migrate to 'hinge'. Backward compatibility is currently enabled."
                      << std::endl;
            revolute_deprecated_warned = true;
        }
        type = "hinge";
    }

    auto find_body_by_id = [&](int body_id) -> std::shared_ptr<Dynamics::RigidBody> {
        for (const auto& body : world.bodies) {
            if (body->id == body_id) {
                return body;
            }
        }
        return nullptr;
    };

    if (type == "hinge") {
        auto ancA = j["anchor_a"];
        auto ancB = j["anchor_b"];
        auto axA = j["axis_a"];
        auto axB = j["axis_b"];

        Eigen::Vector3d anchorA(ancA[0], ancA[1], ancA[2]);
        Eigen::Vector3d anchorB(ancB[0], ancB[1], ancB[2]);
        Eigen::Vector3d axisA(axA[0], axA[1], axA[2]);
        Eigen::Vector3d axisB(axB[0], axB[1], axB[2]);

        int idA = j["body_a"];
        int idB = j["body_b"];
        auto joint = std::make_shared<Dynamics::HingeJoint>(
            idA, idB, anchorA, anchorB, axisA, axisB
        );
        if (j.contains("stiffness")) {
            joint->setStiffness(j["stiffness"].get<double>());
        }
        world.addJoint(joint);
    }
    else if (type == "spherical") {
        auto ancA = j["anchor_a"];
        auto ancB = j["anchor_b"];

        Eigen::Vector3d anchorA(ancA[0], ancA[1], ancA[2]);
        Eigen::Vector3d anchorB(ancB[0], ancB[1], ancB[2]);

        int idA = j["body_a"];
        int idB = j["body_b"];
        auto joint = std::make_shared<Dynamics::SphericalJoint>(
            idA, idB, anchorA, anchorB
        );
        if (j.contains("stiffness")) {
            joint->setStiffness(j["stiffness"].get<double>());
        }
        world.addJoint(joint);
    }
    else if (type == "prismatic") {
        auto ancA = j["anchor_a"];
        auto ancB = j["anchor_b"];
        auto axA = j["axis_a"];
        auto axB = j["axis_b"];

        Eigen::Vector3d anchorA(ancA[0], ancA[1], ancA[2]);
        Eigen::Vector3d anchorB(ancB[0], ancB[1], ancB[2]);
        Eigen::Vector3d axisA(axA[0], axA[1], axA[2]);
        Eigen::Vector3d axisB(axB[0], axB[1], axB[2]);

        int idA = j["body_a"];
        int idB = j["body_b"];
        auto joint = std::make_shared<Dynamics::PrismaticJoint>(
            idA, idB, anchorA, anchorB, axisA, axisB
        );
        if (j.contains("stiffness")) {
            joint->setStiffness(j["stiffness"].get<double>());
        }
        world.addJoint(joint);
    }
    else if (type == "cylindrical") {
        auto ancA = j["anchor_a"];
        auto ancB = j["anchor_b"];
        auto axA = j["axis_a"];
        auto axB = j["axis_b"];

        Eigen::Vector3d anchorA(ancA[0], ancA[1], ancA[2]);
        Eigen::Vector3d anchorB(ancB[0], ancB[1], ancB[2]);
        Eigen::Vector3d axisA(axA[0], axA[1], axA[2]);
        Eigen::Vector3d axisB(axB[0], axB[1], axB[2]);

        int idA = j["body_a"];
        int idB = j["body_b"];
        auto joint = std::make_shared<Dynamics::CylindricalJoint>(
            idA, idB, anchorA, anchorB, axisA, axisB
        );
        if (j.contains("stiffness")) {
            joint->setStiffness(j["stiffness"].get<double>());
        }
        world.addJoint(joint);
    }
    else if (type == "fixed") {
        int bodyId = j["body"];
        auto pos = j["position"];
        Eigen::Vector3d target_pos(pos[0], pos[1], pos[2]);

        Eigen::Quaterniond target_ori = Eigen::Quaterniond::Identity();
        if (j.contains("orientation")) {
            auto ori = j["orientation"];
            target_ori = Eigen::Quaterniond(ori[3], ori[0], ori[1], ori[2]); // w,x,y,z
        }

        auto joint = std::make_shared<Dynamics::FixedJoint>(bodyId, target_pos, target_ori);
        if (j.contains("stiffness")) {
            joint->setStiffness(j["stiffness"].get<double>());
        }
        world.addJoint(joint);
    }
    else if (type == "angular_velocity_drive") {
        int idA = j["body_a"];
        int idB = j["body_b"];
        auto ax = j["axis"];
        Eigen::Vector3d axis(ax[0], ax[1], ax[2]);

        const auto bodyA = find_body_by_id(idA);
        const auto bodyB = find_body_by_id(idB);

        if (!bodyB) {
            std::cout << "Ignoring angular_velocity_drive with invalid body_b id" << std::endl;
            return false;
        }

        const double target = j.value("target_velocity_radps", 0.0);
        const double kv = j.value("kv", j.value("stiffness", 100.0));
        const double max_torque = j.value("max_torque", j.value("max_torque_nm", 0.0));
        const double delay = j.value("delay", j.value("delay_radps", 0.0));
        const double delay_tau = j.value("delay_tau", j.value("delay_seconds", 0.0));

        auto drive = std::make_shared<NexDynIPC::Control::VelocityDriveForm>(
            bodyA, bodyB, axis, kv, target);
        drive->setMaxTorque(max_torque);
        drive->setDelay(delay);
        drive->setDelayTau(delay_tau);
        drive->updateGlobalIndices(world.bodies);
        world.addForm(drive);
    }
    else if (type == "linear_velocity_drive") {
        int idA = j["body_a"];
        int idB = j["body_b"];
        auto ax = j["axis"];
        Eigen::Vector3d axis(ax[0], ax[1], ax[2]);

        const auto bodyA = find_body_by_id(idA);
        const auto bodyB = find_body_by_id(idB);
        if (!bodyB) {
            std::cout << "Ignoring linear_velocity_drive with invalid body_b id" << std::endl;
            return false;
        }

        const double target = j.value("target_velocity_mps", j.value("target_velocity", 0.0));
        const double kv = j.value("kv", j.value("stiffness", 100.0));
        const double max_force = j.value("max_force", j.value("max_force_n", 0.0));
        const double delay = j.value("delay", j.value("delay_mps", 0.0));
        const double delay_tau = j.value("delay_tau", j.value("delay_seconds", 0.0));

        auto drive = std::make_shared<NexDynIPC::Control::LinearVelocityDriveForm>(
            bodyA, bodyB, axis, kv, target);
        drive->setMaxForce(max_force);
        drive->setDelay(delay);
        drive->setDelayTau(delay_tau);
        drive->updateGlobalIndices(world.bodies);
        world.addForm(drive);
    }
    else if (type == "position_drive") {
        int idA = j["body_a"];
        int idB = j["body_b"];
        auto ax = j["axis"];
        Eigen::Vector3d axis(ax[0], ax[1], ax[2]);

        const auto bodyA = find_body_by_id(idA);
        const auto bodyB = find_body_by_id(idB);
        if (!bodyB) {
            std::cout << "Ignoring position_drive with invalid body_b id" << std::endl;
            return false;
        }

        const double target = j.value("target_position_m", j.value("target_position", 0.0));
        const double kp = j.value("kp", j.value("stiffness", 100.0));
        const double max_force = j.value("max_force", j.value("max_force_n", 0.0));
        const double deadzone = j.value("deadzone", j.value("deadzone_m", 0.0));

        auto drive = std::make_shared<NexDynIPC::Control::PositionDriveForm>(
            bodyA, bodyB, axis, kp, target);
        drive->setMaxForce(max_force);
        drive->setDeadzone(deadzone);
        drive->updateGlobalIndices(world.bodies);
        world.addForm(drive);
    }
    else if (type == "force_drive") {
        int id = j.value("body", j.value("body_b", -1));
        const auto body = find_body_by_id(id);
        if (!body) {
            std::cout << "Ignoring force_drive with invalid body id" << std::endl;
            return false;
        }

        Eigen::Vector3d force = Eigen::Vector3d::Zero();
        if (j.contains("force")) {
            auto fv = j["force"];
            force = Eigen::Vector3d(fv[0], fv[1], fv[2]);
        } else if (j.contains("axis")) {
            auto ax = j["axis"];
            Eigen::Vector3d axis(ax[0], ax[1], ax[2]);
            const double force_n = j.value("force_n", j.value("force_magnitude", 0.0));
            force = axis.normalized() * force_n;
        }

        Eigen::Vector3d torque = Eigen::Vector3d::Zero();
        if (j.contains("torque")) {
            auto tv = j["torque"];
            torque = Eigen::Vector3d(tv[0], tv[1], tv[2]);
        }

        auto drive = std::make_shared<NexDynIPC::Control::ForceDriveForm>(body, force, torque);
        drive->updateGlobalIndices(world.bodies);
        world.addForm(drive);
    }
    else if (type == "angle_limit") {
        int idA = j["body_a"];
        int idB = j["body_b"];
        auto ax = j["axis"];
        Eigen::Vector3d axis(ax[0], ax[1], ax[2]);

        if (!find_body_by_id(idA) || !find_body_by_id(idB)) {
            std::cout << "Ignoring angle_limit with invalid body id" << std::endl;
            return false;
        }

        const double min_deg = j.value("min_angle_deg", -180.0);
        const double max_deg = j.value("max_angle_deg", 180.0);
        const double stiffness = j.value("stiffness", 1e5);

        auto limit_joint = std::make_shared<NexDynIPC::Dynamics::AngleLimitJoint>(
            idA,
            idB,
            axis,
            min_deg * std::numbers::pi / 180.0,
            max_deg * std::numbers::pi / 180.0);
        limit_joint->setStiffness(stiffness);
        world.addJoint(limit_joint);
    }
    else if (type == "distance_limit" || type == "distance_limits") {
        int idA = j.value("body_a", -1);
        int idB = j.value("body_b", -1);

        if (!find_body_by_id(idA) || !find_body_by_id(idB)) {
            std::cout << "Ignoring distance_limit with invalid body id" << std::endl;
            return false;
        }

        const double min_d = j.value("min_distance_m", j.value("min_translation_m", 0.0));
        const double max_d = j.value("max_distance_m", j.value("max_translation_m", min_d));
        const double stiffness = j.value("stiffness", 1e5);

        auto limit_joint = std::make_shared<NexDynIPC::Dynamics::DistanceLimitJoint>(
            idA,
            idB,
            min_d,
            max_d);
        limit_joint->setStiffness(stiffness);
        world.addJoint(limit_joint);
    }
    else if (type == "damped_spring") {
        int idA = j.value("body_a", -1);
        int idB = j.value("body_b", -1);

        const auto bodyA = find_body_by_id(idA);
        const auto bodyB = find_body_by_id(idB);
        if (!bodyA || !bodyB) {
            std::cout << "Ignoring damped_spring with invalid body id" << std::endl;
            return false;
        }

        const double rest = j.value("rest_length_m", j.value("rest_length", 0.0));
        const double k = j.value("stiffness_npm", j.value("stiffness", 0.0));
        const double c = j.value("damping_nspm", j.value("damping", 0.0));

        auto spring = std::make_shared<NexDynIPC::Control::DampedSpringForm>(bodyA, bodyB, rest, k, c);
        spring->updateGlobalIndices(world.bodies);
        world.addForm(spring);
    }
    else {
        std::cout << "Ignoring unsupported constraint type: " << type << std::endl;
        return false;
    }

    if (!signature.empty()) {
        signatures.insert(signature);
    }
    return true;
}

} // namespace

void SceneLoader::load(const std::string& filename, Dynamics::World& world) {
    if (filename.length() >= 5 && filename.substr(filename.length() - 5) == ".json") {
        std::cout << "Loading Scene from JSON: " << filename << std::endl;
        std::ifstream f(filename);
        if (!f.is_open()) {
            std::cerr << "Failed to open scene file: " << filename << std::endl;
            return;
        }
        nlohmann::json scene;
        f >> scene;

        // Parse Bodies
        if (scene.contains("bodies")) {
            for (const auto& b : scene["bodies"]) {
                auto body = std::make_shared<Dynamics::RigidBody>();
                body->id = b.value("id", -1);
                body->name = b.value("name", "Body");
                body->mass = b.value("mass", 1.0);
                
                if (b.contains("position")) {
                    auto p = b["position"];
                    body->position = Eigen::Vector3d(p[0], p[1], p[2]);
                }
                
                if (b.contains("inertia")) {
                    auto I = b["inertia"];
                    Eigen::Matrix3d mat;
                    for(int i=0; i<3; ++i) for(int j=0; j<3; ++j) mat(i,j) = I[i][j];
                    body->inertia_body = mat;
                } else {
                    body->inertia_body = Eigen::Matrix3d::Identity();
                }
                
                world.addBody(body);
            }
        }

        std::unordered_set<std::string> signatures;

        if (scene.contains("joints")) {
            for (const auto& j : scene["joints"]) {
                add_joint_from_entry(j, world, signatures);
            }
        }

        if (scene.contains("constraints")) {
            for (const auto& c : scene["constraints"]) {
                add_joint_from_entry(c, world, signatures);
            }
        }
        return;
    }

    if (filename == "double_pendulum") {
        std::cout << "Loading Hardcoded Double Pendulum..." << std::endl;
        
        // 1. Anchor (fixed in space via FixedJoint)
        auto anchor = std::make_shared<Dynamics::RigidBody>();
        anchor->name = "Anchor";
        anchor->position = Eigen::Vector3d(0, 0, 0); 
        anchor->id = 0;
        world.addBody(anchor);
        
        // Add FixedJoint to fix anchor in place
        auto anchor_fixed = std::make_shared<Dynamics::FixedJoint>(
            anchor->id, anchor->position, anchor->orientation);
        world.addJoint(anchor_fixed);

        // 2. Link 1
        auto link1 = std::make_shared<Dynamics::RigidBody>();
        link1->name = "Link1";
        link1->mass = 1.0;
        link1->position = Eigen::Vector3d(0.5, 0, 0);
        link1->inertia_body = Eigen::Matrix3d::Identity(); 
        link1->id = 1;
        world.addBody(link1);

        // 3. Link 2
        auto link2 = std::make_shared<Dynamics::RigidBody>();
        link2->name = "Link2";
        link2->mass = 1.0;
        link2->position = Eigen::Vector3d(1.5, 0, 0);
        link2->inertia_body = Eigen::Matrix3d::Identity();
        link2->id = 2;
        world.addBody(link2);

        auto joint1 = std::make_shared<Dynamics::HingeJoint>(
            anchor->id, link1->id,
            Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(-0.5, 0, 0),
            Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1)
        );
        world.addJoint(joint1);

        auto joint2 = std::make_shared<Dynamics::HingeJoint>(
            link1->id, link2->id,
            Eigen::Vector3d(0.5, 0, 0), Eigen::Vector3d(-0.5, 0, 0),
            Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1)
        );
        world.addJoint(joint2);
        
        return;
    }

    // Default Scene (Falling Cube)
    std::cout << "Loading Default Scene..." << std::endl;

    auto ground = std::make_shared<Dynamics::RigidBody>();
    ground->name = "Ground";
    ground->position = Eigen::Vector3d(0, -1.0, 0); 
    ground->id = 0;
    world.addBody(ground);
    
    // Fix ground in place via FixedJoint
    auto ground_fixed = std::make_shared<Dynamics::FixedJoint>(
        ground->id, ground->position, ground->orientation);
    world.addJoint(ground_fixed);

    auto cube = std::make_shared<Dynamics::RigidBody>();
    cube->name = "Cube";
    cube->mass = 1.0;
    cube->position = Eigen::Vector3d(0, 2.0, 0); 
    cube->id = 1;
    world.addBody(cube);
}

void SceneLoader::load(const std::string& filename, Dynamics::World& world, SimulationConfig& config) {
    // First, load bodies and joints as normal
    load(filename, world);

    // Then, parse "settings" block from JSON (if present)
    if (filename.length() >= 5 && filename.substr(filename.length() - 5) == ".json") {
        std::ifstream f(filename);
        if (!f.is_open()) return;
        nlohmann::json scene;
        f >> scene;

        const std::string model_name = scene.value("case_name", std::filesystem::path(filename).stem().string());

        if (scene.contains("settings")) {
            auto& s = scene["settings"];
            if (s.contains("dt"))               config.dt = s["dt"].get<double>();
            if (s.contains("max_time"))          config.max_time = s["max_time"].get<double>();
            if (s.contains("newmark_gamma"))      config.newmark_gamma = s["newmark_gamma"].get<double>();
            if (s.contains("newmark_beta"))       config.newmark_beta = s["newmark_beta"].get<double>();
            if (s.contains("joint_stiffness"))    config.joint_stiffness = s["joint_stiffness"].get<double>();
            if (s.contains("integrator_type"))    config.integrator_type = s["integrator_type"].get<std::string>();
            if (s.contains("output_name"))        config.output_name = s["output_name"].get<std::string>();
            if (s.contains("output_dir"))         config.output_dir = s["output_dir"].get<std::string>();
            if (s.contains("alm_max_iters"))      config.alm_max_iters = s["alm_max_iters"].get<int>();
            if (s.contains("alm_constraint_tolerance")) config.alm_constraint_tolerance = s["alm_constraint_tolerance"].get<double>();
            if (s.contains("alm_dual_tolerance")) config.alm_dual_tolerance = s["alm_dual_tolerance"].get<double>();
            if (s.contains("alm_hardening_trigger")) config.alm_hardening_trigger = s["alm_hardening_trigger"].get<double>();
            if (s.contains("alm_hardening_ratio")) config.alm_hardening_ratio = s["alm_hardening_ratio"].get<double>();
            if (s.contains("kpi_gate_enabled")) config.kpi_gate_enabled = s["kpi_gate_enabled"].get<bool>();
            if (s.contains("kpi_v_w_max")) config.kpi_v_w_max = s["kpi_v_w_max"].get<double>();
            if (s.contains("kpi_t_sat_max")) config.kpi_t_sat_max = s["kpi_t_sat_max"].get<double>();
            if (s.contains("kpi_r_dual_max")) config.kpi_r_dual_max = s["kpi_r_dual_max"].get<double>();
            
            std::cout << "Applied settings from JSON:" << std::endl;
            std::cout << "  dt=" << config.dt 
                      << " max_time=" << config.max_time
                      << " gamma=" << config.newmark_gamma 
                      << " stiffness=" << config.joint_stiffness << std::endl;
        }

        if (config.output_name == "simulation_results") {
            config.output_name = model_name;
        }
        if (config.output_dir == "output") {
            config.output_dir = "output/" + model_name;
        }
    }
}

} // namespace NexDynIPC::App
