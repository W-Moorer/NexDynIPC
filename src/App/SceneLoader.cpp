#include "NexDynIPC/App/SceneLoader.hpp"
#include "NexDynIPC/App/Simulation.hpp"
#include "NexDynIPC/Dynamics/RigidBody.hpp"
#include "NexDynIPC/Dynamics/Joints/RevoluteJoint.hpp"
#include "NexDynIPC/Dynamics/Joints/FixedJoint.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>

namespace NexDynIPC::App {

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

        // Parse Joints
        if (scene.contains("joints")) {
            for (const auto& j : scene["joints"]) {
                std::string type = j.value("type", "revolute");

                if (type == "revolute" || type == "hinge") {
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
                    auto joint = std::make_shared<Dynamics::RevoluteJoint>(
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
            }
        }
        return;
    }

    if (filename == "double_pendulum") {
        std::cout << "Loading Hardcoded Double Pendulum..." << std::endl;
        // ... (Hardcoded Fallback) ...
        // For brevity in edit, I'll keep the logic or just delegate to JSON if file exists?
        // Let's keep hardcoded logic below for now, but user should prefer JSON.
        
        // 1. Static Anchor
        auto anchor = std::make_shared<Dynamics::RigidBody>();
        anchor->name = "Anchor";
        anchor->is_static = true;
        anchor->position = Eigen::Vector3d(0, 0, 0); 
        anchor->id = 0; // Explicit ID assignment for clarity
        world.addBody(anchor);

        // 2. Link 1
        auto link1 = std::make_shared<Dynamics::RigidBody>();
        link1->name = "Link1";
        link1->is_static = false;
        link1->mass = 1.0;
        link1->position = Eigen::Vector3d(0.5, 0, 0);
        link1->inertia_body = Eigen::Matrix3d::Identity(); 
        link1->id = 1;
        world.addBody(link1);

        // 3. Link 2
        auto link2 = std::make_shared<Dynamics::RigidBody>();
        link2->name = "Link2";
        link2->is_static = false;
        link2->mass = 1.0;
        link2->position = Eigen::Vector3d(1.5, 0, 0);
        link2->inertia_body = Eigen::Matrix3d::Identity();
        link2->id = 2;
        world.addBody(link2);

        auto joint1 = std::make_shared<Dynamics::RevoluteJoint>(
            anchor->id, link1->id,
            Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(-0.5, 0, 0),
            Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1)
        );
        world.addJoint(joint1);

        auto joint2 = std::make_shared<Dynamics::RevoluteJoint>(
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
    ground->is_static = true;
    ground->position = Eigen::Vector3d(0, -1.0, 0); 
    ground->id = 0;
    world.addBody(ground);

    auto cube = std::make_shared<Dynamics::RigidBody>();
    cube->name = "Cube";
    cube->is_static = false;
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

        if (scene.contains("settings")) {
            auto& s = scene["settings"];
            if (s.contains("dt"))               config.dt = s["dt"].get<double>();
            if (s.contains("max_time"))          config.max_time = s["max_time"].get<double>();
            if (s.contains("newmark_gamma"))      config.newmark_gamma = s["newmark_gamma"].get<double>();
            if (s.contains("newmark_beta"))       config.newmark_beta = s["newmark_beta"].get<double>();
            if (s.contains("joint_stiffness"))    config.joint_stiffness = s["joint_stiffness"].get<double>();
            if (s.contains("integrator_type"))    config.integrator_type = s["integrator_type"].get<std::string>();
            if (s.contains("output_name"))        config.output_name = s["output_name"].get<std::string>();
            
            std::cout << "Applied settings from JSON:" << std::endl;
            std::cout << "  dt=" << config.dt 
                      << " max_time=" << config.max_time
                      << " gamma=" << config.newmark_gamma 
                      << " stiffness=" << config.joint_stiffness << std::endl;
        }
    }
}

} // namespace NexDynIPC::App
