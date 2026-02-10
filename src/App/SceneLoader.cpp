#include "NexDynIPC/App/SceneLoader.hpp"
#include "NexDynIPC/Dynamics/RigidBody.hpp"
#include "NexDynIPC/Dynamics/Joints/HingeJoint.hpp"
#include <iostream>

namespace NexDynIPC::App {

void SceneLoader::load(const std::string& filename, Dynamics::World& world) {
    if (filename == "double_pendulum") {
        std::cout << "Loading Double Pendulum Scene..." << std::endl;

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
        link1->position = Eigen::Vector3d(1.0, 0, 0); // Horizontal start
        link1->inertia_body = Eigen::Matrix3d::Identity(); // Simple inertia
        link1->id = 1;
        world.addBody(link1);

        // 3. Link 2
        auto link2 = std::make_shared<Dynamics::RigidBody>();
        link2->name = "Link2";
        link2->is_static = false;
        link2->mass = 1.0;
        link2->position = Eigen::Vector3d(2.0, 0, 0); // Horizontal start
        link2->inertia_body = Eigen::Matrix3d::Identity();
        link2->id = 2;
        world.addBody(link2);

        // Joint 1: Anchor <-> Link 1
        // Pivot at (0,0,0) (World)
        // Anchor Local: (0,0,0)
        // Link 1 Local: (-1,0,0)
        auto joint1 = std::make_shared<Dynamics::HingeJoint>(
            anchor->id, link1->id,
            Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(-1.0, 0, 0),
            Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1) // Z-axis hinge
        );
        world.addJoint(joint1);

        // Joint 2: Link 1 <-> Link 2
        // Pivot at (2,0,0) (World - Wait, Link 1 ends at 2? No, length 2 centered at 1?)
        // Let's assume point masses or rods of length 1 for simplicity.
        // Link 1 pos (1,0,0). Length 2? End points (0,0,0) and (2,0,0).
        // Link 2 pos (3,0,0). Wait, let's chain them.
        
        // Revised Geometry:
        // Anchor at (0,0,0).
        // Link 1 (Length 1) from (0,0,0) to (1,0,0). Center (0.5, 0, 0).
        // Link 2 (Length 1) from (1,0,0) to (2,0,0). Center (1.5, 0, 0).
        
        // Update Link 1
        link1->position = Eigen::Vector3d(0.5, 0, 0);
        // Update Link 2
        link2->position = Eigen::Vector3d(1.5, 0, 0);

        // Joint 1: Anchor (0,0,0) <-> Link 1 (0.5,0,0)
        // Anchor Local (0,0,0). Link 1 Local (-0.5, 0, 0).
        // Actual Joint 1 re-def:
        joint1 = std::make_shared<Dynamics::HingeJoint>(
            anchor->id, link1->id,
            Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(-0.5, 0, 0),
            Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1)
        );
        // Re-add? No, replace.
        world.joints.clear();
        world.addJoint(joint1);

        // Joint 2: Link 1 (0.5,0,0) <-> Link 2 (1.5,0,0)
        // Connection Point: (1.0, 0, 0)
        // Link 1 Local: (0.5, 0, 0)
        // Link 2 Local: (-0.5, 0, 0)
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

} // namespace NexDynIPC::App
