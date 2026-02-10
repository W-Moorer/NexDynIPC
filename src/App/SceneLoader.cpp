#include "NexDynIPC/App/SceneLoader.hpp"
#include "NexDynIPC/Dynamics/RigidBody.hpp"
#include <iostream>

namespace NexDynIPC::App {

void SceneLoader::load(const std::string& filename, Dynamics::World& world) {
    // Placeholder: Hardcode a simple scene for now
    // 1. Static ground plane (box)
    // 2. Dynamic falling cube
    
    std::cout << "Loading scene: " << filename << " (Hardcoded for now)" << std::endl;

    auto ground = std::make_shared<Dynamics::RigidBody>();
    ground->name = "Ground";
    ground->is_static = true;
    ground->position = Eigen::Vector3d(0, -1.0, 0); 
    // TODO: Assign box shape
    world.addBody(ground);

    auto cube = std::make_shared<Dynamics::RigidBody>();
    cube->name = "Cube";
    cube->is_static = false;
    cube->mass = 1.0;
    cube->position = Eigen::Vector3d(0, 2.0, 0); // Start high
    // TODO: Assign box shape
    world.addBody(cube);
}

} // namespace NexDynIPC::App
