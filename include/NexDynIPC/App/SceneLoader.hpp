#pragma once

#include "NexDynIPC/Dynamics/World.hpp"
#include <string>

namespace NexDynIPC::App {

struct SimulationConfig; // Forward declaration

class SceneLoader {
public:
    // Load scene from file and populate world
    static void load(const std::string& filename, Dynamics::World& world);

    // Load scene and also override SimulationConfig from JSON "settings" block
    static void load(const std::string& filename, Dynamics::World& world, SimulationConfig& config);
};

} // namespace NexDynIPC::App
