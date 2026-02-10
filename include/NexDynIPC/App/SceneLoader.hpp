#pragma once

#include "NexDynIPC/Dynamics/World.hpp"
#include <string>

namespace NexDynIPC::App {

class SceneLoader {
public:
    // Load scene from file and populate world
    static void load(const std::string& filename, Dynamics::World& world);
};

} // namespace NexDynIPC::App
