#pragma once

#include "NexDynIPC/Dynamics/World.hpp"
#include <string>

namespace NexDynIPC::App {

class StateExporter {
public:
    explicit StateExporter(const std::string& output_dir, const std::string& output_name);

    // Export current frame
    void exportFrame(const Dynamics::World& world, int frame_idx, double time);

private:
    std::string output_dir_;
    std::string output_name_;
};

} // namespace NexDynIPC::App
