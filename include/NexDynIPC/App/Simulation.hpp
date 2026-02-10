#pragma once

#include "NexDynIPC/Dynamics/World.hpp"
#include "NexDynIPC/Dynamics/ImplicitEuler.hpp"
#include "NexDynIPC/App/StateExporter.hpp"
#include <string>
#include <memory> 

namespace NexDynIPC::App {

struct SimulationConfig {
    double dt = 0.01;
    double max_time = 1.0; 
    std::string output_dir = "output";
    std::string scene_file = "scene.json";
};

class Simulation {
public:
    explicit Simulation(const SimulationConfig& config);

    void run();

private:
    SimulationConfig config_;
    Dynamics::World world_;
    std::unique_ptr<Dynamics::TimeIntegrator> integrator_;
    std::unique_ptr<StateExporter> exporter_;
};

} // namespace NexDynIPC::App
