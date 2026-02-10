#include "NexDynIPC/App/Simulation.hpp"
#include "NexDynIPC/App/SceneLoader.hpp"
#include <iostream>

namespace NexDynIPC::App {

Simulation::Simulation(const SimulationConfig& config) : config_(config) {
    integrator_ = std::make_unique<Dynamics::ImplicitEuler>();
    exporter_ = std::make_unique<StateExporter>(config_.output_dir);
}

void Simulation::run() {
    // 1. Load scene
    SceneLoader::load(config_.scene_file, world_);

    // 2. Loop
    double t = 0.0;
    int frame = 0;
    
    // Export initial state
    exporter_->exportFrame(world_, frame++, t);

    while (t < config_.max_time) {
        std::cout << "Simulating t=" << t << std::endl;
        
        // Step
        integrator_->step(world_, config_.dt);
        
        t += config_.dt;
        
        // Export
        exporter_->exportFrame(world_, frame++, t);
    }
    
    std::cout << "Simulation complete." << std::endl;
}

} // namespace NexDynIPC::App
