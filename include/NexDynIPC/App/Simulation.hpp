#pragma once

#include "NexDynIPC/Dynamics/World.hpp"
#include "NexDynIPC/Dynamics/IPCSolver.hpp"
#include "NexDynIPC/App/StateExporter.hpp"
#include <string>
#include <memory> 

namespace NexDynIPC::App {

struct SimulationConfig {
    double dt = 0.01;
    double max_time = 1.0; 
    std::string output_dir = "output";
    std::string output_name = "simulation_results"; // Default name without extension
    std::string scene_file = "scene.json";
    std::string integrator_type = "ImplicitNewmark"; // "ImplicitEuler", "ImplicitNewmark"

    // Newmark Parameters
    double newmark_beta = 0.25;
    double newmark_gamma = 0.5;

    // Constraint Parameters
    double joint_stiffness = 1000.0;
};

class Simulation {
public:
    explicit Simulation(const SimulationConfig& config);

    void run();

private:
   SimulationConfig config_;
    Dynamics::World world_;
    std::unique_ptr<Dynamics::IPCSolver> solver_; // Make unique_ptr to concrete type to access setIntegrator, or cast it.
    std::unique_ptr<StateExporter> exporter_;
};

} // namespace NexDynIPC::App
