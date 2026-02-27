#pragma once

#include "NexDynIPC/Dynamics/World.h"
#include "NexDynIPC/Dynamics/IPCSolver.h"
#include "NexDynIPC/App/StateExporter.h"
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

    // ALM Parameters
    int alm_max_iters = 10;
    double alm_constraint_tolerance = 1e-4;
    double alm_dual_tolerance = 1e-3;
    double alm_hardening_trigger = 2.0;
    double alm_hardening_ratio = 2.0;

    // KPI gate controls (disabled by default for backward compatibility)
    bool kpi_gate_enabled = false;
    double kpi_v_w_max = 0.2;
    double kpi_t_sat_max = 0.01;
    double kpi_r_dual_max = 1e-3;
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
