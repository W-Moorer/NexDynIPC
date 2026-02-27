#include "NexDynIPC/App/Simulation.h"
#include "NexDynIPC/App/SceneLoader.h"
#include "NexDynIPC/TimeIntegration/ImplicitTimeIntegrator.h"
#include <nlohmann/json.hpp>
#include <iostream>
#include <algorithm>
#include <stdexcept>

namespace NexDynIPC::App {

Simulation::Simulation(const SimulationConfig& config) : config_(config) {
}

void Simulation::run() {
    // 1. Load scene (and override config from JSON "settings" if present)
    SceneLoader::load(config_.scene_file, world_, config_);

    // 2. Apply Global Stiffness (only to joints that still have default stiffness)
    for (auto& joint : world_.joints) {
        if (joint->getStiffness() == 1e8) {
            joint->setStiffness(config_.joint_stiffness);
        }
    }

    // 3. Configure Integrator (using possibly-updated config from JSON)
    solver_ = std::make_unique<Dynamics::IPCSolver>();
    nlohmann::json integrator_config;
    integrator_config["type"] = config_.integrator_type;
    integrator_config["beta"] = config_.newmark_beta;
    integrator_config["gamma"] = config_.newmark_gamma;
    
    auto integrator = TimeIntegration::ImplicitTimeIntegrator::create(integrator_config);
    solver_->setIntegrator(integrator);
    solver_->setALMMaxIters(config_.alm_max_iters);
    solver_->setALMConstraintTolerance(config_.alm_constraint_tolerance);
    solver_->setALMDualTolerance(config_.alm_dual_tolerance);
    solver_->setALMHardeningTrigger(config_.alm_hardening_trigger);
    solver_->setALMHardeningRatio(config_.alm_hardening_ratio);

    // 4. Create Exporter
    exporter_ = std::make_unique<StateExporter>(config_.output_dir, config_.output_name);

    // 5. Simulation Loop
    double t = 0.0;
    int frame = 0;
    double max_v_w = 0.0;
    double max_t_sat = 0.0;
    double max_r_dual = 0.0;
    
    // Export initial state
    exporter_->exportFrame(world_, frame++, t);
    exporter_->exportKPIFrame(0, t, 0.0, 0.0, 0.0, 0.0);

    while (t < config_.max_time) {
        std::cout << "Simulating t=" << t << std::endl;
        
        solver_->step(world_, config_.dt);
        t += config_.dt;
        
        exporter_->exportFrame(world_, frame++, t);
        exporter_->exportKPIFrame(
            frame - 1,
            t,
            solver_->lastAngularVelocityError(),
            solver_->lastTorqueSaturationRatio(),
            solver_->lastDualResidual(),
            solver_->lastMaxConstraintViolation());

        max_v_w = std::max(max_v_w, solver_->lastAngularVelocityError());
        max_t_sat = std::max(max_t_sat, solver_->lastTorqueSaturationRatio());
        max_r_dual = std::max(max_r_dual, solver_->lastDualResidual());
    }

    if (config_.kpi_gate_enabled) {
        const bool pass_v_w = max_v_w <= config_.kpi_v_w_max;
        const bool pass_t_sat = max_t_sat <= config_.kpi_t_sat_max;
        const bool pass_r_dual = max_r_dual <= config_.kpi_r_dual_max;

        std::cout << "KPI Gate Summary:"
                  << " max(V_w)=" << max_v_w << " (<= " << config_.kpi_v_w_max << ")"
                  << " max(T_sat)=" << max_t_sat << " (<= " << config_.kpi_t_sat_max << ")"
                  << " max(R_dual)=" << max_r_dual << " (<= " << config_.kpi_r_dual_max << ")"
                  << std::endl;

        if (!(pass_v_w && pass_t_sat && pass_r_dual)) {
            throw std::runtime_error("KPI gate failed: thresholds exceeded");
        }
    }
    
    std::cout << "Simulation complete." << std::endl;
}

} // namespace NexDynIPC::App
