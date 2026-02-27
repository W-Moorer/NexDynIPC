#pragma once

#include "NexDynIPC/Dynamics/World.h"
#include <string>

namespace NexDynIPC::App {

class StateExporter {
public:
    explicit StateExporter(const std::string& output_dir, const std::string& output_name);

    // Export current frame
    void exportFrame(const Dynamics::World& world, int frame_idx, double time);

    // Export KPI frame
    void exportKPIFrame(int frame_idx,
                        double time,
                        double v_w,
                        double t_sat,
                        double r_dual,
                        double max_constraint_violation,
                        double ccd_toi_ratio,
                        int ccd_substeps,
                        int newton_fallbacks,
                        int solver_converged);

private:
    std::string output_dir_;
    std::string output_name_;
};

} // namespace NexDynIPC::App
