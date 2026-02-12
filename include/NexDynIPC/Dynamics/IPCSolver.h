#pragma once

#include "NexDynIPC/Dynamics/TimeIntegrator.h" // Keep base class if needed, or remove? ImplicitEuler inherited from TimeIntegrator?
// Wait, TimeIntegrator (Dynamics) is different from ImplicitTimeIntegrator (TimeIntegration).
// Dynamics/TimeIntegrator was an abstract base? Let's check below.
#include "NexDynIPC/Math/NewtonSolver.h"
#include "NexDynIPC/TimeIntegration/ImplicitTimeIntegrator.h"
#include <memory>

namespace NexDynIPC::Dynamics {

// Renaming ImplicitEuler to IPCSolver
class IPCSolver : public TimeIntegrator {
public:
    IPCSolver();

    void setIntegrator(std::shared_ptr<TimeIntegration::ImplicitTimeIntegrator> integrator);

    // Step now takes dt from integrator (or passed in, but should match)
    void step(World& world, double dt) override;

private:
    Math::NewtonSolver solver_;
    std::shared_ptr<TimeIntegration::ImplicitTimeIntegrator> integrator_;
};

} // namespace NexDynIPC::Dynamics
