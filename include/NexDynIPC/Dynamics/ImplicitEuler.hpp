#pragma once

#include "NexDynIPC/Dynamics/TimeIntegrator.hpp"
#include "NexDynIPC/Math/NewtonSolver.hpp"

namespace NexDynIPC::Dynamics {

class ImplicitEuler : public TimeIntegrator {
public:
    ImplicitEuler();

    void step(World& world, double dt) override;

private:
    Math::NewtonSolver solver_;
};

} // namespace NexDynIPC::Dynamics
