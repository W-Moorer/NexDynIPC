#pragma once

#include "NexDynIPC/Dynamics/World.hpp"

namespace NexDynIPC::Dynamics {

class TimeIntegrator {
public:
    virtual ~TimeIntegrator() = default;

    // Advance the world state by dt
    virtual void step(World& world, double dt) = 0;
};

} // namespace NexDynIPC::Dynamics
