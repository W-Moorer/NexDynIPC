#pragma once

#include "NexDynIPC/Physics/Contact/Distance.h"
#include "NexDynIPC/Dynamics/RigidBody.h"
#include <vector>
#include <memory>

namespace NexDynIPC::Physics::Contact {

class CollisionCandidatesBuilder {
public:
    std::vector<CollisionCandidate> build(
        const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies,
        const std::vector<std::pair<int, int>>& broadphase_pairs) const;
};

} // namespace NexDynIPC::Physics::Contact
