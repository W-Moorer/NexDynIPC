#include "NexDynIPC/Physics/Contact/CollisionCandidates.h"
#include "NexDynIPC/Physics/Geometry/MeshShape.h"

namespace NexDynIPC::Physics::Contact {

std::vector<CollisionCandidate> CollisionCandidatesBuilder::build(
    const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies,
    const std::vector<std::pair<int, int>>& broadphase_pairs) const
{
    std::vector<CollisionCandidate> candidates;

    for (const auto& pair : broadphase_pairs) {
        const int body_a_idx = pair.first;
        const int body_b_idx = pair.second;

        if (body_a_idx < 0 || body_b_idx < 0
            || body_a_idx >= static_cast<int>(bodies.size())
            || body_b_idx >= static_cast<int>(bodies.size())) {
            continue;
        }

        const auto meshA = std::dynamic_pointer_cast<Physics::MeshShape>(bodies[static_cast<size_t>(body_a_idx)]->shape);
        const auto meshB = std::dynamic_pointer_cast<Physics::MeshShape>(bodies[static_cast<size_t>(body_b_idx)]->shape);
        if (!meshA || !meshB) {
            continue;
        }

        for (int vi = 0; vi < meshA->vertices().rows(); ++vi) {
            for (int fi = 0; fi < meshB->faces().rows(); ++fi) {
                candidates.push_back(CollisionCandidate{
                    body_a_idx,
                    body_b_idx,
                    ContactPrimitiveType::VertexFace,
                    vi,
                    fi
                });
            }
        }

        for (int vi = 0; vi < meshB->vertices().rows(); ++vi) {
            for (int fi = 0; fi < meshA->faces().rows(); ++fi) {
                candidates.push_back(CollisionCandidate{
                    body_b_idx,
                    body_a_idx,
                    ContactPrimitiveType::VertexFace,
                    vi,
                    fi
                });
            }
        }
    }

    return candidates;
}

} // namespace NexDynIPC::Physics::Contact
