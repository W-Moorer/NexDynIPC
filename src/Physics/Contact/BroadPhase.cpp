#include "NexDynIPC/Physics/Contact/BroadPhase.hpp"

namespace NexDynIPC::Physics {

std::vector<std::pair<int, int>> BruteForceBroadPhase::detect(const std::vector<AABB>& aabbs) {
    std::vector<std::pair<int, int>> pairs;
    pairs.reserve(aabbs.size() * aabbs.size() / 2);

    for (size_t i = 0; i < aabbs.size(); ++i) {
        for (size_t j = i + 1; j < aabbs.size(); ++j) {
            if (aabbs[i].intersects(aabbs[j])) {
                pairs.emplace_back(aabbs[i].id, aabbs[j].id);
            }
        }
    }
    return pairs;
}

} // namespace NexDynIPC::Physics
