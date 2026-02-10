#pragma once

#include <Eigen/Core>
#include <vector>
#include <utility>

namespace NexDynIPC::Physics {

struct AABB {
    Eigen::Vector3d min;
    Eigen::Vector3d max;
    int id; // Associated Body ID

    bool intersects(const AABB& other) const {
        return (min.array() <= other.max.array()).all() &&
               (max.array() >= other.min.array()).all();
    }
};

class BroadPhase {
public:
    virtual ~BroadPhase() = default;

    // Detect potential collision pairs
    // Returns a list of pairs of IDs (id1, id2)
    virtual std::vector<std::pair<int, int>> detect(const std::vector<AABB>& aabbs) = 0;
};

// Simple Brute Force Broad Phase
class BruteForceBroadPhase : public BroadPhase {
public:
    std::vector<std::pair<int, int>> detect(const std::vector<AABB>& aabbs) override;
};

} // namespace NexDynIPC::Physics
