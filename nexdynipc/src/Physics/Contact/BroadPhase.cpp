#include "NexDynIPC/Physics/Contact/BroadPhase.h"

namespace NexDynIPC::Physics {

//==============================================================================
// BruteForceBroadPhase
//==============================================================================

std::vector<std::pair<int, int>> BruteForceBroadPhase::detect(
    const std::vector<Geometry::AABB>& aabbs)
{
    std::vector<std::pair<int, int>> pairs;
    pairs.reserve(aabbs.size() * aabbs.size() / 2);

    for (size_t i = 0; i < aabbs.size(); ++i) {
        for (size_t j = i + 1; j < aabbs.size(); ++j) {
            if (aabbs[i].intersects(aabbs[j])) {
                pairs.emplace_back(static_cast<int>(i), static_cast<int>(j));
            }
        }
    }
    return pairs;
}

//==============================================================================
// BVHBroadPhase
//==============================================================================

BVHBroadPhase::BVHBroadPhase(double inflationRadius)
    : inflationRadius_(inflationRadius)
{
}

std::vector<std::pair<int, int>> BVHBroadPhase::detect(
    const std::vector<Geometry::AABB>& aabbs)
{
    std::vector<std::pair<int, int>> result;

    if (aabbs.size() < 2) {
        return result;
    }

    bvh_.init(aabbs);

    for (size_t i = 0; i < aabbs.size(); ++i) {
        Geometry::AABB queryBox = aabbs[i];

        if (inflationRadius_ > 0) {
            queryBox = queryBox.inflated(inflationRadius_);
        }

        std::vector<unsigned int> overlapping;
        bvh_.intersectBox(queryBox, overlapping);

        for (unsigned int j : overlapping) {
            if (static_cast<size_t>(j) > i) {
                result.emplace_back(static_cast<int>(i), static_cast<int>(j));
            }
        }
    }

    return result;
}

std::vector<std::pair<int, int>> BVHBroadPhase::detectWithInflation(
    const std::vector<Geometry::AABB>& aabbs, double inflationRadius)
{
    inflationRadius_ = inflationRadius;
    return detect(aabbs);
}

void BVHBroadPhase::getStatistics(
    int& height, int& leafCount, int& internalCount) const
{
    height = bvh_.height();
    leafCount = bvh_.leafCount();
    internalCount = bvh_.internalCount();
}

//==============================================================================
// IncrementalBVHBroadPhase
//==============================================================================

std::vector<std::pair<int, int>> IncrementalBVHBroadPhase::detect(
    const std::vector<Geometry::AABB>& aabbs)
{
    std::vector<std::pair<int, int>> result;

    if (aabbs.size() < 2) {
        return result;
    }

    if (needsRebuild_ || cachedAABBs_.size() != aabbs.size()) {
        rebuildBVH(aabbs);
    } else {
        bool changed = false;
        for (size_t i = 0; i < aabbs.size(); ++i) {
            if ((aabbs[i].min() - cachedAABBs_[i].min()).norm() > 1e-10
                || (aabbs[i].max() - cachedAABBs_[i].max()).norm() > 1e-10) {
                changed = true;
                break;
            }
        }

        if (changed) {
            rebuildBVH(aabbs);
        }
    }

    for (size_t i = 0; i < aabbs.size(); ++i) {
        Geometry::AABB queryBox = aabbs[i];
        std::vector<unsigned int> overlapping;
        bvh_.intersectBox(queryBox, overlapping);

        for (unsigned int j : overlapping) {
            if (static_cast<size_t>(j) > i) {
                result.emplace_back(static_cast<int>(i), static_cast<int>(j));
            }
        }
    }

    return result;
}

void IncrementalBVHBroadPhase::rebuildBVH(
    const std::vector<Geometry::AABB>& aabbs)
{
    cachedAABBs_ = aabbs;
    bvh_.init(aabbs);
    needsRebuild_ = false;
}

} // namespace NexDynIPC::Physics
