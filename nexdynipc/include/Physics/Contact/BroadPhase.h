#pragma once

#include "NexDynIPC/Physics/Geometry/AABB.h"
#include "NexDynIPC/Physics/Geometry/BVH.h"
#include <vector>
#include <utility>

namespace NexDynIPC::Physics {

/**
 * @brief 宽相位碰撞检测接口
 * 
 * 宽相位用于快速筛选可能发生碰撞的物体对，
 * 避免对所有物体对进行精确的碰撞检测。
 */
class BroadPhase {
public:
    virtual ~BroadPhase() = default;

    /**
     * @brief 检测潜在的碰撞对
     * @param aabbs 包围盒列表（每个包围盒关联一个刚体ID）
     * @return 潜在碰撞对列表 (id1, id2)
     */
    virtual std::vector<std::pair<int, int>> detect(
        const std::vector<Geometry::AABB>& aabbs) = 0;
};

/**
 * @brief 简单暴力宽相位
 * 
 * 对所有物体对进行两两检测，复杂度 O(n²)。
 * 适用于小规模场景或作为基准测试。
 */
class BruteForceBroadPhase : public BroadPhase {
public:
    /**
     * @brief 检测潜在的碰撞对
     * @param aabbs 包围盒列表
     * @return 潜在碰撞对列表
     */
    std::vector<std::pair<int, int>> detect(
        const std::vector<Geometry::AABB>& aabbs) override;
};

/**
 * @brief 基于BVH的宽相位碰撞检测
 * 
 * 使用层次包围盒(BVH)加速碰撞检测。相比暴力方法，
 * BVH可以将检测复杂度从O(n²)降低到O(n log n)。
 * 
 * 性能特点：
 * - 构建复杂度：O(n log n)
 * - 查询复杂度：O(k + log n)，其中k为重叠对数量
 * - 适用于大规模刚体场景
 */
class BVHBroadPhase : public BroadPhase {
public:
    /**
     * @brief 默认构造函数
     */
    BVHBroadPhase() = default;

    /**
     * @brief 使用指定的膨胀半径构造
     * @param inflationRadius 膨胀半径，用于CCD安全边界
     */
    explicit BVHBroadPhase(double inflationRadius);

    /**
     * @brief 检测潜在的碰撞对
     * @param aabbs 包围盒列表
     * @return 潜在碰撞对列表 (id1, id2)
     */
    std::vector<std::pair<int, int>> detect(
        const std::vector<Geometry::AABB>& aabbs) override;

    /**
     * @brief 使用膨胀半径检测碰撞对
     * @param aabbs 包围盒列表
     * @param inflationRadius 膨胀半径
     * @return 潜在碰撞对列表
     */
    std::vector<std::pair<int, int>> detectWithInflation(
        const std::vector<Geometry::AABB>& aabbs, double inflationRadius);

    /**
     * @brief 设置膨胀半径
     * @param radius 膨胀半径
     */
    void setInflationRadius(double radius) { inflationRadius_ = radius; }

    /**
     * @brief 获取膨胀半径
     * @return 膨胀半径
     */
    double getInflationRadius() const { return inflationRadius_; }

    /**
     * @brief 获取内部BVH结构
     * @return BVH引用
     */
    const Geometry::BVH& getBVH() const { return bvh_; }

    /**
     * @brief 获取BVH统计信息
     * @param height 树高度
     * @param leafCount 叶子节点数
     * @param internalCount 内部节点数
     */
    void getStatistics(int& height, int& leafCount, int& internalCount) const;

private:
    Geometry::BVH bvh_;
    double inflationRadius_ = 0.0;
};

/**
 * @brief 增量式BVH宽相位
 * 
 * 支持动态更新刚体位置后的BVH增量更新，
 * 避免每帧完全重建BVH。
 */
class IncrementalBVHBroadPhase : public BroadPhase {
public:
    /**
     * @brief 默认构造函数
     */
    IncrementalBVHBroadPhase() = default;

    /**
     * @brief 检测潜在的碰撞对
     * @param aabbs 包围盒列表
     * @return 潜在碰撞对列表
     */
    std::vector<std::pair<int, int>> detect(
        const std::vector<Geometry::AABB>& aabbs) override;

    /**
     * @brief 强制重建BVH
     * @param aabbs 包围盒列表
     */
    void rebuildBVH(const std::vector<Geometry::AABB>& aabbs);

    /**
     * @brief 标记需要重建
     */
    void markNeedsRebuild() { needsRebuild_ = true; }

    /**
     * @brief 获取内部BVH结构
     * @return BVH引用
     */
    const Geometry::BVH& getBVH() const { return bvh_; }

private:
    Geometry::BVH bvh_;
    std::vector<Geometry::AABB> cachedAABBs_;
    bool needsRebuild_ = true;
};

/**
 * @brief 带ID的包围盒结构
 * 
 * 用于宽相位检测，包含包围盒和关联的刚体ID。
 */
struct BodyAABB {
    Geometry::AABB aabb;
    int bodyId;

    BodyAABB() = default;

    BodyAABB(const Eigen::Vector3d& min, const Eigen::Vector3d& max, int id)
        : aabb(min, max)
        , bodyId(id)
    {
    }

    BodyAABB(const Geometry::AABB& bbox, int id)
        : aabb(bbox)
        , bodyId(id)
    {
    }
};

} // namespace NexDynIPC::Physics
