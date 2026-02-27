#pragma once

#include "NexDynIPC/Physics/Geometry/AABB.h"
#include <algorithm>
#include <array>
#include <stack>
#include <vector>

namespace NexDynIPC::Physics::Geometry {

/**
 * @brief 层次包围盒 (Bounding Volume Hierarchy)
 * 
 * 一种用于加速碰撞检测的树形数据结构。每个节点存储一个AABB，
 * 叶子节点存储实际的几何体引用，内部节点存储子节点的包围盒。
 * 
 * 构建策略：使用表面积启发式(SAH)进行最优分割
 * 查询复杂度：O(log n) 平均情况
 */
class BVH {
public:
    /**
     * @brief BVH节点结构
     */
    struct Node {
        AABB bbox;       ///< 节点的包围盒
        int left = -1;   ///< 左子节点索引（-1表示无）
        int right = -1;  ///< 右子节点索引（-1表示无）
        int parent = -1; ///< 父节点索引（-1表示无）
        int primitive = -1; ///< 图元索引（叶子节点有效，-1表示内部节点）

        /**
         * @brief 判断是否为叶子节点
         * @return 是否为叶子节点
         */
        bool isLeaf() const { return primitive >= 0; }

        /**
         * @brief 判断是否为内部节点
         * @return 是否为内部节点
         */
        bool isInternal() const { return primitive < 0 && left >= 0; }

        /**
         * @brief 判断是否为空节点
         * @return 是否为空节点
         */
        bool isEmpty() const { return primitive < 0 && left < 0; }
    };

    /**
     * @brief 默认构造函数
     */
    BVH() = default;

    /**
     * @brief 从AABB列表初始化BVH
     * @param aabbs AABB列表，每个AABB对应一个图元
     */
    void init(const std::vector<std::array<Eigen::Vector3d, 2>>& aabbs);

    /**
     * @brief 从AABB列表初始化BVH（使用AABB类）
     * @param aabbs AABB列表
     */
    void init(const std::vector<AABB>& aabbs);

    /**
     * @brief 清空BVH
     */
    void clear();

    /**
     * @brief 检测BVH是否为空
     * @return 是否为空
     */
    bool empty() const;

    /**
     * @brief 获取所有节点
     * @return 节点列表
     */
    const std::vector<Node>& nodes() const { return nodes_; }

    /**
     * @brief 获取根节点索引
     * @return 根节点索引
     */
    int root() const { return root_; }

    /**
     * @brief 查询与指定包围盒相交的所有图元
     * @param min 包围盒最小点
     * @param max 包围盒最大点
     * @param result 相交图元索引列表
     */
    void intersectBox(const Eigen::Vector3d& min, const Eigen::Vector3d& max,
        std::vector<unsigned int>& result) const;

    /**
     * @brief 查询与指定包围盒相交的所有图元
     * @param aabb 查询包围盒
     * @param result 相交图元索引列表
     */
    void intersectBox(const AABB& aabb, std::vector<unsigned int>& result) const;

    /**
     * @brief 查询与指定点相交的所有图元
     * @param point 查询点
     * @param result 相交图元索引列表
     */
    void intersectPoint(const Eigen::Vector3d& point,
        std::vector<unsigned int>& result) const;

    /**
     * @brief 查询两个BVH之间的所有重叠图元对
     * @param other 另一个BVH
     * @param result 重叠图元对列表 (图元A索引, 图元B索引)
     */
    void intersectBVH(const BVH& other,
        std::vector<std::pair<int, int>>& result) const;

    /**
     * @brief 获取BVH高度
     * @return 树高度
     */
    int height() const;

    /**
     * @brief 获取叶子节点数量
     * @return 叶子节点数量
     */
    int leafCount() const;

    /**
     * @brief 获取内部节点数量
     * @return 内部节点数量
     */
    int internalCount() const;

    /**
     * @brief 获取总结点数量
     * @return 总结点数量
     */
    int nodeCount() const;

private:
    std::vector<Node> nodes_; ///< 节点数组
    int root_ = -1;           ///< 根节点索引

    /**
     * @brief 递归构建BVH
     * @param primitives 图元索引列表
     * @param aabbs 图元AABB列表
     * @return 构建的节点索引
     */
    int buildRecursive(std::vector<int>& primitives, const std::vector<AABB>& aabbs);

    /**
     * @brief 计算SAH分割位置
     * @param primitives 图元索引列表
     * @param aabbs 图元AABB列表
     * @param axis 分割轴
     * @param splitPos 分割位置（输出）
     * @return 分割索引
     */
    int findSAHSplit(const std::vector<int>& primitives, const std::vector<AABB>& aabbs,
        int axis, double& splitPos);

    /**
     * @brief 计算子树高度
     * @param nodeId 节点索引
     * @return 子树高度
     */
    int heightRecursive(int nodeId) const;
};

} // namespace NexDynIPC::Physics::Geometry
