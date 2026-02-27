#pragma once

#include <Eigen/Core>
#include <limits>
#include <vector>

namespace NexDynIPC::Physics::Geometry {

/**
 * @brief 轴对齐包围盒 (Axis-Aligned Bounding Box)
 * 
 * 用于碰撞检测的加速结构基础组件。AABB 是一个与坐标轴对齐的长方体，
 * 用于快速判断两个物体是否可能发生碰撞。
 */
class AABB {
public:
    /**
     * @brief 默认构造函数，创建一个空的包围盒
     */
    AABB()
        : min_(Eigen::Vector3d::Constant(std::numeric_limits<double>::max()))
        , max_(Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest()))
    {
    }

    /**
     * @brief 从最小和最大点构造包围盒
     * @param min 最小点坐标
     * @param max 最大点坐标
     */
    AABB(const Eigen::Vector3d& min, const Eigen::Vector3d& max)
        : min_(min)
        , max_(max)
    {
    }

    /**
     * @brief 从单个点构造包围盒
     * @param point 点坐标
     */
    explicit AABB(const Eigen::Vector3d& point)
        : min_(point)
        , max_(point)
    {
    }

    /**
     * @brief 从两个包围盒构造合并包围盒
     * @param a 第一个包围盒
     * @param b 第二个包围盒
     */
    AABB(const AABB& a, const AABB& b)
        : min_(a.min_.cwiseMin(b.min_))
        , max_(a.max_.cwiseMax(b.max_))
    {
    }

    /**
     * @brief 从三个包围盒构造合并包围盒
     * @param a 第一个包围盒
     * @param b 第二个包围盒
     * @param c 第三个包围盒
     */
    AABB(const AABB& a, const AABB& b, const AABB& c)
        : min_(a.min_.cwiseMin(b.min_).cwiseMin(c.min_))
        , max_(a.max_.cwiseMax(b.max_).cwiseMax(c.max_))
    {
    }

    /**
     * @brief 获取最小点
     * @return 最小点坐标
     */
    const Eigen::Vector3d& min() const { return min_; }

    /**
     * @brief 获取最大点
     * @return 最大点坐标
     */
    const Eigen::Vector3d& max() const { return max_; }

    /**
     * @brief 获取包围盒中心
     * @return 中心点坐标
     */
    Eigen::Vector3d center() const { return (min_ + max_) * 0.5; }

    /**
     * @brief 获取包围盒尺寸（长宽高）
     * @return 尺寸向量
     */
    Eigen::Vector3d extent() const { return max_ - min_; }

    /**
     * @brief 获取包围盒体积
     * @return 体积值
     */
    double volume() const
    {
        Eigen::Vector3d e = extent();
        return e.x() * e.y() * e.z();
    }

    /**
     * @brief 获取包围盒表面积
     * @return 表面积值
     */
    double surfaceArea() const
    {
        Eigen::Vector3d e = extent();
        return 2.0 * (e.x() * e.y() + e.y() * e.z() + e.z() * e.x());
    }

    /**
     * @brief 扩展包围盒以包含指定点
     * @param point 要包含的点
     */
    void expand(const Eigen::Vector3d& point)
    {
        min_ = min_.cwiseMin(point);
        max_ = max_.cwiseMax(point);
    }

    /**
     * @brief 扩展包围盒以包含另一个包围盒
     * @param other 要包含的包围盒
     */
    void expand(const AABB& other)
    {
        min_ = min_.cwiseMin(other.min_);
        max_ = max_.cwiseMax(other.max_);
    }

    /**
     * @brief 返回与另一个包围盒的合并结果
     * @param other 另一个包围盒
     * @return 合并后的包围盒
     */
    AABB merged(const AABB& other) const
    {
        return AABB(min_.cwiseMin(other.min_), max_.cwiseMax(other.max_));
    }

    /**
     * @brief 检测是否与另一个包围盒相交
     * @param other 另一个包围盒
     * @return 是否相交
     */
    bool intersects(const AABB& other) const
    {
        return (min_.array() <= other.max_.array()).all()
            && (max_.array() >= other.min_.array()).all();
    }

    /**
     * @brief 检测两个包围盒是否重叠（静态方法）
     * @param a 第一个包围盒
     * @param b 第二个包围盒
     * @return 是否重叠
     */
    static bool areOverlapping(const AABB& a, const AABB& b)
    {
        return a.intersects(b);
    }

    /**
     * @brief 检测是否包含指定点
     * @param point 点坐标
     * @return 是否包含
     */
    bool contains(const Eigen::Vector3d& point) const
    {
        return (min_.array() <= point.array()).all()
            && (point.array() <= max_.array()).all();
    }

    /**
     * @brief 检测是否包含另一个包围盒
     * @param other 另一个包围盒
     * @return 是否完全包含
     */
    bool contains(const AABB& other) const
    {
        return (min_.array() <= other.min_.array()).all()
            && (other.max_.array() <= max_.array()).all();
    }

    /**
     * @brief 返回膨胀后的包围盒
     * @param margin 膨胀边距
     * @return 膨胀后的包围盒
     */
    AABB inflated(double margin) const
    {
        return AABB(min_.array() - margin, max_.array() + margin);
    }

    /**
     * @brief 检测包围盒是否为空
     * @return 是否为空
     */
    bool isEmpty() const
    {
        return min_.x() > max_.x() || min_.y() > max_.y() || min_.z() > max_.z();
    }

    /**
     * @brief 获取包围盒的对角线长度
     * @return 对角线长度
     */
    double diagonal() const { return extent().norm(); }

    /**
     * @brief 获取包围盒的最长边长度
     * @return 最长边长度
     */
    double maxExtent() const { return extent().maxCoeff(); }

    /**
     * @brief 获取最长边的轴索引 (0=x, 1=y, 2=z)
     * @return 轴索引
     */
    int maxExtentAxis() const
    {
        Eigen::Vector3d e = extent();
        if (e.x() >= e.y() && e.x() >= e.z())
            return 0;
        if (e.y() >= e.z())
            return 1;
        return 2;
    }

private:
    Eigen::Vector3d min_;
    Eigen::Vector3d max_;
};

/**
 * @brief 从顶点列表构建AABB
 * @param vertices 顶点矩阵 (N x 3)
 * @return 包围所有顶点的AABB
 */
AABB buildAABB(const Eigen::MatrixXd& vertices);

/**
 * @brief 从顶点列表构建带膨胀的AABB
 * @param vertices 顶点矩阵 (N x 3)
 * @param inflationRadius 膨胀半径
 * @return 膨胀后的AABB
 */
AABB buildAABB(const Eigen::MatrixXd& vertices, double inflationRadius);

/**
 * @brief 从顶点索引构建AABB
 * @param vertices 顶点矩阵 (N x 3)
 * @param indices 顶点索引列表
 * @return 包围指定顶点的AABB
 */
AABB buildAABB(const Eigen::MatrixXd& vertices, const std::vector<int>& indices);

} // namespace NexDynIPC::Physics::Geometry
