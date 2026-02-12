#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "NexDynIPC/Physics/Geometry/AABB.h"

using namespace NexDynIPC::Physics::Geometry;
using Catch::Matchers::WithinRel;

/**
 * @brief AABB包围盒单元测试
 * 
 * 测试覆盖：
 * - 构造函数（默认、点、两点、合并）
 * - 基本属性（中心、尺寸、体积、表面积）
 * - 包围盒操作（扩展、合并、相交检测）
 * - 工具函数（从顶点构建）
 */
TEST_CASE("AABB default construction", "[geometry][aabb]")
{
    AABB aabb;

    SECTION("Empty AABB has infinite min and max")
    {
        REQUIRE(aabb.min().x() >= std::numeric_limits<double>::max() / 2);
        REQUIRE(aabb.max().x() <= std::numeric_limits<double>::lowest() / 2);
        REQUIRE(aabb.isEmpty());
    }
}

TEST_CASE("AABB construction from point", "[geometry][aabb]")
{
    Eigen::Vector3d point(1.0, 2.0, 3.0);
    AABB aabb(point);

    SECTION("Point AABB has same min and max")
    {
        REQUIRE(aabb.min().isApprox(point));
        REQUIRE(aabb.max().isApprox(point));
        REQUIRE(aabb.center().isApprox(point));
        REQUIRE(aabb.extent().isApprox(Eigen::Vector3d::Zero()));
    }
}

TEST_CASE("AABB construction from min and max", "[geometry][aabb]")
{
    Eigen::Vector3d min(0.0, 0.0, 0.0);
    Eigen::Vector3d max(2.0, 3.0, 4.0);
    AABB aabb(min, max);

    SECTION("Basic properties")
    {
        REQUIRE(aabb.min().isApprox(min));
        REQUIRE(aabb.max().isApprox(max));
        REQUIRE(aabb.center().isApprox(Eigen::Vector3d(1.0, 1.5, 2.0)));
        REQUIRE(aabb.extent().isApprox(Eigen::Vector3d(2.0, 3.0, 4.0)));
    }

    SECTION("Volume and surface area")
    {
        REQUIRE_THAT(aabb.volume(), WithinRel(24.0, 1e-10));
        REQUIRE_THAT(aabb.surfaceArea(), WithinRel(52.0, 1e-10));
    }

    SECTION("Diagonal and max extent")
    {
        REQUIRE_THAT(aabb.diagonal(), WithinRel(std::sqrt(29.0), 1e-10));
        REQUIRE_THAT(aabb.maxExtent(), WithinRel(4.0, 1e-10));
        REQUIRE(aabb.maxExtentAxis() == 2);
    }
}

TEST_CASE("AABB construction from two AABBs", "[geometry][aabb]")
{
    AABB a(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
    AABB b(Eigen::Vector3d(0.5, 0.5, 0.5), Eigen::Vector3d(2, 2, 2));
    AABB merged(a, b);

    SECTION("Merged AABB contains both")
    {
        REQUIRE(merged.min().isApprox(Eigen::Vector3d(0, 0, 0)));
        REQUIRE(merged.max().isApprox(Eigen::Vector3d(2, 2, 2)));
    }
}

TEST_CASE("AABB construction from three AABBs", "[geometry][aabb]")
{
    AABB a(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
    AABB b(Eigen::Vector3d(2, 0, 0), Eigen::Vector3d(3, 1, 1));
    AABB c(Eigen::Vector3d(0, 2, 0), Eigen::Vector3d(1, 3, 1));
    AABB merged(a, b, c);

    SECTION("Merged AABB contains all three")
    {
        REQUIRE(merged.min().isApprox(Eigen::Vector3d(0, 0, 0)));
        REQUIRE(merged.max().isApprox(Eigen::Vector3d(3, 3, 1)));
    }
}

TEST_CASE("AABB expand operations", "[geometry][aabb]")
{
    AABB aabb;

    SECTION("Expand by point")
    {
        aabb.expand(Eigen::Vector3d(1, 2, 3));
        REQUIRE(aabb.min().isApprox(Eigen::Vector3d(1, 2, 3)));
        REQUIRE(aabb.max().isApprox(Eigen::Vector3d(1, 2, 3)));

        aabb.expand(Eigen::Vector3d(0, 3, 2));
        REQUIRE(aabb.min().isApprox(Eigen::Vector3d(0, 2, 2)));
        REQUIRE(aabb.max().isApprox(Eigen::Vector3d(1, 3, 3)));
    }

    SECTION("Expand by another AABB")
    {
        aabb.expand(AABB(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)));
        aabb.expand(AABB(Eigen::Vector3d(0.5, 0.5, 0.5), Eigen::Vector3d(2, 2, 2)));

        REQUIRE(aabb.min().isApprox(Eigen::Vector3d(0, 0, 0)));
        REQUIRE(aabb.max().isApprox(Eigen::Vector3d(2, 2, 2)));
    }
}

TEST_CASE("AABB intersection test", "[geometry][aabb]")
{
    AABB a(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));

    SECTION("Overlapping AABBs")
    {
        AABB b(Eigen::Vector3d(0.5, 0.5, 0.5), Eigen::Vector3d(1.5, 1.5, 1.5));
        REQUIRE(a.intersects(b));
        REQUIRE(b.intersects(a));
        REQUIRE(AABB::areOverlapping(a, b));
    }

    SECTION("Adjacent AABBs (touching)")
    {
        AABB b(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(2, 1, 1));
        REQUIRE(a.intersects(b));
    }

    SECTION("Non-overlapping AABBs")
    {
        AABB b(Eigen::Vector3d(2, 2, 2), Eigen::Vector3d(3, 3, 3));
        REQUIRE_FALSE(a.intersects(b));
        REQUIRE_FALSE(AABB::areOverlapping(a, b));
    }

    SECTION("One AABB contains another")
    {
        AABB b(Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.8, 0.8, 0.8));
        REQUIRE(a.intersects(b));
        REQUIRE(a.contains(b));
        REQUIRE_FALSE(b.contains(a));
    }
}

TEST_CASE("AABB contains test", "[geometry][aabb]")
{
    AABB aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2));

    SECTION("Contains point inside")
    {
        REQUIRE(aabb.contains(Eigen::Vector3d(1, 1, 1)));
        REQUIRE(aabb.contains(Eigen::Vector3d(0, 0, 0)));
        REQUIRE(aabb.contains(Eigen::Vector3d(2, 2, 2)));
    }

    SECTION("Does not contain point outside")
    {
        REQUIRE_FALSE(aabb.contains(Eigen::Vector3d(3, 1, 1)));
        REQUIRE_FALSE(aabb.contains(Eigen::Vector3d(-1, 0, 0)));
    }

    SECTION("Contains another AABB")
    {
        AABB inner(Eigen::Vector3d(0.5, 0.5, 0.5), Eigen::Vector3d(1.5, 1.5, 1.5));
        REQUIRE(aabb.contains(inner));

        AABB outer(Eigen::Vector3d(-1, 0, 0), Eigen::Vector3d(3, 2, 2));
        REQUIRE_FALSE(aabb.contains(outer));
    }
}

TEST_CASE("AABB inflation", "[geometry][aabb]")
{
    AABB aabb(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3));
    AABB inflated = aabb.inflated(0.5);

    SECTION("Inflated AABB has expanded bounds")
    {
        REQUIRE(inflated.min().isApprox(Eigen::Vector3d(0.5, 0.5, 0.5)));
        REQUIRE(inflated.max().isApprox(Eigen::Vector3d(3.5, 3.5, 3.5)));
    }

    SECTION("Original AABB unchanged")
    {
        REQUIRE(aabb.min().isApprox(Eigen::Vector3d(1, 1, 1)));
        REQUIRE(aabb.max().isApprox(Eigen::Vector3d(3, 3, 3)));
    }
}

TEST_CASE("AABB merged operation", "[geometry][aabb]")
{
    AABB a(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
    AABB b(Eigen::Vector3d(2, 2, 2), Eigen::Vector3d(3, 3, 3));
    AABB merged = a.merged(b);

    SECTION("Merged AABB contains both original")
    {
        REQUIRE(merged.min().isApprox(Eigen::Vector3d(0, 0, 0)));
        REQUIRE(merged.max().isApprox(Eigen::Vector3d(3, 3, 3)));
        REQUIRE(merged.contains(a));
        REQUIRE(merged.contains(b));
    }

    SECTION("Original AABBs unchanged")
    {
        REQUIRE(a.min().isApprox(Eigen::Vector3d(0, 0, 0)));
        REQUIRE(b.min().isApprox(Eigen::Vector3d(2, 2, 2)));
    }
}

TEST_CASE("buildAABB from vertices", "[geometry][aabb]")
{
    Eigen::MatrixXd vertices(4, 3);
    vertices << 0, 0, 0,
        1, 0, 0,
        0, 1, 0,
        0, 0, 1;

    AABB aabb = buildAABB(vertices);

    SECTION("AABB contains all vertices")
    {
        REQUIRE(aabb.min().isApprox(Eigen::Vector3d(0, 0, 0)));
        REQUIRE(aabb.max().isApprox(Eigen::Vector3d(1, 1, 1)));
    }
}

TEST_CASE("buildAABB with inflation", "[geometry][aabb]")
{
    Eigen::MatrixXd vertices(2, 3);
    vertices << 0, 0, 0,
        1, 1, 1;

    AABB aabb = buildAABB(vertices, 0.5);

    SECTION("Inflated AABB has expanded bounds")
    {
        REQUIRE(aabb.min().isApprox(Eigen::Vector3d(-0.5, -0.5, -0.5)));
        REQUIRE(aabb.max().isApprox(Eigen::Vector3d(1.5, 1.5, 1.5)));
    }
}

TEST_CASE("buildAABB from indexed vertices", "[geometry][aabb]")
{
    Eigen::MatrixXd vertices(6, 3);
    vertices << 0, 0, 0,
        1, 0, 0,
        0, 1, 0,
        0, 0, 1,
        10, 10, 10,
        -10, -10, -10;

    std::vector<int> indices = { 0, 1, 2, 3 };
    AABB aabb = buildAABB(vertices, indices);

    SECTION("AABB contains only indexed vertices")
    {
        REQUIRE(aabb.min().isApprox(Eigen::Vector3d(0, 0, 0)));
        REQUIRE(aabb.max().isApprox(Eigen::Vector3d(1, 1, 1)));
    }
}

TEST_CASE("AABB max extent axis", "[geometry][aabb]")
{
    SECTION("X axis is longest")
    {
        AABB aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(5, 1, 1));
        REQUIRE(aabb.maxExtentAxis() == 0);
    }

    SECTION("Y axis is longest")
    {
        AABB aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 5, 1));
        REQUIRE(aabb.maxExtentAxis() == 1);
    }

    SECTION("Z axis is longest")
    {
        AABB aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 5));
        REQUIRE(aabb.maxExtentAxis() == 2);
    }
}
