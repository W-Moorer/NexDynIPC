#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "NexDynIPC/Physics/Geometry/BVH.h"
#include "NexDynIPC/Physics/Contact/BroadPhase.h"

#include <algorithm>
#include <chrono>
#include <random>
#include <set>

using namespace NexDynIPC::Physics::Geometry;
using namespace NexDynIPC::Physics;
using Catch::Matchers::WithinRel;

/**
 * @brief BVH层次包围盒单元测试
 * 
 * 测试覆盖：
 * - BVH构建（空、单个、多个图元）
 * - BVH查询（包围盒相交、点相交、BVH间相交）
 * - BVH统计（高度、节点计数）
 * - BVHBroadPhase集成测试
 * - 性能测试
 */
TEST_CASE("BVH empty construction", "[geometry][bvh]")
{
    BVH bvh;

    SECTION("Empty BVH has no nodes")
    {
        REQUIRE(bvh.empty());
        REQUIRE(bvh.nodeCount() == 0);
        REQUIRE(bvh.root() < 0);
    }

    SECTION("Empty BVH returns empty results")
    {
        std::vector<unsigned int> result;
        bvh.intersectBox(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1), result);
        REQUIRE(result.empty());
    }
}

TEST_CASE("BVH construction from single AABB", "[geometry][bvh]")
{
    std::vector<AABB> aabbs = {
        AABB(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1))
    };

    BVH bvh;
    bvh.init(aabbs);

    SECTION("Single AABB creates single leaf node")
    {
        REQUIRE_FALSE(bvh.empty());
        REQUIRE(bvh.nodeCount() == 1);
        REQUIRE(bvh.leafCount() == 1);
        REQUIRE(bvh.internalCount() == 0);
        REQUIRE(bvh.height() == 1);
    }

    SECTION("Query finds the single primitive")
    {
        std::vector<unsigned int> result;
        bvh.intersectBox(Eigen::Vector3d(0.5, 0.5, 0.5), Eigen::Vector3d(1.5, 1.5, 1.5), result);
        REQUIRE(result.size() == 1);
        REQUIRE(result[0] == 0);
    }
}

TEST_CASE("BVH construction from multiple AABBs", "[geometry][bvh]")
{
    std::vector<AABB> aabbs = {
        AABB(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)),
        AABB(Eigen::Vector3d(2, 0, 0), Eigen::Vector3d(3, 1, 1)),
        AABB(Eigen::Vector3d(0, 2, 0), Eigen::Vector3d(1, 3, 1)),
        AABB(Eigen::Vector3d(2, 2, 0), Eigen::Vector3d(3, 3, 1))
    };

    BVH bvh;
    bvh.init(aabbs);

    SECTION("Four AABBs create proper tree structure")
    {
        REQUIRE_FALSE(bvh.empty());
        REQUIRE(bvh.leafCount() == 4);
        REQUIRE(bvh.internalCount() == 3);
        REQUIRE(bvh.nodeCount() == 7);
        REQUIRE(bvh.height() >= 2);
        REQUIRE(bvh.height() <= 4);
    }
}

TEST_CASE("BVH construction from array format", "[geometry][bvh]")
{
    std::vector<std::array<Eigen::Vector3d, 2>> aabbs = {
        { Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1) },
        { Eigen::Vector3d(2, 0, 0), Eigen::Vector3d(3, 1, 1) }
    };

    BVH bvh;
    bvh.init(aabbs);

    SECTION("Array format works correctly")
    {
        REQUIRE_FALSE(bvh.empty());
        REQUIRE(bvh.leafCount() == 2);
    }
}

TEST_CASE("BVH box intersection query", "[geometry][bvh]")
{
    std::vector<AABB> aabbs = {
        AABB(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)),
        AABB(Eigen::Vector3d(2, 0, 0), Eigen::Vector3d(3, 1, 1)),
        AABB(Eigen::Vector3d(0, 2, 0), Eigen::Vector3d(1, 3, 1)),
        AABB(Eigen::Vector3d(2, 2, 0), Eigen::Vector3d(3, 3, 1))
    };

    BVH bvh;
    bvh.init(aabbs);

    SECTION("Query overlapping first AABB")
    {
        std::vector<unsigned int> result;
        bvh.intersectBox(Eigen::Vector3d(0.5, 0.5, 0), Eigen::Vector3d(0.6, 0.6, 1), result);
        REQUIRE(result.size() == 1);
        REQUIRE(result[0] == 0);
    }

    SECTION("Query overlapping multiple AABBs")
    {
        std::vector<unsigned int> result;
        bvh.intersectBox(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(3, 3, 1), result);
        REQUIRE(result.size() == 4);
    }

    SECTION("Query no overlap")
    {
        std::vector<unsigned int> result;
        bvh.intersectBox(Eigen::Vector3d(10, 10, 10), Eigen::Vector3d(11, 11, 11), result);
        REQUIRE(result.empty());
    }

    SECTION("Query with AABB object")
    {
        AABB query(Eigen::Vector3d(0.5, 0.5, 0), Eigen::Vector3d(2.5, 0.6, 1));
        std::vector<unsigned int> result;
        bvh.intersectBox(query, result);
        REQUIRE(result.size() == 2);
    }
}

TEST_CASE("BVH point intersection query", "[geometry][bvh]")
{
    std::vector<AABB> aabbs = {
        AABB(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)),
        AABB(Eigen::Vector3d(2, 0, 0), Eigen::Vector3d(3, 1, 1))
    };

    BVH bvh;
    bvh.init(aabbs);

    SECTION("Point inside first AABB")
    {
        std::vector<unsigned int> result;
        bvh.intersectPoint(Eigen::Vector3d(0.5, 0.5, 0.5), result);
        REQUIRE(result.size() == 1);
        REQUIRE(result[0] == 0);
    }

    SECTION("Point inside second AABB")
    {
        std::vector<unsigned int> result;
        bvh.intersectPoint(Eigen::Vector3d(2.5, 0.5, 0.5), result);
        REQUIRE(result.size() == 1);
        REQUIRE(result[0] == 1);
    }

    SECTION("Point outside all AABBs")
    {
        std::vector<unsigned int> result;
        bvh.intersectPoint(Eigen::Vector3d(5, 5, 5), result);
        REQUIRE(result.empty());
    }
}

TEST_CASE("BVH to BVH intersection", "[geometry][bvh]")
{
    std::vector<AABB> aabbsA = {
        AABB(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)),
        AABB(Eigen::Vector3d(2, 0, 0), Eigen::Vector3d(3, 1, 1))
    };

    std::vector<AABB> aabbsB = {
        AABB(Eigen::Vector3d(0.5, 0.5, 0), Eigen::Vector3d(1.5, 1.5, 1)),
        AABB(Eigen::Vector3d(5, 5, 5), Eigen::Vector3d(6, 6, 6))
    };

    BVH bvhA, bvhB;
    bvhA.init(aabbsA);
    bvhB.init(aabbsB);

    SECTION("Find overlapping pairs")
    {
        std::vector<std::pair<int, int>> result;
        bvhA.intersectBVH(bvhB, result);

        REQUIRE(result.size() == 1);
        REQUIRE(result[0].first == 0);
        REQUIRE(result[0].second == 0);
    }
}

TEST_CASE("BVH clear operation", "[geometry][bvh]")
{
    std::vector<AABB> aabbs = {
        AABB(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1))
    };

    BVH bvh;
    bvh.init(aabbs);
    REQUIRE_FALSE(bvh.empty());

    bvh.clear();

    SECTION("Clear makes BVH empty")
    {
        REQUIRE(bvh.empty());
        REQUIRE(bvh.nodeCount() == 0);
        REQUIRE(bvh.root() < 0);
    }
}

TEST_CASE("BVHBroadPhase basic detection", "[geometry][bvh][broadphase]")
{
    std::vector<AABB> aabbs = {
        AABB(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)),
        AABB(Eigen::Vector3d(0.5, 0.5, 0.5), Eigen::Vector3d(1.5, 1.5, 1.5)),
        AABB(Eigen::Vector3d(5, 5, 5), Eigen::Vector3d(6, 6, 6))
    };

    BVHBroadPhase broadPhase;
    auto pairs = broadPhase.detect(aabbs);

    SECTION("Detects overlapping pair")
    {
        REQUIRE(pairs.size() == 1);
        REQUIRE(pairs[0].first == 0);
        REQUIRE(pairs[0].second == 1);
    }
}

TEST_CASE("BVHBroadPhase with inflation", "[geometry][bvh][broadphase]")
{
    std::vector<AABB> aabbs = {
        AABB(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)),
        AABB(Eigen::Vector3d(1.1, 1.1, 1.1), Eigen::Vector3d(2, 2, 2))
    };

    BVHBroadPhase broadPhase;

    SECTION("Without inflation - no overlap")
    {
        auto pairs = broadPhase.detect(aabbs);
        REQUIRE(pairs.empty());
    }

    SECTION("With inflation - detects overlap")
    {
        auto pairs = broadPhase.detectWithInflation(aabbs, 0.2);
        REQUIRE(pairs.size() == 1);
    }
}

TEST_CASE("BVHBroadPhase statistics", "[geometry][bvh][broadphase]")
{
    std::vector<AABB> aabbs;
    for (int i = 0; i < 10; ++i) {
        aabbs.push_back(AABB(Eigen::Vector3d(i, 0, 0), Eigen::Vector3d(i + 0.5, 0.5, 0.5)));
    }

    BVHBroadPhase broadPhase;
    broadPhase.detect(aabbs);

    int height, leafCount, internalCount;
    broadPhase.getStatistics(height, leafCount, internalCount);

    SECTION("Statistics are valid")
    {
        REQUIRE(height >= 1);
        REQUIRE(leafCount == 10);
        REQUIRE(internalCount >= 0);
    }
}

TEST_CASE("IncrementalBVHBroadPhase basic operation", "[geometry][bvh][broadphase]")
{
    IncrementalBVHBroadPhase broadPhase;

    std::vector<AABB> aabbs1 = {
        AABB(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)),
        AABB(Eigen::Vector3d(0.5, 0.5, 0.5), Eigen::Vector3d(1.5, 1.5, 1.5))
    };

    auto pairs1 = broadPhase.detect(aabbs1);
    REQUIRE(pairs1.size() == 1);

    SECTION("Same AABBs reuse cached BVH")
    {
        auto pairs2 = broadPhase.detect(aabbs1);
        REQUIRE(pairs2.size() == 1);
    }

    SECTION("Changed AABBs trigger rebuild")
    {
        std::vector<AABB> aabbs2 = {
            AABB(Eigen::Vector3d(5, 5, 5), Eigen::Vector3d(6, 6, 6)),
            AABB(Eigen::Vector3d(7, 7, 7), Eigen::Vector3d(8, 8, 8))
        };

        auto pairs2 = broadPhase.detect(aabbs2);
        REQUIRE(pairs2.empty());
    }
}

TEST_CASE("BVH performance comparison", "[geometry][bvh][performance]")
{
    const int numObjects = 1000;
    std::vector<AABB> aabbs;

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(0, 100);

    for (int i = 0; i < numObjects; ++i) {
        double x = dist(rng);
        double y = dist(rng);
        double z = dist(rng);
        aabbs.push_back(AABB(Eigen::Vector3d(x, y, z), Eigen::Vector3d(x + 1, y + 1, z + 1)));
    }

    SECTION("BVH build time")
    {
        auto start = std::chrono::high_resolution_clock::now();

        BVH bvh;
        bvh.init(aabbs);

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        REQUIRE_FALSE(bvh.empty());
        REQUIRE(bvh.leafCount() == numObjects);
    }

    SECTION("BVH query vs brute force")
    {
        BVH bvh;
        bvh.init(aabbs);

        AABB queryBox(Eigen::Vector3d(25, 25, 25), Eigen::Vector3d(75, 75, 75));

        auto startBVH = std::chrono::high_resolution_clock::now();
        std::vector<unsigned int> bvhResult;
        bvh.intersectBox(queryBox, bvhResult);
        auto endBVH = std::chrono::high_resolution_clock::now();

        auto startBrute = std::chrono::high_resolution_clock::now();
        std::vector<unsigned int> bruteResult;
        for (size_t i = 0; i < aabbs.size(); ++i) {
            if (queryBox.intersects(aabbs[i])) {
                bruteResult.push_back(static_cast<unsigned int>(i));
            }
        }
        auto endBrute = std::chrono::high_resolution_clock::now();

        std::set<unsigned int> bvhSet(bvhResult.begin(), bvhResult.end());
        std::set<unsigned int> bruteSet(bruteResult.begin(), bruteResult.end());
        REQUIRE(bvhSet == bruteSet);
    }
}

TEST_CASE("BVH handles degenerate cases", "[geometry][bvh]")
{
    SECTION("All AABBs at same location")
    {
        std::vector<AABB> aabbs = {
            AABB(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)),
            AABB(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)),
            AABB(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1))
        };

        BVH bvh;
        bvh.init(aabbs);

        REQUIRE(bvh.leafCount() == 3);

        std::vector<unsigned int> result;
        bvh.intersectBox(Eigen::Vector3d(0.5, 0.5, 0.5), Eigen::Vector3d(0.6, 0.6, 0.6), result);
        REQUIRE(result.size() == 3);
    }

    SECTION("Very thin AABBs")
    {
        std::vector<AABB> aabbs = {
            AABB(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1e-10, 1, 1)),
            AABB(Eigen::Vector3d(2, 0, 0), Eigen::Vector3d(2 + 1e-10, 1, 1))
        };

        BVH bvh;
        bvh.init(aabbs);

        REQUIRE(bvh.leafCount() == 2);
    }
}

TEST_CASE("BruteForceBroadPhase basic detection", "[geometry][broadphase]")
{
    std::vector<AABB> aabbs = {
        AABB(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)),
        AABB(Eigen::Vector3d(0.5, 0.5, 0.5), Eigen::Vector3d(1.5, 1.5, 1.5)),
        AABB(Eigen::Vector3d(5, 5, 5), Eigen::Vector3d(6, 6, 6))
    };

    BruteForceBroadPhase broadPhase;
    auto pairs = broadPhase.detect(aabbs);

    SECTION("Detects overlapping pair")
    {
        REQUIRE(pairs.size() == 1);
        REQUIRE(pairs[0].first == 0);
        REQUIRE(pairs[0].second == 1);
    }
}

TEST_CASE("BroadPhase interface consistency", "[geometry][broadphase]")
{
    std::vector<AABB> aabbs = {
        AABB(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)),
        AABB(Eigen::Vector3d(0.5, 0.5, 0.5), Eigen::Vector3d(1.5, 1.5, 1.5)),
        AABB(Eigen::Vector3d(5, 5, 5), Eigen::Vector3d(6, 6, 6))
    };

    BruteForceBroadPhase bruteForce;
    BVHBroadPhase bvh;
    IncrementalBVHBroadPhase incremental;

    auto brutePairs = bruteForce.detect(aabbs);
    auto bvhPairs = bvh.detect(aabbs);
    auto incPairs = incremental.detect(aabbs);

    SECTION("All implementations return same results")
    {
        REQUIRE(brutePairs.size() == bvhPairs.size());
        REQUIRE(brutePairs.size() == incPairs.size());

        for (const auto& p : brutePairs) {
            bool found = false;
            for (const auto& bp : bvhPairs) {
                if (bp.first == p.first && bp.second == p.second) {
                    found = true;
                    break;
                }
            }
            REQUIRE(found);
        }
    }
}
