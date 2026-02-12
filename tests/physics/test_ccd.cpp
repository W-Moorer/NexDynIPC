#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "NexDynIPC/Physics/Contact/CCD/Impact.h"
#include "NexDynIPC/Physics/Contact/CCD/IntervalRootFinder.h"
#include "NexDynIPC/Physics/Contact/CCD/RigidTrajectoryAABB.h"
#include "NexDynIPC/Physics/Contact/CCD/TimeOfImpact.h"
#include "NexDynIPC/Physics/Contact/CCD/CCD.h"
#include "NexDynIPC/Physics/Contact/AdaptiveBarrier.h"
#include "NexDynIPC/Dynamics/RigidBody.h"
#include "NexDynIPC/Physics/Geometry/MeshShape.h"

using namespace NexDynIPC::Physics::CCD;
using namespace NexDynIPC::Physics::Contact;
using namespace NexDynIPC::Math;
using namespace Catch::Matchers;

TEST_CASE("Impact structures basic functionality", "[ccd][impact]")
{
    SECTION("EdgeVertexImpact construction and comparison")
    {
        EdgeVertexImpact impact(0.5, 1, 0.25, 2);
        
        REQUIRE(impact.time == 0.5);
        REQUIRE(impact.edge_index == 1);
        REQUIRE(impact.alpha == 0.25);
        REQUIRE(impact.vertex_index == 2);
        
        EdgeVertexImpact same_impact(0.5, 1, 0.25, 2);
        REQUIRE(impact == same_impact);
    }
    
    SECTION("EdgeEdgeImpact construction and comparison")
    {
        EdgeEdgeImpact impact(0.3, 0, 0.5, 1, 0.5);
        
        REQUIRE(impact.time == 0.3);
        REQUIRE(impact.impacted_edge_index == 0);
        REQUIRE(impact.impacting_edge_index == 1);
        
        EdgeEdgeImpact same_impact(0.3, 0, 0.5, 1, 0.5);
        REQUIRE(impact == same_impact);
    }
    
    SECTION("FaceVertexImpact construction and comparison")
    {
        FaceVertexImpact impact(0.7, 2, 0.3, 0.3, 5);
        
        REQUIRE(impact.time == 0.7);
        REQUIRE(impact.face_index == 2);
        REQUIRE(impact.u == 0.3);
        REQUIRE(impact.v == 0.3);
        REQUIRE(impact.vertex_index == 5);
        
        FaceVertexImpact same_impact(0.7, 2, 0.3, 0.3, 5);
        REQUIRE(impact == same_impact);
    }
    
    SECTION("Impacts container operations")
    {
        Impacts impacts;
        
        REQUIRE(impacts.empty());
        REQUIRE(impacts.size() == 0);
        
        impacts.ev_impacts.emplace_back(0.1, 0, 0.5, 1);
        impacts.ee_impacts.emplace_back(0.2, 0, 0.5, 1, 0.5);
        impacts.fv_impacts.emplace_back(0.3, 0, 0.33, 0.33, 2);
        
        REQUIRE(!impacts.empty());
        REQUIRE(impacts.size() == 3);
        
        impacts.clear();
        REQUIRE(impacts.empty());
    }
}

TEST_CASE("IntervalRootFinder basic tests", "[ccd][interval]")
{
    SECTION("Find root of simple linear function")
    {
        auto f = [](const Interval& x) -> Interval {
            return x - Interval(0.5);
        };
        
        Interval x0(0, 1);
        Interval root;
        
        bool found = intervalRootFinder(f, x0, 1e-8, root);
        
        REQUIRE(found);
        REQUIRE_THAT(lower(root), WithinAbs(0.5, 1e-6));
        REQUIRE_THAT(upper(root), WithinAbs(0.5, 1e-6));
    }
    
    SECTION("Find root of quadratic function")
    {
        auto f = [](const Interval& x) -> Interval {
            return x * x - Interval(0.25);
        };
        
        Interval x0(0, 1);
        Interval root;
        
        bool found = intervalRootFinder(f, x0, 1e-8, root);
        
        REQUIRE(found);
        REQUIRE_THAT(lower(root), WithinAbs(0.5, 1e-6));
    }
    
    SECTION("No root when function doesn't cross zero")
    {
        auto f = [](const Interval& x) -> Interval {
            return x * x + Interval(1.0);
        };
        
        Interval x0(0, 1);
        Interval root;
        
        bool found = intervalRootFinder(f, x0, 1e-8, root);
        
        REQUIRE(!found);
    }
}

TEST_CASE("RigidTrajectoryAABB basic tests", "[ccd][trajectory]")
{
    using namespace NexDynIPC::Dynamics;
    
    RigidBody body;
    body.position = Eigen::Vector3d(0, 0, 0);
    body.orientation = Eigen::Quaterniond::Identity();
    
    SECTION("PoseInterval construction")
    {
        Eigen::Vector3d pos_t0(0, 0, 0);
        Eigen::Vector3d pos_t1(1, 0, 0);
        Eigen::Quaterniond rot = Eigen::Quaterniond::Identity();
        
        PoseInterval pose(pos_t0, pos_t1, rot, rot);
        
        REQUIRE_THAT(lower(pose.position.x()), WithinAbs(0.0, 1e-10));
        REQUIRE_THAT(upper(pose.position.x()), WithinAbs(1.0, 1e-10));
    }
    
    SECTION("Vertex trajectory AABB")
    {
        Eigen::Vector3d pos_t0(0, 0, 0);
        Eigen::Vector3d pos_t1(1, 0, 0);
        Eigen::Quaterniond rot = Eigen::Quaterniond::Identity();
        
        PoseInterval pose(pos_t0, pos_t1, rot, rot);
        Eigen::Vector3d local_vertex(0, 0, 0);
        
        Eigen::Vector3I trajectory = vertexTrajectoryAABB(body, pose, local_vertex);
        
        REQUIRE_THAT(lower(trajectory.x()), WithinAbs(0.0, 1e-10));
        REQUIRE_THAT(upper(trajectory.x()), WithinAbs(1.0, 1e-10));
    }
}

TEST_CASE("AdaptiveBarrier basic tests", "[ccd][barrier]")
{
    SECTION("AdaptiveBarrier initialization")
    {
        AdaptiveBarrier barrier;
        barrier.initialize(1.0, 10.0);
        
        REQUIRE(barrier.minStiffness() > 0);
        REQUIRE(barrier.currentStiffness() == barrier.minStiffness());
    }
    
    SECTION("AdaptiveBarrier stiffness update")
    {
        AdaptiveBarrier barrier;
        barrier.initialize(1.0, 10.0);
        
        double initial_kappa = barrier.currentStiffness();
        
        barrier.updateStiffness(1e-12);
        
        REQUIRE(barrier.currentStiffness() > initial_kappa);
    }
    
    SECTION("AdaptiveBarrier value computation")
    {
        AdaptiveBarrier barrier;
        barrier.setDhat(1.0);
        barrier.setStiffness(1.0);
        
        double d = 0.5;
        double val = barrier.value(d);
        
        REQUIRE(val > 0);
        
        double grad = barrier.gradient(d);
        REQUIRE(grad < 0);
    }
    
    SECTION("AdaptiveBarrier is active")
    {
        AdaptiveBarrier barrier;
        barrier.setDhat(1.0);
        
        REQUIRE(barrier.isActive(0.5));
        REQUIRE(!barrier.isActive(1.5));
        REQUIRE(!barrier.isActive(-0.1));
    }
}

TEST_CASE("TimeOfImpactCalculator basic tests", "[ccd][toi]")
{
    using namespace NexDynIPC::Dynamics;
    
    SECTION("TimeOfImpactCalculator default tolerance")
    {
        TimeOfImpactCalculator calc;
        REQUIRE_THAT(calc.toiTolerance(), WithinAbs(1e-8, 1e-10));
    }
    
    SECTION("TimeOfImpactCalculator set tolerance")
    {
        TimeOfImpactCalculator calc;
        calc.setTOITolerance(1e-6);
        REQUIRE_THAT(calc.toiTolerance(), WithinAbs(1e-6, 1e-10));
    }
}

TEST_CASE("CCDSystem basic tests", "[ccd][system]")
{
    using namespace NexDynIPC::Dynamics;
    
    SECTION("CCDSystem default construction")
    {
        CCDSystem ccd;
        
        std::vector<std::shared_ptr<RigidBody>> bodies;
        double toi = ccd.computeEarliestTOI(bodies, 0.01);
        
        REQUIRE_THAT(toi, WithinAbs(1.0, 1e-10));
    }
    
    SECTION("CCDSystem empty scene")
    {
        CCDSystem ccd;
        
        std::vector<std::shared_ptr<RigidBody>> bodies;
        Impacts impacts = ccd.detectCollisions(bodies, 0.01);
        
        REQUIRE(impacts.empty());
    }
}

TEST_CASE("Edge-Vertex TOI computation", "[ccd][toi][edge-vertex]")
{
    using namespace NexDynIPC::Dynamics;
    
    Eigen::Vector3d posA_t0(0, 0, 0);
    Eigen::Quaterniond rotA = Eigen::Quaterniond::Identity();
    Eigen::Vector3d posA_t1(0, 0, 0);
    
    Eigen::Vector3d posB_t0(2, 0, 0);
    Eigen::Vector3d posB_t1(0, 0, 0);
    Eigen::Quaterniond rotB = Eigen::Quaterniond::Identity();
    
    RigidBody bodyA, bodyB;
    bodyA.position = posA_t0;
    bodyA.orientation = rotA;
    bodyB.position = posB_t0;
    bodyB.orientation = rotB;
    
    Eigen::Vector3d vertex(0, 0, 0);
    Eigen::Vector3d edge_v0(0, -1, 0);
    Eigen::Vector3d edge_v1(0, 1, 0);
    
    double toi;
    bool collision = computeEdgeVertexTOI(
        bodyA, posA_t0, rotA, posA_t1, rotA, vertex,
        bodyB, posB_t0, rotB, posB_t1, rotB, edge_v0, edge_v1,
        toi);
    
    SECTION("Collision detected for approaching bodies")
    {
        REQUIRE(collision);
        REQUIRE(toi >= 0);
        REQUIRE(toi <= 1);
    }
}

TEST_CASE("AdaptiveBarrier stiffness scaling", "[ccd][barrier][scaling]")
{
    SECTION("Stiffness grows when distance is small")
    {
        AdaptiveBarrier barrier;
        barrier.initialize(1.0, 1.0);
        
        double kappa0 = barrier.currentStiffness();
        
        barrier.updateStiffness(1e-10);
        double kappa1 = barrier.currentStiffness();
        
        barrier.updateStiffness(1e-10);
        double kappa2 = barrier.currentStiffness();
        
        REQUIRE(kappa1 > kappa0);
        REQUIRE(kappa2 > kappa1);
    }
    
    SECTION("Stiffness capped at maximum")
    {
        AdaptiveBarrierParams params;
        params.max_stiffness = 1e15;
        AdaptiveBarrier barrier(params);
        barrier.initialize(1.0, 1.0);
        
        for (int i = 0; i < 100; ++i) {
            barrier.updateStiffness(1e-15);
        }
        
        REQUIRE(barrier.currentStiffness() <= barrier.maxStiffness());
    }
}
