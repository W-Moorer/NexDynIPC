#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "NexDynIPC/Physics/Contact/Friction.h"
#include "NexDynIPC/Physics/Contact/FrictionCone.h"
#include "NexDynIPC/Dynamics/Forms/FrictionForm.h"
#include "NexDynIPC/Dynamics/World.h"
#include "NexDynIPC/Dynamics/RigidBody.h"

using namespace NexDynIPC::Physics::Contact;
using namespace NexDynIPC::Dynamics;
using namespace Catch::Matchers;

TEST_CASE("FrictionPotential basic functionality", "[friction][potential]")
{
    SECTION("Friction potential value is positive")
    {
        Eigen::Vector3d u(0.1, 0, 0);
        double N = 10.0;
        double mu = 0.5;
        
        double val = FrictionPotential::value(u, N, mu);
        
        REQUIRE(val > 0);
    }
    
    SECTION("Friction potential is zero with zero normal force")
    {
        Eigen::Vector3d u(0.1, 0, 0);
        double N = 0.0;
        double mu = 0.5;
        
        double val = FrictionPotential::value(u, N, mu);
        
        REQUIRE(val == 0.0);
    }
    
    SECTION("Friction potential increases with displacement")
    {
        double N = 10.0;
        double mu = 0.5;
        
        Eigen::Vector3d u1(0.1, 0, 0);
        Eigen::Vector3d u2(0.2, 0, 0);
        
        double val1 = FrictionPotential::value(u1, N, mu);
        double val2 = FrictionPotential::value(u2, N, mu);
        
        REQUIRE(val2 > val1);
    }
    
    SECTION("Friction gradient points opposite to displacement")
    {
        Eigen::Vector3d u(0.1, 0, 0);
        double N = 10.0;
        double mu = 0.5;
        
        Eigen::Vector3d grad = FrictionPotential::gradient(u, N, mu);
        
        // Gradient should be in same direction as u (for potential)
        REQUIRE(grad.x() > 0);
        REQUIRE_THAT(grad.y(), WithinAbs(0.0, 1e-10));
        REQUIRE_THAT(grad.z(), WithinAbs(0.0, 1e-10));
    }
    
    SECTION("Friction Hessian is positive semi-definite")
    {
        Eigen::Vector3d u(0.1, 0.05, 0);
        double N = 10.0;
        double mu = 0.5;
        
        Eigen::Matrix3d hess = FrictionPotential::hessian(u, N, mu);
        
        // Check symmetry
        REQUIRE((hess - hess.transpose()).norm() < 1e-10);
        
        // Check positive semi-definite (all eigenvalues >= 0)
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(hess);
        for (int i = 0; i < 3; ++i) {
            REQUIRE(solver.eigenvalues()[i] >= -1e-10);
        }
    }
}

TEST_CASE("FrictionPotential smooth approximation", "[friction][smooth]")
{
    SECTION("Smooth friction value")
    {
        Eigen::Vector3d u(0.1, 0, 0);
        double N = 10.0;
        double mu = 0.5;
        
        double val = FrictionPotential::valueSmooth(u, N, mu);
        
        REQUIRE(val > 0);
    }
    
    SECTION("Smooth friction is quadratic for small displacements")
    {
        double N = 10.0;
        double mu = 0.5;
        double eps = 0.1;
        
        // Small displacement (within eps)
        Eigen::Vector3d u(0.05, 0, 0);
        
        double val = FrictionPotential::valueSmooth(u, N, mu, eps);
        double expected = 0.5 * mu * N / eps * u.squaredNorm();
        
        REQUIRE_THAT(val, WithinAbs(expected, 1e-10));
    }
    
    SECTION("Smooth friction is linear for large displacements")
    {
        double N = 10.0;
        double mu = 0.5;
        double eps = 0.1;
        
        // Large displacement (beyond eps)
        Eigen::Vector3d u(0.5, 0, 0);
        
        double val = FrictionPotential::valueSmooth(u, N, mu, eps);
        double expected = mu * N * (u.norm() - 0.5 * eps);
        
        REQUIRE_THAT(val, WithinAbs(expected, 1e-10));
    }
}

TEST_CASE("FrictionCone basic functionality", "[friction][cone]")
{
    SECTION("Project inside cone returns same vector")
    {
        Eigen::Vector3d f(1.0, 0, 0);
        double fn = 10.0;
        double mu = 0.5; // max ||ft|| = 5.0
        
        Eigen::Vector3d projected = FrictionCone::projectToCone(f, fn, mu);
        
        REQUIRE((projected - f).norm() < 1e-10);
    }
    
    SECTION("Project outside cone scales to boundary")
    {
        Eigen::Vector3d f(10.0, 0, 0);
        double fn = 5.0;
        double mu = 0.5; // max ||ft|| = 2.5
        
        Eigen::Vector3d projected = FrictionCone::projectToCone(f, fn, mu);
        
        REQUIRE_THAT(projected.norm(), WithinAbs(2.5, 1e-10));
        REQUIRE((projected.normalized() - f.normalized()).norm() < 1e-10);
    }
    
    SECTION("Is inside cone")
    {
        Eigen::Vector3d f(2.0, 0, 0);
        double fn = 10.0;
        double mu = 0.5; // max ||ft|| = 5.0
        
        bool inside = FrictionCone::isInsideCone(f, fn, mu);
        
        REQUIRE(inside);
    }
    
    SECTION("Is outside cone")
    {
        Eigen::Vector3d f(10.0, 0, 0);
        double fn = 5.0;
        double mu = 0.5; // max ||ft|| = 2.5
        
        bool inside = FrictionCone::isInsideCone(f, fn, mu);
        
        REQUIRE(!inside);
    }
    
    SECTION("Distance to cone - inside")
    {
        Eigen::Vector3d f(2.0, 0, 0);
        double fn = 10.0;
        double mu = 0.5; // max ||ft|| = 5.0
        
        double dist = FrictionCone::distanceToCone(f, fn, mu);
        
        REQUIRE(dist < 0); // Negative when inside
    }
    
    SECTION("Distance to cone - outside")
    {
        Eigen::Vector3d f(10.0, 0, 0);
        double fn = 5.0;
        double mu = 0.5; // max ||ft|| = 2.5
        
        double dist = FrictionCone::distanceToCone(f, fn, mu);
        
        REQUIRE(dist > 0); // Positive when outside
        REQUIRE_THAT(dist, WithinAbs(7.5, 1e-10));
    }
    
    SECTION("Cone radius")
    {
        double fn = 10.0;
        double mu = 0.3;
        
        double radius = FrictionCone::coneRadius(fn, mu);
        
        REQUIRE_THAT(radius, WithinAbs(3.0, 1e-10));
    }
    
    SECTION("Zero normal force - no friction")
    {
        Eigen::Vector3d f(5.0, 0, 0);
        double fn = 0.0;
        double mu = 0.5;
        
        Eigen::Vector3d projected = FrictionCone::projectToCone(f, fn, mu);
        
        REQUIRE_THAT(projected.norm(), WithinAbs(0.0, 1e-10));
    }
}

TEST_CASE("FrictionForm basic functionality", "[friction][form]")
{
    World world;
    
    // Create two bodies
    auto body1 = std::make_shared<RigidBody>();
    body1->id = 0;
    body1->position = Eigen::Vector3d(0, 0, 0);
    body1->velocity = Eigen::Vector3d(0, 0, 0);
    body1->mass = 1.0;
    world.bodies.push_back(body1);
    
    auto body2 = std::make_shared<RigidBody>();
    body2->id = 1;
    body2->position = Eigen::Vector3d(1, 0, 0);
    body2->velocity = Eigen::Vector3d(0, 0, 0);
    body2->mass = 1.0;
    world.bodies.push_back(body2);
    
    SECTION("FrictionForm construction")
    {
        FrictionForm form(world, 0.5);
        
        REQUIRE(form.frictionCoefficient() == 0.5);
    }
    
    SECTION("Set friction coefficient")
    {
        FrictionForm form(world, 0.5);
        form.setFrictionCoefficient(0.3);
        
        REQUIRE(form.frictionCoefficient() == 0.3);
    }
    
    SECTION("Set smoothing parameter")
    {
        FrictionForm form(world, 0.5);
        form.setSmoothingParameter(1e-6);
        
        REQUIRE(form.smoothingParameter() == 1e-6);
    }
}

TEST_CASE("Friction gradient finite difference check", "[friction][gradient]")
{
    SECTION("Gradient matches finite difference")
    {
        Eigen::Vector3d u(0.1, 0.05, 0.02);
        double N = 10.0;
        double mu = 0.5;
        double eps = 1e-8;
        
        Eigen::Vector3d grad = FrictionPotential::gradient(u, N, mu, eps);
        
        // Finite difference check
        double h = 1e-6;
        for (int i = 0; i < 3; ++i) {
            Eigen::Vector3d u_plus = u;
            Eigen::Vector3d u_minus = u;
            u_plus[i] += h;
            u_minus[i] -= h;
            
            double val_plus = FrictionPotential::value(u_plus, N, mu, eps);
            double val_minus = FrictionPotential::value(u_minus, N, mu, eps);
            
            double fd_grad = (val_plus - val_minus) / (2 * h);
            
            REQUIRE_THAT(grad[i], WithinAbs(fd_grad, 1e-5));
        }
    }
}

TEST_CASE("Friction Hessian finite difference check", "[friction][hessian]")
{
    SECTION("Hessian matches finite difference")
    {
        Eigen::Vector3d u(0.1, 0.05, 0.02);
        double N = 10.0;
        double mu = 0.5;
        double eps = 1e-8;
        
        Eigen::Matrix3d hess = FrictionPotential::hessian(u, N, mu, eps);
        
        // Finite difference check
        double h = 1e-6;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                Eigen::Vector3d u_plus = u;
                Eigen::Vector3d u_minus = u;
                u_plus[j] += h;
                u_minus[j] -= h;
                
                Eigen::Vector3d grad_plus = FrictionPotential::gradient(u_plus, N, mu, eps);
                Eigen::Vector3d grad_minus = FrictionPotential::gradient(u_minus, N, mu, eps);
                
                double fd_hess = (grad_plus[i] - grad_minus[i]) / (2 * h);
                
                REQUIRE_THAT(hess(i, j), WithinAbs(fd_hess, 1e-4));
            }
        }
    }
}
