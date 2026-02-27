#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "NexDynIPC/Control/LinearVelocityDriveForm.h"
#include "NexDynIPC/Control/PositionDriveForm.h"
#include "NexDynIPC/Control/ForceDriveForm.h"
#include "NexDynIPC/Control/DampedSpringForm.h"
#include "NexDynIPC/Dynamics/RigidBody.h"

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <cmath>
#include <limits>

using namespace NexDynIPC::Control;
using namespace NexDynIPC::Dynamics;
using Catch::Approx;

namespace {

std::shared_ptr<RigidBody> makeBody(int id) {
    auto body = std::make_shared<RigidBody>();
    body->id = id;
    body->mass = 1.0;
    body->position = Eigen::Vector3d::Zero();
    body->velocity = Eigen::Vector3d::Zero();
    return body;
}

Eigen::VectorXd denseHessianVectorProduct(
    const std::vector<Eigen::Triplet<double>>& triplets,
    int n,
    const Eigen::VectorXd& vec) {
    Eigen::SparseMatrix<double> H(n, n);
    H.setFromTriplets(triplets.begin(), triplets.end());
    return H * vec;
}

} // namespace

TEST_CASE("LinearVelocityDriveForm - gradient and saturation behavior", "[control][linear_velocity_drive]") {
    auto bodyA = makeBody(0);
    auto bodyB = makeBody(1);
    std::vector<std::shared_ptr<RigidBody>> bodies{bodyA, bodyB};

    SECTION("Gradient follows expected axis force mapping") {
        LinearVelocityDriveForm form(bodyA, bodyB, Eigen::Vector3d::UnitX(), 100.0, 1.0);
        form.setTimeStep(0.1);
        form.setMaxForce(5.0);
        form.updateGlobalIndices(bodies);

        Eigen::VectorXd x_prev = Eigen::VectorXd::Zero(12);
        Eigen::VectorXd x_curr = Eigen::VectorXd::Zero(12);
        x_curr(6) = 0.05; // bodyB x position

        // Seed previous state for velocity estimation
        (void)form.value(x_prev);

        Eigen::VectorXd grad = Eigen::VectorXd::Zero(12);
        form.gradient(x_curr, grad);

        const double v = (0.05 - 0.0) / 0.1;
        const double v_error = 1.0 - v;
        const double force = 5.0 * std::tanh((100.0 * v_error) / 5.0);
        const double coeff = force / 0.1;

        REQUIRE(grad(6) == Approx(-coeff).margin(1e-6));
        REQUIRE(grad(0) == Approx(coeff).margin(1e-6));
        REQUIRE(grad(7) == Approx(0.0).margin(1e-12));
        REQUIRE(grad(8) == Approx(0.0).margin(1e-12));
    }

    SECTION("Deadzone and saturation clamp drive force") {
        LinearVelocityDriveForm form(bodyA, bodyB, Eigen::Vector3d::UnitX(), 80.0, 1.0);
        form.setDelay(0.1);
        form.setMaxForce(3.0);
        form.updateGlobalIndices(bodies);

        bodyA->velocity = Eigen::Vector3d::Zero();
        bodyB->velocity = Eigen::Vector3d(0.95, 0.0, 0.0);
        REQUIRE(form.getDriveForceFromBodies() == Approx(0.0).margin(1e-12));

        bodyB->velocity = Eigen::Vector3d(-2.0, 0.0, 0.0);
        const double force = form.getDriveForceFromBodies();
        REQUIRE(std::abs(force) <= Approx(3.0).margin(1e-8));
        REQUIRE(form.isForceSaturatedFromBodies());
    }

    SECTION("Delay tau filters target velocity") {
        LinearVelocityDriveForm form(bodyA, bodyB, Eigen::Vector3d::UnitX(), 10.0, 0.0);
        form.setTimeStep(0.1);
        form.setDelayTau(0.2);

        form.advanceControlState();
        REQUIRE(form.getEffectiveTargetVelocity() == Approx(0.0));

        form.setTargetVelocity(1.0);
        form.advanceControlState();

        const double alpha = 0.1 / (0.2 + 0.1);
        REQUIRE(form.getEffectiveTargetVelocity() == Approx(alpha).margin(1e-9));
    }

    SECTION("Hessian matches finite-difference of gradient (unsaturated)") {
        LinearVelocityDriveForm form(bodyA, bodyB, Eigen::Vector3d::UnitX(), 12.0, 1.5);
        form.setTimeStep(0.2);
        form.setDelay(0.0);
        form.setMaxForce(std::numeric_limits<double>::infinity());

        const Eigen::VectorXd x_prev_seed = Eigen::VectorXd::Zero(12);

        auto eval_grad = [&](const Eigen::VectorXd& x_eval) {
            form.updateGlobalIndices(bodies);
            (void)form.value(x_prev_seed); // seed previous state for velocity estimate
            Eigen::VectorXd g = Eigen::VectorXd::Zero(12);
            form.gradient(x_eval, g);
            return g;
        };

        auto eval_hv = [&](const Eigen::VectorXd& x_eval, const Eigen::VectorXd& d) {
            form.updateGlobalIndices(bodies);
            (void)form.value(x_prev_seed);
            std::vector<Eigen::Triplet<double>> triplets;
            form.hessian(x_eval, triplets);
            return denseHessianVectorProduct(triplets, 12, d);
        };

        Eigen::VectorXd x = Eigen::VectorXd::Zero(12);
        x(0) = 0.1;
        x(6) = 0.4;

        Eigen::VectorXd d = Eigen::VectorXd::Zero(12);
        d(0) = -0.3;
        d(6) = 0.7;

        const double eps = 1e-6;
        const Eigen::VectorXd g_plus = eval_grad(x + eps * d);
        const Eigen::VectorXd g_minus = eval_grad(x - eps * d);
        const Eigen::VectorXd fd_hv = (g_plus - g_minus) / (2.0 * eps);
        const Eigen::VectorXd hv = eval_hv(x, d);

        REQUIRE(hv(0) == Approx(fd_hv(0)).margin(1e-3));
        REQUIRE(hv(6) == Approx(fd_hv(6)).margin(1e-3));
    }
}

TEST_CASE("PositionDriveForm - gradient and saturation behavior", "[control][position_drive]") {
    auto bodyA = makeBody(0);
    auto bodyB = makeBody(1);
    std::vector<std::shared_ptr<RigidBody>> bodies{bodyA, bodyB};

    SECTION("Gradient matches finite difference in unsaturated linear region") {
        PositionDriveForm form(bodyA, bodyB, Eigen::Vector3d::UnitX(), 25.0, 0.2);
        form.setDeadzone(0.0);
        form.setMaxForce(std::numeric_limits<double>::infinity());
        form.updateGlobalIndices(bodies);

        Eigen::VectorXd x = Eigen::VectorXd::Zero(12);
        x(6) = 0.7;

        Eigen::VectorXd grad = Eigen::VectorXd::Zero(12);
        form.gradient(x, grad);

        const double eps = 1e-6;
        Eigen::VectorXd x_plus = x;
        Eigen::VectorXd x_minus = x;
        x_plus(6) += eps;
        x_minus(6) -= eps;
        const double fd = (form.value(x_plus) - form.value(x_minus)) / (2.0 * eps);

        REQUIRE(grad(6) == Approx(fd).margin(1e-4));
    }

    SECTION("Deadzone zeroes force and saturation limits magnitude") {
        PositionDriveForm form(bodyA, bodyB, Eigen::Vector3d::UnitX(), 100.0, 0.0);
        form.setDeadzone(0.1);
        form.setMaxForce(10.0);
        form.updateGlobalIndices(bodies);

        Eigen::VectorXd x_inside_deadzone = Eigen::VectorXd::Zero(12);
        x_inside_deadzone(6) = -0.05;
        REQUIRE(form.getDriveForce(x_inside_deadzone) == Approx(0.0).margin(1e-12));

        Eigen::VectorXd x_large_error = Eigen::VectorXd::Zero(12);
        x_large_error(6) = 1.0;
        const double saturated_force = form.getDriveForce(x_large_error);
        REQUIRE(std::abs(saturated_force) <= Approx(10.0).margin(1e-8));
        REQUIRE(std::abs(saturated_force) > 9.0);
    }

    SECTION("Hessian matches finite-difference of gradient") {
        PositionDriveForm form(bodyA, bodyB, Eigen::Vector3d::UnitX(), 30.0, 0.2);
        form.setDeadzone(0.0);
        form.setMaxForce(std::numeric_limits<double>::infinity());
        form.updateGlobalIndices(bodies);

        Eigen::VectorXd x = Eigen::VectorXd::Zero(12);
        x(0) = 0.1;
        x(6) = 0.55;

        Eigen::VectorXd d = Eigen::VectorXd::Zero(12);
        d(0) = -0.4;
        d(6) = 0.9;

        Eigen::VectorXd g_plus = Eigen::VectorXd::Zero(12);
        Eigen::VectorXd g_minus = Eigen::VectorXd::Zero(12);
        const double eps = 1e-6;
        form.gradient(x + eps * d, g_plus);
        form.gradient(x - eps * d, g_minus);
        const Eigen::VectorXd fd_hv = (g_plus - g_minus) / (2.0 * eps);

        std::vector<Eigen::Triplet<double>> triplets;
        form.hessian(x, triplets);
        const Eigen::VectorXd hv = denseHessianVectorProduct(triplets, 12, d);

        REQUIRE(hv(0) == Approx(fd_hv(0)).margin(1e-3));
        REQUIRE(hv(6) == Approx(fd_hv(6)).margin(1e-3));
    }
}

TEST_CASE("ForceDriveForm - gradient is constant without saturation", "[control][force_drive]") {
    auto body = makeBody(0);
    std::vector<std::shared_ptr<RigidBody>> bodies{body};

    const Eigen::Vector3d force(123.0, -45.0, 9.0);
    const Eigen::Vector3d torque(-3.0, 8.0, 2.5);

    ForceDriveForm form(body, force, torque);
    form.updateGlobalIndices(bodies);

    Eigen::VectorXd x1 = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd x2 = Eigen::VectorXd::Constant(6, 1.2345);
    Eigen::VectorXd g1 = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd g2 = Eigen::VectorXd::Zero(6);

    form.gradient(x1, g1);
    form.gradient(x2, g2);

    REQUIRE(g1(0) == Approx(-123.0));
    REQUIRE(g1(1) == Approx(45.0));
    REQUIRE(g1(2) == Approx(-9.0));
    REQUIRE(g1(3) == Approx(3.0));
    REQUIRE(g1(4) == Approx(-8.0));
    REQUIRE(g1(5) == Approx(-2.5));

    // No state-dependent behavior or saturation: gradient is identical for different x
    REQUIRE(g1.isApprox(g2, 1e-12));
}

TEST_CASE("DampedSpringForm - gradient and hessian behavior", "[control][damped_spring]") {
    auto bodyA = makeBody(0);
    auto bodyB = makeBody(1);
    std::vector<std::shared_ptr<RigidBody>> bodies{bodyA, bodyB};

    SECTION("Gradient matches spring force direction") {
        DampedSpringForm form(bodyA, bodyB, 0.5, 100.0, 0.0);
        form.updateGlobalIndices(bodies);

        Eigen::VectorXd x = Eigen::VectorXd::Zero(12);
        x(6) = 1.0;

        Eigen::VectorXd grad = Eigen::VectorXd::Zero(12);
        form.gradient(x, grad);

        // extension = 1.0 - 0.5 = 0.5 => traction = 100 * 0.5 = 50 along +x
        REQUIRE(grad(0) == Approx(-50.0).margin(1e-9));
        REQUIRE(grad(6) == Approx(50.0).margin(1e-9));
    }

    SECTION("Hessian matches finite-difference of gradient") {
        DampedSpringForm form(bodyA, bodyB, 0.4, 40.0, 0.0);
        form.updateGlobalIndices(bodies);

        Eigen::VectorXd x = Eigen::VectorXd::Zero(12);
        x(0) = 0.2;
        x(6) = 1.1;

        Eigen::VectorXd d = Eigen::VectorXd::Zero(12);
        d(0) = -0.6;
        d(6) = 0.8;

        const double eps = 1e-6;
        Eigen::VectorXd g_plus = Eigen::VectorXd::Zero(12);
        Eigen::VectorXd g_minus = Eigen::VectorXd::Zero(12);
        form.gradient(x + eps * d, g_plus);
        form.gradient(x - eps * d, g_minus);
        const Eigen::VectorXd fd_hv = (g_plus - g_minus) / (2.0 * eps);

        std::vector<Eigen::Triplet<double>> triplets;
        form.hessian(x, triplets);
        const Eigen::VectorXd hv = denseHessianVectorProduct(triplets, 12, d);

        REQUIRE(hv(0) == Approx(fd_hv(0)).margin(1e-3));
        REQUIRE(hv(6) == Approx(fd_hv(6)).margin(1e-3));
    }
}
