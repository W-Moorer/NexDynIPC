#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "NexDynIPC/Control/ExternalForceForm.h"
#include "NexDynIPC/Dynamics/RigidBody.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace NexDynIPC::Control;
using namespace NexDynIPC::Dynamics;
using Catch::Approx;

TEST_CASE("ExternalForceForm - Basic functionality", "[force_control]") {
    ExternalForceForm force_form;
    
    auto body1 = std::make_shared<RigidBody>();
    body1->id = 0;
    body1->mass = 1.0;
    
    auto body2 = std::make_shared<RigidBody>();
    body2->id = 1;
    body2->mass = 2.0;
    
    SECTION("Add and remove bodies") {
        force_form.addBody(body1);
        force_form.addBody(body2);
        
        // 设置力
        force_form.setForce(body1, Eigen::Vector3d(10.0, 0.0, 0.0));
        force_form.setForce(body2, Eigen::Vector3d(0.0, 5.0, 0.0));
        
        REQUIRE(force_form.getForce(body1).isApprox(Eigen::Vector3d(10.0, 0.0, 0.0)));
        REQUIRE(force_form.getForce(body2).isApprox(Eigen::Vector3d(0.0, 5.0, 0.0)));
        
        // 移除body1
        force_form.removeBody(body1);
        REQUIRE(force_form.getForce(body1).isApprox(Eigen::Vector3d::Zero()));
        REQUIRE(force_form.getForce(body2).isApprox(Eigen::Vector3d(0.0, 5.0, 0.0)));
    }
    
    SECTION("Set force and torque") {
        force_form.addBody(body1);
        
        // 同时设置力和力矩
        force_form.setForceAndTorque(
            body1,
            Eigen::Vector3d(1.0, 2.0, 3.0),   // 力
            Eigen::Vector3d(0.1, 0.2, 0.3),   // 力矩
            Eigen::Vector3d::Zero()            // 作用点
        );
        
        REQUIRE(force_form.getForce(body1).isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
        REQUIRE(force_form.getTorque(body1).isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
    }
    
    SECTION("Clear forces") {
        force_form.addBody(body1);
        force_form.setForce(body1, Eigen::Vector3d(10.0, 0.0, 0.0));
        force_form.setTorque(body1, Eigen::Vector3d(0.0, 1.0, 0.0));
        
        // 清除力
        force_form.clearForces(body1);
        
        REQUIRE(force_form.getForce(body1).isApprox(Eigen::Vector3d::Zero()));
        REQUIRE(force_form.getTorque(body1).isApprox(Eigen::Vector3d::Zero()));
    }
    
    SECTION("Gradient computation") {
        // 创建两个刚体
        std::vector<std::shared_ptr<RigidBody>> bodies = {body1, body2};
        
        force_form.addBody(body1);
        force_form.addBody(body2);
        
        // 设置力
        force_form.setForce(body1, Eigen::Vector3d(10.0, 0.0, 0.0));  // X方向10N
        force_form.setForce(body2, Eigen::Vector3d(0.0, 5.0, 0.0));   // Y方向5N
        force_form.setTorque(body1, Eigen::Vector3d(0.0, 0.0, 2.0));  // Z方向2Nm
        
        // 更新全局索引
        force_form.updateGlobalIndices(bodies);
        
        // 计算梯度
        Eigen::VectorXd grad(12);  // 2个刚体 * 6DOF
        grad.setZero();
        
        Eigen::VectorXd x(12);  // 虚拟状态向量
        x.setZero();
        
        force_form.gradient(x, grad);
        
        // 验证梯度：grad = -force
        // body1: idx 0-2 是位置，idx 3-5 是旋转
        REQUIRE(grad(0) == Approx(-10.0));  // -Fx
        REQUIRE(grad(1) == Approx(0.0));
        REQUIRE(grad(2) == Approx(0.0));
        REQUIRE(grad(3) == Approx(0.0));
        REQUIRE(grad(4) == Approx(0.0));
        REQUIRE(grad(5) == Approx(-2.0));   // -Tz
        
        // body2: idx 6-11
        REQUIRE(grad(6) == Approx(0.0));
        REQUIRE(grad(7) == Approx(-5.0));   // -Fy
        REQUIRE(grad(8) == Approx(0.0));
    }
}

TEST_CASE("ExternalForceForm - Force application simulation", "[force_control][simulation]") {
    // 这个测试演示如何使用力控制
    // 在实际仿真中，力会通过 ExternalForceForm 施加到刚体上
    
    ExternalForceForm force_form;
    auto body = std::make_shared<RigidBody>();
    body->id = 0;
    body->mass = 1.0;  // 1kg
    body->position = Eigen::Vector3d::Zero();
    body->velocity = Eigen::Vector3d::Zero();
    
    std::vector<std::shared_ptr<RigidBody>> bodies = {body};
    force_form.addBody(body);
    force_form.updateGlobalIndices(bodies);
    
    SECTION("Constant force produces constant acceleration (F=ma)") {
        // 施加恒定力 F = 10N (X方向)
        force_form.setForce(body, Eigen::Vector3d(10.0, 0.0, 0.0));
        
        // 计算梯度
        Eigen::VectorXd grad(6);
        grad.setZero();
        Eigen::VectorXd x(6);
        x.setZero();
        force_form.gradient(x, grad);
        
        // 力 = -grad = 10N
        // 加速度 a = F/m = 10/1 = 10 m/s²
        REQUIRE(-grad(0) == Approx(10.0));
        
        // 在实际仿真中，这个梯度会被牛顿求解器使用
        // 计算新的速度和位置
    }
    
    SECTION("Time-varying force") {
        // 模拟时变力：F(t) = sin(t)
        double t = 0.0;
        double dt = 0.1;
        
        for (int i = 0; i < 10; ++i) {
            double force_magnitude = 10.0 * std::sin(t);
            force_form.setForce(body, Eigen::Vector3d(force_magnitude, 0.0, 0.0));
            
            // 验证力已设置
            REQUIRE(force_form.getForce(body).x() == Approx(force_magnitude));
            
            t += dt;
        }
    }
}

TEST_CASE("ExternalForceForm - Multiple bodies with different forces", "[force_control]") {
    ExternalForceForm force_form;
    
    // 创建多个刚体
    auto box = std::make_shared<RigidBody>();
    box->id = 0;
    box->mass = 5.0;
    
    auto sphere = std::make_shared<RigidBody>();
    sphere->id = 1;
    sphere->mass = 2.0;
    
    auto cylinder = std::make_shared<RigidBody>();
    cylinder->id = 2;
    cylinder->mass = 3.0;
    
    std::vector<std::shared_ptr<RigidBody>> bodies = {box, sphere, cylinder};
    
    // 添加所有刚体
    for (auto& body : bodies) {
        force_form.addBody(body);
    }
    force_form.updateGlobalIndices(bodies);
    
    SECTION("Different forces on different bodies") {
        // 对不同的刚体施加不同的力
        force_form.setForce(box, Eigen::Vector3d(10.0, 0.0, 0.0));      // X方向推力
        force_form.setForce(sphere, Eigen::Vector3d(0.0, -9.8 * 2.0, 0.0));  // 重力补偿
        force_form.setForceAndTorque(
            cylinder,
            Eigen::Vector3d(0.0, 0.0, 5.0),   // Z方向推力
            Eigen::Vector3d(0.0, 2.0, 0.0),   // Y方向力矩
            Eigen::Vector3d::Zero()
        );
        
        // 验证所有力
        REQUIRE(force_form.getForce(box).isApprox(Eigen::Vector3d(10.0, 0.0, 0.0)));
        REQUIRE(force_form.getForce(sphere).isApprox(Eigen::Vector3d(0.0, -19.6, 0.0)));
        REQUIRE(force_form.getForce(cylinder).isApprox(Eigen::Vector3d(0.0, 0.0, 5.0)));
        REQUIRE(force_form.getTorque(cylinder).isApprox(Eigen::Vector3d(0.0, 2.0, 0.0)));
        
        // 计算总梯度
        Eigen::VectorXd grad(18);  // 3 bodies * 6 DOF
        grad.setZero();
        Eigen::VectorXd x(18);
        x.setZero();
        
        force_form.gradient(x, grad);
        
        // box: idx 0-5
        REQUIRE(grad(0) == Approx(-10.0));
        
        // sphere: idx 6-11
        REQUIRE(grad(7) == Approx(19.6));  // -(-19.6) = 19.6
        
        // cylinder: idx 12-17
        REQUIRE(grad(14) == Approx(-5.0));   // Fz
        REQUIRE(grad(16) == Approx(-2.0));   // Ty
    }
}
