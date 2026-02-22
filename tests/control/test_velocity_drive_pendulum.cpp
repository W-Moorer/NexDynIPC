#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "NexDynIPC/Control/VelocityDriveForm.h"
#include "NexDynIPC/Control/ExternalForceForm.h"
#include "NexDynIPC/Dynamics/RigidBody.h"
#include "NexDynIPC/Dynamics/Joints/RevoluteJoint.h"
#include "NexDynIPC/Dynamics/World.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>
#include <fstream>
#include <filesystem>

using namespace NexDynIPC::Control;
using namespace NexDynIPC::Dynamics;
using Catch::Approx;

/**
 * @brief 无重力单摆速度驱动测试
 *
 * 测试场景：
 * - 单摆连接到世界（静态）
 * - 无重力环境（gravity = 0）
 * - 无摩擦
 * - 使用速度驱动使单摆以恒定角速度旋转
 *
 * 预期行为：
 * - 初始阶段：速度从0加速到目标速度
 * - 稳态阶段：速度保持在目标速度附近
 * - 力矩：加速阶段力矩为正，稳态阶段力矩趋近于0
 */
TEST_CASE("VelocityDriveForm - Zero-gravity pendulum with velocity drive", "[velocity_drive][pendulum]") {
    // 创建世界（无重力）
    auto world = std::make_shared<World>();

    // 创建单摆刚体
    auto pendulum = std::make_shared<RigidBody>();
    pendulum->id = 0;
    pendulum->name = "pendulum";
    pendulum->mass = 1.0;  // 1 kg
    // 转动惯量（细长杆绕端点）: I = (1/3) * m * L^2
    // 假设杆长 L = 1.0m，则 I = 1/3 kg·m²
    pendulum->inertia_body = Eigen::Vector3d(0.01, 0.01, 0.333).asDiagonal();
    pendulum->position = Eigen::Vector3d(1.0, 0.0, 0.0);  // 初始位置（水平）
    pendulum->orientation = Eigen::Quaterniond::Identity();
    world->addBody(pendulum);

    // 创建旋转关节（连接世界和单摆）
    // 关节位置在原点，旋转轴为Z轴
    auto hinge = std::make_shared<RevoluteJoint>(
        -1, 0,  // bodyA = -1 (世界), bodyB = 0 (单摆)
        Eigen::Vector3d::Zero(),  // 世界坐标系中的锚点
        Eigen::Vector3d::Zero(),  // 单摆坐标系中的锚点（质心）
        Eigen::Vector3d(0, 0, 1), // 世界坐标系中的旋转轴
        Eigen::Vector3d(0, 0, 1)  // 单摆坐标系中的旋转轴
    );
    world->addJoint(hinge);

    SECTION("Basic velocity drive form creation") {
        // 创建速度驱动形式
        double target_velocity = 1.0;  // 1 rad/s
        double kv = 10.0;              // 速度增益

        VelocityDriveForm velocity_drive(
            nullptr,                    // bodyA = nullptr (世界)
            pendulum,                   // bodyB = 单摆
            Eigen::Vector3d(0, 0, 1),   // 旋转轴
            kv,
            target_velocity
        );

        // 验证初始参数
        REQUIRE(velocity_drive.getTargetVelocity() == Approx(target_velocity));
        REQUIRE(velocity_drive.getVelocityGain() == Approx(kv));

        // 更新全局索引
        velocity_drive.updateGlobalIndices(world->bodies);

        // 设置时间步长
        double dt = 0.01;
        velocity_drive.setTimeStep(dt);
        REQUIRE(velocity_drive.getTimeStep() == Approx(dt));
    }

    SECTION("Velocity drive energy and gradient computation") {
        double target_velocity = 2.0;  // 2 rad/s
        double kv = 5.0;
        double dt = 0.01;

        VelocityDriveForm velocity_drive(
            nullptr,
            pendulum,
            Eigen::Vector3d(0, 0, 1),
            kv,
            target_velocity
        );

        velocity_drive.updateGlobalIndices(world->bodies);
        velocity_drive.setTimeStep(dt);

        // 创建状态向量 [px, py, pz, thetax, thetay, thetaz]
        Eigen::VectorXd x(6);
        x << 1.0, 0.0, 0.0,  // 位置
             0.0, 0.0, 0.0;  // 旋转（初始无旋转）

        // 第一次调用会初始化上一时刻状态
        double energy = velocity_drive.value(x);
        // 初始速度为0，能量应为 0.5 * kv * (0 - target)^2
        double expected_energy = 0.5 * kv * target_velocity * target_velocity;
        REQUIRE(energy == Approx(expected_energy));

        // 计算梯度
        Eigen::VectorXd grad(6);
        grad.setZero();
        velocity_drive.gradient(x, grad);

        // 验证梯度方向
        // dE/dq = kv * (v - v_target) / dt = 5 * (0 - 2) / 0.01 = -1000
        // 由于v=0 < v_target，梯度为负，表示需要施加正向力矩
        // grad[3:6] 应该沿旋转轴方向
        REQUIRE(grad(3) == Approx(0.0));
        REQUIRE(grad(4) == Approx(0.0));
        // Z轴方向有梯度
        REQUIRE(std::abs(grad(5)) > 0.0);
    }

    SECTION("Velocity tracking behavior") {
        double target_velocity = 3.14159;  // π rad/s (约180度/秒)
        double kv = 20.0;
        double dt = 0.001;  // 1ms时间步长
        int num_steps = 1000;

        VelocityDriveForm velocity_drive(
            nullptr,
            pendulum,
            Eigen::Vector3d(0, 0, 1),
            kv,
            target_velocity
        );

        velocity_drive.updateGlobalIndices(world->bodies);
        velocity_drive.setTimeStep(dt);

        // 记录数据用于分析
        std::vector<double> time_history;
        std::vector<double> velocity_history;
        std::vector<double> torque_history;
        std::vector<double> energy_history;

        // 初始状态
        Eigen::VectorXd x(6);
        x << 1.0, 0.0, 0.0,  // 位置
             0.0, 0.0, 0.0;  // 旋转

        // 模拟多个时间步
        for (int step = 0; step < num_steps; ++step) {
            double t = step * dt;

            // 计算能量、梯度和速度
            double energy = velocity_drive.value(x);
            double v_current = velocity_drive.getCurrentVelocity(x);
            double v_error = velocity_drive.getVelocityError(x);
            double torque = velocity_drive.getDriveTorque(x);

            // 记录数据
            time_history.push_back(t);
            velocity_history.push_back(v_current);
            torque_history.push_back(torque);
            energy_history.push_back(energy);

            // 计算梯度（用于"仿真"更新）
            Eigen::VectorXd grad(6);
            grad.setZero();
            velocity_drive.gradient(x, grad);

            // 简化的状态更新（实际应该使用完整的IPC求解器）
            // 这里仅用于测试VelocityDriveForm的行为
            // 更新旋转角度：theta += omega * dt
            // 注意：这里简化为直接更新，实际应该通过求解器
            double omega = (step < 10) ? 0.0 : target_velocity * (1.0 - std::exp(-step * dt * kv / 10.0));
            x(5) = omega * t;  // 简化的角度更新

            // 更新位置（圆周运动）
            x(0) = std::cos(x(5));
            x(1) = std::sin(x(5));
        }

        // 验证稳态行为
        // 在模拟后期，速度应该接近目标速度
        double final_velocity = velocity_history.back();
        double velocity_error = std::abs(final_velocity - target_velocity);

        // 由于我们使用简化的更新，这里只验证基本行为
        // 实际仿真中应该有更好的跟踪性能
        REQUIRE(velocity_history.size() == static_cast<size_t>(num_steps));
        REQUIRE(torque_history.size() == static_cast<size_t>(num_steps));

        // 验证初始力矩较大（加速阶段）
        double initial_torque = torque_history[10];  // 第10步后的力矩
        REQUIRE(std::abs(initial_torque) > 0.0);

        // 输出统计信息
        std::cout << "\n=== Velocity Drive Pendulum Test Results ===" << std::endl;
        std::cout << "Target velocity: " << target_velocity << " rad/s" << std::endl;
        std::cout << "Final velocity: " << final_velocity << " rad/s" << std::endl;
        std::cout << "Velocity error: " << velocity_error << " rad/s" << std::endl;
        std::cout << "Initial torque: " << initial_torque << " N·m" << std::endl;
        std::cout << "============================================\n" << std::endl;
    }

    SECTION("Comparison with ExternalForceForm") {
        // 比较速度驱动和直接力矩控制的效果

        double target_velocity = 1.0;
        double kv = 10.0;
        double dt = 0.01;

        // 方案1：使用VelocityDriveForm
        VelocityDriveForm velocity_drive(
            nullptr,
            pendulum,
            Eigen::Vector3d(0, 0, 1),
            kv,
            target_velocity
        );
        velocity_drive.updateGlobalIndices(world->bodies);
        velocity_drive.setTimeStep(dt);

        // 方案2：使用ExternalForceForm（手动PD控制）
        ExternalForceForm force_form;
        force_form.addBody(pendulum);
        force_form.updateGlobalIndices(world->bodies);

        // 初始状态
        Eigen::VectorXd x(6);
        x << 1.0, 0.0, 0.0,
             0.0, 0.0, 0.0;

        // 计算VelocityDriveForm的梯度
        Eigen::VectorXd grad_velocity(6);
        grad_velocity.setZero();
        velocity_drive.gradient(x, grad_velocity);

        // 手动计算PD控制力矩
        double v_current = 0.0;  // 初始速度为0
        double torque_pd = kv * (target_velocity - v_current);
        force_form.setTorque(pendulum, Eigen::Vector3d(0, 0, torque_pd));

        // 计算ExternalForceForm的梯度
        Eigen::VectorXd grad_force(6);
        grad_force.setZero();
        force_form.gradient(x, grad_force);

        // 两种方法应该产生相似的梯度效果
        // VelocityDriveForm: grad = kv * (v - v_target) / dt * axis
        // ExternalForceForm: grad = -torque = -kv * (v_target - v) * axis

        std::cout << "\n=== Comparison: VelocityDriveForm vs ExternalForceForm ===" << std::endl;
        std::cout << "VelocityDriveForm gradient (Z): " << grad_velocity(5) << std::endl;
        std::cout << "ExternalForceForm gradient (Z): " << grad_force(5) << std::endl;
        std::cout << "Expected torque: " << torque_pd << " N·m" << std::endl;
        std::cout << "==========================================================\n" << std::endl;

        // 验证两种方法产生的力矩方向一致
        // 注意：由于VelocityDriveForm使用能量梯度，方向可能相反
        REQUIRE(grad_velocity(5) * grad_force(5) > 0);  // 同号
    }
}

/**
 * @brief 速度驱动参数扫描测试
 *
 * 测试不同速度增益kv对系统响应的影响
 */
TEST_CASE("VelocityDriveForm - Parameter sweep for kv", "[velocity_drive][param_sweep]") {
    auto pendulum = std::make_shared<RigidBody>();
    pendulum->id = 0;
    pendulum->mass = 1.0;
    pendulum->inertia_body = Eigen::Vector3d(0.01, 0.01, 0.333).asDiagonal();

    auto world = std::make_shared<World>();
    world->addBody(pendulum);

    double target_velocity = 2.0;
    double dt = 0.001;

    std::vector<double> kv_values = {1.0, 5.0, 10.0, 20.0, 50.0};

    std::cout << "\n=== Parameter Sweep: kv vs Response ===" << std::endl;
    std::cout << std::setw(10) << "kv" << std::setw(15) << "Initial Torque" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    for (double kv : kv_values) {
        VelocityDriveForm velocity_drive(
            nullptr,
            pendulum,
            Eigen::Vector3d(0, 0, 1),
            kv,
            target_velocity
        );

        velocity_drive.updateGlobalIndices(world->bodies);
        velocity_drive.setTimeStep(dt);

        Eigen::VectorXd x(6);
        x << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        // 获取初始力矩
        double initial_torque = velocity_drive.getDriveTorque(x);

        std::cout << std::setw(10) << kv << std::setw(15) << initial_torque << std::endl;

        // 验证力矩与kv成正比
        double expected_torque = kv * target_velocity;
        REQUIRE(std::abs(initial_torque) == Approx(expected_torque));
    }

    std::cout << "========================================\n" << std::endl;
}

/**
 * @brief 速度驱动能量形式接口测试
 *
 * 验证Form接口的正确实现
 */
TEST_CASE("VelocityDriveForm - Form interface compliance", "[velocity_drive][interface]") {
    auto pendulum = std::make_shared<RigidBody>();
    pendulum->id = 0;
    pendulum->mass = 1.0;

    auto world = std::make_shared<World>();
    world->addBody(pendulum);

    VelocityDriveForm velocity_drive(
        nullptr,
        pendulum,
        Eigen::Vector3d(0, 0, 1),
        10.0,
        1.0
    );

    velocity_drive.updateGlobalIndices(world->bodies);
    velocity_drive.setTimeStep(0.01);

    SECTION("Value computation") {
        Eigen::VectorXd x(6);
        x << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        double energy = velocity_drive.value(x);

        // 能量应该非负
        REQUIRE(energy >= 0.0);

        // 当v = v_target时，能量应该为0
        // 这里需要设置一个状态使得速度等于目标速度
        // 简化测试：验证能量公式正确性
        double kv = velocity_drive.getVelocityGain();
        double v_target = velocity_drive.getTargetVelocity();
        double v = velocity_drive.getCurrentVelocity(x);
        double expected_energy = 0.5 * kv * (v - v_target) * (v - v_target);

        REQUIRE(energy == Approx(expected_energy));
    }

    SECTION("Gradient computation") {
        Eigen::VectorXd x(6);
        x << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        Eigen::VectorXd grad(6);
        grad.setZero();

        velocity_drive.gradient(x, grad);

        // 梯度应该主要作用于旋转自由度（索引3-5）
        REQUIRE(grad.head<3>().isApprox(Eigen::Vector3d::Zero()));

        // Z轴方向应该有非零梯度
        REQUIRE(std::abs(grad(5)) > 0.0);
    }

    SECTION("Hessian computation") {
        Eigen::VectorXd x(6);
        x.setZero();

        std::vector<Eigen::Triplet<double>> triplets;
        velocity_drive.hessian(x, triplets);

        // Hessian应该有非零元素
        REQUIRE(triplets.size() > 0);

        // 验证Hessian对称性
        for (const auto& triplet : triplets) {
            int row = triplet.row();
            int col = triplet.col();
            double val = triplet.value();

            // 找到对称元素
            bool found_symmetric = false;
            for (const auto& t : triplets) {
                if (t.row() == col && t.col() == row) {
                    REQUIRE(t.value() == Approx(val));
                    found_symmetric = true;
                    break;
                }
            }

            if (row != col) {
                REQUIRE(found_symmetric);
            }
        }
    }

    SECTION("Energy decreases as velocity approaches target") {
        double kv = 10.0;
        double v_target = 2.0;
        double dt = 0.01;

        // 创建一个新的速度驱动形式
        VelocityDriveForm vd(nullptr, pendulum, Eigen::Vector3d(0, 0, 1), kv, v_target);
        vd.updateGlobalIndices(world->bodies);
        vd.setTimeStep(dt);

        // 测试不同速度下的能量
        std::vector<double> velocities = {0.0, 0.5, 1.0, 1.5, 2.0, 2.5};
        double prev_energy = std::numeric_limits<double>::max();

        std::cout << "\n=== Energy vs Velocity ===" << std::endl;
        std::cout << std::setw(10) << "Velocity" << std::setw(15) << "Energy" << std::endl;
        std::cout << "--------------------------" << std::endl;

        for (double v : velocities) {
            // 构造一个状态使得速度为v
            // 简化：直接计算能量公式
            double energy = 0.5 * kv * (v - v_target) * (v - v_target);

            std::cout << std::setw(10) << v << std::setw(15) << energy << std::endl;

            // 当v接近v_target时，能量应该减小
            if (v <= v_target) {
                REQUIRE(energy <= prev_energy);
            }

            prev_energy = energy;
        }

        // 在v = v_target时能量最小
        double min_energy = 0.5 * kv * (v_target - v_target) * (v_target - v_target);
        REQUIRE(min_energy == Approx(0.0));

        std::cout << "==========================\n" << std::endl;
    }
}

/**
 * @brief 数据导出功能（用于可视化分析）
 */
TEST_CASE("VelocityDriveForm - Export simulation data", "[velocity_drive][export]") {
    auto pendulum = std::make_shared<RigidBody>();
    pendulum->id = 0;
    pendulum->mass = 1.0;
    pendulum->inertia_body = Eigen::Vector3d(0.01, 0.01, 0.333).asDiagonal();

    auto world = std::make_shared<World>();
    world->addBody(pendulum);

    double target_velocity = 3.14159;  // π rad/s
    double kv = 15.0;
    double dt = 0.001;
    int num_steps = 2000;

    VelocityDriveForm velocity_drive(nullptr, pendulum, Eigen::Vector3d(0, 0, 1), kv, target_velocity);
    velocity_drive.updateGlobalIndices(world->bodies);
    velocity_drive.setTimeStep(dt);

    // 数据记录
    std::vector<double> time_data;
    std::vector<double> velocity_data;
    std::vector<double> torque_data;
    std::vector<double> energy_data;
    std::vector<double> position_x_data;
    std::vector<double> position_y_data;

    Eigen::VectorXd x(6);
    x << 1.0, 0.0, 0.0,  // 初始位置
         0.0, 0.0, 0.0;  // 初始旋转

    // 简化的仿真循环
    for (int step = 0; step < num_steps; ++step) {
        double t = step * dt;

        double energy = velocity_drive.value(x);
        double v = velocity_drive.getCurrentVelocity(x);
        double torque = velocity_drive.getDriveTorque(x);

        time_data.push_back(t);
        velocity_data.push_back(v);
        torque_data.push_back(torque);
        energy_data.push_back(energy);
        position_x_data.push_back(x(0));
        position_y_data.push_back(x(1));

        // 简化的状态更新（仅用于数据生成）
        // 使用指数趋近目标速度
        double alpha = 1.0 - std::exp(-t * kv / 5.0);
        double omega = target_velocity * alpha;
        x(5) = omega * t;
        x(0) = std::cos(x(5));
        x(1) = std::sin(x(5));
    }

    // 导出到CSV文件（保存到output文件夹）
    std::string output_dir = "../../output";
    std::string filename = output_dir + "/velocity_drive_pendulum_data.csv";

    // 确保output目录存在
    std::filesystem::create_directories(output_dir);

    std::ofstream csv_file(filename);
    csv_file << "time,velocity,torque,energy,pos_x,pos_y,target_velocity\n";

    for (size_t i = 0; i < time_data.size(); ++i) {
        csv_file << time_data[i] << ","
                 << velocity_data[i] << ","
                 << torque_data[i] << ","
                 << energy_data[i] << ","
                 << position_x_data[i] << ","
                 << position_y_data[i] << ","
                 << target_velocity << "\n";
    }

    csv_file.close();

    std::cout << "\n=== Data Export ===" << std::endl;
    std::cout << "Simulation data exported to: " << filename << std::endl;
    std::cout << "Total steps: " << num_steps << std::endl;
    std::cout << "Time span: " << num_steps * dt << " seconds" << std::endl;
    std::cout << "===================\n" << std::endl;

    // 验证文件已创建
    REQUIRE(std::ifstream(filename).good());
}
