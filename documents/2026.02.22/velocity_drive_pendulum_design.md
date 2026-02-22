# NexDynIPC 速度驱动与力驱动实现方案

## 基于 MuJoCo 架构的无重力单摆测试设计

---

## 目录
1. [设计目标与参考架构](#1-设计目标与参考架构)
2. [MuJoCo 速度驱动机制分析](#2-mujoco-速度驱动机制分析)
3. [NexDynIPC 速度驱动架构设计](#3-nexdynipc-速度驱动架构设计)
4. [无重力单摆测试实现](#4-无重力单摆测试实现)
5. [测试验证方案](#5-测试验证方案)
6. [参考代码实现](#6-参考代码实现)

---

## 1. 设计目标与参考架构

### 1.1 设计目标

实现一个**无重力环境下的单摆系统**，支持：
1. **速度驱动**：通过 PD 控制器使单摆以指定角速度旋转
2. **力矩驱动**：直接施加力矩控制单摆运动
3. **自由转动**：无重力、无摩擦的理想环境

### 1.2 参考架构（MuJoCo）

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         MuJoCo 执行器系统架构                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   XML 定义：                                                                 │
│   <motor name="drive" joint="hinge" gear="1500" ctrlrange="-1 1"/>         │
│                                                                             │
│   计算流程：                                                                 │
│   ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐             │
│   │  ctrl    │───▶│  gain    │───▶│  force   │───▶│  torque  │             │
│   │ (控制)   │    │ (增益)   │    │ (力)     │    │ (力矩)   │             │
│   └──────────┘    └──────────┘    └──────────┘    └──────────┘             │
│        │               │               │               │                   │
│        ▼               ▼               ▼               ▼                   │
│   速度误差计算    gain = kp*length    force = gain    torque = gear         │
│   或位置误差           + kv*velocity         *ctrl           *force         │
│                                                                             │
│   数学模型：                                                                 │
│   • 位置控制：ctrl = kp*(q_target - q) + kv*(v_target - v)                  │
│   • 速度控制：ctrl = kv*(v_target - v)                                      │
│   • 力矩输出：τ = gear * (gain * ctrl + bias)                               │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. MuJoCo 速度驱动机制分析

### 2.1 核心数学模型

#### 速度驱动 PD 控制
```
控制输入计算：
    ctrl = kv * (v_target - v_current)

执行器力计算：
    force = gain * ctrl + bias
    
关节力矩计算：
    torque = gear_ratio * force

其中：
    kv: 速度增益（阻尼系数）
    v_target: 目标角速度
    v_current: 当前角速度
    gear_ratio: 传动比/齿轮比
```

#### MuJoCo 代码实现（参考）

```c
// mj_fwdActuation - 执行器力计算
void mj_fwdActuation(const mjModel* m, mjData* d) {
    for (int i = 0; i < nu; i++) {
        // 1. 计算增益（Affine 类型）
        // gain = prm[0] + prm[1]*length + prm[2]*velocity
        double gain = prm[0] + prm[1]*d->actuator_length[i] 
                           + prm[2]*d->actuator_velocity[i];
        
        // 2. 计算偏置
        // bias = prm[0] + prm[1]*length + prm[2]*velocity
        double bias = bias_prm[0] + bias_prm[1]*d->actuator_length[i] 
                                  + bias_prm[2]*d->actuator_velocity[i];
        
        // 3. 计算执行器力
        // force = gain * ctrl + bias
        double force = gain * ctrl[i] + bias;
        
        // 4. 映射到关节空间
        // qfrc_actuator = moment' * force
        // 对于 hinge 关节：moment = gear[0]
        d->qfrc_actuator[dof_adr] = gear[0] * force;
    }
}
```

### 2.2 传动映射系统

```c
// mj_transmission - 传动计算
void mj_transmission(const mjModel* m, mjData* d) {
    for (int i = 0; i < nu; i++) {
        int id = m->actuator_trnid[2*i];  // 关节 ID
        double* gear = m->actuator_gear + 6*i;
        
        if (m->jnt_type[id] == mjJNT_HINGE) {
            // 标量传动
            // length = gear * qpos (传动长度)
            length[i] = d->qpos[m->jnt_qposadr[id]] * gear[0];
            
            // moment = gear (力矩臂)
            moment[adr] = gear[0];
        }
    }
}
```

---

## 3. NexDynIPC 速度驱动架构设计

### 3.1 架构设计

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      NexDynIPC 速度驱动架构                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                     VelocityController                               │   │
│  │  速度控制器：实现 PD 速度控制，参考 MuJoCo 的 gain/bias 分离          │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                              │                                              │
│           ┌──────────────────┼──────────────────┐                          │
│           ▼                  ▼                  ▼                          │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐                   │
│  │  Gain        │   │  Bias        │   │ Transmission │                   │
│  │  增益计算     │   │  偏置计算     │   │ 传动映射      │                   │
│  └──────────────┘   └──────────────┘   └──────────────┘                   │
│           │                  │                  │                          │
│           └──────────────────┼──────────────────┘                          │
│                              ▼                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                     JointActuator                                    │   │
│  │  关节驱动器：集成到现有系统，支持 TORQUE/POSITION/VELOCITY 模式      │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                              │                                              │
│                              ▼                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                     RevoluteJoint                                    │   │
│  │  旋转关节：通过 ExternalForceForm 或约束乘子施加力矩                 │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 核心组件设计

#### 3.2.1 VelocityController（新增）

```cpp
// include/NexDynIPC/Control/VelocityController.h
#pragma once

#include <Eigen/Core>

namespace NexDynIPC::Control {

/**
 * @brief 速度控制器：实现 PD 速度控制
 * 
 * 参考 MuJoCo 的 mjGAIN_AFFINE 和 mjBIAS_AFFINE 设计
 * 
 * 控制律：
 *   ctrl = kv * (v_target - v_current)
 *   force = gain * ctrl + bias
 *   torque = gear_ratio * force
 */
class VelocityController {
public:
    VelocityController();
    ~VelocityController() = default;

    /**
     * @brief 设置速度增益 kv
     */
    void setVelocityGain(double kv) { kv_ = kv; }
    double getVelocityGain() const { return kv_; }

    /**
     * @brief 设置目标速度
     */
    void setTargetVelocity(double velocity) { target_velocity_ = velocity; }
    double getTargetVelocity() const { return target_velocity_; }

    /**
     * @brief 设置传动比（齿轮比）
     */
    void setGearRatio(double ratio) { gear_ratio_ = ratio; }
    double getGearRatio() const { return gear_ratio_; }

    /**
     * @brief 设置增益参数（Affine 类型）
     * @param c0 常数项
     * @param c1 位置系数（通常用于位置控制）
     * @param c2 速度系数（速度控制主要用这个）
     * 
     * gain = c0 + c1 * length + c2 * velocity
     */
    void setGainParams(double c0, double c1, double c2) {
        gain_c0_ = c0;
        gain_c1_ = c1;
        gain_c2_ = c2;
    }

    /**
     * @brief 设置偏置参数（Affine 类型）
     * bias = c0 + c1 * length + c2 * velocity
     */
    void setBiasParams(double c0, double c1, double c2) {
        bias_c0_ = c0;
        bias_c1_ = c1;
        bias_c2_ = c2;
    }

    /**
     * @brief 计算控制力矩
     * @param current_position 当前关节位置
     * @param current_velocity 当前关节速度
     * @return 控制力矩
     */
    double computeTorque(double current_position, double current_velocity) const;

    /**
     * @brief 计算控制流程（调试用）
     */
    struct ControlOutput {
        double ctrl;           // 控制输入
        double gain;           // 增益值
        double bias;           // 偏置值
        double force;          // 执行器力
        double torque;         // 最终力矩
    };
    ControlOutput computeDetailed(double current_position, double current_velocity) const;

private:
    // 控制参数
    double kv_ = 10.0;           // 速度增益（类似 MuJoCo 的 kv）
    double target_velocity_ = 0.0; // 目标速度
    double gear_ratio_ = 1.0;    // 传动比

    // 增益参数（Affine: c0 + c1*length + c2*velocity）
    double gain_c0_ = 0.0;
    double gain_c1_ = 0.0;
    double gain_c2_ = 1.0;  // 速度控制时主要用这个

    // 偏置参数（Affine）
    double bias_c0_ = 0.0;
    double bias_c1_ = 0.0;
    double bias_c2_ = 0.0;
};

} // namespace NexDynIPC::Control
```

#### 3.2.2 改进的 JointActuator

```cpp
// include/NexDynIPC/Control/JointActuator.h（改进版）
#pragma once

#include "NexDynIPC/Dynamics/Joints/Joint.h"
#include "NexDynIPC/Control/VelocityController.h"
#include <memory>

namespace NexDynIPC::Control {

/**
 * @brief 关节驱动器：支持力矩/位置/速度控制
 * 
 * 参考 MuJoCo 的 actuator 系统设计：
 * - 传动映射（Transmission）
 * - 增益/偏置分离（Gain/Bias）
 * - 多种控制模式
 */
class JointActuator {
public:
    enum class ControlMode {
        TORQUE_CONTROL,    // 直接力矩控制
        POSITION_CONTROL,  // PD位置控制
        VELOCITY_CONTROL   // PD速度控制（参考 MuJoCo）
    };

    JointActuator(std::shared_ptr<Dynamics::Joint> joint);
    ~JointActuator() = default;

    // ========== 控制模式 ==========
    void setControlMode(ControlMode mode);
    ControlMode getControlMode() const { return mode_; }

    // ========== 力矩控制 ==========
    void setTargetTorque(double torque);
    double getTargetTorque() const { return target_torque_; }

    // ========== 位置控制 ==========
    void setTargetPosition(double position);
    double getTargetPosition() const { return target_position_; }

    // ========== 速度控制（新增）==========
    void setTargetVelocity(double velocity);
    double getTargetVelocity() const { return target_velocity_; }
    
    /**
     * @brief 获取速度控制器（用于高级配置）
     */
    VelocityController& getVelocityController() { return velocity_controller_; }
    const VelocityController& getVelocityController() const { return velocity_controller_; }

    // ========== PD 参数 ==========
    void setPDGains(double kp, double kd);
    double getKp() const { return kp_; }
    double getKd() const { return kd_; }

    // ========== 传动比（参考 MuJoCo gear）==========
    void setGearRatio(double ratio);
    double getGearRatio() const { return gear_ratio_; }

    // ========== 力矩限制 ==========
    void setTorqueLimits(double min_torque, double max_torque);
    double getMinTorque() const { return min_torque_; }
    double getMaxTorque() const { return max_torque_; }

    // ========== 状态获取 ==========
    double getCurrentPosition() const;
    double getCurrentVelocity() const;

    // ========== 控制输出计算 ==========
    /**
     * @brief 计算并应用控制输出
     * @return 实际应用的力矩
     * 
     * 计算流程（参考 MuJoCo）：
     * 1. 根据控制模式计算控制输入 ctrl
     * 2. 计算增益 gain 和偏置 bias
     * 3. 计算执行器力 force = gain * ctrl + bias
     * 4. 应用传动比 torque = gear_ratio * force
     * 5. 力矩饱和处理
     */
    double computeControlOutput();

    std::shared_ptr<Dynamics::Joint> getJoint() const { return joint_; }

    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }

private:
    std::shared_ptr<Dynamics::Joint> joint_;
    ControlMode mode_ = ControlMode::TORQUE_CONTROL;
    
    // 目标值
    double target_torque_ = 0.0;
    double target_position_ = 0.0;
    double target_velocity_ = 0.0;
    
    // PD 参数
    double kp_ = 100.0;
    double kd_ = 10.0;
    
    // 传动比（参考 MuJoCo gear）
    double gear_ratio_ = 1.0;
    
    // 力矩限制
    double min_torque_ = -std::numeric_limits<double>::max();
    double max_torque_ = std::numeric_limits<double>::max();
    
    // 速度控制器（VELOCITY_CONTROL 模式使用）
    VelocityController velocity_controller_;
    
    bool enabled_ = true;
    
    // 辅助函数
    double saturateTorque(double torque) const;
    
    // 不同模式的计算
    double computeTorqueControl() const;
    double computePositionControl() const;
    double computeVelocityControl() const;
};

} // namespace NexDynIPC::Control
```

### 3.3 控制流程设计

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      速度驱动控制流程                                        │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  输入：目标角速度 ω_target                                                   │
│       当前角速度 ω_current                                                   │
│       当前角度 θ                                                             │
│                                                                             │
│  步骤 1：计算控制输入（类似 MuJoCo ctrl）                                     │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │ ctrl = kv * (ω_target - ω_current)                                  │   │
│  │                                                                     │   │
│  │ 物理意义：速度误差越大，控制输入越大                                  │   │
│  │          kv 越大，系统响应越快，但可能振荡                            │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                              │                                              │
│                              ▼                                              │
│  步骤 2：计算增益和偏置（Affine 类型）                                        │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │ gain = c0 + c1 * length + c2 * velocity                             │   │
│  │ bias = c0 + c1 * length + c2 * velocity                             │   │
│  │                                                                     │   │
│  │ 简化配置（纯速度控制）：                                             │   │
│  │   gain = 1.0（固定增益）                                            │   │
│  │   bias = 0.0（无偏置）                                              │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                              │                                              │
│                              ▼                                              │
│  步骤 3：计算执行器力                                                        │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │ force = gain * ctrl + bias                                          │   │
│  │       = gain * kv * (ω_target - ω_current)                          │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                              │                                              │
│                              ▼                                              │
│  步骤 4：应用传动比（齿轮比）                                                 │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │ torque = gear_ratio * force                                         │   │
│  │        = gear_ratio * gain * kv * (ω_target - ω_current)            │   │
│  │                                                                     │   │
│  │ 等效 PD 参数：                                                       │   │
│  │   Kp_velocity = gear_ratio * gain * kv                              │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                              │                                              │
│                              ▼                                              │
│  步骤 5：力矩饱和                                                            │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │ if torque > max_torque: torque = max_torque                         │   │
│  │ if torque < min_torque: torque = min_torque                         │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                              │                                              │
│                              ▼                                              │
│  输出：关节力矩 τ                                                            │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 4. 无重力单摆测试实现

### 4.1 测试场景设计

#### 场景描述
- **环境**：无重力（gravity = [0, 0, 0]）
- **单摆**：一个旋转关节连接的摆杆
- **驱动**：速度驱动，使单摆以恒定角速度旋转
- **目标**：验证速度控制器的正确性

#### 物理参数
```
摆杆参数：
- 质量：m = 1.0 kg
- 长度：L = 1.0 m
- 转动惯量：I = (1/3) * m * L^2 = 0.333 kg·m²

关节参数：
- 类型：RevoluteJoint（旋转关节）
- 轴：[0, 0, 1]（绕 Z 轴旋转）
- 阻尼：0（无摩擦）

控制参数：
- 目标角速度：ω_target = 1.0 rad/s（约 57.3 度/秒）
- 速度增益：kv = 10.0
- 传动比：gear = 1.0
```

### 4.2 测试案例代码

#### 4.2.1 场景配置文件（JSON）

```json
{
    "name": "Zero-Gravity Pendulum with Velocity Drive",
    "description": "Test velocity control on a free pendulum without gravity",
    "settings": {
        "dt": 0.01,
        "max_time": 10.0,
        "gravity": [0.0, 0.0, 0.0],
        "integrator_type": "ImplicitNewmark",
        "newmark_beta": 0.25,
        "newmark_gamma": 0.5
    },
    "bodies": [
        {
            "id": 0,
            "name": "Anchor",
            "type": "static",
            "mass": 1.0,
            "position": [0.0, 0.0, 0.0],
            "shape": {
                "type": "sphere",
                "radius": 0.05
            }
        },
        {
            "id": 1,
            "name": "Pendulum",
            "type": "dynamic",
            "mass": 1.0,
            "inertia": [0.083, 0.333, 0.333],
            "position": [0.5, 0.0, 0.0],
            "shape": {
                "type": "capsule",
                "radius": 0.05,
                "height": 1.0
            }
        }
    ],
    "joints": [
        {
            "id": 0,
            "name": "HingeJoint",
            "type": "revolute",
            "body_a": 0,
            "body_b": 1,
            "anchor_a": [0.0, 0.0, 0.0],
            "anchor_b": [-0.5, 0.0, 0.0],
            "axis_a": [0.0, 0.0, 1.0],
            "axis_b": [0.0, 0.0, 1.0],
            "damping": 0.0
        }
    ],
    "actuators": [
        {
            "id": 0,
            "name": "VelocityDrive",
            "type": "motor",
            "joint_id": 0,
            "control_mode": "velocity",
            "target_velocity": 1.0,
            "kv": 10.0,
            "gear_ratio": 1.0,
            "torque_limit": 100.0
        }
    ]
}
```

#### 4.2.2 C++ 测试代码

```cpp
// tests/test_velocity_drive_pendulum.cpp
#include <iostream>
#include <fstream>
#include <cmath>
#include <memory>
#include <vector>

#include "NexDynIPC/Dynamics/World.h"
#include "NexDynIPC/Dynamics/RigidBody.h"
#include "NexDynIPC/Dynamics/Joints/RevoluteJoint.h"
#include "NexDynIPC/Dynamics/IPCSolver.h"
#include "NexDynIPC/Control/JointActuator.h"
#include "NexDynIPC/Control/VelocityController.h"
#include "NexDynIPC/TimeIntegration/ImplicitNewmark.h"

using namespace NexDynIPC;

/**
 * @brief 无重力单摆速度驱动测试
 * 
 * 测试目标：
 * 1. 验证速度控制器能使单摆达到目标角速度
 * 2. 验证力矩驱动能正确控制单摆运动
 * 3. 在无重力环境下，单摆应能自由旋转
 */
class VelocityDrivePendulumTest {
public:
    struct TestResult {
        double final_velocity;
        double velocity_error;
        double max_torque;
        double avg_torque;
        bool passed;
        std::string message;
    };

    VelocityDrivePendulumTest() = default;

    /**
     * @brief 设置测试参数
     */
    void setup(double target_velocity = 1.0,
               double kv = 10.0,
               double gear_ratio = 1.0,
               double torque_limit = 100.0) {
        target_velocity_ = target_velocity;
        kv_ = kv;
        gear_ratio_ = gear_ratio;
        torque_limit_ = torque_limit;
    }

    /**
     * @brief 创建测试场景
     */
    void createScene() {
        // 1. 创建锚点（静态）
        auto anchor = std::make_shared<Dynamics::RigidBody>();
        anchor->id = 0;
        anchor->name = "Anchor";
        anchor->mass = 1.0;
        anchor->position = Eigen::Vector3d(0.0, 0.0, 0.0);
        anchor->velocity = Eigen::Vector3d::Zero();
        anchor->angular_velocity = Eigen::Vector3d::Zero();
        anchor->orientation = Eigen::Quaterniond::Identity();
        anchor->inertia_body = Eigen::Matrix3d::Identity() * 0.1;
        world_.addBody(anchor);

        // 2. 创建摆杆（动态）
        auto pendulum = std::make_shared<Dynamics::RigidBody>();
        pendulum->id = 1;
        pendulum->name = "Pendulum";
        pendulum->mass = 1.0;
        // 初始位置：水平向右
        pendulum->position = Eigen::Vector3d(0.5, 0.0, 0.0);
        pendulum->velocity = Eigen::Vector3d::Zero();
        pendulum->angular_velocity = Eigen::Vector3d::Zero();
        pendulum->orientation = Eigen::Quaterniond::Identity();
        // 转动惯量（细长杆绕端点）
        pendulum->inertia_body = Eigen::Vector3d(0.083, 0.333, 0.333).asDiagonal();
        world_.addBody(pendulum);

        // 3. 创建旋转关节
        auto joint = std::make_shared<Dynamics::RevoluteJoint>(
            0, 1,  // bodyA, bodyB
            Eigen::Vector3d(0.0, 0.0, 0.0),  // anchorA (局部)
            Eigen::Vector3d(-0.5, 0.0, 0.0), // anchorB (局部)
            Eigen::Vector3d(0.0, 0.0, 1.0),  // axisA (局部)
            Eigen::Vector3d(0.0, 0.0, 1.0)   // axisB (局部)
        );
        world_.addJoint(joint);

        // 4. 创建速度驱动器
        actuator_ = std::make_shared<Control::JointActuator>(joint);
        actuator_->setControlMode(Control::JointActuator::ControlMode::VELOCITY_CONTROL);
        actuator_->setTargetVelocity(target_velocity_);
        actuator_->setGearRatio(gear_ratio_);
        actuator_->setTorqueLimits(-torque_limit_, torque_limit_);
        
        // 配置速度控制器
        auto& vc = actuator_->getVelocityController();
        vc.setVelocityGain(kv_);
        vc.setTargetVelocity(target_velocity_);
        vc.setGearRatio(gear_ratio_);
        // 简化配置：固定增益 1.0，无偏置
        vc.setGainParams(0.0, 0.0, 1.0);
        vc.setBiasParams(0.0, 0.0, 0.0);

        // 5. 配置求解器
        solver_ = std::make_shared<Dynamics::IPCSolver>();
        solver_->setTimeStep(dt_);
        
        // 使用隐式 Newmark 积分器
        auto integrator = std::make_shared<TimeIntegration::ImplicitNewmark>();
        integrator->setBeta(0.25);
        integrator->setGamma(0.5);
        solver_->setIntegrator(integrator);
    }

    /**
     * @brief 运行测试
     */
    TestResult run(double duration = 5.0) {
        TestResult result;
        std::vector<double> velocities;
        std::vector<double> torques;
        std::vector<double> times;

        double t = 0.0;
        int step = 0;

        // 记录初始状态
        recordState(t);

        while (t < duration) {
            // 1. 计算控制输出
            double torque = actuator_->computeControlOutput();
            torques.push_back(torque);

            // 2. 施加力矩到关节
            applyTorqueToJoint(torque);

            // 3. 仿真步进
            solver_->step(world_, dt_);

            // 4. 记录状态
            t += dt_;
            step++;
            
            double current_vel = actuator_->getCurrentVelocity();
            velocities.push_back(current_vel);
            times.push_back(t);

            // 每 100 步输出一次
            if (step % 100 == 0) {
                std::cout << "t=" << t << "s, "
                          << "vel=" << current_vel << "rad/s, "
                          << "torque=" << torque << "Nm"
                          << std::endl;
            }

            recordState(t);
        }

        // 分析结果
        analyzeResult(velocities, torques, result);
        
        // 导出数据
        exportData(times, velocities, torques);

        return result;
    }

    /**
     * @brief 运行力矩驱动测试（对比）
     */
    TestResult runTorqueDrive(double constant_torque = 0.333, double duration = 5.0) {
        // 切换到力矩控制模式
        actuator_->setControlMode(Control::JointActuator::ControlMode::TORQUE_CONTROL);
        actuator_->setTargetTorque(constant_torque);
        
        return run(duration);
    }

private:
    Dynamics::World world_;
    std::shared_ptr<Dynamics::IPCSolver> solver_;
    std::shared_ptr<Control::JointActuator> actuator_;
    
    double dt_ = 0.01;
    double target_velocity_ = 1.0;
    double kv_ = 10.0;
    double gear_ratio_ = 1.0;
    double torque_limit_ = 100.0;
    
    std::ofstream log_file_;

    void recordState(double t) {
        if (!log_file_.is_open()) {
            log_file_.open("pendulum_velocity_drive_log.csv");
            log_file_ << "time,position,velocity,torque\n";
        }
        
        log_file_ << t << ","
                  << actuator_->getCurrentPosition() << ","
                  << actuator_->getCurrentVelocity() << ","
                  << actuator_->computeControlOutput() << "\n";
    }

    void applyTorqueToJoint(double torque) {
        // 将力矩应用到关节
        // 这里通过 ExternalForceForm 或直接修改关节约束实现
        // 具体实现取决于 NexDynIPC 的力施加机制
    }

    void analyzeResult(const std::vector<double>& velocities,
                       const std::vector<double>& torques,
                       TestResult& result) {
        // 计算最终速度（最后 1 秒的平均值）
        double sum_vel = 0.0;
        int count = 0;
        for (size_t i = velocities.size() - 100; i < velocities.size(); ++i) {
            sum_vel += velocities[i];
            count++;
        }
        result.final_velocity = sum_vel / count;
        
        // 速度误差
        result.velocity_error = std::abs(result.final_velocity - target_velocity_);
        
        // 力矩统计
        result.max_torque = *std::max_element(torques.begin(), torques.end(),
            [](double a, double b) { return std::abs(a) < std::abs(b); });
        result.avg_torque = std::accumulate(torques.begin(), torques.end(), 0.0) / torques.size();
        
        // 判断测试是否通过
        result.passed = (result.velocity_error < 0.1);  // 误差小于 10%
        result.message = result.passed ? 
            "Velocity control converged successfully" : 
            "Velocity control did not converge to target";
    }

    void exportData(const std::vector<double>& times,
                    const std::vector<double>& velocities,
                    const std::vector<double>& torques) {
        std::ofstream file("velocity_drive_data.csv");
        file << "time,velocity,target_velocity,torque\n";
        for (size_t i = 0; i < times.size(); ++i) {
            file << times[i] << ","
                 << velocities[i] << ","
                 << target_velocity_ << ","
                 << torques[i] << "\n";
        }
        file.close();
        std::cout << "Data exported to velocity_drive_data.csv" << std::endl;
    }
};

/**
 * @brief 主测试函数
 */
int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Zero-Gravity Pendulum Velocity Drive Test" << std::endl;
    std::cout << "========================================" << std::endl;

    // 测试 1：速度驱动
    std::cout << "\n[Test 1] Velocity Drive" << std::endl;
    std::cout << "------------------------" << std::endl;
    
    VelocityDrivePendulumTest test1;
    test1.setup(1.0,  // 目标速度 1 rad/s
                10.0, // kv = 10
                1.0,  // gear = 1
                100.0 // 力矩限制
               );
    test1.createScene();
    
    auto result1 = test1.run(5.0);
    
    std::cout << "\nResults:" << std::endl;
    std::cout << "  Final velocity: " << result1.final_velocity << " rad/s" << std::endl;
    std::cout << "  Target velocity: 1.0 rad/s" << std::endl;
    std::cout << "  Velocity error: " << result1.velocity_error << std::endl;
    std::cout << "  Max torque: " << result1.max_torque << " Nm" << std::endl;
    std::cout << "  Avg torque: " << result1.avg_torque << " Nm" << std::endl;
    std::cout << "  Status: " << (result1.passed ? "PASSED" : "FAILED") << std::endl;
    std::cout << "  Message: " << result1.message << std::endl;

    // 测试 2：力矩驱动（对比）
    std::cout << "\n[Test 2] Torque Drive (Constant Torque)" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    
    VelocityDrivePendulumTest test2;
    test2.setup(1.0, 10.0, 1.0, 100.0);
    test2.createScene();
    
    // 计算使单摆达到目标速度所需的力矩
    // I * alpha = torque, alpha = dv/dt
    // 假设 1 秒内达到目标速度，alpha = 1 rad/s²
    // I = 0.333 kg·m²
    // torque = 0.333 Nm
    auto result2 = test2.runTorqueDrive(0.333, 5.0);
    
    std::cout << "\nResults:" << std::endl;
    std::cout << "  Final velocity: " << result2.final_velocity << " rad/s" << std::endl;
    std::cout << "  Max torque: " << result2.max_torque << " Nm" << std::endl;
    std::cout << "  Status: " << (result2.passed ? "PASSED" : "FAILED") << std::endl;

    std::cout << "\n========================================" << std::endl;
    std::cout << "Test completed" << std::endl;
    std::cout << "========================================" << std::endl;

    return result1.passed && result2.passed ? 0 : 1;
}
```

---

## 5. 测试验证方案

### 5.1 验证指标

| 指标 | 期望值 | 容差 | 说明 |
|------|--------|------|------|
| 稳态速度 | 1.0 rad/s | ±0.1 rad/s | 达到目标速度的 90% 以上 |
| 上升时间 | < 1.0 s | - | 从 0 到 90% 目标速度 |
| 稳态力矩 | ~0.0 Nm | ±0.1 Nm | 无摩擦时维持匀速需要零力矩 |
| 超调量 | < 10% | - | 速度不应超过目标值太多 |

### 5.2 预期结果分析

#### 速度驱动模式
```
物理分析：
- 无重力、无摩擦环境下
- 达到稳态后，维持匀速旋转不需要力矩
- 速度控制器应输出接近零的力矩

预期行为：
1. 初始阶段：速度误差大，控制器输出较大力矩加速
2. 过渡阶段：速度误差减小，力矩逐渐减小
3. 稳态阶段：速度达到目标值，力矩趋近于零
```

#### 力矩驱动模式
```
物理分析：
- 恒定力矩 τ 产生恒定角加速度 α = τ / I
- 速度线性增加：ω(t) = α * t
- 无速度限制，会一直加速

预期行为：
1. 速度线性增长
2. 5 秒后速度约为：ω = (0.333 / 0.333) * 5 = 5 rad/s
```

### 5.3 可视化脚本（Python）

```python
# tools/plot_pendulum_test.py
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 读取数据
data = pd.read_csv('velocity_drive_data.csv')

# 创建图形
fig, axes = plt.subplots(2, 1, figsize=(10, 8))

# 速度曲线
axes[0].plot(data['time'], data['velocity'], 'b-', label='Actual Velocity')
axes[0].axhline(y=1.0, color='r', linestyle='--', label='Target Velocity')
axes[0].set_xlabel('Time (s)')
axes[0].set_ylabel('Angular Velocity (rad/s)')
axes[0].set_title('Pendulum Angular Velocity')
axes[0].legend()
axes[0].grid(True)

# 力矩曲线
axes[1].plot(data['time'], data['torque'], 'g-', label='Control Torque')
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Torque (Nm)')
axes[1].set_title('Control Torque')
axes[1].legend()
axes[1].grid(True)

plt.tight_layout()
plt.savefig('pendulum_velocity_drive.png', dpi=150)
plt.show()

# 计算性能指标
final_vel = data['velocity'].iloc[-100:].mean()
print(f"Final velocity: {final_vel:.3f} rad/s")
print(f"Velocity error: {abs(final_vel - 1.0):.3f} rad/s")
print(f"Max torque: {data['torque'].abs().max():.3f} Nm")
```

---

## 6. 参考代码实现

### 6.1 VelocityController 实现

```cpp
// src/Control/VelocityController.cpp
#include "NexDynIPC/Control/VelocityController.h"

namespace NexDynIPC::Control {

VelocityController::VelocityController()
    : kv_(10.0)
    , target_velocity_(0.0)
    , gear_ratio_(1.0)
    , gain_c0_(0.0)
    , gain_c1_(0.0)
    , gain_c2_(1.0)
    , bias_c0_(0.0)
    , bias_c1_(0.0)
    , bias_c2_(0.0)
{
}

double VelocityController::computeTorque(double current_position, 
                                          double current_velocity) const {
    // 步骤 1：计算控制输入（速度误差）
    double velocity_error = target_velocity_ - current_velocity;
    double ctrl = kv_ * velocity_error;
    
    // 步骤 2：计算增益（Affine 类型）
    // gain = c0 + c1 * length + c2 * velocity
    // 对于旋转关节，length = position
    double gain = gain_c0_ + gain_c1_ * current_position + gain_c2_ * current_velocity;
    
    // 步骤 3：计算偏置
    double bias = bias_c0_ + bias_c1_ * current_position + bias_c2_ * current_velocity;
    
    // 步骤 4：计算执行器力
    double force = gain * ctrl + bias;
    
    // 步骤 5：应用传动比
    double torque = gear_ratio_ * force;
    
    return torque;
}

VelocityController::ControlOutput 
VelocityController::computeDetailed(double current_position, 
                                    double current_velocity) const {
    ControlOutput output;
    
    // 控制输入
    output.ctrl = kv_ * (target_velocity_ - current_velocity);
    
    // 增益
    output.gain = gain_c0_ + gain_c1_ * current_position + gain_c2_ * current_velocity;
    
    // 偏置
    output.bias = bias_c0_ + bias_c1_ * current_position + bias_c2_ * current_velocity;
    
    // 执行器力
    output.force = output.gain * output.ctrl + output.bias;
    
    // 最终力矩
    output.torque = gear_ratio_ * output.force;
    
    return output;
}

} // namespace NexDynIPC::Control
```

### 6.2 JointActuator 改进实现

```cpp
// src/Control/JointActuator.cpp（改进版）
#include "NexDynIPC/Control/JointActuator.h"
#include <algorithm>
#include <limits>

namespace NexDynIPC::Control {

JointActuator::JointActuator(std::shared_ptr<Dynamics::Joint> joint)
    : joint_(joint)
    , mode_(ControlMode::TORQUE_CONTROL)
    , target_torque_(0.0)
    , target_position_(0.0)
    , target_velocity_(0.0)
    , kp_(100.0)
    , kd_(10.0)
    , gear_ratio_(1.0)
    , min_torque_(-std::numeric_limits<double>::max())
    , max_torque_(std::numeric_limits<double>::max())
    , enabled_(true)
{
    // 初始化速度控制器
    velocity_controller_.setGearRatio(1.0);
    velocity_controller_.setVelocityGain(kd_);
}

void JointActuator::setControlMode(ControlMode mode) {
    mode_ = mode;
}

void JointActuator::setTargetVelocity(double velocity) {
    target_velocity_ = velocity;
    velocity_controller_.setTargetVelocity(velocity);
}

void JointActuator::setGearRatio(double ratio) {
    gear_ratio_ = ratio;
    velocity_controller_.setGearRatio(ratio);
}

double JointActuator::computeControlOutput() {
    if (!enabled_) {
        return 0.0;
    }
    
    double torque = 0.0;
    
    switch (mode_) {
        case ControlMode::TORQUE_CONTROL:
            torque = computeTorqueControl();
            break;
        case ControlMode::POSITION_CONTROL:
            torque = computePositionControl();
            break;
        case ControlMode::VELOCITY_CONTROL:
            torque = computeVelocityControl();
            break;
    }
    
    return saturateTorque(torque);
}

double JointActuator::computeTorqueControl() const {
    // 直接力矩控制：τ = target_torque * gear_ratio
    return gear_ratio_ * target_torque_;
}

double JointActuator::computePositionControl() const {
    // PD 位置控制
    double pos_error = target_position_ - getCurrentPosition();
    double vel_error = target_velocity_ - getCurrentVelocity();
    
    // 计算控制输入
    double ctrl = kp_ * pos_error + kd_ * vel_error;
    
    // 应用传动比
    return gear_ratio_ * ctrl;
}

double JointActuator::computeVelocityControl() const {
    // 使用 VelocityController 计算
    return velocity_controller_.computeTorque(
        getCurrentPosition(), 
        getCurrentVelocity()
    );
}

double JointActuator::saturateTorque(double torque) const {
    return std::max(min_torque_, std::min(max_torque_, torque));
}

double JointActuator::getCurrentPosition() const {
    // 从关节获取当前位置
    // 具体实现取决于 NexDynIPC 的关节状态获取机制
    // 这里假设可以通过 joint_ 获取
    return 0.0;  // 占位符
}

double JointActuator::getCurrentVelocity() const {
    // 从关节获取当前速度
    return 0.0;  // 占位符
}

} // namespace NexDynIPC::Control
```

---

## 7. 总结

### 7.1 设计要点

1. **参考 MuJoCo 架构**：
   - 传动-增益-偏置分离设计
   - 清晰的控制流程：ctrl → gain/bias → force → torque
   - 灵活的参数配置

2. **速度控制核心**：
   - PD 控制律：ctrl = kv * (v_target - v)
   - 稳态时力矩趋近于零（无摩擦环境）
   - 传动比影响等效增益

3. **测试验证**：
   - 无重力单摆是理想的测试场景
   - 可以清晰验证速度控制器的收敛性
   - 力矩驱动模式作为对比

### 7.2 实现优先级

1. **高优先级**：
   - 实现 `VelocityController` 类
   - 改进 `JointActuator` 支持速度模式
   - 创建无重力单摆测试

2. **中优先级**：
   - 添加传动比支持
   - 实现力矩限制
   - 数据导出和可视化

3. **低优先级**：
   - 更多增益类型（非线性增益）
   - 自适应控制参数
   - 多关节协调控制

### 7.3 与 MuJoCo 的差异

| 特性 | MuJoCo | NexDynIPC 实现 |
|------|--------|----------------|
| 坐标系统 | 广义坐标 | 最大坐标 |
| 约束求解 | LCP | IPC + ALM |
| 力施加 | 直接修改 qfrc | ExternalForceForm |
| 积分器 | 半隐式欧拉 | 隐式 Newmark |
| 控制架构 | 传动-增益-偏置 | 类似设计 |

通过借鉴 MuJoCo 的控制架构，NexDynIPC 可以获得更灵活、更强大的控制能力，同时保持 IPC 接触模型的优势。
