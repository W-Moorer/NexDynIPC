# NexDynIPC 速度驱动能量框架适配方案

## 核心问题：MuJoCo方案的能量化改造

由于NexDynIPC采用**基于能量的IPC框架**，MuJoCo的力反馈速度控制需要转化为**能量形式**才能集成。

---

## 1. 框架差异对比

### 1.1 数学本质差异

| 方面 | MuJoCo (约束求解) | NexDynIPC (能量最小化) |
|------|------------------|----------------------|
| **动力学方程** | `M·v̇ + C = τ + Jᵀ·λ` | `min E(x) = E_inertia + E_potential + E_constraint` |
| **力如何引入** | 直接作为广义力 `τ` | 作为能量梯度 `τ = -∇E` |
| **速度控制** | `τ = kv·(v_target - v)` | 需要构造 `E(v)` 使得 `-∇E = kv·(v_target - v)` |

### 1.2 关键洞察

MuJoCo的速度驱动公式：
```
τ = kv * (v_target - v)
```

对应NexDynIPC需要的能量形式：
```
E(v) = 0.5 * kv * (v - v_target)²
∇E = kv * (v - v_target) = -kv * (v_target - v)
F_drive = -∇E = kv * (v_target - v)  ✓ 等价
```

---

## 2. 能量形式设计方案

### 2.1 速度驱动能量形式 (VelocityDriveForm)

```cpp
#pragma once

#include "NexDynIPC/Dynamics/Forms/Form.h"
#include "NexDynIPC/Dynamics/Joints/Joint.h"
#include <memory>

namespace NexDynIPC::Control {

/**
 * @brief 速度驱动能量形式
 * 
 * 将速度控制目标转化为能量最小化问题。
 * 
 * 物理原理：
 * 目标速度 v_target，当前速度 v，速度增益 kv
 * 
 * 能量函数（二次势）：
 *   E(v) = 0.5 * kv * (v - v_target)²
 * 
 * 能量梯度（对应阻尼力）：
 *   dE/dv = kv * (v - v_target)
 * 
 * 驱动力（负梯度方向）：
 *   F_drive = -dE/dv = kv * (v_target - v)
 * 
 * 这与MuJoCo的PD速度控制完全等价：
 *   MuJoCo: τ = kv * (v_target - v)
 *   NexDynIPC: F_drive = kv * (v_target - v)
 * 
 * 时间离散化（隐式欧拉）：
 *   v^{n+1} = (q^{n+1} - q^n) / dt
 *   E(q^{n+1}) = 0.5 * kv * ((q^{n+1} - q^n)/dt - v_target)²
 */
class VelocityDriveForm : public Dynamics::Form {
public:
    /**
     * @brief 构造函数
     * @param joint 目标关节
     * @param kv 速度增益（阻尼系数）
     * @param v_target 目标速度
     */
    VelocityDriveForm(std::shared_ptr<Dynamics::Joint> joint, 
                      double kv, 
                      double v_target);

    /**
     * @brief 设置目标速度
     */
    void setTargetVelocity(double v_target);

    /**
     * @brief 获取目标速度
     */
    double getTargetVelocity() const { return v_target_; }

    /**
     * @brief 设置速度增益
     */
    void setVelocityGain(double kv);

    /**
     * @brief 获取速度增益
     */
    double getVelocityGain() const { return kv_; }

    /**
     * @brief 获取当前速度误差
     */
    double getVelocityError(const Eigen::VectorXd& x) const;

    // Form 接口实现
    double value(const Eigen::VectorXd& x) const override;
    void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const override;
    void hessian(const Eigen::VectorXd& x, 
                 std::vector<Eigen::Triplet<double>>& triplets) const override;

private:
    std::shared_ptr<Dynamics::Joint> joint_;
    double kv_;           // 速度增益
    double v_target_;     // 目标速度
    double dt_;           // 时间步长（用于速度计算）

    /**
     * @brief 计算关节速度
     * 
     * 从状态向量中提取关节速度：
     * v = (q - q_prev) / dt
     * 
     * 对于旋转关节：角速度 ω = dθ/dt
     */
    double computeVelocity(const Eigen::VectorXd& x) const;

    /**
     * @brief 计算速度对位置的导数 dv/dq = 1/dt
     */
    double computeVelocityGradient() const { return 1.0 / dt_; }
};

} // namespace NexDynIPC::Control
```

### 2.2 实现代码

```cpp
// VelocityDriveForm.cpp
#include "NexDynIPC/Control/VelocityDriveForm.h"
#include "NexDynIPC/Dynamics/Joints/RevoluteJoint.h"

namespace NexDynIPC::Control {

VelocityDriveForm::VelocityDriveForm(std::shared_ptr<Dynamics::Joint> joint,
                                      double kv, double v_target)
    : joint_(joint), kv_(kv), v_target_(v_target), dt_(0.01) {}

void VelocityDriveForm::setTargetVelocity(double v_target) {
    v_target_ = v_target;
}

void VelocityDriveForm::setVelocityGain(double kv) {
    kv_ = kv;
}

double VelocityDriveForm::getVelocityError(const Eigen::VectorXd& x) const {
    double v_current = computeVelocity(x);
    return v_target_ - v_current;
}

double VelocityDriveForm::computeVelocity(const Eigen::VectorXd& x) const {
    // 对于旋转关节，从状态中提取角速度
    // 注意：在最大坐标系中，需要从刚体状态计算关节相对速度
    
    if (auto revolute = std::dynamic_pointer_cast<Dynamics::RevoluteJoint>(joint_)) {
        // 获取父刚体和子刚体的角速度
        // 在隐式欧拉中：ω = (θ^{n+1} - θ^n) / dt
        
        // 这里简化处理：假设状态x包含旋转向量
        // 实际实现需要从World获取上一时刻状态
        int idx = revolute->getGlobalIndex();
        if (idx >= 0 && idx + 3 <= x.size()) {
            Eigen::Vector3d theta = x.segment<3>(idx + 3);
            // 需要从World获取上一时刻的theta_prev
            // 这里使用简化计算
            return theta.norm() / dt_;  // 简化：假设旋转向量大小代表角度变化
        }
    }
    
    return 0.0;
}

/**
 * @brief 计算速度驱动能量值
 * 
 * E = 0.5 * kv * (v - v_target)²
 */
double VelocityDriveForm::value(const Eigen::VectorXd& x) const {
    double v = computeVelocity(x);
    double dv = v - v_target_;
    return 0.5 * kv_ * dv * dv;
}

/**
 * @brief 计算能量梯度
 * 
 * dE/dq = dE/dv * dv/dq
 *       = kv * (v - v_target) * (1/dt)
 *       = (kv/dt) * (v - v_target)
 * 
 * 注意：这是势能梯度，负梯度方向才是驱动力方向
 * F_drive = -dE/dq = (kv/dt) * (v_target - v)
 */
void VelocityDriveForm::gradient(const Eigen::VectorXd& x, 
                                  Eigen::VectorXd& grad) const {
    double v = computeVelocity(x);
    double dv = v - v_target_;
    
    // dE/dv = kv * (v - v_target)
    double dE_dv = kv_ * dv;
    
    // dv/dq = 1/dt (隐式欧拉)
    double dv_dq = 1.0 / dt_;
    
    // 链式法则：dE/dq = dE/dv * dv/dq
    double dE_dq = dE_dv * dv_dq;
    
    // 应用到关节对应的自由度
    int idx = joint_->getGlobalIndex();
    if (idx >= 0 && idx < grad.size()) {
        // 对于旋转关节，应用到旋转自由度
        // 注意：这里假设旋转向量的方向代表旋转轴
        grad[idx + 3] += dE_dq;  // 旋转自由度索引偏移3
    }
}

/**
 * @brief 计算Hessian（用于牛顿法）
 * 
 * H = d²E/dq² = kv * (dv/dq)² = kv / dt²
 */
void VelocityDriveForm::hessian(const Eigen::VectorXd& x,
                                 std::vector<Eigen::Triplet<double>>& triplets) const {
    (void)x;
    
    double H = kv_ / (dt_ * dt_);
    
    int idx = joint_->getGlobalIndex();
    if (idx >= 0) {
        int rot_idx = idx + 3;  // 旋转自由度
        triplets.emplace_back(rot_idx, rot_idx, H);
    }
}

} // namespace NexDynIPC::Control
```

---

## 3. 与MuJoCo方案的对比

### 3.1 数学等价性证明

| 步骤 | MuJoCo | NexDynIPC (能量形式) |
|------|--------|---------------------|
| **控制目标** | 使 `v → v_target` | 最小化 `E = 0.5*kv*(v-v_target)²` |
| **控制律** | `τ = kv*(v_target - v)` | `F = -∇E = kv*(v_target - v)` |
| **稳态** | `v = v_target` 时 `τ = 0` | `v = v_target` 时 `∇E = 0` |
| **动态响应** | 一阶指数收敛 | 能量最小化驱动相同收敛 |

**结论**：两种方法在数学上完全等价，只是表达形式不同。

### 3.2 实现复杂度对比

| 方面 | MuJoCo | NexDynIPC能量形式 |
|------|--------|------------------|
| **代码复杂度** | 低（直接计算力） | 中（需要实现Form接口） |
| **与求解器集成** | 直接加入广义力 | 自动通过能量框架集成 |
| **Hessian支持** | 需要额外计算 | 天然支持（优化需要） |
| **多约束协调** | 需要手动处理优先级 | 能量自动平衡各约束 |

---

## 4. 无重力单摆测试的能量形式实现

### 4.1 测试场景配置

```cpp
// VelocityDrivePendulumTest_Energy.cpp
#include "NexDynIPC/Control/VelocityDriveForm.h"
#include "NexDynIPC/Dynamics/World.h"
#include "NexDynIPC/Dynamics/Forms/InertiaForm.h"
#include "NexDynIPC/Dynamics/Forms/ConstraintForm.h"

class VelocityDrivePendulumTest {
public:
    void setup(double target_velocity, double kv) {
        // 1. 创建世界（无重力）
        world_ = std::make_shared<Dynamics::World>();
        world_->gravity = Eigen::Vector3d::Zero();
        
        // 2. 创建单摆刚体
        auto pendulum = std::make_shared<Dynamics::RigidBody>();
        pendulum->mass = 1.0;
        pendulum->inertia_body = Eigen::Vector3d(0.01, 0.01, 0.01).asDiagonal();
        world_->addBody(pendulum);
        
        // 3. 创建旋转关节（连接world和pendulum）
        auto hinge = std::make_shared<Dynamics::RevoluteJoint>(
            nullptr, pendulum,
            Eigen::Vector3d::Zero(),      // 锚点
            Eigen::Vector3d(0, 0, 1)      // 旋转轴（Z轴）
        );
        world_->addJoint(hinge);
        
        // 4. 创建速度驱动能量形式
        velocity_drive_ = std::make_shared<VelocityDriveForm>(
            hinge, kv, target_velocity
        );
        world_->addForm(velocity_drive_);
        
        // 5. 创建约束能量形式
        constraint_form_ = std::make_shared<Dynamics::ConstraintForm>();
        constraint_form_->addJoint(hinge);
        world_->addForm(constraint_form_);
    }
    
    void run(double duration, double dt) {
        // 隐式时间积分
        auto integrator = std::make_shared<TimeIntegration::ImplicitEuler>(dt);
        
        for (double t = 0; t < duration; t += dt) {
            // 构建优化问题
            SimulationProblem problem(*world_, integrator);
            
            // 求解：min E_total(x)
            Eigen::VectorXd x = solveOptimization(problem);
            
            // 更新状态
            world_->setState(x);
            
            // 记录数据
            double v = velocity_drive_->getVelocityError(x);
            std::cout << "t=" << t << ", v_error=" << v << std::endl;
        }
    }

private:
    std::shared_ptr<Dynamics::World> world_;
    std::shared_ptr<VelocityDriveForm> velocity_drive_;
    std::shared_ptr<Dynamics::ConstraintForm> constraint_form_;
};
```

### 4.2 能量组成分析

在每一时间步，总能量为：

```
E_total = E_inertia + E_constraint + E_velocity_drive

其中：
• E_inertia = 0.5 * (x - x_hat)ᵀ * M * (x - x_hat)
  （惯性能量，驱动状态向预测状态收敛）

• E_constraint = λᵀ*C + 0.5*μ*Cᵀ*C
  （关节约束能量，保持连接）

• E_velocity_drive = 0.5 * kv * (v - v_target)²
  （速度驱动能量，驱动速度向目标收敛）
```

**优化求解**：
```
x* = argmin E_total(x)
```

通过最小化总能量，系统自动平衡：
- 惯性项要求状态平滑变化
- 约束项要求关节连接保持
- 速度驱动项要求达到目标速度

---

## 5. 力矩驱动的简化实现

### 5.1 直接使用 ExternalForceForm

力矩驱动不需要特殊能量形式，直接施加即可：

```cpp
// 创建外力形式
auto force_form = std::make_shared<ExternalForceForm>();
world_->addForm(force_form);

// 施加恒定力矩
force_form->setTorque(pendulum, Eigen::Vector3d(0, 0, target_torque));

// 或在仿真循环中动态更新
void updateTorque(double current_velocity) {
    // 简单的速度跟踪
    double torque = kv * (v_target - current_velocity);
    force_form->setTorque(pendulum, Eigen::Vector3d(0, 0, torque));
}
```

### 5.2 ExternalForceForm 的能量解释

```cpp
// ExternalForceForm 的梯度计算
void ExternalForceForm::gradient(const Eigen::VectorXd& x, 
                                  Eigen::VectorXd& grad) const {
    for (const auto& entry : entries_) {
        int idx = entry.global_idx;
        
        // 力对应的能量梯度：∇E = -F
        // 所以 grad -= force 等价于 ∇E = -F
        grad.segment<3>(idx) -= entry.force;
        
        // 力矩同理
        grad.segment<3>(idx + 3) -= entry.torque;
    }
}
```

**能量解释**：
- 外力形式隐式定义了能量 `E = -F·x`
- 梯度 `∇E = -F`，所以负梯度方向就是力的方向
- 这与速度驱动形式的数学结构一致

---

## 6. 关键设计决策

### 6.1 何时使用 VelocityDriveForm vs ExternalForceForm

| 场景 | 推荐方案 | 原因 |
|------|---------|------|
| **纯速度跟踪** | VelocityDriveForm | 能量形式自动处理与约束的协调 |
| **力矩限制严格** | ExternalForceForm | 直接控制力矩，易于限制 |
| **复杂控制律** | ExternalForceForm | 灵活实现任意控制算法 |
| **需要Hessian** | VelocityDriveForm | 能量形式天然提供Hessian |

### 6.2 速度计算的关键问题

在最大坐标系（NexDynIPC）vs 广义坐标系（MuJoCo）中，速度计算方式不同：

```cpp
// MuJoCo (广义坐标)
v = d->qvel[joint_dof_idx];  // 直接读取

// NexDynIPC (最大坐标)
// 需要从两个刚体的状态计算相对速度
Eigen::Vector3d omega_parent = computeAngularVelocity(body_parent, dt);
Eigen::Vector3d omega_child = computeAngularVelocity(body_child, dt);
Eigen::Vector3d omega_relative = omega_child - omega_parent;
double v_joint = omega_relative.dot(joint_axis);  // 投影到关节轴
```

**实现建议**：
- 在 `VelocityDriveForm` 中缓存上一时刻状态
- 或使用 `World` 提供的速度查询接口

---

## 7. 总结

### 7.1 适配要点

1. **MuJoCo的速度控制律** `τ = kv*(v_target - v)` **可以直接使用**，但需要包装为能量形式

2. **能量形式的优势**：
   - 自动与IPC约束求解框架集成
   - 天然支持Hessian计算（优化收敛更快）
   - 能量自动平衡多个竞争目标

3. **实现路径**：
   - 创建 `VelocityDriveForm` 继承自 `Form`
   - 实现 `value()`、`gradient()`、`hessian()` 三个接口
   - 在 `gradient()` 中实现 `F = kv*(v_target - v)`

### 7.2 与原始设计的差异

| 原始设计 (MuJoCo风格) | 能量适配设计 |
|---------------------|-------------|
| `VelocityController` 计算力矩 | `VelocityDriveForm` 计算能量和梯度 |
| 直接施加力矩到关节 | 通过能量梯度间接施加力 |
| 需要手动处理与约束的关系 | 能量框架自动协调 |

### 7.3 推荐实现优先级

1. **高优先级**：实现 `VelocityDriveForm` 类
2. **中优先级**：修改无重力单摆测试使用能量形式
3. **低优先级**：优化速度计算效率（缓存、预计算）

---

## 附录：完整类图

```
┌─────────────────────────────────────────────────────────────────┐
│                     NexDynIPC 能量框架                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────┐                                            │
│  │   Form (基类)   │◄─────────────────────────────────────┐     │
│  │─────────────────│                                    │     │
│  │ + value()       │                                    │     │
│  │ + gradient()    │                                    │     │
│  │ + hessian()     │                                    │     │
│  └─────────────────┘                                    │     │
│           ▲                                             │     │
│           │ 继承                                         │     │
│    ┌──────┴──────┬──────────────┬──────────────┐       │     │
│    │             │              │              │       │     │
│    ▼             ▼              ▼              ▼       │     │
│ ┌──────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐  │     │
│ │Inertia│   │ Gravity  │   │Constraint│   │Velocity  │  │     │
│ │ Form  │   │  Form    │   │  Form    │   │ DriveForm│  │     │
│ └──────┘   └──────────┘   └──────────┘   └──────────┘  │     │
│                                              ▲         │     │
│                                              │         │     │
│  ┌──────────────────────────────────────┐    │         │     │
│  │         VelocityDriveForm            │────┘         │     │
│  │──────────────────────────────────────│              │     │
│  │ - kv: double                         │              │     │
│  │ - v_target: double                   │              │     │
│  │ - joint: Joint*                      │              │     │
│  │──────────────────────────────────────│              │     │
│  │ + value(x) = 0.5*kv*(v-v_target)²   │              │     │
│  │ + gradient(x) → kv*(v-v_target)/dt  │──────────────┘     │
│  │ + hessian(x) → kv/dt²               │                    │
│  └──────────────────────────────────────┘                    │
│                                                              │
│  说明：gradient() 的负方向即为驱动力方向                      │
│        F_drive = -gradient = kv*(v_target - v)               │
│                                                              │
└─────────────────────────────────────────────────────────────────┘
```
