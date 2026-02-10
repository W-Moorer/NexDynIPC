# 参考项目时间积分方案分析

## 1. PolyFEM

**仓库路径**：`polyfem/src/polyfem/time_integrator`

### 1.1 支持的方案

PolyFEM 采用面向对象设计，通过 `ImplicitTimeIntegrator` 基类支持以下方案：

| 方案名称 | 类名 | 说明 |
|---------|------|------|
| **隐式欧拉** | `ImplicitEuler` | 这里就是标准的 Backward Euler。 |
| **隐式 Newmark** | `ImplicitNewmark` | 支持 Newmark-β 家族方案。 |
| **BDF** | `BDF` | 后向差分公式，支持 1~6 阶（代码中 `BDF1` ~ `BDF6`）。 |

### 1.2 关键实现细节 (ImplicitNewmark)

- **默认参数**：$\beta = 0.25, \gamma = 0.5$（即平均加速度法 / 梯形法），这是二阶、无条件稳定、能量守恒（无阻尼）到的配置。
- **加速度更新**：标准的 Newmark 预测-校正格式。
- **⚠️ 潜在问题**：在 `set_parameters` 函数中，参数读取似乎存在**键值互换**的嫌疑：
  ```cpp
  beta_ = params.at("gamma");
  gamma_ = params.at("beta");
  ```
  如果用户在 JSON 中指定标准参数（如 `{"beta": 0.25, "gamma": 0.5}`），会导致内部变量被错误赋值为 $\beta=0.5, \gamma=0.25$。这可能导致算法不稳定。在使用或参考时需特别注意这一点。

---

## 2. Rigid-IPC

**仓库路径**：`rigid-ipc/src/time_stepper`

### 2.1 核心方案：DMV (Discrete Mean Value)

Rigid-IPC 默认且主要使用的是 **DMV (Discrete Mean Value)** 积分器（`dmv_time_stepper.cpp`）。

- **类型**：几何变分积分器（Geometric Variational Integrator）。
- **特点**：
  - 专门针对刚体旋转群 $SO(3)$ 设计。
  - **保辛（Symplectic）**：长时间仿真中保持几何结构和能量性质。
  - **显式/半隐式结构**：类似于 Kick-Drift-Kick 的分裂算法。
    1. **Kick**: 更新动量 $p_{n+1/2} = p_n + \frac{h}{2} F(q_n)$
    2. **Drift**: 更新位置/旋转 $q_{n+1} = \text{DMVSolve}(q_n, p_{n+1/2})$
    3. **Kick**: 更新动量 $p_{n+1} = p_{n+1/2} + \frac{h}{2} F(q_{n+1})$
  - 能够很好地处理刚体高速旋转时的进动和章动，优于传统的欧拉角或四元数直接积分。

### 2.2 其他支持方案

- **Exponential Euler** (`exponential_euler_time_stepper`): 用于旋转李代数积分的一阶方法，作为 DMV 失败时的回退（Failsafe）。
- **Symplectic Euler** (`sympletic_euler_time_stepper`): 标准的辛欧拉法，主要用于简单测试。
- **Verlet** (`verlet_time_stepper`): 经典的二阶辛积分器，常用于分子动力学。

### 3. 对 NexDynIPC 的启示

1. **PolyFEM 选择 Newmark**：作为一个通用有限元求解器，PolyFEM 选择了 Newmark-β 作为主力二阶求解器，因为其在结构动力学中非常成熟且灵活（可调阻尼）。对于 NexDynIPC 若涉及柔性体或通用 PDE，Newmark 是稳妥之选。

2. **Rigid-IPC 选择 DMV**：作为一个专门的刚体模拟器，Rigid-IPC 选择了更高级的 DMV 变分积分器。这说明对于**纯刚体**系统，**保结构（Structure-Preserving）/ 保辛**方法是获得高质量长时间仿真的关键。

**推荐路径**：
- 短期：实现 **隐式中点法**（即 Newmark $\beta=0.25, \gamma=0.5$ 的特例），这也是最简单的保辛二阶方法。
- 长期（如果专注于刚体）：研究并实现类似 DMV 的李群变分积分器。
