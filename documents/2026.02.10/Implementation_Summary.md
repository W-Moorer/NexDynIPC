# NexDynIPC 实现总结
**日期：** 2026-02-10
**时间：** 15:30 (大约)

## 概述
本文档总结了截至 2026 年 2 月 10 日，**NexDynIPC** 物理引擎已实现的代码框架和逻辑。该系统被设计为一个模块化的、无头模式（headless）运行的物理仿真引擎，核心采用隐式欧拉积分（Implicit Euler），并以支持 IPC（增量势能接触）为目标。

## 模块架构

项目结构分为四个主要模块，遵循严格的分层依赖关系：`App -> Dynamics -> Physics -> Math`。

### 1. 数学模块 (`NexDynIPC::Math`)
**职责：** 核心数学工具和求解器。
- **`LinearSolver`**：线性方程组 ($Ax = b$) 的求解接口。
  - 实现了对 **SuiteSparse/CHOLMOD** 的封装，用于高效的稀疏 Cholesky 分解，这对于隐式积分中的 Hessian 矩阵求解至关重要。
- **`NewtonSolver`**：用于无约束优化的鲁棒阻尼牛顿法（Newton-Raphson）求解器。
  - 最小化由 `OptimizationProblem` 定义的目标函数。
  - 包含 **回溯线搜索（Backtracking Line Search）** 以确保全局收敛（能量下降）。
- **`SparseMatrixUtils`**：处理 Eigen 稀疏矩阵的工具函数。

### 2. 物理模块 (`NexDynIPC::Physics`)
**职责：** 几何、接触检测和空间划分。
- **`Shape`**：几何图元的抽象基类。
- **`MeshShape`**：三角形网格的具体实现（目前为基础版本）。
- **`BarrierPotential`**：（计划中）用于 IPC 接触处理的对数障碍函数。
- **`BroadPhase`**：宽阶段碰撞检测。
  - 目前实现了 **暴力（Brute-Force）** 方法 ($O(N^2)$)，结构简单，适用于小规模测试。

### 3. 动力学模块 (`NexDynIPC::Dynamics`)
**职责：** 物理仿真逻辑、运动方程和时间积分。
- **`RigidBody`**：表示刚体，包含状态（位置 $x$、旋转 $q$、速度 $v$、角速度 $\omega$）和属性（质量、惯性张量）。
  - 最近已更新，增加了 **线性加速度 ($a$)** 和 **角加速度 ($\alpha$)** 的存储，便于分析。
- **`World`**：刚体和物理形式（势能项）的容器。
- **`Form`**：能量势能（增量势能）的抽象接口。
  - **`InertiaForm`**：实现了“惯性势能” ($ \frac{1}{2} (x - \hat{x})^T M (x - \hat{x}) $)，它将运动方程封装在优化框架中。
    - 经过优化，已将静态物体从活动变量集中剔除。
  - **`GravityForm`**：实现了重力势能 ($ U = m g^T x $)，并在目标函数中正确应用了 $dt^2$ 缩放。
- **`TimeIntegrator`**：时间步进方案的接口。
  - **`ImplicitEuler`**：全隐式后向欧拉积分器。
    - 将时间步进公式化为优化问题：$\min_x E(x) = \text{Inertia}(x) + h^2 V(x)$。
    - 使用 `NewtonSolver` 求解下一时刻的状态 $x_{n+1}$。
    - 处理状态的收集（Gather）与分发（Scatter），即在全局优化向量和独立刚体对象之间进行映射。

### 4. 应用模块 (`NexDynIPC::App`)
**职责：** 仿真循环、配置和输入/输出。
- **`Simulation`**：主入口类。管理 `World` 和 `TimeIntegrator`，运行仿真主循环。
- **`SceneLoader`**：加载仿真场景。
  - 目前通过硬编码创建一个测试场景（地面 + 下落的立方体）。
- **`StateExporter`**：导出仿真结果。
  - **CSV 导出**：将每一帧的状态（位置、旋转、速度、加速度）写入 `simulation_results.csv`，方便数据分析。

## 核心逻辑与算法

### 隐式积分流程 (Implicit Integration Flow)
1. **预测 (Prediction)**：计算预测状态 $\hat{x} = x_n + h v_n$。
2. **构建优化 (Optimization Construction)**：
   - 构建结合了惯性项和势能项（重力等）的目标函数。
   - 静态物体被排除在优化变量之外，以防止 Hessian 矩阵奇异。
3. **牛顿求解 (Newton Solve)**：
   - 计算梯度 $\nabla E$ 和 Hessian $\nabla^2 E$。
   - 对势能项应用 $h^2$（时间步长的平方）进行缩放，以确保正确的物理单位和动力学行为。
   - 求解线性系统 $H \Delta x = -g$。
   - 更新候选解 $x$ 并检查收敛性。
4. **状态更新 (State Update)**：
   - 更新位置 $x_{n+1}$。
   - 更新速度 $v_{n+1} = (x_{n+1} - x_n) / h$。
   - 更新加速度 $a_{n+1} = (v_{n+1} - v_n) / h$。

## 近期更新 (2026.02.10)
- **重构优化过程**：通过从优化状态中移除静态物体，修复了求解器发散的问题。
- **修正物理逻辑**：通过对势能项应用正确的 $dt^2$ 缩放，修复了物体下落速度过快的问题。
- **增强输出功能**：将输出从多文件文本格式更改为统一的 **CSV** 格式，并包含加速度数据。

## 下一步计划
- 实现 IPC 碰撞约束 (`BarrierForm`)。
- 升级 `SceneLoader` 以支持解析 JSON/XML 文件。
- 为 `BroadPhase` 实现空间哈希（Spatial Hashing）或 BVH 加速结构。
