# Rigid IPC 深度解析与移植指南 (Deep Dive & Porting Guide)

本文档基于 `temp/rigid-ipc-main` 源码，对 Rigid IPC 参考实现的架构、核心算法及数据流进行了深度解析，旨在为 NexDynIPC 项目提供 **可执行的移植与开发指南**。

## 1. 核心架构总览 (Core Architecture)

Rigid IPC 的核心思想是将刚体动力学建模为一个 **非线性优化问题** (Implict Time Integration)，通过 **牛顿法** (Newton's Method) 求解下一帧的状态。

### 数据流图 (Data Flow)

```mermaid
graph TD
    Input[Scene JSON / Meshes] -->|read_rb_scene| Assembler[RigidBodyAssembler]
    Assembler -->|Construct Global Vector| x_vec[Global DOF Vector x]
    x_vec -->|NewtonSolver::solve| Optimizer[Newton Solver]
    Optimizer -->|Compute Gradient/Hessian| Barrier[IPC Barrier Potential] & Inertia[Inertial Potential]
    Optimizer -->|Line Search| CCD[Continuous Collision Detection]
    Optimizer -->|Update x| NewState[New State x_{n+1}]
```

---

## 2. 物理核心模块 (`src/physics`)

此模块是刚体模拟的基础，建议优先移植。

### 2.1 `Pose` 类 (位姿管理)
*   **文件**: `src/physics/pose.hpp`
*   **功能**: 统一管理 2D/3D 位姿，屏蔽旋转表示差异（旋转向量/四元数）。
*   **核心成员**:
    *   `VectorMax3 position`: 位移 (2D/3D)。
    *   `VectorMax3 rotation`: 旋转 (2D为标量，3D为旋转向量)。
*   **关键方法**:
    *   `construct_rotation_matrix()`: 转为旋转矩阵 ($R \in \mathbb{R}^{3 \times 3}$).
    *   `construct_quaternion()`: 转为 Eigen 四元数。
    *   `dofs_to_poses(dof)`: 将求解器的平坦向量转为 Pose 对象数组。

### 2.2 `RigidBody` 类 (刚体状态)
*   **文件**: `src/physics/rigid_body.hpp`
*   **功能**: 单个刚体的完整状态容器。
*   **核心成员**:
    *   `vertices / edges / faces`: 局部坐标系下的几何。
    *   `pose / velocity / force`: 当前时间步状态。
    *   `pose_prev / velocity_prev`: 上一步状态 (用于 CCD 和惯性项计算)。
    *   `mass / moment_of_inertia`: 质量属性。
*   **核心算法**:
    *   **顶点世界坐标更新**: `world_vertices(pose)`
        $$ v_{world} = R(q) \cdot v_{local} + p $$
    *   **微分接口 (Crucial)**: `world_vertices_diff`
        *   使用自动微分 (`autodiff`) 计算 $\frac{\partial v_{world}}{\partial \text{DOF}}$。
        *   这是 IPC 势能导数计算的链式法则起点。

### 2.3 `RigidBodyAssembler` (系统组装)
*   **文件**: `src/physics/rigid_body_assembler.cpp`
*   **功能**: 将 $N$ 个刚体组装成一个巨大的系统，提供全局向量 ($x \in \mathbb{R}^{12N}$) 与局部刚体状态的双向映射。
*   **关键逻辑**:
    *   **全局映射**: `m_body_vertex_id` 数组存储了每个刚体顶点在全局顶点向量 $V$ 中的偏移量。
    *   **碰撞对筛选**: `close_bodies` 使用 BVH 或 HashGrid 筛选潜在碰撞对。

---

## 3. 求解器模块 (`src/solvers`)

IPC 的稳定性极度依赖于求解器的鲁棒性。

### 3.1 `NewtonSolver` (牛顿法求解器)
*   **文件**: `src/solvers/newton_solver.cpp`
*   **算法流程**:
    1.  **计算梯度与 Hessian**: $\nabla E(x), \nabla^2 E(x)$。
    2.  **正则化 (Make PSD)**: 检查 Hessian 是否正定。若非正定，添加 $\mu I$ 直到正定。
        *   实现函数: `make_matrix_positive_definite()`
        *   策略: 对角占优法 (Diagonal Dominance)。
    3.  **求解方向**: sc = $\text{Solve}(\nabla^2 E \cdot \Delta x = -\nabla E)$。
        *   使用 `Eigen::SimplicialLDLT` (稀疏 Cholesky 分解)。
    4.  **线搜索 (Line Search)**:
        *   **CCD 约束**: 步长 $\alpha$ 必须保证 $x_n$ 到 $x_n + \alpha \Delta x$ 的路径无穿透。
        *   **Armijo 准则**: 能量必需充分下降。

### 3.2 移植建议
*   **Hessian 正则化**是不可或缺的。直接使用未处理的 Hessian 会导致牛顿法在非凸区域（如接触分离瞬间）失效。
*   **线搜索中的 CCD**: 必须先计算最大无碰撞步长 (TOI)，将线搜索范围限制在 $[0, \text{TOI}]$ 内。

---

## 4. 数据加载模块 (`src/io`)

### 4.1 `read_rb_scene.cpp` (场景加载)
*   **功能**: 解析 JSON 场景文件。
*   **亮点**:
    *   支持 `mesh` 路径加载 (.obj)。
    *   支持动态/静态/运动学 (`type` 字段) 设置。
    *   支持初速度 (`linear_velocity`, `angular_velocity`) 设置。
    *   自动处理单位转换 (角度 -> 弧度)。

---

## 5. 移植步骤 (Action Plan)

若要将上述功能集成到 NexDynIPC，建议遵循以下顺序：

1.  **基础数据结构移植**:
    *   复制 `Pose`, `RigidBody` 类定义。
    *   确保依赖库 (`Eigen`, `nlohmann/json`, `spdlog`) 已配置。

2.  **物理状态管理**:
    *   实现 `RigidBodyAssembler` 的简化版，至少包含：
        *   刚体列表管理 (`std::vector<RigidBody>`)。
        *   全局 DOF 向量与其更新逻辑。

3.  **时间积分循环**:
    *   实现 Backward Euler 积分器接口。
    *   集成 `NewtonSolver`，重点复用其 **Hessian 正定化** 和 **带 CCD 的线搜索** 逻辑。

4.  **对接 IPC Toolkit**:
    *   (详见 `detailed_ipc_toolkit_analysis.md`) 连接 Barrier Potential 和 CCD 模块。
