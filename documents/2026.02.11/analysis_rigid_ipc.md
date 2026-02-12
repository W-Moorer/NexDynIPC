# Rigid IPC 项目深度分析

## 项目概述
`rigid-ipc-main` 是一个基于 IPC (Incremental Potential Contact) 方法的刚体动力学仿真器的参考实现。它提供了从物理状态管理、碰撞检测、接触响应到时间积分的完整管线。

## 目录结构分析 (`src/`)

### 1. 物理核心 (`src/physics`)
这是项目中最核心的模块，定义了刚体动力学的基本数据结构。

*   **`pose.hpp` / `Pose` 类**:
    *   **核心功能**: 统一封装了位置（Vector）和旋转（Quaternion/Rotation Vector）。
    *   **亮点**:
        *   支持 2D (xy, theta) 和 3D (xyz, quaternion) 两种模式。
        *   提供了 `dofs_to_poses` 和 `poses_to_dofs` 静态方法，方便在广义坐标向量（如 Solver 需要的 `x` 向量）和对象化表示之间转换。
        *   实现了 `construct_rotation_matrix`，方便获取旋转矩阵。
    *   **复用价值**: 高。可以直接作为物理引擎的基础位姿类。

*   **`rigid_body.hpp` / `RigidBody` 类**:
    *   **核心功能**: 单个刚体的完整状态容器。
    *   **成员变量**:
        *   `vertices`, `edges`, `faces`: 几何数据。
        *   `pose`, `velocity`, `force`: 当前状态。
        *   `pose_prev`, `velocity_prev`: 上一帧状态（用于 CCD 和隐式积分）。
        *   `mass`, `moment_of_inertia`: 质量属性。
        *   `type`: Dynamic / Static / Kinematic。
    *   **关键方法**:
        *   `world_vertices()`: 计算世界坐标系下的顶点位置。
        *   `compute_bounding_box()`: 计算 AABB。
    *   **复用价值**: 极高。设计完善的刚体类。

*   **`rigid_body_assembler.hpp` / `RigidBodyAssembler` 类**:
    *   **核心功能**: 管理多刚体系统，将所有刚体的自由度 (DOF) 拼装成由 Solver 处理的大向量。
    *   **关键方法**:
        *   `world_vertices_diff()`: 计算世界坐标顶点位置对刚体 DOF 的 Jacobian 和 Hessian。这是隐式积分中计算接触势能导数的关键。
        *   `close_bodies()`: 基于 Broad Phase (Hash Grid, BVH) 筛选潜在碰撞对。
    *   **复用价值**: 高。如果你需要处理多刚体系统，这是一个很好的管理器模板。

### 2. 求解器 (`src/solvers`)
实现了用于隐式时间积分的非线性优化求解器。

*   **`newton_solver.hpp` / `NewtonSolver` 类**:
    *   **功能**: 标准的牛顿法求解器，带有线搜索 (Line Search)。
    *   **特性**:
        *   支持 `VELOCITY` 或 `ENERGY` 收敛准则。
        *   集成了 `make_matrix_positive_definite`，确保 Hessian 正定，保证牛顿方向是下降方向。
        *   `compute_direction()`: 求解线性方程组 $H \Delta x = -g$。
    *   **复用价值**: 中。依赖于具体的 `OptimizationProblem` 接口，但其牛顿法和正定化逻辑通用于所有优化问题。

### 3. 时间积分器 (`src/time_stepper`)
定义了仿真步进的策略。

*   **`time_stepper.hpp` / `TimeStepper` 接口**:
    *   定义了统一的 `step()` 接口。
*   **实现类**:
    *   `Euler` (Implicit Euler): 无条件稳定，但数值阻尼大。
    *   `Verlet`: 显式积分，简单但需小步长。
    *   **复用价值**: 其架构模式（将积分逻辑与物理状态分离）值得参考。

### 4. 其他实用模块
*   **`src/io`**: 提供了 `.obj` (Mesh), `.json` (Scene), `.gltf` (Animation) 的读写支持。
*   **`src/utils`**: 包含 `Eigen` 扩展函数、Logger 配置等。

## 关键代码复用建议

1.  **数据与逻辑分离**: 通过 `RigidBodyAssembler` 将分散的 `RigidBody` 对象聚合为代数求解器可见的向量 (`dof`)，这种设计模式是实现高效求解器的关键。
2.  **微分计算**: `RigidBodyAssembler::world_vertices_diff` 展示了如何链式法则计算 `d(WorldVertex)/d(BodyDOF)`，这是实现隐式 IPC 的数学核心，建议详细研读。
3.  **位姿管理**: `Pose` 类屏蔽了 2D/3D 差异和四元数运算的复杂性，是一个非常优秀的工具类，建议直接移植。
