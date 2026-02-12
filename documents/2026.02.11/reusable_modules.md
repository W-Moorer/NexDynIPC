# 可复用模块分析报告

本文档基于 `temps` 目录下的代码实现，梳理了可供当前项目（NexDynIPC）参考或直接复制使用的模块。

## 1. 独立原型验证模块 (Python)

### `temps/ipc+sdf-simple/ipc_rigid_sdf_framework.py`

这是一个独立的、单文件的 Python 脚本，实现了基于解析 SDF (Signed Distance Field) 的 3D 刚体动力学与 IPC (Incremental Potential Contact) 接触解算。

*   **功能亮点**:
    *   **刚体动力学**: 支持动态 (Free) 和静态 (Fixed) 刚体。
    *   **状态表示**: 使用位移 + 四元数 (Quaternion) 表示姿态。
    *   **隐式积分器**: 基于 Backward Euler 的隐式积分，使用牛顿法求解速度。
    *   **接触模型**: 实现了 IPC 的对数势垒 (Log Barriers) 和正则化库伦摩擦。
    *   **解析形状**: 内置球体 (Sphere) 和有向包围盒 (Box) 的 SDF 及梯度计算。
    *   **可视化与输出**: 包含简易的 Matplotlib 绘图和 CSV 数据导出。

*   **复用建议**:
    *   **算法验证**: 该脚本非常适合作为核心算法（特别是接触势能和隐式积分求解器）的 **Reference Implementation**。
    *   **快速原型**: 如果需要快速测试新的接触公式或积分策略，可以直接修改此脚本进行验证，无需编译 C++ 项目。
    *   **移植参考**: 其中的 `Pose` 更新逻辑（四元数积分）、IPC 势垒函数 (`barrier_dB_dd`) 和摩擦力计算逻辑可以直接翻译成 C++。

## 2. 刚体动力学核心组件 (C++)

### 来源: `temps/rigid-ipc-main/rigid-ipc-main/src/physics`

该目录下包含了成熟的 C++ 刚体动力学实现，可以直接参考或移植到当前项目的物理引擎中。

*   **核心类 (Classes)**:
    *   **`RigidBody` (`rigid_body.hpp`, `rigid_body.cpp`)**:
        *   封装了刚体的所有状态：质量、惯性张量、位置、姿态 (Pose)、速度、受力等。
        *   包含了顶点变换逻辑 (`world_vertices`)，这是碰撞检测的基础。
        *   支持动态 (Dynamic)、静态 (Static) 和运动学 (Kinematic) 类型。
    *   **`Pose` (`pose.hpp`)**:
        *   封装了位置 (Vector) 和旋转 (Quaternion/Matrix) 的组合，提供了便捷的坐标变换接口。
    *   **`Mass` (`mass.hpp`, `mass.cpp`)**:
        *   提供了计算网格质量属性（质心、惯性张量）的工具函数。

*   **复用建议**:
    *   **数据结构**: 如果当前项目的 `RigidBody` 类功能尚不完善，可以参考此处的成员变量设计和状态更新方法。
    *   **物理计算**: 惯性张量的变换 (`I_world_inv`) 和牛顿-欧拉方程的实现细节值得参考。

## 3. IPC 接触算法核心 (C++)

### 来源: `temps/ipc-toolkit-main/ipc-toolkit-main/src/ipc`

IPC Toolkit 是一个通过 CMake 引入的库，但在需要手写核心逻辑时，其部分源码极具参考价值。

*   **关键模块**:
    *   **`barrier/`**: 包含了 IPC 核心的势垒函数及其导数（Gradient, Hessian）的实现。
    *   **`ccd/`**: 连续碰撞检测 (Continuous Collision Detection) 的接口和实现。
    *   **`distance/`**: 点-三角、边-边等图元间的距离计算函数。
    *   **`friction/`**: 摩擦力势能及其导数的计算。

*   **复用建议**:
    *   **公式实现**: 若项目中需要手写 IPC 能量项，请直接参考 `barrier` 和 `friction` 目录下的具体公式实现，确保数值计算的稳定性。
    *   **接口设计**: 参考 `ipc.hpp` 中的接口设计 (`is_step_collision_free`, `compute_collision_free_stepsize`) 来组织碰撞检测模块。

## 4. 实用工具 (Utilities)

### `temps/compare_results.py`

*   **功能**: 一个用于对比仿真结果 (CSV) 与参考数据 (CSV) 的 Python 脚本。
*   **适用场景**: 在做 "Comparing Simulation Results" 任务时，该脚本可以直接作为 `tools` 目录下的验证工具，用于生成误差分析报告。
