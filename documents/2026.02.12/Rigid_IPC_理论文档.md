# Rigid IPC 项目理论文档

## 目录

1. [项目概述](#1-项目概述)
2. [项目架构与目录结构](#2-项目架构与目录结构)
3. [核心理论基础](#3-核心理论基础)
4. [刚体动力学系统](#4-刚体动力学系统)
5. [碰撞检测系统](#5-碰撞检测系统)
6. [IPC障碍函数方法](#6-ipc障碍函数方法)
7. [优化求解器](#7-优化求解器)
8. [时间积分器](#8-时间积分器)
9. [自动微分系统](#9-自动微分系统)
10. [区间算术与根查找](#10-区间算术与根查找)
11. [输入输出系统](#11-输入输出系统)
12. [工具与辅助模块](#12-工具与辅助模块)
13. [依赖库说明](#13-依赖库说明)
14. [使用示例](#14-使用示例)

---

## 1. 项目概述

### 1.1 项目简介

**Rigid IPC** 是一个用于刚体动力学模拟的开源项目，实现了 **SIGGRAPH 2021** 论文《Intersection-free Rigid Body Dynamics》的核心算法。该项目的核心特点是能够进行**鲁棒、无交叉的刚体模拟**，确保在整个模拟过程中刚体之间不会发生穿透现象。

### 1.2 核心特性

- **无穿透保证**：使用 IPC（Incremental Potential Contact）方法确保模拟过程中不会产生几何交叉
- **支持2D和3D**：同时支持二维和三维刚体模拟
- **多种碰撞检测方法**：支持 BVH（层次包围盒）、Hash Grid（哈希网格）等多种碰撞检测策略
- **连续碰撞检测（CCD）**：精确计算碰撞发生的时间点
- **摩擦与恢复系数**：支持摩擦力和弹性碰撞的模拟
- **并行计算**：使用 TBB 进行多线程并行加速

### 1.3 应用场景

- 物理仿真与动画
- 游戏物理引擎
- 机器人学仿真
- 虚拟现实应用
- 科学计算与工程仿真

---

## 2. 项目架构与目录结构

### 2.1 目录结构概览

```
rigid-ipc-main/
├── src/                    # 源代码目录
│   ├── autodiff/          # 自动微分模块
│   ├── autogen/           # 自动生成的代码
│   ├── barrier/           # 障碍函数实现
│   ├── ccd/               # 连续碰撞检测
│   ├── geometry/          # 几何计算
│   ├── interval/          # 区间算术
│   ├── io/                # 输入输出
│   ├── opt/               # 优化问题定义
│   ├── physics/           # 物理模拟核心
│   ├── problems/          # 问题定义
│   ├── solvers/           # 求解器实现
│   ├── time_stepper/      # 时间积分器
│   ├── tools/             # 工具程序
│   ├── utils/             # 工具函数
│   └── viewer/            # 可视化界面
├── fixtures/              # 测试场景文件
├── meshes/                # 网格模型文件
├── tests/                 # 单元测试
├── cmake/                 # CMake配置
├── python/                # Python绑定
└── comparisons/           # 与其他引擎的对比
```

### 2.2 核心模块说明

| 模块 | 功能描述 |
|------|----------|
| `physics/` | 刚体定义、位姿表示、质量计算等物理核心 |
| `ccd/` | 连续碰撞检测，包括宽相位和窄相位 |
| `barrier/` | IPC障碍函数的实现 |
| `solvers/` | Newton求解器、IPC求解器、同伦求解器 |
| `opt/` | 优化问题定义和约束处理 |
| `time_stepper/` | Verlet积分器和DMV积分器 |
| `autodiff/` | 自动微分支持，用于计算梯度和Hessian |

---

## 3. 核心理论基础

### 3.1 IPC（Incremental Potential Contact）方法概述

IPC是一种基于优化的碰撞处理方法，其核心思想是将非穿透约束转化为一个**障碍函数（Barrier Function）**，并将其加入到优化目标函数中。

#### 3.1.1 核心思想

传统的碰撞处理方法通常采用以下策略：
- **惩罚方法**：在穿透后施加惩罚力
- **约束方法**：将碰撞作为硬约束求解

IPC方法的创新在于：
1. 使用**障碍函数**在距离接近零时产生无穷大的势能
2. 将碰撞避免问题转化为**无约束优化问题**
3. 保证在整个模拟过程中**不会产生穿透**

#### 3.1.2 数学形式

IPC的目标函数形式为：

$$f(x) = E(x) + \kappa \sum_{k \in C} b(d(x_k))$$

其中：
- $E(x)$ 是能量项（动能、势能等）
- $\kappa$ 是障碍函数的刚度系数
- $b(d)$ 是障碍函数
- $d(x_k)$ 是第 $k$ 对碰撞候选之间的距离
- $C$ 是碰撞候选集合

### 3.2 障碍函数类型

项目支持三种障碍函数：

#### 3.2.1 IPC障碍函数

$$b_{IPC}(d, \hat{d}) = \begin{cases} 
-(\hat{d} - d)^2 \ln(\frac{d}{\hat{d}}) & d < \hat{d} \\
0 & d \geq \hat{d}
\end{cases}$$

#### 3.2.2 Poly-Log障碍函数

$$b_{polylog}(d, s) = -\ln(y)(2y^3 - 3y^2 + 1), \quad y = \frac{d}{s}$$

#### 3.2.3 Spline障碍函数

$$g(x, s) = \frac{1}{s^3}x^3 - \frac{3}{s^2}x^2 + \frac{3}{s}x$$

$$\phi_{spline}(x, s) = \frac{1}{g(x, s)} - 1$$

### 3.3 关键参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `dhat` | 障碍函数激活距离 | 1e-3 |
| `kappa` | 障碍函数刚度 | 自适应计算 |
| `minimum_separation_distance` | 最小分离距离 | 0 |
| `barrier_type` | 障碍函数类型 | IPC |

---

## 4. 刚体动力学系统

### 4.1 刚体类（RigidBody）

刚体是模拟的基本单元，定义在 [rigid_body.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/physics/rigid_body.hpp) 中。

#### 4.1.1 刚体类型

```cpp
enum RigidBodyType { 
    STATIC,     // 静态刚体：完全固定不动
    KINEMATIC,  // 运动学刚体：按预定轨迹运动
    DYNAMIC     // 动力学刚体：受力和碰撞影响
};
```

#### 4.1.2 刚体属性

| 属性 | 类型 | 说明 |
|------|------|------|
| `vertices` | MatrixXd | 刚体局部坐标系下的顶点位置 |
| `edges` | MatrixXi | 边的连接关系 |
| `faces` | MatrixXi | 面的连接关系 |
| `mass` | double | 总质量 |
| `moment_of_inertia` | VectorMax3d | 绕主轴的转动惯量 |
| `pose` | PoseD | 当前时刻的位姿 |
| `velocity` | PoseD | 当前时刻的速度 |
| `force` | PoseD | 作用力 |

#### 4.1.3 刚体状态

刚体的状态由**位姿（Pose）**描述，包含：
- **位置（Position）**：质心的位置坐标
- **旋转（Rotation）**：使用旋转向量表示的旋转

### 4.2 位姿类（Pose）

位姿类定义在 [pose.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/physics/pose.hpp) 中。

#### 4.2.1 自由度

| 维度 | 位置自由度 | 旋转自由度 | 总自由度 |
|------|-----------|-----------|----------|
| 2D | 2 (x, y) | 1 (θ) | 3 |
| 3D | 3 (x, y, z) | 3 (θx, θy, θz) | 6 |

#### 4.2.2 核心操作

```cpp
// 构造旋转矩阵
MatrixMax3<T> construct_rotation_matrix() const;

// 构造四元数
Eigen::Quaternion<T> construct_quaternion() const;

// 位姿插值
static Pose<T> interpolate(const Pose<T>& pose0, const Pose<T>& pose1, T t);
```

### 4.3 刚体组装器（RigidBodyAssembler）

刚体组装器负责管理多个刚体的全局状态，定义在 [rigid_body_assembler.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/physics/rigid_body_assembler.hpp) 中。

#### 4.3.1 主要功能

1. **全局顶点管理**：将所有刚体的顶点合并到全局坐标系
2. **索引映射**：维护全局索引到局部索引的映射
3. **质量矩阵组装**：构建全局质量矩阵

#### 4.3.2 关键方法

```cpp
// 获取世界坐标系下的顶点
Eigen::MatrixXd world_vertices(const Poses<T>& poses) const;

// 全局到局部索引转换
void global_to_local_vertex(long global_id, long& body_id, long& local_id) const;

// 获取刚体位姿
PosesD rb_poses(const bool previous = false) const;
```

### 4.4 质量属性计算

质量属性计算模块定义在 [mass.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/physics/mass.hpp) 中。

#### 4.4.1 计算内容

```cpp
void compute_mass_properties(
    const Eigen::MatrixXd& vertices,
    const Eigen::MatrixXi& facets,
    double& total_mass,           // 总质量
    VectorMax3d& center_of_mass,  // 质心位置
    MatrixMax3d& moment_of_inertia // 转动惯量矩阵
);
```

#### 4.4.2 计算方法

对于三角形网格，使用**体积积分**方法计算：
- 总质量 = 密度 × 体积
- 质心 = ∫r·dV / 体积
- 转动惯量 = ∫(r² - r·r')·dV

---

## 5. 碰撞检测系统

### 5.1 碰撞检测概述

碰撞检测系统分为两个阶段：
1. **宽相位（Broad Phase）**：快速筛选可能碰撞的候选对
2. **窄相位（Narrow Phase）**：精确计算碰撞时间和位置

### 5.2 检测方法

定义在 [ccd.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/ccd/ccd.hpp) 中。

#### 5.2.1 宽相位检测方法

```cpp
enum DetectionMethod {
    BRUTE_FORCE,  // 暴力检测：检查所有可能的对
    HASH_GRID,    // 哈希网格：空间划分加速
    BVH           // 层次包围盒：树形结构加速
};
```

#### 5.2.2 轨迹类型

```cpp
enum TrajectoryType {
    LINEAR,           // 线性轨迹：旋转线性化
    PIECEWISE_LINEAR, // 分段线性：保守估计
    RIGID,            // 刚体轨迹：完整非线性
    REDON             // Redon方法：使用Redon算法
};
```

### 5.3 碰撞类型

#### 5.3.1 2D碰撞类型

- **边-点碰撞（Edge-Vertex）**：顶点与边的碰撞

#### 5.3.2 3D碰撞类型

- **边-边碰撞（Edge-Edge）**：两条边之间的碰撞
- **面-点碰撞（Face-Vertex）**：顶点与面的碰撞

### 5.4 连续碰撞检测（CCD）

CCD模块定义在 [ccd/rigid/time_of_impact.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/ccd/rigid/time_of_impact.hpp) 中。

#### 5.4.1 碰撞时间计算

```cpp
// 边-点碰撞时间
bool compute_edge_vertex_time_of_impact(
    const RigidBody& bodyA, const Pose<double>& poseA_t0, const Pose<double>& poseA_t1,
    size_t vertex_id,
    const RigidBody& bodyB, const Pose<double>& poseB_t0, const Pose<double>& poseB_t1,
    size_t edge_id,
    double& toi  // 输出：碰撞时间
);

// 边-边碰撞时间
bool compute_edge_edge_time_of_impact(...);

// 面-点碰撞时间
bool compute_face_vertex_time_of_impact(...);
```

#### 5.4.2 碰撞冲击数据结构

定义在 [impact.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/ccd/impact.hpp) 中。

```cpp
// 边-点碰撞冲击
struct EdgeVertexImpact {
    double time;        // 碰撞时间
    long edge_index;    // 被碰撞的边索引
    double alpha;       // 边上的参数位置
    long vertex_index;  // 碰撞顶点索引
};

// 边-边碰撞冲击
struct EdgeEdgeImpact {
    double time;
    long impacted_edge_index;
    double impacted_alpha;
    long impacting_edge_index;
    double impacting_alpha;
};

// 面-点碰撞冲击
struct FaceVertexImpact {
    double time;
    long face_index;
    double u, v;        // 面上的重心坐标
    long vertex_index;
};
```

### 5.5 宽相位碰撞检测

定义在 [ccd/rigid/broad_phase.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/ccd/rigid/broad_phase.hpp) 中。

#### 5.5.1 候选对生成

```cpp
// 生成碰撞候选对
void detect_collision_candidates_rigid(
    const RigidBodyAssembler& bodies,
    const PosesD& poses_t0,
    const PosesD& poses_t1,
    const int collision_types,
    Candidates& candidates,
    DetectionMethod method,
    const double inflation_radius = 0.0
);
```

#### 5.5.2 BVH方法

BVH（层次包围盒）方法通过构建树形结构来加速碰撞检测：
1. 为每个刚体构建局部BVH
2. 在模拟过程中更新包围盒
3. 通过树遍历快速排除不可能碰撞的对

### 5.6 距离计算

定义在 [geometry/distance.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/geometry/distance.hpp) 中。

```cpp
// 点-线有符号距离
template <typename T>
T point_line_signed_distance(
    const Vector2<T>& point,
    const Vector2<T>& line_point0,
    const Vector2<T>& line_point1
);

// 线-线有符号距离
template <typename T>
T line_line_signed_distance(
    const Vector3<T>& line0_point0, const Vector3<T>& line0_point1,
    const Vector3<T>& line1_point0, const Vector3<T>& line1_point1
);

// 点-面有符号距离
template <typename T>
T point_plane_signed_distance(
    const Vector3<T>& point,
    const Vector3<T>& triangle_vertex0,
    const Vector3<T>& triangle_vertex1,
    const Vector3<T>& triangle_vertex2
);
```

---

## 6. IPC障碍函数方法

### 6.1 障碍函数实现

定义在 [barrier/barrier.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/barrier/barrier.hpp) 中。

#### 6.1.1 障碍函数接口

```cpp
// 障碍函数值
template <typename T> 
T barrier(const T& x, double s, BarrierType barrier_type);

// 障碍函数梯度
double barrier_gradient(double x, double s, BarrierType barrier_type);

// 障碍函数Hessian
double barrier_hessian(double x, double s, BarrierType barrier_type);
```

#### 6.1.2 IPC障碍函数特性

IPC障碍函数具有以下重要特性：
1. **C²连续性**：函数、一阶导数、二阶导数都连续
2. **紧支撑**：只在距离小于阈值时激活
3. **无穿透保证**：距离趋近于零时函数值趋向无穷大

### 6.2 距离障碍约束

定义在 [opt/distance_barrier_constraint.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/opt/distance_barrier_constraint.hpp) 中。

#### 6.2.1 约束类定义

```cpp
class DistanceBarrierConstraint : public CollisionConstraint {
public:
    // 障碍函数激活距离
    double barrier_activation_distance() const;
    
    // 检测是否有活跃碰撞
    bool has_active_collisions(
        const RigidBodyAssembler& bodies,
        const PosesD& poses_t0,
        const PosesD& poses_t1
    ) const;
    
    // 计算最早碰撞时间
    double compute_earliest_toi(
        const RigidBodyAssembler& bodies,
        const PosesD& poses_t0,
        const PosesD& poses_t1
    ) const;
    
    // 计算障碍约束
    void compute_constraints(
        const RigidBodyAssembler& bodies,
        const PosesD& poses,
        Eigen::VectorXd& barriers
    );
    
    // 计算最小距离
    double compute_minimum_distance(
        const RigidBodyAssembler& bodies,
        const PosesD& poses
    ) const;
};
```

#### 6.2.2 关键参数

| 参数 | 说明 |
|------|------|
| `initial_barrier_activation_distance` | 初始障碍函数激活距离 |
| `minimum_separation_distance` | 最小分离距离 |
| `barrier_type` | 障碍函数类型 |

### 6.3 障碍问题定义

定义在 [problems/barrier_problem.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/problems/barrier_problem.hpp) 中。

#### 6.3.1 目标函数计算

```cpp
double BarrierProblem::compute_objective(
    const Eigen::VectorXd& x,
    Eigen::VectorXd& grad,
    Eigen::SparseMatrix<double>& hess,
    bool compute_grad,
    bool compute_hess
) {
    // 计算能量项 E(x)
    double Ex = compute_energy_term(x, grad, hess, compute_grad, compute_hess);
    
    // 计算障碍项 ∑b(d(x_k))
    double Bx = compute_barrier_term(x, grad_Bx, hess_Bx, num_constraints, compute_grad, compute_hess);
    
    // 组合：f(x) = E(x) + κ * B(x)
    if (compute_grad) grad += kappa * grad_Bx;
    if (compute_hess) hess += kappa * hess_Bx;
    
    return Ex + kappa * Bx;
}
```

#### 6.3.2 虚函数接口

```cpp
// 能量项计算（由子类实现）
virtual double compute_energy_term(
    const Eigen::VectorXd& x,
    Eigen::VectorXd& grad,
    Eigen::SparseMatrix<double>& hess,
    bool compute_grad,
    bool compute_hess
) = 0;

// 障碍项计算（由子类实现）
virtual double compute_barrier_term(
    const Eigen::VectorXd& x,
    Eigen::VectorXd& grad,
    Eigen::SparseMatrix<double>& hess,
    int& num_constraints,
    bool compute_grad,
    bool compute_hess
) = 0;
```

---

## 7. 优化求解器

### 7.1 求解器概述

项目提供了三种求解器：

| 求解器 | 说明 | 适用场景 |
|--------|------|----------|
| NewtonSolver | 基础牛顿法 | 无约束优化 |
| IPCSolver | IPC自适应求解器 | 碰撞约束优化 |
| HomotopySolver | 同伦方法求解器 | 复杂约束问题 |

### 7.2 牛顿求解器（NewtonSolver）

定义在 [solvers/newton_solver.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/solvers/newton_solver.hpp) 中。

#### 7.2.1 算法流程

```
1. 初始化 x = x0
2. while 未收敛:
   a. 计算梯度 g = ∇f(x)
   b. 计算Hessian H = ∇²f(x)
   c. 求解方向 Δx = -H⁻¹g
   d. 线搜索确定步长 α
   e. 更新 x = x + α * Δx
3. 返回结果
```

#### 7.2.2 收敛准则

```cpp
enum ConvergenceCriteria {
    VELOCITY,  // 位置变化量收敛
    ENERGY     // 能量变化量收敛
};
```

#### 7.2.3 线搜索

```cpp
virtual bool line_search(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& dir,
    const double fx,
    const Eigen::VectorXd& grad_fx,
    double& step_length
);
```

### 7.3 IPC求解器（IPCSolver）

定义在 [solvers/ipc_solver.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/solvers/ipc_solver.hpp) 中。

#### 7.3.1 自适应刚度更新

IPCSolver的核心特性是**自适应调整障碍函数刚度**：

```cpp
void IPCSolver::post_step_update() {
    double min_distance = barrier_problem_ptr()->compute_min_distance();
    
    // 根据最小距离调整刚度
    if (min_distance < dhat_epsilon) {
        // 增加刚度以确保无穿透
        kappa = min(kappa * 2, max_barrier_stiffness);
    }
}
```

#### 7.3.2 关键参数

| 参数 | 说明 |
|------|------|
| `min_barrier_stiffness_scale` | 最小障碍刚度比例 |
| `dhat_epsilon` | 自适应刚度激活距离 |
| `max_barrier_stiffness` | 最大障碍刚度 |

### 7.4 同伦求解器（HomotopySolver）

定义在 [solvers/homotopy_solver.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/solvers/homotopy_solver.hpp) 中。

#### 7.4.1 同伦方法原理

同伦方法通过逐渐增加约束强度来求解问题：

$$H(x, t) = E(x) + t \cdot \kappa \sum b(d(x_k))$$

从 $t=0$（无约束）逐渐增加到 $t=1$（完全约束）。

#### 7.4.2 参数说明

```cpp
double tinit;   // 初始t值
double t;       // 当前t值
double m;       // 精度参数
double c;       // 收敛比例
double e_b;     // 基础精度
double t_inc;   // t增量
```

### 7.5 优化结果

```cpp
struct OptimizationResults {
    Eigen::VectorXd x;       // 最优解
    double minf;             // 最小目标值
    bool success;            // 是否成功
    int num_iterations;      // 迭代次数
};
```

---

## 8. 时间积分器

### 8.1 时间积分器概述

时间积分器负责在时间步进过程中更新刚体的状态。定义在 [time_stepper/time_stepper.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/time_stepper/time_stepper.hpp) 中。

### 8.2 基类接口

```cpp
class TimeStepper {
public:
    // 单个刚体时间步进
    virtual void step(
        RigidBody& body,
        const VectorMax3d& gravity,
        const double& time_step
    ) const;
    
    // 多刚体时间步进（并行）
    virtual void step(
        RigidBodyAssembler& bodies,
        const VectorMax3d& gravity,
        const double& time_step
    ) const;
    
    virtual std::string name() const = 0;
};
```

### 8.3 Verlet积分器

定义在 [time_stepper/verlet_time_stepper.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/time_stepper/verlet_time_stepper.hpp) 中。

#### 8.3.1 Verlet算法（2D）

Verlet积分是一种**辛积分**方法，适用于2D模拟：

$$x_{n+1} = 2x_n - x_{n-1} + a_n \Delta t^2$$

#### 8.3.2 特点

- **时间可逆**：具有良好的能量守恒性
- **二阶精度**：误差为 $O(\Delta t^2)$
- **简单高效**：计算量小

### 8.4 DMV积分器

定义在 [time_stepper/dmv_time_stepper.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/time_stepper/dmv_time_stepper.hpp) 中。

#### 8.4.1 DMV算法（3D）

DMV（Dullweber-Murray-Young）积分器专门用于3D刚体旋转：

- 使用**四元数**表示旋转
- 保持**角动量守恒**
- 适用于快速旋转的刚体

#### 8.4.2 特点

- **辛积分**：长期模拟能量稳定
- **角动量守恒**：适合旋转动力学
- **高精度**：适用于复杂运动

---

## 9. 自动微分系统

### 9.1 自动微分概述

自动微分系统定义在 [autodiff/autodiff.h](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/autodiff/autodiff.h) 中，用于精确计算目标函数的梯度和Hessian矩阵。

### 9.2 微分标量类型

#### 9.2.1 一阶微分（DScalar1）

```cpp
template <typename Scalar, typename Gradient = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>
struct DScalar1 : public DiffScalarBase {
    Scalar value;      // 函数值
    Gradient grad;     // 梯度向量
    
    // 支持的运算：+, -, *, /, sqrt, pow, exp, log, sin, cos, ...
};
```

#### 9.2.2 二阶微分（DScalar2）

```cpp
template <typename Scalar, typename Gradient, typename Hessian>
struct DScalar2 : public DiffScalarBase {
    Scalar value;      // 函数值
    Gradient grad;     // 梯度向量
    Hessian hess;      // Hessian矩阵
};
```

### 9.3 使用方法

```cpp
// 设置变量数量
DiffScalarBase::setVariableCount(n);

// 创建独立变量
DScalar2 x(0, x_value);  // 第0个变量
DScalar2 y(1, y_value);  // 第1个变量

// 计算函数
DScalar2 f = x * x + sin(y);

// 获取结果
double value = f.getValue();
Vector grad = f.getGradient();
Matrix hess = f.getHessian();
```

### 9.4 支持的运算

| 运算 | 公式 | 梯度 | Hessian |
|------|------|------|---------|
| 加法 | f + g | ∇f + ∇g | ∇²f + ∇²g |
| 乘法 | f * g | f∇g + g∇f | f∇²g + g∇²f + ∇f∇gᵀ + ∇g∇fᵀ |
| sqrt | √f | ∇f/(2√f) | ... |
| exp | eᶠ | eᶠ∇f | eᶠ(∇f∇fᵀ + ∇²f) |
| sin | sin(f) | cos(f)∇f | ... |
| cos | cos(f) | -sin(f)∇f | ... |

---

## 10. 区间算术与根查找

### 10.1 区间算术

定义在 [interval/interval.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/interval/interval.hpp) 中。

#### 10.1.1 区间类型

```cpp
typedef boost::numeric::interval<
    double,
    boost::numeric::interval_lib::policies<
        boost::numeric::interval_lib::save_state<FILibRounding>,
        interval_options::CheckingPolicy
    >
> Interval;
```

#### 10.1.2 区间向量类型

```cpp
typedef Vector2<Interval> Vector2I;
typedef Vector3<Interval> Vector3I;
typedef VectorX<Interval> VectorXI;
typedef Matrix3<Interval> Matrix3I;
```

#### 10.1.3 区间运算

区间运算保证结果包含真实值：

```cpp
Interval a(1.0, 2.0);
Interval b(3.0, 4.0);

Interval c = a + b;  // [4.0, 6.0]
Interval d = a * b;  // [3.0, 8.0]
Interval e = sin(a); // [sin(1.0), sin(2.0)]
```

### 10.2 区间根查找

定义在 [interval/interval_root_finder.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/interval/interval_root_finder.hpp) 中。

#### 10.2.1 一维根查找

```cpp
bool interval_root_finder(
    const std::function<Interval(const Interval&)>& f,
    const Interval& x0,
    double tol,
    Interval& x,
    int max_iterations
);
```

#### 10.2.2 多维根查找

```cpp
bool interval_root_finder(
    const std::function<VectorMax3I(const VectorMax3I&)>& f,
    const VectorMax3I& x0,
    VectorMax3d tol,
    VectorMax3I& x,
    int max_iterations
);
```

#### 10.2.3 带约束的根查找

```cpp
bool interval_root_finder(
    const std::function<Interval(const Interval&)>& f,
    const std::function<bool(const Interval&)>& constraint_predicate,
    const Interval& x0,
    double tol,
    Interval& x
);
```

### 10.3 在CCD中的应用

区间算术在连续碰撞检测中的应用：

1. **保守估计**：区间运算提供保守的距离范围
2. **根查找**：使用区间方法查找碰撞时间
3. **可靠性保证**：不会漏检碰撞

---

## 11. 输入输出系统

### 11.1 场景文件格式

场景使用JSON格式描述，定义在 [io/read_rb_scene.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/io/read_rb_scene.hpp) 中。

#### 11.1.1 场景文件结构

```json
{
    "scene_type": "distance_barrier_rb_problem",
    "max_iterations": 1000,
    "timestep": 0.01,
    "rigid_body_problem": {
        "coefficient_restitution": -1,
        "coefficient_friction": 0.0,
        "gravity": [0, -9.8, 0],
        "rigid_bodies": [
            {
                "mesh": "cube.obj",
                "position": [0, 1, 0],
                "rotation": [0, 0, 0],
                "linear_velocity": [0, 0, 0],
                "angular_velocity": [0, 0, 0],
                "density": 1.0,
                "type": "dynamic"
            },
            {
                "mesh": "plane.obj",
                "is_dof_fixed": [true, true, true, true, true, true],
                "position": [0, 0, 0],
                "type": "static"
            }
        ]
    },
    "newton_solver": {
        "max_iterations": 3000,
        "convergence_criteria": "velocity"
    },
    "distance_barrier_constraint": {
        "detection_method": "bvh",
        "initial_barrier_activation_distance": 1e-3
    }
}
```

#### 11.1.2 刚体配置参数

| 参数 | 类型 | 说明 |
|------|------|------|
| `mesh` | string | 网格文件路径 |
| `position` | [double] | 初始位置 |
| `rotation` | [double] | 初始旋转（欧拉角） |
| `linear_velocity` | [double] | 初始线速度 |
| `angular_velocity` | [double] | 初始角速度 |
| `density` | double | 密度 |
| `type` | string | 刚体类型 |
| `is_dof_fixed` | [bool] | 固定的自由度 |

### 11.2 输出格式

#### 11.2.1 模拟结果文件

```json
{
    "args": { /* 原始场景配置 */ },
    "animation": {
        "state_sequence": [ /* 每帧的刚体状态 */ ]
    },
    "stats": {
        "dim": 3,
        "num_bodies": 2,
        "num_vertices": 100,
        "num_timesteps": 1000,
        "step_timings": [/* 每帧耗时 */],
        "solver_iterations": [/* 求解器迭代次数 */],
        "num_contacts": [/* 碰撞数量 */],
        "step_minimum_distances": [/* 最小距离 */]
    }
}
```

#### 11.2.2 GLTF动画输出

支持导出为GLTF格式，可在各种3D查看器中播放动画。

### 11.3 网格文件支持

支持的网格格式：
- OBJ格式
- STL格式
- VTK格式

---

## 12. 工具与辅助模块

### 12.1 日志系统

使用 spdlog 库实现日志功能：

```cpp
// 设置日志级别
set_logger_level(spdlog::level::info);

// 日志输出
spdlog::info("Starting simulation");
spdlog::debug("Iteration {}", i);
spdlog::error("Failed to load scene");
```

### 12.2 性能分析器

定义在 `profiler.hpp` 中：

```cpp
PROFILE_START();
PROFILE_POINT("simulation_step");
// ... 代码 ...
PROFILE_END();
LOG_PROFILER("simulation.json");
```

### 12.3 可视化工具

#### 12.3.1 OpenGL查看器

支持实时可视化：
- 刚体渲染
- 碰撞点显示
- 交互式控制

#### 12.3.2 软件渲染器

用于无GUI环境的渲染：
- PNG输出
- 自定义相机视角

### 12.4 命令行工具

| 工具 | 功能 |
|------|------|
| `cli_ccd` | CCD测试工具 |
| `cli_min_distance` | 最小距离计算 |
| `time_ccd` | CCD性能测试 |
| `sim_to_gltf` | 模拟结果转GLTF |
| `obj_sequence` | OBJ序列生成 |

---

## 13. 依赖库说明

### 13.1 核心依赖

| 库 | 用途 |
|---|------|
| **Eigen** | 线性代数运算 |
| **IPC Toolkit** | IPC通用函数 |
| **libigl** | 几何处理和可视化 |
| **TBB** | 并行计算 |
| **spdlog** | 日志系统 |

### 13.2 碰撞检测依赖

| 库 | 用途 |
|---|------|
| **Tight Inclusion CCD** | 精确连续碰撞检测 |
| **filib** | 区间算术 |

### 13.3 输入输出依赖

| 库 | 用途 |
|---|------|
| **nlohmann/json** | JSON解析 |
| **tinygltf** | GLTF导出 |
| **tinyxml2** | XML解析 |

### 13.4 可选依赖

| 库 | 用途 |
|---|------|
| **Catch2** | 单元测试 |
| **Polyscope** | 可视化 |
| **pybind11** | Python绑定 |

---

## 14. 使用示例

### 14.1 基本使用流程

```
1. 准备场景JSON文件
2. 运行模拟器
3. 查看结果
```

### 14.2 命令行运行

```bash
# 带GUI运行
./rigid-ipc --gui scene.json

# 无GUI运行
./rigid-ipc --ngui -i scene.json -o output/ -f result.json

# 指定线程数
./rigid-ipc --ngui -i scene.json -o output/ --nthreads 8
```

### 14.3 Python接口

```python
import rigid_ipc

# 加载场景
sim = rigid_ipc.Simulation()
sim.load_scene("scene.json")

# 运行模拟
for i in range(1000):
    sim.step()
    
# 保存结果
sim.save_simulation("result.json")
```

### 14.4 典型场景示例

#### 14.4.1 自由落体

```json
{
    "timestep": 0.01,
    "rigid_body_problem": {
        "gravity": [0, -9.8, 0],
        "rigid_bodies": [{
            "mesh": "cube.obj",
            "position": [0, 5, 0]
        }]
    }
}
```

#### 14.4.2 链条模拟

```json
{
    "timestep": 0.001,
    "rigid_body_problem": {
        "gravity": [0, -9.8, 0],
        "rigid_bodies": [
            {"mesh": "link.obj", "position": [0, 0, 0], "is_dof_fixed": [true, true, true, true, true, true]},
            {"mesh": "link.obj", "position": [0, -1, 0]},
            {"mesh": "link.obj", "position": [0, -2, 0]}
        ]
    }
}
```

---

## 附录A：数学符号说明

| 符号 | 说明 |
|------|------|
| $x$ | 优化变量（刚体位姿） |
| $E(x)$ | 能量函数 |
| $b(d)$ | 障碍函数 |
| $d$ | 距离 |
| $\kappa$ | 障碍刚度 |
| $\hat{d}$ | 障碍激活距离 |
| $\Delta t$ | 时间步长 |
| $t$ | 时间参数 |
| $\nabla$ | 梯度算子 |
| $\nabla^2$ | Hessian算子 |

## 附录B：重要常数

定义在 [constants.hpp](file:///e:/workspace/NexDynIPC/temps/rigid-ipc-main/rigid-ipc-main/src/constants.hpp) 中：

| 常数 | 值 | 说明 |
|------|-----|------|
| `DEFAULT_NEWTON_ENERGY_CONVERGENCE_TOL` | 1e-5 | 能量收敛容差 |
| `DEFAULT_NEWTON_VELOCITY_CONVERGENCE_TOL` | 1e-2 | 速度收敛容差 |
| `DEFAULT_LINE_SEARCH_LOWER_BOUND` | 1e-12 | 线搜索下界 |
| `RIGID_CCD_TOI_TOL` | 1e-4 | CCD时间容差 |
| `INTERVAL_ROOT_FINDER_TOL` | 1e-8 | 区间根查找容差 |
| `DEFAULT_MIN_BARRIER_STIFFNESS_SCALE` | 1e11 | 最小障碍刚度比例 |

---

## 附录C：参考文献

1. **IPC原始论文**：
   - Li, M., Ferguson, Z., Schneider, T., et al. "Incremental Potential Contact: Intersection-and Inversion-free Large Deformation Dynamics." SIGGRAPH 2020.

2. **刚体IPC论文**：
   - Ferguson, Z., Li, M., Schneider, T., et al. "Intersection-free Rigid Body Dynamics." SIGGRAPH 2021.

3. **连续碰撞检测**：
   - Wang, H., et al. "A Large Scale Benchmark and an Inclusion-Based Algorithm for Continuous Collision Detection." SIGGRAPH 2021.

---

*文档版本：1.0*  
*生成日期：2026-02-12*  
*基于项目：Rigid IPC (SIGGRAPH 2021)*
