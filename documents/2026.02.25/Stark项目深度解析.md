# Stark 物理仿真框架深度解析

## 目录
1. [项目概述](#1-项目概述)
2. [框架设计架构](#2-框架设计架构)
3. [核心算法理论](#3-核心算法理论)
4. [模块详细分析](#4-模块详细分析)
5. [关键设计模式](#5-关键设计模式)
6. [性能优化策略](#6-性能优化策略)
7. [扩展开发指南](#7-扩展开发指南)

---

## 1. 项目概述

### 1.1 项目定位

Stark 是一个基于 C++ 和 Python 的高性能物理仿真平台，专注于刚体与可变形物体的**强耦合仿真**。该项目由 RWTH Aachen University 开发，并在 ICRA 2024 上发表了相关论文。Stark 的核心创新在于：

- **统一仿真框架**：将刚体动力学、可变形体（FEM）、布料（离散壳）和摩擦接触统一在一个隐式时间积分框架内
- **符号微分引擎**：通过 SymX 符号计算引擎自动生成能量函数的梯度和 Hessian 矩阵
- **IPC 接触处理**：采用 Incremental Potential Contact 方法实现无穿透的鲁棒接触仿真

### 1.2 技术特点

| 特性 | 描述 |
|------|------|
| 编程语言 | C++17 / Python 3.8+ |
| 构建系统 | CMake 3.15+ |
| 数值计算 | Eigen 3.x |
| 符号计算 | 自研 SymX 引擎 |
| 碰撞检测 | 自研 TriangleMeshCollisionDetection |
| 并行计算 | OpenMP 多线程 |

### 1.3 项目结构

```
stark-main/
├── stark/                    # 核心库
│   ├── src/
│   │   ├── core/            # 核心引擎（牛顿法、设置、日志）
│   │   ├── models/          # 物理模型
│   │   │   ├── deformables/ # 可变形体（FEM、壳、点动力学）
│   │   │   ├── rigidbodies/ # 刚体动力学
│   │   │   ├── interactions/# 相互作用（接触、摩擦、附件）
│   │   │   └── presets/     # 预设配置
│   │   └── utils/           # 工具函数
│   └── extern/              # 外部依赖
│       ├── symx/            # 符号微分引擎
│       ├── Eigen/           # 线性代数库
│       ├── TriangleMeshCollisionDetection/  # 碰撞检测
│       └── fmt/             # 格式化库
├── pystark/                  # Python 绑定
├── examples/                 # C++ 示例
└── docs/                     # 文档和图片
```

---

## 2. 框架设计架构

### 2.1 整体架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                        Simulation Layer                          │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    Simulation (高层API)                    │   │
│  │  ├── Deformables (可变形体管理)                            │   │
│  │  ├── RigidBodies (刚体管理)                                │   │
│  │  ├── Interactions (相互作用管理)                           │   │
│  │  └── Presets (预设配置)                                    │   │
│  └──────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────┤
│                        Core Engine Layer                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐  │
│  │    Stark     │  │ NewtonsMethod│  │   GlobalEnergy       │  │
│  │ (仿真主循环)  │──│ (牛顿迭代器) │──│ (全局能量组装)        │  │
│  └──────────────┘  └──────────────┘  └──────────────────────┘  │
│         │                  │                      │              │
│         ▼                  ▼                      ▼              │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐  │
│  │   Console    │  │   Logger     │  │     Assembly         │  │
│  │ (控制台输出)  │  │ (日志记录)    │  │ (稀疏矩阵组装)        │  │
│  └──────────────┘  └──────────────┘  └──────────────────────┘  │
├─────────────────────────────────────────────────────────────────┤
│                        Symbolic Layer                            │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                      SymX Engine                           │   │
│  │  ├── Energy (能量定义)                                     │   │
│  │  ├── Scalar/Vector/Matrix (符号类型)                       │   │
│  │  ├── diff (自动微分)                                       │   │
│  │  └── Compilation (代码生成与编译)                          │   │
│  └──────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────┤
│                        Physics Models Layer                      │
│  ┌────────────────┐ ┌────────────────┐ ┌────────────────────┐  │
│  │ EnergyTetStrain│ │EnergyDiscrete  │ │EnergyFrictional    │  │
│  │ (四面体应变)    │ │Shells(离散壳)  │ │Contact(摩擦接触)   │  │
│  └────────────────┘ └────────────────┘ └────────────────────┘  │
│  ┌────────────────┐ ┌────────────────┐ ┌────────────────────┐  │
│  │ PointDynamics  │ │RigidBodyDynamics│ │EnergyAttachments  │  │
│  │ (点动力学)      │ │(刚体动力学)     │ │(附件约束)          │  │
│  └────────────────┘ └────────────────┘ └────────────────────┘  │
├─────────────────────────────────────────────────────────────────┤
│                        External Libraries                        │
│  ┌────────────┐ ┌─────────────────────┐ ┌──────────────────┐   │
│  │   Eigen    │ │TriangleMeshCollision│ │       fmt        │   │
│  │(线性代数)  │ │Detection(碰撞检测)   │ │   (格式化输出)   │   │
│  └────────────┘ └─────────────────────┘ └──────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 核心类设计

#### 2.2.1 Stark 核心类

`Stark` 类是整个仿真引擎的核心控制器，负责管理仿真主循环：

```cpp
class Stark {
public:
    const Settings settings;           // 仿真设置
    symx::GlobalEnergy global_energy;  // 全局能量系统
    Callbacks callbacks;               // 回调函数
    Console console;                   // 控制台输出
    Logger logger;                     // 日志记录
    EventDrivenScript script;          // 事件驱动脚本
    
    double current_time = 0.0;         // 当前时间
    int current_frame = 0;             // 当前帧
    double dt = -1.0;                  // 时间步长
    Eigen::Vector3d gravity;           // 重力加速度
    
    bool run(double duration);         // 运行仿真
    bool run_one_step();               // 执行单步
    
private:
    NewtonsMethod newton;              // 牛顿法求解器
    void _initialize();                // 初始化
    void _write_frame();               // 写入帧数据
};
```

**设计要点**：
- 采用组合模式，将各个子系统作为成员变量
- 使用回调机制实现松耦合
- 支持事件驱动的脚本系统

#### 2.2.2 Simulation 高层接口

`Simulation` 类提供用户友好的高层 API：

```cpp
class Simulation {
public:
    std::shared_ptr<Deformables> deformables;    // 可变形体
    std::shared_ptr<RigidBodies> rigidbodies;    // 刚体
    std::shared_ptr<Interactions> interactions;  // 相互作用
    std::shared_ptr<Presets> presets;            // 预设
    
    void run(double duration);                   // 运行仿真
    void add_time_event(double t0, double t1, 
        std::function<void(double)> action);     // 添加时间事件
    
private:
    core::Stark stark;                           // 核心引擎
};
```

### 2.3 数据流架构

```
用户代码
    │
    ▼
┌─────────────────┐
│   Simulation    │ 创建场景、添加物体
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│     Stark       │ 初始化、编译能量函数
└────────┬────────┘
         │
         ▼
┌─────────────────────────────────────────────┐
│              仿真主循环                       │
│  ┌─────────────────────────────────────┐    │
│  │  1. 更新碰撞检测                     │    │
│  │  2. 牛顿迭代求解                     │    │
│  │     ├── 计算能量/梯度/Hessian        │    │
│  │     ├── 求解线性系统                 │    │
│  │     └── 线搜索                       │    │
│  │  3. 更新状态                         │    │
│  │  4. 输出帧数据                       │    │
│  └─────────────────────────────────────┘    │
└─────────────────────────────────────────────┘
         │
         ▼
    输出文件 (VTK)
```

---

## 3. 核心算法理论

### 3.1 隐式时间积分

Stark 采用**隐式欧拉法**进行时间积分，这是处理刚性系统的标准方法。

#### 3.1.1 运动方程

对于可变形体，运动方程为：

$$M\ddot{x} + f_{int}(x) = f_{ext}$$

其中：
- $M$ 是质量矩阵
- $x$ 是位置向量
- $f_{int}$ 是内力（来自弹性势能）
- $f_{ext}$ 是外力（如重力）

#### 3.1.2 隐式离散化

采用隐式欧拉法，速度和位置更新为：

$$v^{n+1} = v^n + \Delta t \cdot a^{n+1}$$
$$x^{n+1} = x^n + \Delta t \cdot v^{n+1}$$

#### 3.1.3 变分形式

Stark 将问题转化为**能量最小化**问题：

$$\min_{v^{n+1}} E(v^{n+1}) = E_{inertia} + E_{elastic} + E_{contact} + E_{friction}$$

其中：
- $E_{inertia}$：惯性势能
- $E_{elastic}$：弹性势能
- $E_{contact}$：接触势能（IPC）
- $E_{friction}$：摩擦势能

**时间积分实现**：

```cpp
symx::Vector time_integration(const symx::Vector& x0, 
                               const symx::Vector& v1, 
                               const symx::Scalar& dt) {
    return x0 + v1 * dt;
}
```

### 3.2 牛顿迭代法

Stark 使用牛顿法求解非线性优化问题。

#### 3.2.1 算法流程

```
输入: 初始猜测 v^0, 容差 ε
输出: 收敛解 v*

1. 初始化: k = 0
2. while not converged:
   a. 计算梯度 g^k = ∇E(v^k)
   b. 计算残差 r^k = |g^k| / Δt
   c. if r^k < ε: converged = true; break
   d. 计算 Hessian H^k = ∇²E(v^k)
   e. 求解线性系统: H^k Δv = -g^k
   f. 线搜索确定步长 α
   g. 更新: v^{k+1} = v^k + α Δv
   h. k = k + 1
```

#### 3.2.2 核心实现

```cpp
NewtonState NewtonsMethod::solve(const double& dt, 
                                  symx::GlobalEnergy& global_energy, ...) {
    while (newton_state == NewtonState::Running) {
        step_newton_it++;
        
        // 1. 组装线性系统
        symx::Assembled assembled = _evaluate_E_grad_hess();
        
        // 2. 计算残差
        residual = _compute_residual(*assembled.grad, dt);
        
        // 3. 检查收敛
        if (residual_max < epsilon_residual) {
            newton_state = NewtonState::Successful;
            break;
        }
        
        // 4. 求解线性系统
        _solve_linear_system(du, assembled, dt);
        
        // 5. 线搜索
        _inplace_backtracking_line_search(du, E0, E, ...);
    }
    return newton_state;
}
```

#### 3.2.3 线性求解器

Stark 支持两种线性求解器：

**1. 直接求解器 (Direct LU)**
```cpp
Eigen::SparseLU<Eigen::SparseMatrix<double>> lu;
lu.analyzePattern(s);
lu.factorize(s);
du = lu.solve(rhs);
```

**2. 迭代求解器 (Conjugate Gradient)**
```cpp
cg::solve<double>(du.data(), rhs.data(), size, cg_tol, max_iterations,
    [&](double* b, const double* x) { lhs.spmxv_from_ptr(b, x, n_threads); },
    [&](double* z, const double* r) { lhs.apply_preconditioning(z, r, n_threads); }
);
```

### 3.3 IPC 接触模型

IPC (Incremental Potential Contact) 是 Stark 的核心创新之一，实现了无穿透的鲁棒接触处理。

#### 3.3.1 势垒函数

IPC 使用势垒函数来防止穿透。Stark 支持两种势垒类型：

**Cubic 势垒**：
$$b(d) = \frac{k}{3}(d̂ - d)^3, \quad d < d̂$$

**Log 势垒**：
$$b(d) = -k(d̂ - d)^2 \ln\left(\frac{d}{d̂}\right), \quad d < d̂$$

其中：
- $d$ 是距离
- $d̂$ 是接触厚度
- $k$ 是接触刚度

**实现代码**：
```cpp
symx::Scalar _barrier_potential(const symx::Scalar& d, 
                                 const symx::Scalar& dhat, 
                                 const symx::Scalar& k) {
    if (ipc_barrier_type == IPCBarrierType::Cubic) {
        return k * (dhat - d).powN(3) / 3.0;
    }
    else if (ipc_barrier_type == IPCBarrierType::Log) {
        return -k * (dhat - d).powN(2) * log(d / dhat);
    }
}
```

#### 3.3.2 距离计算

IPC 需要计算各种几何图元之间的距离：

**点-点距离**：
$$d_{pp} = \|p - q\|$$

**点-边距离**：
$$d_{pe} = \min_{t \in [0,1]} \|p - (e_0 + t(e_1 - e_0))\|$$

**点-三角形距离**：
$$d_{pt} = \min_{u,v,w \geq 0, u+v+w=1} \|p - (u t_0 + v t_1 + w t_2)\|$$

**边-边距离**：
$$d_{ee} = \min_{s,t \in [0,1]} \|(e_a^0 + s(e_a^1 - e_a^0)) - (e_b^0 + t(e_b^1 - e_b^0))\|$$

#### 3.3.3 边-边软化器

为处理边-边接触的退化情况，IPC 引入软化器：

$$n(e_a, e_b) = (e_a^1 - e_a^0) \times (e_b^1 - e_b^0)$$
$$\epsilon_x = 10^{-3} \|e_a\|^2 \|e_b\|^2$$
$$m(x) = \begin{cases} 1 & x > \epsilon_x \\ (-x/\epsilon_x + 2)(x/\epsilon_x) & x \leq \epsilon_x \end{cases}$$

### 3.4 摩擦模型

Stark 实现了 Coulomb 摩擦模型，支持 C0 和 C1 两种连续化版本。

#### 3.4.1 摩擦势能

**C0 摩擦模型**：
$$E_f = \begin{cases} \frac{1}{2}k u^2 & u < \epsilon_u \\ \mu f_n (u - \epsilon) & u \geq \epsilon_u \end{cases}$$

**C1 摩擦模型**：
$$E_f = \begin{cases} \mu f_n \left(-\frac{u^3}{3\epsilon_u^2} + \frac{u^2}{\epsilon_u} + \frac{\epsilon_u}{3}\right) & u < \epsilon_u \\ \mu f_n u & u \geq \epsilon_u \end{cases}$$

其中：
- $u$ 是切向位移
- $\mu$ 是摩擦系数
- $f_n$ 是法向力
- $\epsilon_u$ 是粘滞-滑动阈值

**实现代码**：
```cpp
symx::Scalar _friction_potential(const symx::Vector& v, 
                                  const symx::Scalar& fn, 
                                  const symx::Scalar& mu, ...) {
    symx::Vector vt = T * v;  // 切向速度
    symx::Scalar u = (vt * dt).norm();  // 切向位移
    
    if (ipc_friction_type == IPCFrictionType::C0) {
        symx::Scalar E_stick = 0.5 * k * u.powN(2);
        symx::Scalar E_slide = mu * fn * (u - eps);
        return symx::branch(u < epsu, E_stick, E_slide);
    }
    // C1 版本...
}
```

### 3.5 弹性材料模型

#### 3.5.1 稳定 Neo-Hookean 模型

Stark 采用 Smith 等人 2022 年提出的稳定 Neo-Hookean 模型：

$$\Psi = \frac{\mu'}{2}(I_c - 3) + \frac{\lambda'}{2}(J - \alpha)^2 - \frac{\mu'}{2}\ln(I_c + 1)$$

其中：
- $I_c = \|F\|_F^2$ 是变形梯度的 Frobenius 范数平方
- $J = \det(F)$ 是变形梯度的行列式
- $\mu' = \frac{4}{3}\mu$
- $\lambda' = \lambda + \frac{5}{6}\mu$
- $\alpha = 1 + \frac{\mu'}{\lambda'} - \frac{\mu'}{4\lambda'}$

**实现代码**：
```cpp
symx::Scalar elastic_energy_density = 
    0.5 * mu_ * (Ic - 3.0) + 
    0.5 * lambda_ * (detF - alpha).powN(2) - 
    0.5 * mu_ * symx::log(Ic + 1.0);
```

#### 3.5.2 应变阻尼

Stark 支持材料阻尼，通过应变率实现：

$$E_{damping} = \frac{1}{2}\beta V_0 \|\dot{E}\|_F^2$$

其中 $\beta$ 是阻尼系数，$V_0$ 是参考体积。

#### 3.5.3 应变限制

为防止过度变形，Stark 实现了应变限制：

$$E_{limit} = \frac{k_{limit}}{3}\sum_{i=1}^{3} \max(0, \lambda_i - \lambda_{max})^3$$

其中 $\lambda_i$ 是 Green-Lagrange 应变张量的特征值。

---

## 4. 模块详细分析

### 4.1 SymX 符号计算引擎

SymX 是 Stark 的核心创新，实现了符号微分和代码生成。

#### 4.1.1 架构设计

```
┌─────────────────────────────────────────────────────────────┐
│                    SymX Architecture                         │
│                                                              │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────┐ │
│  │   Scalar    │    │   Vector    │    │     Matrix      │ │
│  │  (标量符号) │    │  (向量符号) │    │   (矩阵符号)    │ │
│  └──────┬──────┘    └──────┬──────┘    └────────┬────────┘ │
│         │                  │                     │          │
│         └──────────────────┼─────────────────────┘          │
│                            │                                │
│                            ▼                                │
│                   ┌─────────────────┐                       │
│                   │     Energy      │                       │
│                   │   (能量定义)    │                       │
│                   └────────┬────────┘                       │
│                            │                                │
│                            ▼                                │
│                   ┌─────────────────┐                       │
│                   │      diff       │                       │
│                   │   (自动微分)    │                       │
│                   └────────┬────────┘                       │
│                            │                                │
│                            ▼                                │
│                   ┌─────────────────┐                       │
│                   │  Compilation    │                       │
│                   │  (代码生成)     │                       │
│                   └────────┬────────┘                       │
│                            │                                │
│                            ▼                                │
│                   ┌─────────────────┐                       │
│                   │   Compiled      │                       │
│                   │  (编译执行)     │                       │
│                   └─────────────────┘                       │
└─────────────────────────────────────────────────────────────┘
```

#### 4.1.2 核心类

**GlobalEnergy** - 全局能量管理：
```cpp
class GlobalEnergy {
public:
    std::vector<std::unique_ptr<Energy>> energies;  // 能量列表
    std::vector<std::function<double*()>> dof_data; // 自由度数据
    Assembly assembly;                              // 组装结果
    
    void add_energy(std::string name, ...);         // 添加能量
    void compile(std::string working_directory);    // 编译
    Assembled evaluate_E_grad_hess();               // 计算导数
};
```

**Energy** - 单个能量定义：
```cpp
class Energy {
public:
    CompiledInLoop<double> E;    // 能量函数
    CompiledInLoop<double> dE;   // 能量+梯度
    CompiledInLoop<double> hE;   // 能量+梯度+Hessian
    
    void set(const Scalar& expr);  // 设置能量表达式
    
    // 创建符号
    Scalar make_scalar(...);
    Vector make_vector(...);
    Matrix make_matrix(...);
};
```

#### 4.1.3 使用示例

定义四面体应变能量：
```cpp
stark.global_energy.add_energy("EnergyTetStrain", conn,
    [&](symx::Energy& energy, symx::Element& conn) {
        // 创建符号变量
        std::vector<symx::Vector> v1 = energy.make_dof_vectors(dof, v1_data, tet);
        std::vector<symx::Vector> x0 = energy.make_vectors(x0_data, tet);
        std::vector<symx::Vector> X = energy.make_vectors(X_data, tet);
        
        // 时间积分
        std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);
        
        // 计算变形梯度
        symx::Matrix F = compute_deformation_gradient(x1, X);
        
        // 计算能量密度
        symx::Scalar psi = neo_hookean_energy(F, mu, lambda);
        
        // 设置总能量
        energy.set(tet_volume * psi);
    }
);
```

### 4.2 碰撞检测模块

#### 4.2.1 架构设计

```
TriangleMeshCollisionDetection/
├── BroadPhaseBase.h        # 宽相检测基类
├── BroadPhaseET.h          # 边-三角形宽相
├── BroadPhasePTEE.h        # 点-三角形/边-边宽相
├── ProximityDetection.h    # 近距离检测
├── IntersectionDetection.h # 穿透检测
├── Octree.h                # 八叉树加速结构
├── AABBs.h                 # 轴对齐包围盒
└── Meshes.h                # 网格数据结构
```

#### 4.2.2 检测流程

```
1. 宽相检测 (Broad Phase)
   ├── 构建八叉树/AABB
   └── 快速排除不可能碰撞的图元对
   
2. 窄相检测 (Narrow Phase)
   ├── 点-三角形检测
   └── 边-边检测
   
3. 距离计算
   ├── 点-点距离
   ├── 点-边距离
   ├── 点-三角形距离
   └── 边-边距离
```

#### 4.2.3 近距离检测

```cpp
const tmcd::ProximityResults& _run_proximity_detection(core::Stark& stark, double dt) {
    _update_vertices(stark, dt);
    
    pd.set_n_threads(stark.settings.execution.n_threads);
    pd.activate_point_triangle(global_params.triangle_point_enabled);
    pd.activate_edge_edge(global_params.edge_edge_enabled);
    
    double max_thickness = *std::max_element(contact_thicknesses.begin(), 
                                              contact_thicknesses.end());
    return pd.run(2.0 * max_thickness);
}
```

### 4.3 可变形体模块

#### 4.3.1 PointDynamics - 点动力学

管理所有点的位置、速度、加速度：

```cpp
class PointDynamics {
public:
    IntervalVector<Eigen::Vector3d> X;   // 静止位置
    IntervalVector<Eigen::Vector3d> x0;  // 当前位置 (时刻 n)
    IntervalVector<Eigen::Vector3d> v1;  // 当前速度 (时刻 n+1)
    symx::DoF dof;                       // 自由度句柄
    
    PointSetHandler add(const std::vector<Eigen::Vector3d>& x);
    Eigen::Vector3d get_x1(int global_index, double dt) const;
};
```

#### 4.3.2 EnergyTetStrain - 四面体应变

四面体有限元应变能量：

```cpp
class EnergyTetStrain {
public:
    struct Params {
        double youngs_modulus;    // 杨氏模量
        double poissons_ratio;    // 泊松比
        double damping;           // 阻尼系数
        double strain_limit;      // 应变限制
    };
    
    Handler add(const PointSetHandler& set, 
                const std::vector<std::array<int, 4>>& tets,
                const Params& params);
};
```

#### 4.3.3 EnergyDiscreteShells - 离散壳

布料和薄壳的弯曲能量：

```cpp
class EnergyDiscreteShells {
public:
    struct Params {
        double stiffness;         // 弯曲刚度
        double damping;           // 阻尼系数
        bool flat_rest_angle;     // 是否使用平坦静止角
    };
    
    Handler add(const PointSetHandler& set,
                const std::vector<std::array<int, 3>>& triangles,
                const Params& params);
};
```

### 4.4 刚体模块

#### 4.4.1 RigidBodyDynamics - 刚体动力学

```cpp
class RigidBodyDynamics {
public:
    std::vector<Eigen::Vector3d> t0, t1;      // 平移
    std::vector<Eigen::Quaterniond> q0, q1;   // 四元数旋转
    std::vector<Eigen::Vector3d> v0, v1;      // 线速度
    std::vector<Eigen::Vector3d> w0, w1;      // 角速度
    symx::DoF dof_v, dof_w;                   // 自由度
    
    // 获取世界坐标位置
    symx::Vector get_x1(symx::Energy& energy, 
                        const symx::Index& rb_idx,
                        const symx::Vector& x_loc, 
                        const symx::Scalar& dt);
};
```

#### 4.4.2 刚体变换

```cpp
// 局部坐标到世界坐标
Eigen::Vector3d local_to_global_point(const Eigen::Vector3d& x_loc,
                                       const Eigen::Matrix3d& R,
                                       const Eigen::Vector3d& t) {
    return R * x_loc + t;
}

// 四元数时间积分
Eigen::Quaterniond quat_time_integration(const Eigen::Quaterniond& q0,
                                          const Eigen::Vector3d& w,
                                          double dt) {
    Eigen::Quaterniond dq;
    dq.w() = 0.0;
    dq.vec() = 0.5 * w;
    Eigen::Quaterniond q1 = q0 + dt * (dq * q0);
    return q1.normalized();
}
```

---

## 5. 关键设计模式

### 5.1 Handler 模式

Stark 使用 Handler 模式管理对象引用：

```cpp
struct Handler {
private:
    EnergyTetStrain* model;
    int idx;
    
public:
    int get_idx() const { return idx; }
    bool is_valid() const { return model != nullptr; }
    void exit_if_not_valid(const std::string& where) const;
};
```

**优点**：
- 类型安全
- 易于扩展
- 支持链式调用

### 5.2 回调机制

```cpp
class Callbacks {
public:
    std::vector<std::function<void()>> before_time_step;
    std::vector<std::function<void()>> before_energy_evaluation;
    std::vector<std::function<bool()>> is_intermidiate_state_valid;
    std::vector<std::function<void()>> on_time_step_accepted;
    
    void run_before_time_step();
    void run_before_energy_evaluation();
    bool run_is_intermidiate_state_valid();
};
```

**使用场景**：
- 碰撞检测更新
- 状态验证
- 事件触发

### 5.3 参数验证宏

```cpp
#define STARK_PARAM_POSITIVE(type, name, default) \
    type name = default; \
    void set_##name(const type& v) { \
        if (v <= 0) { /* 错误处理 */ } \
        this->name = v; \
    }

#define STARK_PARAM_NON_NEGATIVE(type, name, default) \
    type name = default; \
    void set_##name(const type& v) { \
        if (v < 0) { /* 错误处理 */ } \
        this->name = v; \
    }
```

### 5.4 IntervalVector 数据结构

用于高效管理分组数据：

```cpp
template<typename T>
class IntervalVector {
public:
    std::vector<T> data;
    std::vector<int> begin_indices;
    std::vector<int> end_indices;
    
    T& get(int group, int local_idx);
    int get_global_index(int group, int local_idx);
    void append(const std::vector<T>& items);
};
```

---

## 6. 性能优化策略

### 6.1 并行计算

#### 6.1.1 OpenMP 多线程

```cpp
#pragma omp parallel for schedule(static) num_threads(n_threads)
for (int i = 0; i < n; i++) {
    // 并行计算
}
```

#### 6.1.2 块稀疏矩阵

```cpp
// 使用 3x3 块结构提高缓存效率
bsm::BlockedSparseMatrix<3, 3, double> hess;

// 块对角预条件
lhs.set_preconditioner(bsm::Preconditioner::BlockDiagonal);
lhs.prepare_preconditioning(n_threads);
```

### 6.2 内存管理

#### 6.2.1 预分配缓冲区

```cpp
// NewtonsMethod 预分配缓冲区
Eigen::VectorXd du;
Eigen::VectorXd u0, u1;
Eigen::VectorXd residual;

du.resize(ndofs);
u0.resize(ndofs);
```

#### 6.2.2 避免数据拷贝

```cpp
// 使用引用和指针
std::vector<symx::Vector> make_vectors(std::function<const double*()> data, ...);
```

### 6.3 自适应刚度

```cpp
void _on_intermidiate_state_invalid(core::Stark& stark) {
    // 穿透无法避免时，增加接触刚度
    contact_stiffness *= 2.0;
}

void _on_time_step_accepted(core::Stark& stark) {
    // 时间步成功后，逐渐降低刚度
    contact_stiffness = std::max(min_contact_stiffness, 
                                  0.99 * contact_stiffness);
}
```

---

## 7. 扩展开发指南

### 7.1 添加新的能量模型

**步骤 1**：创建能量类

```cpp
class EnergyMyModel {
public:
    struct Params { /* 参数定义 */ };
    struct Handler { /* 句柄定义 */ };
    
    EnergyMyModel(core::Stark& stark, spPointDynamics dyn);
    Handler add(const PointSetHandler& set, const Params& params);
};
```

**步骤 2**：定义能量函数

```cpp
stark.global_energy.add_energy("EnergyMyModel", conn,
    [&](symx::Energy& energy, symx::Element& conn) {
        // 1. 创建符号变量
        // 2. 计算能量表达式
        // 3. 设置能量
        energy.set(E);
    }
);
```

**步骤 3**：注册回调（如需要）

```cpp
stark.callbacks.add_before_time_step([&]() { /* ... */ });
```

### 7.2 添加新的约束

```cpp
class EnergyMyConstraint {
    // 约束通常作为软约束实现
    // E = 0.5 * k * (constraint_value)^2
    
    symx::Scalar constraint_value = /* 约束表达式 */;
    symx::Scalar E = 0.5 * stiffness * constraint_value.powN(2);
    energy.set(E);
};
```

### 7.3 Python 绑定

使用 nanobind 创建 Python 绑定：

```cpp
#include <nanobind/nanobind.h>
namespace nb = nanobind;

NB_MODULE(pystark, m) {
    nb::class_<EnergyMyModel>(m, "EnergyMyModel")
        .def("add", &EnergyMyModel::add);
}
```

---

## 8. 总结

### 8.1 核心优势

1. **统一的仿真框架**：刚体、可变形体、接触摩擦统一处理
2. **符号微分引擎**：自动生成导数，简化模型开发
3. **鲁棒的接触处理**：IPC 方法保证无穿透
4. **高性能实现**：并行计算、块稀疏矩阵、自适应策略

### 8.2 适用场景

- 机器人与可变形物体交互仿真
- 布料和软体仿真
- 刚体动力学与约束
- 研究和开发新的物理模型

### 8.3 学习建议

1. **入门**：从 Python API 开始，运行示例场景
2. **进阶**：阅读 C++ 源码，理解 SymX 引擎
3. **深入**：研究 IPC 论文，理解接触处理
4. **实践**：尝试添加新的能量模型

---

## 参考文献

1. Fernández-Fernández et al., "STARK: A Unified Framework for Strongly Coupled Simulation of Rigid and Deformable Bodies with Frictional Contact", ICRA 2024
2. Smith et al., "Stable Neo-Hookean Flesh Simulation", ACM TOG 2022
3. Li et al., "Incremental Potential Contact", ACM TOG 2020
4. Grinspun et al., "Discrete Shells", SCA 2003
