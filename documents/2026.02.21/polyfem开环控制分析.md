# PolyFEM 开环控制详细分析

## 1. 概述

PolyFEM 是一个基于有限元方法（FEM）的多物理场模拟框架，支持线性和非线性问题、稳态和瞬态分析。该系统的"开环控制"主要体现在**障碍物（Obstacle）**和**边界条件（Boundary Conditions）**的控制机制上，通过预设的位移序列或时间相关的边界条件来驱动模拟，而不受系统内部物理状态的反馈影响。

---

## 2. 核心架构与模块划分

### 2.1 系统架构图

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              State                                       │
│                         (主状态管理类)                                    │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐     │
│  │    Mesh     │  │    Bases    │  │  Assembler  │  │  Problem    │     │
│  │   (网格)     │  │   (基函数)   │  │   (装配器)   │  │  (问题定义)  │     │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘     │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
            ┌───────────────────────┼───────────────────────┐
            ▼                       ▼                       ▼
┌───────────────────┐   ┌───────────────────┐   ┌───────────────────┐
│     Obstacle      │   │  SolveData        │   │  TimeIntegrator   │
│    (障碍物/开环)   │   │  (求解数据)        │   │  (时间积分器)      │
└───────────────────┘   └───────────────────┘   └───────────────────┘
            │                       │                       │
            ▼                       ▼                       ▼
┌───────────────────┐   ┌───────────────────┐   ┌───────────────────┐
│  BCLagrangianForm │   │   NLProblem       │   │ ImplicitEuler/    │
│  (边界约束形式)    │   │  (非线性问题)      │   │ ImplicitNewmark   │
└───────────────────┘   └───────────────────┘   └───────────────────┘
```

### 2.2 关键类职责说明

| 类名 | 文件路径 | 职责 |
|------|----------|------|
| `State` | `State.hpp/cpp` | 主状态管理类，包含网格、基函数、装配器、问题定义等所有状态 |
| `Obstacle` | `mesh/Obstacle.hpp/cpp` | **开环控制的核心**，管理障碍物的几何和位移序列 |
| `BCLagrangianForm` | `solver/forms/lagrangian/BCLagrangianForm.hpp/cpp` | 边界条件的增广拉格朗日形式，强制执行Dirichlet边界条件 |
| `ImplicitTimeIntegrator` | `time_integrator/ImplicitTimeIntegrator.hpp` | 隐式时间积分器基类，管理时间步进 |
| `ImplicitEuler` | `time_integrator/ImplicitEuler.hpp/cpp` | 隐式欧拉时间积分器 |
| `ImplicitNewmark` | `time_integrator/ImplicitNewmark.hpp/cpp` | 隐式Newmark时间积分器 |
| `SolveData` | `solver/SolveData.hpp/cpp` | 求解数据，包含所有形式（Form）和求解器配置 |
| `NLProblem` | `solver/NLProblem.hpp/cpp` | 非线性问题定义，组合所有形式构成优化问题 |

---

## 3. 开环控制的核心机制

### 3.1 障碍物（Obstacle）开环控制

PolyFEM 中的**障碍物（Obstacle）**是实现开环控制的主要机制。障碍物可以具有预设的位移序列，这些位移完全由用户定义，不受系统内部物理状态的影响。

#### 3.1.1 数据结构定义

在 `Obstacle` 类中（`mesh/Obstacle.hpp`），开环控制的数据定义如下：

```cpp
class Obstacle {
private:
    int dim_;                           // 维度 (2D或3D)
    Eigen::MatrixXd v_;                 // 顶点位置
    Eigen::VectorXi codim_v_;           // 共维顶点
    Eigen::MatrixXi f_;                 // 面
    Eigen::MatrixXi e_;                 // 边
    
    // 连接性信息
    Eigen::VectorXi in_v_;              // 顶点连接性
    Eigen::MatrixXi in_f_;              // 面连接性
    Eigen::MatrixXi in_e_;              // 边连接性
    
    // 关键：位移定义（开环控制的核心）
    std::vector<assembler::TensorBCValue> displacements_;
    
    std::vector<int> endings_;          // 每个网格的结束索引
    std::vector<Plane> planes_;         // 平面障碍物
};
```

#### 3.1.2 位移定义方式

PolyFEM 支持三种开环控制模式：

**模式一：JSON表达式驱动**

通过JSON配置定义位移表达式：

```cpp
void Obstacle::append_mesh(
    const Eigen::MatrixXd &vertices,
    const Eigen::VectorXi &codim_vertices,
    const Eigen::MatrixXi &codim_edges,
    const Eigen::MatrixXi &faces,
    const json &displacement)
{
    append_mesh(vertices, codim_vertices, codim_edges, faces);
    
    displacements_.emplace_back();
    for (size_t d = 0; d < dim_; ++d) {
        assert(displacement["value"].is_array());
        displacements_.back().value[d].init(displacement["value"][d]);
    }
    
    // 插值方式
    if (displacement.contains("interpolation")) {
        // 处理插值...
    }
}
```

**模式二：关键帧序列驱动**

通过顶点序列定义关键帧动画：

```cpp
void Obstacle::append_mesh_sequence(
    const std::vector<Eigen::MatrixXd> &vertices,  // 顶点序列（关键帧）
    const Eigen::VectorXi &codim_vertices,
    const Eigen::MatrixXi &codim_edges,
    const Eigen::MatrixXi &faces,
    const int fps)  // 帧率
{
    if (vertices.size() == 0 || vertices[0].size() == 0)
        return;
    
    append_mesh(vertices[0], codim_vertices, codim_edges, faces);
    
    // 计算每帧的位移
    std::array<std::vector<Eigen::MatrixXd>, 3> displacements_xyz;
    for (size_t i = 0; i < vertices.size(); ++i) {
        Eigen::MatrixXd displacement = vertices[i] - vertices[0];
        for (size_t d = 0; d < dim_; ++d)
            displacements_xyz[d].push_back(displacement.col(d));
    }
    
    // 定义时间相关的位移函数
    displacements_.emplace_back();
    for (size_t d = 0; d < dim_; ++d) {
        const std::vector<Eigen::MatrixXd> displacements = displacements_xyz[d];
        displacements_.back().value[d].init(
            [displacements, fps](double x, double y, double z, double t, int index) -> double {
                const double frame = t * fps;
                const double interp = frame - floor(frame);
                const int frame0 = (int)floor(frame);
                const int frame1 = (int)ceil(frame);
                if (frame1 >= displacements.size())
                    return displacements.back()(index);
                const double u0 = displacements[frame0](index);
                const double u1 = displacements[frame1](index);
                return (u1 - u0) * interp + u0;  // 线性插值
            });
    }
}
```

**模式三：函数对象驱动**

通过C++函数对象定义任意位移：

```cpp
void Obstacle::change_displacement(
    const int oid, 
    const std::function<Eigen::MatrixXd(double x, double y, double z, double t)> &func,
    const std::shared_ptr<Interpolation> &interp)
{
    for (size_t k = 0; k < displacements_.back().value.size(); ++k)
        displacements_[oid].value[k].init(func, k);
    
    displacements_[oid].interpolation.clear();
    displacements_[oid].interpolation.push_back(interp);
}
```

#### 3.1.3 位移更新机制

在每个时间步，障碍物的位移会被更新：

```cpp
void Obstacle::update_displacement(const double t, Eigen::MatrixXd &sol) const
{
    // 假设障碍物位移存储在解向量的底部
    const int offset = sol.rows() - v_.rows() * (sol.cols() == 1 ? dim_ : 1);
    
    int start = 0;
    for (int k = 0; k < endings_.size(); ++k) {
        const int to = endings_[k];
        const auto &disp = displacements_[k];
        
        for (int i = start; i < to; ++i) {
            for (int d = 0; d < dim_; ++d) {
                const int sol_row = sol.cols() == 1 ? (offset + i * dim_ + d) : (offset + i);
                const int sol_col = sol.cols() == 1 ? 0 : d;
                // 计算并应用位移
                sol(sol_row, sol_col) = disp.eval(v_.row(i), d, t, i - start);
            }
        }
        start = to;
    }
}
```

### 3.2 边界条件（Boundary Condition）开环控制

#### 3.2.1 Dirichlet边界条件

Dirichlet边界条件是另一种开环控制机制，通过 `BCLagrangianForm` 实现：

```cpp
class BCLagrangianForm : public AugmentedLagrangianForm {
private:
    const std::vector<int> &boundary_nodes_;        // 边界节点索引
    const assembler::RhsAssembler *rhs_assembler_;  // RHS装配器
    const bool is_time_dependent_;                  // 是否时间相关
    
    Eigen::VectorXi constraints_;                   // 约束索引
    Eigen::VectorXi not_constraints_;               // 非约束索引
    
    StiffnessMatrix masked_lumped_mass_sqrt_;       // 质量矩阵平方根
    StiffnessMatrix masked_lumped_mass_;            // 质量矩阵
    
    // 目标值（开环控制输入）
    Eigen::MatrixXd b_;                             // 目标边界值
    Eigen::MatrixXd b_proj_;                        // 投影后的目标值
};
```

#### 3.2.2 增广拉格朗日约束实现

```cpp
double BCLagrangianForm::value_unweighted(const Eigen::VectorXd &x) const
{
    // 计算当前解与目标值的差
    const Eigen::VectorXd dist = A_ * x - b_;
    
    // 拉格朗日惩罚项: -λᵀ√M·dist
    const double L_penalty = -lagr_mults_.transpose() * masked_lumped_mass_sqrt_ * dist;
    
    // 增广惩罚项: 0.5·distᵀ·M·dist
    const double A_penalty = 0.5 * dist.transpose() * masked_lumped_mass_ * dist;
    
    return L_weight() * L_penalty + A_weight() * A_penalty;
}

void BCLagrangianForm::first_derivative_unweighted(
    const Eigen::VectorXd &x, 
    Eigen::VectorXd &gradv) const
{
    // 梯度计算
    gradv = L_weight() * A_.transpose() 
            * (-(masked_lumped_mass_sqrt_ * lagr_mults_) 
               + A_weight() * (masked_lumped_mass_ * (A_ * x - b_)));
}
```

#### 3.2.3 时间相关的边界条件更新

```cpp
void BCLagrangianForm::update_target(const double t)
{
    assert(rhs_assembler_ != nullptr);
    
    b_.setZero(n_dofs_, 1);
    
    // 从RHS装配器获取时间t的边界条件
    rhs_assembler_->set_bc(
        *local_boundary_, boundary_nodes_, n_boundary_samples_,
        *local_neumann_boundary_, b_, Eigen::MatrixXd(), t);
    
    b_proj_ = b_;
    b_ = igl::slice(b_, constraints_, 1);
}
```

---

## 4. 耦合关系分析

### 4.1 控制-物理耦合层次

```
┌────────────────────────────────────────────────────────────────────────┐
│                         开环控制层 (Obstacle/BC)                        │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                     │
│  │  JSON表达式  │  │  关键帧序列  │  │  函数对象   │                     │
│  │  驱动       │  │  驱动       │  │  驱动       │                     │
│  │             │  │             │  │             │                     │
│  │ displacements_│ │ vertices[]  │  │ std::function│                     │
│  └─────────────┘  └─────────────┘  └─────────────┘                     │
└────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼ (通过增广拉格朗日约束耦合)
┌────────────────────────────────────────────────────────────────────────┐
│                      优化求解层 (NLProblem)                             │
│  ┌──────────────────────────────────────────────────────────────┐     │
│  │  Objective: f(x) = Σ wᵢ·fᵢ(x)                                 │     │
│  │  - ElasticForm: 弹性势能                                      │     │
│  │  - InertiaForm: 惯性项                                        │     │
│  │  - ContactForm: 接触约束 (IPC障碍)                             │     │
│  │  - BCLagrangianForm: 边界条件约束                              │     │
│  │  - BodyForm: 体积力                                           │     │
│  └──────────────────────────────────────────────────────────────┘     │
└────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼ (通过时间积分器耦合)
┌────────────────────────────────────────────────────────────────────────┐
│                      时间推进层 (Time Integration)                      │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                     │
│  │  Implicit   │  │  Implicit   │  │     BDF     │                     │
│  │  Euler      │  │  Newmark    │  │             │                     │
│  │             │  │             │  │             │                     │
│  │ v=(x-x₀)/h  │  │ v=2(x-x₀)/h │  │ 多步积分    │                     │
│  │ -v₀        │  │ -v₀         │  │             │                     │
│  └─────────────┘  └─────────────┘  └─────────────┘                     │
└────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼ (通过牛顿迭代耦合)
┌────────────────────────────────────────────────────────────────────────┐
│                      求解器层 (Solver)                                  │
│  ┌──────────────────────────────────────────────────────────────┐     │
│  │  ALSolver: 增广拉格朗日求解器                                  │     │
│  │  - 内部迭代: 非线性求解 (Newton/LBFGS)                        │     │
│  │  - 外部迭代: 更新拉格朗日乘子                                 │     │
│  │  - 摩擦迭代: 处理摩擦滞后                                     │     │
│  └──────────────────────────────────────────────────────────────┘     │
└────────────────────────────────────────────────────────────────────────┘
```

### 4.2 关键耦合方程

#### 4.2.1 障碍物约束耦合

障碍物的位移作为硬约束嵌入优化问题：

```
minimize    f(x) = E_elastic(x) + E_inertia(x) + E_contact(x)
subject to  x_obstacle = x_preset(t),  ∀obstacle vertices
```

通过增广拉格朗日方法转化为无约束问题：

```
L_AL(x, λ, κ) = f(x) + Σᵢ [κ/2·||xᵢ - x_presetᵢ(t)||² - λᵢ·(xᵢ - x_presetᵢ(t))]
```

#### 4.2.2 时间积分耦合

隐式欧拉时间积分：

```
x_tilde = x_prev + h·v_prev                    (预测位置)
v = (x - x_prev) / h                           (速度更新)
a = (v - v_prev) / h                           (加速度更新)

惯性势能:
E_inertia = 0.5·(x - x_tilde)ᵀ·M·(x - x_tilde) / (h²·β)
```

隐式Newmark时间积分：

```
x_tilde = x_prev + h·v_prev + 0.5·h²·a_prev     (预测位置)
v = γ·(x - x_tilde) / (β·h)                     (速度更新)
a = (x - x_tilde) / (β·h²)                      (加速度更新)
```

#### 4.2.3 接触-控制耦合

IPC（增量势能接触）障碍函数：

```
B(x) = Σ_{k ∈ C} b(d(x_k))

其中:
- b(d) = -(d - d̂)²·ln(d/d̂),  0 < d < d̂  (对数障碍函数)
- d(x_k) = ||Vᵢ(x) - Vⱼ(x)||              (顶点间距离)
- d̂ = barrier_activation_distance         (障碍激活距离)
```

### 4.3 耦合数据流

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          时间步进流程                                     │
└─────────────────────────────────────────────────────────────────────────┘

Time Step t → t+1:

1. 初始化求解
   ┌────────────────────────────────────────┐
   │ init_nonlinear_tensor_solve(sol, t)    │
   │ - 初始化时间积分器                      │
   │ - 初始化所有Form                       │
   │ - 构建NLProblem                        │
   └────────────────────────────────────────┘

2. 更新障碍物位移 (开环控制输入)
   ┌────────────────────────────────────────┐
   │ obstacle.update_displacement(t, sol)   │
   │ - 根据预设位移更新障碍物位置            │
   │ - 不受系统状态影响                      │
   └────────────────────────────────────────┘

3. 更新边界条件 (开环控制输入)
   ┌────────────────────────────────────────┐
   │ bc_form.update_quantities(t, sol)      │
   │ - 更新时间相关的Dirichlet边界           │
   │ - 计算目标边界值 b_                     │
   └────────────────────────────────────────┘

4. 求解非线性问题
   ┌────────────────────────────────────────┐
   │ solve_tensor_nonlinear(sol, t)         │
   │                                        │
   │ AL迭代:                                │
   │   1. 最小化 L(x) = f(x) + AL(x)        │
   │   2. 更新拉格朗日乘子 λ                │
   │   3. 增加惩罚系数 κ                    │
   │                                        │
   │ 摩擦滞后迭代:                          │
   │   1. 更新摩擦约束                      │
   │   2. 检查收敛                          │
   └────────────────────────────────────────┘

5. 更新时间积分器状态
   ┌────────────────────────────────────────┐
   │ time_integrator->update_quantities(sol)│
   │ - 更新 x_prev, v_prev, a_prev          │
   │ - 为下一时间步做准备                    │
   └────────────────────────────────────────┘

6. 保存结果
   ┌────────────────────────────────────────┐
   │ save_timestep(t0 + dt * t, t, ...)     │
   │ - 保存当前时间步的解                    │
   └────────────────────────────────────────┘
```

---

## 5. 理论基础

### 5.1 增量势能接触 (IPC) 理论

PolyFEM 使用 IPC 框架处理接触问题，核心思想是通过**对数障碍函数**在碰撞发生前产生排斥力。

#### 5.1.1 障碍函数

```
b(d) = {
    -(d - d̂)²·ln(d/d̂),  0 < d < d̂
    0,                  d ≥ d̂
}

特性：
- 当 d → 0⁺ 时，b(d) → +∞（防止穿透）
- 当 d = d̂ 时，b(d) = 0（障碍边界）
- 当 d > d̂ 时，b(d) = 0（无障碍作用）
```

#### 5.1.2 接触势能

```
E_contact(x) = κ·Σ_{k ∈ C} b(d(x_k))

其中:
- κ = barrier_stiffness  (障碍刚度)
- C = {(i,j) | d(x_i, x_j) < d̂}  (活跃约束集)
```

### 5.2 时间积分理论

#### 5.2.1 隐式欧拉 (Implicit Euler)

```cpp
void ImplicitEuler::update_quantities(const Eigen::VectorXd &x)
{
    const Eigen::VectorXd v = compute_velocity(x);
    set_a_prev(compute_acceleration(v));
    set_v_prev(v);
    set_x_prev(x);
}

Eigen::VectorXd ImplicitEuler::x_tilde() const
{
    return x_prev() + dt() * v_prev();  // 预测位置
}

Eigen::VectorXd ImplicitEuler::compute_velocity(const Eigen::VectorXd &x) const
{
    return (x - x_prev()) / dt();  // 速度 = 位移变化/时间步长
}

Eigen::VectorXd ImplicitEuler::compute_acceleration(const Eigen::VectorXd &v) const
{
    return (v - v_prev()) / dt();  // 加速度 = 速度变化/时间步长
}

double ImplicitEuler::acceleration_scaling() const
{
    return dt() * dt();  // 加速度缩放因子
}
```

#### 5.2.2 隐式Newmark (Implicit Newmark)

```cpp
void ImplicitNewmark::update_quantities(const Eigen::VectorXd &x)
{
    const Eigen::VectorXd v = compute_velocity(x);
    set_a_prev(compute_acceleration(v));
    set_v_prev(v);
    set_x_prev(x);
}

Eigen::VectorXd ImplicitNewmark::x_tilde() const
{
    // 预测位置包含速度和加速度贡献
    return x_prev() + dt() * v_prev() + (0.5 - beta_) * dt() * dt() * a_prev();
}

Eigen::VectorXd ImplicitNewmark::compute_velocity(const Eigen::VectorXd &x) const
{
    // 速度更新包含加速度贡献
    return gamma_ * (x - x_tilde()) / (beta_ * dt()) 
           + (1 - gamma_ / (2 * beta_)) * v_prev()
           + (1 - gamma_ / (2 * beta_)) * dt() * a_prev();
}

double ImplicitNewmark::acceleration_scaling() const
{
    return beta_ * dt() * dt();  // 加速度缩放因子
}
```

### 5.3 优化理论

#### 5.3.1 目标函数组成

```
f(x) = w_elastic·E_elastic(x) 
     + w_inertia·E_inertia(x)
     + w_contact·E_contact(x)
     + w_friction·E_friction(x)
     + w_body·E_body(x)
     + w_AL·E_AL(x)

其中:
- E_elastic: 弹性势能
- E_inertia: 惯性势能 (时间相关)
- E_contact: 接触势能 (IPC障碍)
- E_friction: 摩擦耗散
- E_body: 体积力势能
- E_AL: 增广拉格朗日约束势能
```

#### 5.3.2 增广拉格朗日方法

```
// 增广拉格朗日函数
L(x, λ, κ) = f(x) + λᵀ·c(x) + κ/2·c(x)ᵀ·c(x)

其中:
- c(x) = A·x - b  (约束违反量)
- λ: 拉格朗日乘子
- κ: 惩罚系数

// 乘子更新
λ_{k+1} = λ_k - κ·c(x_k)

// 惩罚系数更新
κ_{k+1} = min(κ_max, scaling·κ_k)  (如果收敛慢)
```

---

## 6. 代码关键路径分析

### 6.1 瞬态非线性求解主流程

```cpp
void State::solve_transient_tensor_nonlinear(
    const int time_steps, 
    const double t0, 
    const double dt, 
    Eigen::MatrixXd &sol)
{
    // 初始化求解
    init_nonlinear_tensor_solve(sol, t0 + dt);
    
    // 保存初始解
    save_timestep(t0, save_i, t0, dt, sol, Eigen::MatrixXd());
    
    // 时间步进循环
    for (int t = 1; t <= time_steps; ++t) {
        // 1. 求解当前时间步
        solve_tensor_nonlinear(sol, t);
        
        // 2. 重网格化 (如果启用)
        if (remesh_enabled) {
            remesh(t0 + dt * t, dt, sol);
            solve_tensor_nonlinear(sol, t, false);  // 全局松弛
        }
        
        // 3. 保存结果
        save_timestep(t0 + dt * t, t, t0, dt, sol, Eigen::MatrixXd());
        
        // 4. 更新时间积分器
        solve_data.time_integrator->update_quantities(sol);
        
        // 5. 更新问题参数
        solve_data.nl_problem->update_quantities(t0 + (t + 1) * dt, sol);
        solve_data.update_dt();
        solve_data.update_barrier_stiffness(sol);
    }
}
```

### 6.2 非线性求解初始化

```cpp
void State::init_nonlinear_tensor_solve(
    Eigen::MatrixXd &sol, 
    const double t, 
    const bool init_time_integrator)
{
    // 1. 检查初始穿插
    if (is_contact_enabled()) {
        const Eigen::MatrixXd displaced = collision_mesh.displace_vertices(
            utils::unflatten(sol, mesh->dimension()));
        
        if (ipc::has_intersections(collision_mesh, displaced, ...)) {
            log_and_throw_error("Unable to solve, initial solution has intersections!");
        }
    }
    
    // 2. 初始化时间积分器
    if (problem->is_time_dependent()) {
        solve_data.time_integrator = 
            ImplicitTimeIntegrator::construct_time_integrator(args["time"]["integrator"]);
        
        Eigen::MatrixXd solution, velocity, acceleration;
        initial_solution(solution);
        initial_velocity(velocity);
        initial_acceleration(acceleration);
        
        const double dt = args["time"]["dt"];
        solve_data.time_integrator->init(solution, velocity, acceleration, dt);
    }
    
    // 3. 初始化所有Form
    const std::vector<std::shared_ptr<Form>> forms = solve_data.init_forms(
        // 各种参数...
        obstacle.ndof(),                    // 障碍物自由度
        args["constraints"]["hard"],        // 硬约束
        args["constraints"]["soft"],        // 软约束
        args["contact"]["enabled"],         // 接触启用
        // ... 更多参数
    );
    
    // 4. 初始化非线性问题
    solve_data.nl_problem = std::make_shared<NLProblem>(
        ndof, periodic_bc, t, forms, solve_data.al_form, ...);
    solve_data.nl_problem->init(sol);
    solve_data.nl_problem->update_quantities(t, sol);
}
```

### 6.3 单时间步非线性求解

```cpp
void State::solve_tensor_nonlinear(Eigen::MatrixXd &sol, const int t, const bool init_lagging)
{
    NLProblem &nl_problem = *(solve_data.nl_problem);
    
    // 1. 初始化滞后 (摩擦)
    if (nl_problem.uses_lagging() && init_lagging) {
        nl_problem.init_lagging(sol);
    }
    
    // 2. 创建增广拉格朗日求解器
    ALSolver al_solver(
        solve_data.al_form,
        args["solver"]["augmented_lagrangian"]["initial_weight"],
        args["solver"]["augmented_lagrangian"]["scaling"],
        args["solver"]["augmented_lagrangian"]["max_weight"],
        args["solver"]["augmented_lagrangian"]["eta"],
        [&](const Eigen::VectorXd &x) {
            this->solve_data.update_barrier_stiffness(sol);
        });
    
    // 3. 求解增广拉格朗日问题
    al_solver.solve_al(nl_problem, sol, ...);
    
    // 4. 求解降维问题
    al_solver.solve_reduced(nl_problem, sol, ...);
    
    // 5. 摩擦滞后迭代
    bool lagging_converged = !nl_problem.uses_lagging();
    for (int lag_i = 1; !lagging_converged; lag_i++) {
        // 更新滞后
        nl_problem.update_lagging(tmp_sol, lag_i);
        
        // 检查收敛
        nl_problem.gradient(tmp_sol, grad);
        if (grad.norm() <= lagging_tol) {
            lagging_converged = true;
            break;
        }
        
        // 继续迭代
        nl_solver->minimize(nl_problem, tmp_sol);
        sol = nl_problem.reduced_to_full(tmp_sol);
    }
}
```

---

## 7. 配置与使用

### 7.1 JSON 配置示例

```json
{
    "geometry": {
        "mesh": "mesh.obj",
        "obstacles": [
            {
                "mesh": "obstacle.obj",
                "displacement": {
                    "value": ["0", "0", "sin(t)"],
                    "interpolation": "linear"
                }
            }
        ]
    },
    "boundary_conditions": {
        "dirichlet_boundary": [
            {
                "id": 1,
                "value": ["0", "0", "0.1*t"]
            }
        ],
        "neumann_boundary": [
            {
                "id": 2,
                "value": ["0", "0", "-10"]
            }
        ]
    },
    "time": {
        "t0": 0,
        "dt": 0.01,
        "time_steps": 100,
        "integrator": "ImplicitEuler"
    },
    "solver": {
        "augmented_lagrangian": {
            "initial_weight": 1e4,
            "scaling": 10,
            "max_weight": 1e12,
            "eta": 0.5
        }
    },
    "contact": {
        "enabled": true,
        "dhat": 1e-3,
        "friction_coefficient": 0.5
    }
}
```

### 7.2 障碍物定义示例

```json
{
    "obstacles": [
        {
            "mesh": "moving_platform.obj",
            "type": "mesh",
            "displacement": {
                "value": ["0.1*t", "0", "0"],
                "interpolation": "linear"
            }
        },
        {
            "mesh": "rotating_cylinder.obj",
            "type": "mesh_sequence",
            "sequence": "cylinder_sequence_%04d.obj",
            "fps": 30
        },
        {
            "type": "plane",
            "point": [0, 0, 0],
            "normal": [0, 0, 1]
        }
    ]
}
```

---

## 8. 总结

### 8.1 开环控制特点

1. **预设轨迹**: 障碍物和边界条件的运动完全由用户预设的表达式、关键帧序列或函数对象决定
2. **无反馈**: 不受系统内部物理状态（如碰撞力、变形）的反馈影响
3. **硬约束**: 通过增广拉格朗日方法将开环控制作为硬约束强制执行
4. **时间驱动**: 所有开环控制都基于时间t，与系统状态解耦

### 8.2 耦合关系总结

| 耦合类型 | 耦合机制 | 数学表达 |
|----------|----------|----------|
| 控制-优化 | 增广拉格朗日 | `E_AL = κ/2·||x - x_preset(t)||² - λ·(x - x_preset(t))` |
| 优化-接触 | IPC障碍 | `E_contact = κ·Σ b(d(x_k))` |
| 优化-惯性 | 时间积分 | `E_inertia = 0.5·(x - x_tilde)ᵀ·M·(x - x_tilde)/(h²·β)` |
| 时间-空间 | 时间步进 | `x_tilde = x_prev + h·v_prev + (0.5-β)·h²·a_prev` |

### 8.3 应用场景

- **机械测试**: 预设夹具运动，测试材料响应
- **接触模拟**: 移动障碍物与变形体交互
- **边界驱动**: 通过边界条件驱动流体或固体流动
- **动画制作**: 关键帧驱动的物理动画

---

## 9. 参考文件

| 文件 | 路径 |
|------|------|
| 主状态类 | `src/polyfem/State.hpp/cpp` |
| 障碍物定义 | `src/polyfem/mesh/Obstacle.hpp/cpp` |
| 边界约束形式 | `src/polyfem/solver/forms/lagrangian/BCLagrangianForm.hpp/cpp` |
| 时间积分器基类 | `src/polyfem/time_integrator/ImplicitTimeIntegrator.hpp` |
| 隐式欧拉 | `src/polyfem/time_integrator/ImplicitEuler.hpp/cpp` |
| 隐式Newmark | `src/polyfem/time_integrator/ImplicitNewmark.hpp/cpp` |
| 非线性求解 | `src/polyfem/state/StateSolveNonlinear.cpp` |
| 求解数据 | `src/polyfem/solver/SolveData.hpp/cpp` |
| 非线性问题 | `src/polyfem/solver/NLProblem.hpp/cpp` |
| 增广拉格朗日求解器 | `src/polyfem/solver/ALSolver.hpp/cpp` |
