# PolyFEM 有限元库完整理论文档

## 目录

1. [项目概述](#1-项目概述)
2. [理论基础](#2-理论基础)
3. [系统架构](#3-系统架构)
4. [核心模块详解](#4-核心模块详解)
5. [支持的物理问题](#5-支持的物理问题)
6. [数值求解器](#6-数值求解器)
7. [输入输出系统](#7-输入输出系统)
8. [高级功能](#8-高级功能)
9. [使用示例](#9-使用示例)
10. [依赖库说明](#10-依赖库说明)

---

## 1. 项目概述

### 1.1 项目简介

PolyFEM 是一个功能强大的 C++ 有限元方法（FEM）库，由纽约大学库朗数学科学研究所几何计算实验室和加拿大维多利亚大学共同开发。该项目的名称 "PolyFEM" 中的 "Poly" 代表 "Polyvalent"（多功能的），体现了该库的设计理念——提供一个通用、灵活且高效的有限元求解框架。

### 1.2 项目特点

- **多元素支持**：支持多种有限元类型，包括拉格朗日元、样条元、多边形单元等
- **多物理场**：支持标量场问题（如拉普拉斯方程）和张量场问题（如弹性力学）
- **非线性求解**：内置牛顿法等非线性求解器，支持几何和材料非线性
- **接触力学**：集成 IPC（Incremental Potential Contact）工具包，支持接触和摩擦
- **自适应网格**：支持网格重划分和 p-细化
- **时间积分**：支持瞬态问题的多种时间积分方案

### 1.3 开源协议

PolyFEM 采用 MIT 开源协议，允许自由使用、修改和分发。

---

## 2. 理论基础

### 2.1 有限元方法基本原理

有限元方法（Finite Element Method, FEM）是一种数值求解偏微分方程的方法。其核心思想是将连续的求解域离散化为有限个相互连接的子域（单元），在每个单元上用近似函数表示未知场变量。

#### 2.1.1 弱形式推导

以泊松方程为例：

$$-\nabla \cdot (k \nabla u) = f \quad \text{in } \Omega$$

边界条件：
- Dirichlet 边界：$u = g$ on $\Gamma_D$
- Neumann 边界：$k \frac{\partial u}{\partial n} = h$ on $\Gamma_N$

**第一步：乘以测试函数并积分**

将方程两边乘以测试函数 $v$（在 Dirichlet 边界上为零），并在域上积分：

$$\int_\Omega (-\nabla \cdot (k \nabla u)) v \, d\Omega = \int_\Omega f v \, d\Omega$$

**第二步：分部积分（格林公式）**

应用分部积分：

$$\int_\Omega k \nabla u \cdot \nabla v \, d\Omega - \int_{\Gamma_N} h v \, d\Gamma = \int_\Omega f v \, d\Omega$$

这就是泊松方程的弱形式。

#### 2.1.2 有限元离散

将未知函数 $u$ 和测试函数 $v$ 用基函数展开：

$$u_h(x) = \sum_{i=1}^{n} u_i \phi_i(x)$$

$$v_h(x) = \phi_j(x), \quad j = 1, \ldots, n$$

代入弱形式，得到线性方程组：

$$\mathbf{K} \mathbf{u} = \mathbf{f}$$

其中刚度矩阵元素：

$$K_{ij} = \int_\Omega k \nabla \phi_i \cdot \nabla \phi_j \, d\Omega$$

载荷向量元素：

$$f_j = \int_\Omega f \phi_j \, d\Omega + \int_{\Gamma_N} h \phi_j \, d\Gamma$$

### 2.2 单元类型与基函数

#### 2.2.1 拉格朗日单元

拉格朗日单元是最常用的有限元类型，其基函数满足：
- 在对应节点值为 1
- 在其他节点值为 0
- 在单元边界连续

**一维拉格朗日基函数（P1）**：

$$\phi_1(\xi) = \frac{1 - \xi}{2}, \quad \phi_2(\xi) = \frac{1 + \xi}{2}$$

**二维三角形拉格朗日基函数（P1）**：

$$\phi_1(\xi, \eta) = 1 - \xi - \eta, \quad \phi_2(\xi, \eta) = \xi, \quad \phi_3(\xi, \eta) = \eta$$

PolyFEM 支持 1 到 8 阶的拉格朗日单元。

#### 2.2.2 样条单元（IGA）

等几何分析（Isogeometric Analysis, IGA）使用 B 样条或 NURBS 作为基函数：

**一维 B 样条基函数递推公式**：

$$N_{i,0}(\xi) = \begin{cases} 1 & \text{if } \xi_i \leq \xi < \xi_{i+1} \\ 0 & \text{otherwise} \end{cases}$$

$$N_{i,p}(\xi) = \frac{\xi - \xi_i}{\xi_{i+p} - \xi_i} N_{i,p-1}(\xi) + \frac{\xi_{i+p+1} - \xi}{\xi_{i+p+1} - \xi_{i+1}} N_{i+1,p-1}(\xi)$$

PolyFEM 支持 1 到 4 阶的样条单元。

#### 2.2.3 多边形单元

多边形单元用于处理复杂网格，PolyFEM 支持三种多边形基函数：

1. **MFSHarmonic（调和基函数）**：使用基本解方法构造调和基函数
2. **MeanValue（均值坐标）**：基于均值坐标的插值
3. **Wachspress**：Wachspress 坐标

### 2.3 数值积分（求积规则）

有限元方法需要在单元上进行数值积分。PolyFEM 支持多种求积规则：

| 单元类型 | 求积规则 |
|---------|---------|
| 线段 | Gauss-Legendre 求积 |
| 三角形 | Dunavant 求积 |
| 四边形 | 张量积 Gauss 求积 |
| 四面体 | Keaster 求积 |
| 六面体 | 张量积 Gauss 求积 |
| 棱柱 | 组合求积 |
| 金字塔 | 专用求积规则 |
| 多边形 | 三角化后求积 |
| 多面体 | 四面体化后求积 |

求积阶数的选择通常遵循：$n_{quad} \geq 2p - 1$，其中 $p$ 是基函数阶数。

### 2.4 线性弹性力学

#### 2.4.1 应变与应力

**应变张量**：

小变形假设下的应变张量：

$$\boldsymbol{\varepsilon} = \frac{1}{2}(\nabla \mathbf{u} + (\nabla \mathbf{u})^T)$$

**应力张量**：

柯西应力张量 $\boldsymbol{\sigma}$ 通过本构关系与应变关联。

#### 2.4.2 本构模型

**线弹性（胡克定律）**：

$$\boldsymbol{\sigma} = \lambda \text{tr}(\boldsymbol{\varepsilon}) \mathbf{I} + 2\mu \boldsymbol{\varepsilon}$$

其中 $\lambda$ 和 $\mu$ 是 Lamé 常数，与杨氏模量 $E$ 和泊松比 $\nu$ 的关系：

$$\lambda = \frac{E\nu}{(1+\nu)(1-2\nu)}, \quad \mu = \frac{E}{2(1+\nu)}$$

**Neo-Hookean 超弹性**：

应变能密度函数：

$$W = \frac{\mu}{2}(I_1 - 3) - \mu \ln J + \frac{\lambda}{2}(\ln J)^2$$

其中 $I_1 = \text{tr}(\mathbf{C})$，$J = \det(\mathbf{F})$，$\mathbf{F}$ 是变形梯度张量。

**Saint Venant-Kirchhoff**：

应变能密度函数：

$$W = \frac{\lambda}{2}(\text{tr}(\mathbf{E}))^2 + \mu \text{tr}(\mathbf{E}^2)$$

其中 $\mathbf{E} = \frac{1}{2}(\mathbf{F}^T\mathbf{F} - \mathbf{I})$ 是 Green-Lagrange 应变张量。

### 2.5 时间积分

对于瞬态问题，PolyFEM 支持多种隐式时间积分方案：

#### 2.5.1 隐式欧拉法（Backward Euler）

$$\mathbf{M}\frac{\mathbf{u}^{n+1} - \mathbf{u}^n}{\Delta t} + \mathbf{K}\mathbf{u}^{n+1} = \mathbf{f}^{n+1}$$

一阶精度，无条件稳定。

#### 2.5.2 BDF（Backward Differentiation Formula）

$k$ 阶 BDF 公式：

$$\sum_{i=0}^{k} \alpha_i \mathbf{u}^{n+1-i} = \Delta t \mathbf{f}^{n+1}$$

PolyFEM 支持 BDF1 到 BDF6。

#### 2.5.3 隐式 Newmark 方法

$$\mathbf{u}^{n+1} = \mathbf{u}^n + \Delta t \mathbf{v}^n + \frac{\Delta t^2}{2}[(1-2\beta)\mathbf{a}^n + 2\beta\mathbf{a}^{n+1}]$$

$$\mathbf{v}^{n+1} = \mathbf{v}^n + \Delta t[(1-\gamma)\mathbf{a}^n + \gamma\mathbf{a}^{n+1}]$$

当 $\gamma = 0.5, \beta = 0.25$ 时为平均加速度法，二阶精度，无条件稳定。

---

## 3. 系统架构

### 3.1 整体架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                          PolyFEM_bin                            │
│                      (主程序/命令行接口)                          │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                        polyfem::State                           │
│                    (核心状态管理类)                               │
│  ┌──────────────┬──────────────┬──────────────┬──────────────┐ │
│  │    mesh      │    basis     │  assembler   │   solver     │ │
│  │   网格数据    │   基函数     │   组装器     │   求解器     │ │
│  └──────────────┴──────────────┴──────────────┴──────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                                │
        ┌───────────────────────┼───────────────────────┐
        ▼                       ▼                       ▼
┌───────────────┐     ┌───────────────┐     ┌───────────────┐
│  polyfem::    │     │  polyfem::    │     │  polyfem::    │
│    mesh       │     │    basis      │     │  assembler    │
│               │     │               │     │               │
│ - Mesh2D      │     │ - Basis       │     │ - Assembler   │
│ - Mesh3D      │     │ - ElementBases│     │ - LinearAsm   │
│ - CMesh2D     │     │ - LagrangeBasis│    │ - NLAssembler │
│ - NCMesh2D    │     │ - SplineBasis │     │ - Problem     │
│ - CMesh3D     │     │ - PolygonalBasis│   │ - RhsAssembler│
│ - NCMesh3D    │     │               │     │               │
└───────────────┘     └───────────────┘     └───────────────┘
        │                       │                       │
        ▼                       ▼                       ▼
┌───────────────┐     ┌───────────────┐     ┌───────────────┐
│  polyfem::    │     │  polyfem::    │     │  polyfem::    │
│  quadrature   │     │     io        │     │   solver      │
│               │     │               │     │               │
│ - TriQuad     │     │ - MshReader   │     │ - NLProblem   │
│ - TetQuad     │     │ - MshWriter   │     │ - FullNLProb  │
│ - HexQuad     │     │ - OBJReader   │     │ - Forms       │
│ - PrismQuad   │     │ - OBJWriter   │     │ - ALSolver    │
└───────────────┘     └───────────────┘     └───────────────┘
```

### 3.2 核心类说明

#### 3.2.1 State 类

`State` 类是 PolyFEM 的核心，管理整个求解过程的状态：

```cpp
class State {
public:
    // 初始化
    void init(const json &args, const bool strict_validation);
    
    // 主要求解流程
    void load_mesh(...);
    void build_basis();
    void assemble_rhs();
    void assemble_mass_mat();
    void solve_problem(Eigen::MatrixXd &sol, Eigen::MatrixXd &pressure);
    
    // 核心数据成员
    std::unique_ptr<mesh::Mesh> mesh;           // 网格
    std::vector<basis::ElementBases> bases;      // 有限元基
    std::shared_ptr<assembler::Assembler> assembler; // 组装器
    std::shared_ptr<assembler::Problem> problem;     // 问题定义
    StiffnessMatrix mass;                       // 质量矩阵
    Eigen::MatrixXd rhs;                        // 右端项
    json args;                                  // 输入参数
};
```

#### 3.2.2 求解流程

```
1. init()          - 初始化参数
       │
       ▼
2. load_mesh()     - 加载网格
       │
       ▼
3. build_basis()   - 构建基函数
       │
       ▼
4. assemble_rhs()  - 组装右端项
       │
       ▼
5. assemble_mass_mat() - 组装质量矩阵（瞬态问题）
       │
       ▼
6. solve_problem() - 求解问题
       │
       ├── solve_linear()          - 线性问题
       ├── solve_tensor_nonlinear() - 非线性弹性
       ├── solve_navier_stokes()   - Navier-Stokes
       └── solve_transient_*()     - 瞬态问题
```

### 3.3 模块依赖关系

```
                    ┌─────────────┐
                    │   State     │
                    └──────┬──────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
        ▼                  ▼                  ▼
┌───────────────┐  ┌───────────────┐  ┌───────────────┐
│     Mesh      │  │    Basis      │  │   Assembler   │
└───────┬───────┘  └───────┬───────┘  └───────┬───────┘
        │                  │                  │
        │                  │                  │
        ▼                  ▼                  ▼
┌───────────────┐  ┌───────────────┐  ┌───────────────┐
│  Quadrature   │  │  Quadrature   │  │   Quadrature  │
└───────────────┘  └───────────────┘  └───────────────┘
```

---

## 4. 核心模块详解

### 4.1 网格模块（mesh）

#### 4.1.1 网格类型

PolyFEM 支持两种网格类型：

**1. 一致网格（Conforming Mesh, CMesh）**

- 所有单元共享节点
- 无悬挂节点
- 适用于标准有限元

**2. 非一致网格（Non-Conforming Mesh, NCMesh）**

- 支持悬挂节点
- 用于自适应 h-细化
- 仅支持单纯形网格

#### 4.1.2 元素类型

```cpp
enum class ElementType {
    SIMPLEX,                    // 三角形/四面体
    REGULAR_INTERIOR_CUBE,      // 规则内部四边形/六面体
    SIMPLE_SINGULAR_INTERIOR_CUBE,
    MULTI_SINGULAR_INTERIOR_CUBE,
    REGULAR_BOUNDARY_CUBE,
    SIMPLE_SINGULAR_BOUNDARY_CUBE,
    MULTI_SINGULAR_BOUNDARY_CUBE,
    INTERFACE_CUBE,             // 与多边形相邻的单元
    INTERIOR_POLYTOPE,          // 内部多边形
    BOUNDARY_POLYTOPE,          // 边界多边形
    PRISM,                      // 棱柱
    UNDEFINED
};
```

#### 4.1.3 网格接口

```cpp
class Mesh {
public:
    // 基本信息
    virtual bool is_volume() const = 0;
    virtual int n_vertices() const = 0;
    virtual int n_edges() const = 0;
    virtual int n_faces() const = 0;
    virtual int n_cells() const = 0;
    
    // 几何查询
    virtual RowVectorNd point(const int global_index) const = 0;
    virtual RowVectorNd edge_barycenter(const int e) const = 0;
    virtual RowVectorNd face_barycenter(const int f) const = 0;
    virtual RowVectorNd cell_barycenter(const int c) const = 0;
    
    // 边界判断
    virtual bool is_boundary_vertex(const int vertex_global_id) const = 0;
    virtual bool is_boundary_edge(const int edge_global_id) const = 0;
    virtual bool is_boundary_face(const int face_global_id) const = 0;
    
    // 拓扑查询
    virtual int edge_vertex(const int e_id, const int lv_id) const = 0;
    virtual int face_vertex(const int f_id, const int lv_id) const = 0;
    virtual int cell_vertex(const int c_id, const int lv_id) const = 0;
};
```

### 4.2 基函数模块（basis）

#### 4.2.1 Basis 类

```cpp
class Basis {
public:
    // 基函数求值
    void eval_basis(const Eigen::MatrixXd &uv, Eigen::MatrixXd &val) const;
    
    // 基函数梯度求值
    void eval_grad(const Eigen::MatrixXd &uv, Eigen::MatrixXd &val) const;
    
    // 局部到全局映射
    const std::vector<Local2Global> &global() const;
    
private:
    std::vector<Local2Global> global_;  // 局部到全局映射
    Fun basis_;                          // 基函数
    Fun grad_;                           // 梯度函数
};
```

#### 4.2.2 ElementBases 类

```cpp
class ElementBases {
public:
    std::vector<Basis> bases;           // 该单元的所有基函数
    Quadrature quadrature;              // 求积规则
    bool has_parameterization;          // 是否有参数化
    
    // 评估单元上的基函数值
    void evaluate_bases(const Eigen::MatrixXd &uv, 
                        Eigen::MatrixXd &val) const;
    
    // 评估梯度
    void evaluate_grads(const Eigen::MatrixXd &uv, 
                        Eigen::MatrixXd &grad) const;
};
```

#### 4.2.3 支持的基函数类型

| 维度 | 元素类型 | 基函数类型 | 阶数 |
|-----|---------|-----------|------|
| 2D | 三角形 | Lagrange | 1-8 |
| 2D | 四边形 | Lagrange | 1-8 |
| 2D | 四边形 | Spline | 1-4 |
| 2D | 多边形 | MFSHarmonic/MeanValue/Wachspress | - |
| 3D | 四面体 | Lagrange | 1-8 |
| 3D | 六面体 | Lagrange | 1-8 |
| 3D | 六面体 | Spline | 1-4 |
| 3D | 多面体 | MFSHarmonic/MeanValue/Wachspress | - |

### 4.3 组装器模块（assembler）

#### 4.3.1 组装器层次结构

```
                    Assembler (抽象基类)
                         │
         ┌───────────────┼───────────────┐
         │               │               │
    LinearAssembler  NLAssembler  MixedAssembler
         │               │
         │               │
    ┌────┴────┐    ┌────┴────┐
    │         │    │         │
Laplacian  Linear  NeoHookean SaintVenant
           Elasticity   Elasticity
```

#### 4.3.2 线性组装器

```cpp
class LinearAssembler : virtual public Assembler {
public:
    // 组装刚度矩阵
    void assemble(
        const bool is_volume,
        const int n_basis,
        const std::vector<basis::ElementBases> &bases,
        const std::vector<basis::ElementBases> &gbases,
        const AssemblyValsCache &cache,
        const double t,
        StiffnessMatrix &stiffness,
        const bool is_mass = false) const override;
    
    // 局部刚度矩阵计算（子类实现）
    virtual Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 9, 1> 
        assemble(const LinearAssemblerData &data) const = 0;
};
```

**Laplacian 组装器示例**：

局部刚度矩阵元素：

$$K_{ij}^e = \int_{\Omega_e} \nabla \phi_i \cdot \nabla \phi_j \, d\Omega$$

#### 4.3.3 非线性组装器

```cpp
class NLAssembler : virtual public Assembler {
public:
    // 能量计算
    double assemble_energy(
        const bool is_volume,
        const std::vector<basis::ElementBases> &bases,
        const std::vector<basis::ElementBases> &gbases,
        const AssemblyValsCache &cache,
        const double t,
        const double dt,
        const Eigen::MatrixXd &displacement,
        const Eigen::MatrixXd &displacement_prev) const override;
    
    // 梯度（残差）计算
    void assemble_gradient(...) const override;
    
    // Hessian（切线刚度矩阵）计算
    void assemble_hessian(...) const override;
    
    // 子类实现的能量、梯度、Hessian
    virtual double compute_energy(const NonLinearAssemblerData &data) const = 0;
    virtual Eigen::VectorXd assemble_gradient(const NonLinearAssemblerData &data) const = 0;
    virtual Eigen::MatrixXd assemble_hessian(const NonLinearAssemblerData &data) const = 0;
};
```

#### 4.3.4 Problem 类

Problem 类定义了边界条件和右端项：

```cpp
class Problem {
public:
    // 问题类型
    virtual bool is_scalar() const = 0;
    virtual bool is_time_dependent() const { return false; }
    
    // 右端项
    virtual void rhs(const assembler::Assembler &assembler, 
                     const Eigen::MatrixXd &pts, 
                     const double t, 
                     Eigen::MatrixXd &val) const = 0;
    
    // 边界条件
    virtual void dirichlet_bc(const mesh::Mesh &mesh, 
                              const Eigen::MatrixXi &global_ids,
                              const Eigen::MatrixXd &uv, 
                              const Eigen::MatrixXd &pts, 
                              const double t, 
                              Eigen::MatrixXd &val) const = 0;
    
    virtual void neumann_bc(...) const {}
    
    // 精确解（用于验证）
    virtual bool has_exact_sol() const = 0;
    virtual void exact(const Eigen::MatrixXd &pts, 
                       const double t, 
                       Eigen::MatrixXd &val) const {}
    
    // 初始条件
    virtual void initial_solution(...) const {}
    virtual void initial_velocity(...) const {}
};
```

### 4.4 求解器模块（solver）

#### 4.4.1 Form 类

Form 是求解器中能量项的基类：

```cpp
class Form {
public:
    // 能量值
    virtual double value(const Eigen::VectorXd &x) const;
    
    // 一阶导数（梯度）
    virtual void first_derivative(const Eigen::VectorXd &x, 
                                   Eigen::VectorXd &gradv) const;
    
    // 二阶导数（Hessian）
    virtual void second_derivative(const Eigen::VectorXd &x, 
                                    StiffnessMatrix &hessian) const;
    
    // 约束检查
    virtual bool is_step_valid(const Eigen::VectorXd &x0, 
                               const Eigen::VectorXd &x1) const;
    virtual double max_step_size(const Eigen::VectorXd &x0, 
                                 const Eigen::VectorXd &x1) const;
};
```

#### 4.4.2 具体的 Form 类型

| Form 类型 | 功能 | 适用问题 |
|----------|------|---------|
| ElasticForm | 弹性能量 | 弹性力学 |
| BodyForm | 体力/面力 | 所有问题 |
| InertiaForm | 惯性项 | 瞬态问题 |
| ContactForm | 接触势能 | 接触问题 |
| FrictionForm | 摩擦势能 | 摩擦接触 |
| PressureForm | 压力边界 | 流固耦合 |
| BarrierContactForm | 障碍接触 | IPC 接触 |

#### 4.4.3 NLProblem 类

非线性问题类：

```cpp
class NLProblem : public FullNLProblem {
public:
    // 能量函数
    double value(const TVector &x) override;
    
    // 梯度
    void gradient(const TVector &x, TVector &gradv) override;
    
    // Hessian
    void hessian(const TVector &x, THessian &hessian) override;
    
    // 约束处理
    bool is_step_valid(const TVector &x0, const TVector &x1) override;
    double max_step_size(const TVector &x0, const TVector &x1) override;
    
    // 周期边界处理
    TVector full_to_reduced(const TVector &full) const;
    TVector reduced_to_full(const TVector &reduced) const;
};
```

### 4.5 求积模块（quadrature）

#### 4.5.1 求积规则接口

```cpp
class Quadrature {
public:
    Eigen::MatrixXd points;   // 求积点（参考坐标）
    Eigen::VectorXd weights;  // 求积权重
    
    // 获取求积规则
    static void get_quadrature(int order, Quadrature &quad);
};
```

#### 4.5.2 各单元求积规则

**三角形求积（TriQuadrature）**：

使用 Dunavant 求积规则，精度可达 20 阶。

**四边形求积（QuadQuadrature）**：

使用张量积 Gauss 求积。

**四面体求积（TetQuadrature）**：

使用 Keaster 求积规则。

**六面体求积（HexQuadrature）**：

使用张量积 Gauss 求积。

---

## 5. 支持的物理问题

### 5.1 标量问题

#### 5.1.1 拉普拉斯方程

$$-\Delta u = f$$

弱形式：

$$\int_\Omega \nabla u \cdot \nabla v \, d\Omega = \int_\Omega f v \, d\Omega$$

#### 5.1.2 Helmholtz 方程

$$-\Delta u + k^2 u = f$$

弱形式：

$$\int_\Omega (\nabla u \cdot \nabla v + k^2 u v) \, d\Omega = \int_\Omega f v \, d\Omega$$

#### 5.1.3 双调和方程

$$\Delta^2 u = f$$

弱形式：

$$\int_\Omega \Delta u \Delta v \, d\Omega = \int_\Omega f v \, d\Omega$$

需要 $C^1$ 连续的基函数或混合方法。

### 5.2 弹性力学问题

#### 5.2.1 线弹性

平衡方程：

$$\nabla \cdot \boldsymbol{\sigma} + \mathbf{f} = \mathbf{0}$$

弱形式：

$$\int_\Omega \boldsymbol{\sigma} : \boldsymbol{\varepsilon}(\mathbf{v}) \, d\Omega = \int_\Omega \mathbf{f} \cdot \mathbf{v} \, d\Omega + \int_{\Gamma_N} \mathbf{t} \cdot \mathbf{v} \, d\Gamma$$

#### 5.2.2 Saint Venant-Kirchhoff 弹性

适用于大变形、小应变情况。

应变能：

$$W = \frac{\lambda}{2}(\text{tr}(\mathbf{E}))^2 + \mu \text{tr}(\mathbf{E}^2)$$

#### 5.2.3 Neo-Hookean 弹性

适用于大变形、大应变情况。

应变能：

$$W = \frac{\mu}{2}(I_1 - 3) - \mu \ln J + \frac{\lambda}{2}(\ln J)^2$$

#### 5.2.4 Mooney-Rivlin 弹性

适用于橡胶类材料：

$$W = C_1(I_1 - 3) + C_2(I_2 - 3) + \frac{\lambda}{2}(J - 1)^2$$

#### 5.2.5 Ogden 弹性

$$W = \sum_{p=1}^{N} \frac{\mu_p}{\alpha_p}(\lambda_1^{\alpha_p} + \lambda_2^{\alpha_p} + \lambda_3^{\alpha_p} - 3) + \frac{\kappa}{2}(J - 1)^2$$

### 5.3 流体力学问题

#### 5.3.1 Stokes 方程

$$-\mu \Delta \mathbf{u} + \nabla p = \mathbf{f}$$

$$\nabla \cdot \mathbf{u} = 0$$

使用混合有限元方法，速度用高阶元，压力用低阶元。

#### 5.3.2 Navier-Stokes 方程

$$\rho \frac{\partial \mathbf{u}}{\partial t} + \rho (\mathbf{u} \cdot \nabla) \mathbf{u} = -\nabla p + \mu \Delta \mathbf{u} + \mathbf{f}$$

$$\nabla \cdot \mathbf{u} = 0$$

### 5.4 接触问题

#### 5.4.1 IPC（Incremental Potential Contact）

PolyFEM 集成了 IPC 工具包，实现了无穿透接触：

**障碍函数**：

$$b(d) = -\kappa (d - \hat{d})^2 \ln(d)$$

其中 $d$ 是距离，$\hat{d}$ 是激活距离，$\kappa$ 是刚度。

**摩擦模型**：

使用增广拉格朗日方法处理摩擦约束。

---

## 6. 数值求解器

### 6.1 线性求解器

PolyFEM 使用 PolySolve 库提供多种线性求解器：

#### 6.1.1 直接求解器

| 求解器 | 说明 |
|-------|------|
| Eigen::SparseLU | Eigen 库的稀疏 LU 分解 |
| Eigen::CholmodSupernodalLLT | Cholesky 分解（需要 SuiteSparse） |
| Pardiso | Intel MKL Pardiso |
| UMFPack | SuiteSparse UMFPack |
| CHOLMOD | SuiteSparse CHOLMOD |

#### 6.1.2 迭代求解器

| 求解器 | 说明 |
|-------|------|
| CG | 共轭梯度法 |
| MINRES | 最小残差法 |
| GMRES | 广义最小残差法 |
| BiCGSTAB | 双共轭梯度稳定法 |
| AMGCL | 代数多重网格（可选） |

### 6.2 非线性求解器

#### 6.2.1 牛顿法

```
给定初始值 x_0
for k = 0, 1, 2, ...
    计算残差 r(x_k) = ∇E(x_k)
    计算切线刚度矩阵 K(x_k) = ∇²E(x_k)
    求解 K(x_k) Δx = -r(x_k)
    线搜索：找到 α 使得 E(x_k + α Δx) < E(x_k)
    更新：x_{k+1} = x_k + α Δx
    检验收敛
end
```

#### 6.2.2 增广拉格朗日方法

用于处理约束（如 Dirichlet 边界条件）：

$$\min_{\mathbf{x}} E(\mathbf{x}) + \frac{\alpha}{2} \|C(\mathbf{x})\|^2 - \boldsymbol{\lambda}^T C(\mathbf{x})$$

迭代更新：
1. 固定 $\boldsymbol{\lambda}$，求解增广问题
2. 更新 $\boldsymbol{\lambda} \leftarrow \boldsymbol{\lambda} - \alpha C(\mathbf{x})$
3. 如需要，增加 $\alpha$

### 6.3 接触求解

#### 6.3.1 连续碰撞检测（CCD）

检测两个时间步之间是否发生碰撞：

- 宽阶段：使用空间哈希或 BVH 快速排除
- 窄阶段：精确计算碰撞时间

#### 6.3.2 摩擦迭代

摩擦力使用滞后（lagging）方法处理：

```
for iter = 1 to max_friction_iterations
    计算当前摩擦力
    更新摩擦方向
    求解非线性问题
    检查收敛
end
```

---

## 7. 输入输出系统

### 7.1 输入格式

#### 7.1.1 JSON 配置文件

PolyFEM 使用 JSON 格式的配置文件：

```json
{
    "geometry": {
        "mesh": "model.obj",
        "surface_selection": {
            "threshold": 1e-8
        }
    },
    "materials": {
        "type": "LinearElasticity",
        "E": 1e6,
        "nu": 0.3
    },
    "boundary_conditions": {
        "dirichlet_boundary": [
            {"id": 1, "value": [0, 0, 0]},
            {"id": 2, "value": [0, 0, 1]}
        ],
        "rhs": [0, -9.8, 0]
    },
    "space": {
        "discr_order": 2
    },
    "solver": {
        "linear": {
            "solver": "Eigen::SparseLU"
        }
    },
    "output": {
        "paraview": {
            "file_name": "result.vtu"
        }
    }
}
```

#### 7.1.2 支持的网格格式

| 格式 | 扩展名 | 说明 |
|-----|-------|------|
| OBJ | .obj | Wavefront OBJ 格式 |
| MSH | .msh | GMSH 格式（2.2 版本） |
| STL | .stl | 立体光刻格式 |
| PLY | .ply | Polygon 文件格式 |
| MESH | .mesh | Medit 格式 |

### 7.2 输出格式

#### 7.2.1 ParaView 输出

输出 VTU/VTK 格式，可在 ParaView 中可视化：

- 节点解
- 单元解
- 应力/应变
- Von Mises 应力
- 接触力

#### 7.2.2 JSON 统计输出

包含求解统计信息：

```json
{
    "time_steps": 10,
    "solving_time": 12.5,
    "num_dofs": 10000,
    "num_nonzeros": 500000,
    "errors": {
        "l2": 1e-6,
        "h1": 1e-4
    }
}
```

---

## 8. 高级功能

### 8.1 自适应网格重划分

PolyFEM 支持基于物理的自适应网格重划分：

#### 8.1.1 操作类型

| 操作 | 说明 |
|-----|------|
| Split | 边分裂，增加网格密度 |
| Collapse | 边坍缩，减少网格密度 |
| Swap | 边/面交换，改善网格质量 |
| Smooth | 节点平滑，优化网格形状 |

#### 8.1.2 能量准则

操作接受准则：

$$\Delta E < \epsilon_{accept}$$

操作剔除准则：

$$\frac{E_{local}}{E_{max}} < \epsilon_{cull}$$

### 8.2 周期边界条件

支持周期边界条件，用于均质化分析：

```json
{
    "boundary_conditions": {
        "periodic_boundary": {
            "enabled": true,
            "linear_displacement_offset": [[1, 0], [0, 1]]
        }
    }
}
```

### 8.3 自动微分

PolyFEM 使用自动微分计算导数：

- 能量对位移的一阶导数（梯度/残差）
- 能量对位移的二阶导数（Hessian/刚度矩阵）
- 应力对材料参数的导数（用于优化）

### 8.4 伴随灵敏度分析

支持目标函数对设计变量的灵敏度分析：

$$\frac{dJ}{d\mathbf{p}} = \frac{\partial J}{\partial \mathbf{p}} - \boldsymbol{\lambda}^T \frac{\partial \mathbf{R}}{\partial \mathbf{p}}$$

其中 $\boldsymbol{\lambda}$ 是伴随变量，满足：

$$\mathbf{K}^T \boldsymbol{\lambda} = \frac{\partial J}{\partial \mathbf{u}}$$

---

## 9. 使用示例

### 9.1 简单拉普拉斯问题

```json
{
    "geometry": {
        "mesh": "plate_hole.obj",
        "advanced": {
            "normalize_mesh": true
        }
    },
    "materials": {
        "type": "Laplacian"
    },
    "boundary_conditions": {
        "dirichlet_boundary": [
            {"id": 1, "value": 0},
            {"id": 4, "value": 1}
        ],
        "rhs": 10
    },
    "output": {
        "paraview": {
            "file_name": "result.vtu"
        }
    }
}
```

### 9.2 线弹性静力学

```json
{
    "geometry": {
        "mesh": "beam.msh"
    },
    "materials": {
        "type": "LinearElasticity",
        "E": 2e11,
        "nu": 0.3
    },
    "boundary_conditions": {
        "dirichlet_boundary": [
            {"id": 1, "value": [0, 0, 0]}
        ],
        "neumann_boundary": [
            {"id": 2, "value": [0, -1000, 0]}
        ]
    },
    "space": {
        "discr_order": 2
    }
}
```

### 9.3 瞬态弹性动力学

```json
{
    "geometry": {
        "mesh": "impact.msh"
    },
    "materials": {
        "type": "LinearElasticity",
        "E": 1e7,
        "nu": 0.35,
        "rho": 1000
    },
    "time": {
        "tend": 1.0,
        "dt": 0.001,
        "integrator": "ImplicitNewmark"
    },
    "boundary_conditions": {
        "dirichlet_boundary": [
            {"id": 1, "value": [0, 0, 0]}
        ]
    },
    "initial_conditions": {
        "velocity": [
            {"id": 2, "value": [0, -10, 0]}
        ]
    }
}
```

### 9.4 接触问题

```json
{
    "geometry": {
        "mesh": "two_blocks.msh"
    },
    "materials": {
        "type": "NeoHookean",
        "E": 1e6,
        "nu": 0.45
    },
    "contact": {
        "enabled": true,
        "dhat": 0.01,
        "friction_coefficient": 0.5
    },
    "boundary_conditions": {
        "dirichlet_boundary": [
            {"id": 1, "value": [0, 0, 0]},
            {"id": 2, "value": [0, -0.1, 0]}
        ]
    }
}
```

### 9.5 C++ API 使用

```cpp
#include <polyfem/State.hpp>

int main() {
    polyfem::State state;
    
    // 加载配置
    json args;
    args["geometry"]["mesh"] = "model.obj";
    args["materials"]["type"] = "Laplacian";
    // ... 更多配置
    
    // 初始化
    state.init(args, true);
    
    // 加载网格
    state.load_mesh();
    
    // 构建基函数
    state.build_basis();
    
    // 组装右端项
    state.assemble_rhs();
    
    // 组装质量矩阵（瞬态问题）
    state.assemble_mass_mat();
    
    // 求解
    Eigen::MatrixXd sol, pressure;
    state.solve_problem(sol, pressure);
    
    // 导出结果
    state.export_data(sol, pressure);
    
    return 0;
}
```

---

## 10. 依赖库说明

### 10.1 核心依赖

| 库名 | 用途 | 许可证 |
|-----|------|-------|
| Eigen | 线性代数 | MPL-2.0 |
| libigl | 几何处理 | MPL-2.0 |
| PolySolve | 线性求解器 | MIT |
| IPC Toolkit | 接触处理 | MIT |
| spdlog | 日志 | MIT |
| CLI11 | 命令行解析 | BSD-3 |

### 10.2 可选依赖

| 库名 | 用途 | 许可证 |
|-----|------|-------|
| TBB | 并行化 | Apache-2.0 |
| Geogram | 几何处理 | BSD-3 |
| MMG | 网格重划分 | LGPL-3 |
| ParaViewo | ParaView 输出 | BSD-3 |
| yaml-cpp | YAML 解析 | MIT |
| nanospline | B 样条 | MIT |

### 10.3 构建系统

PolyFEM 使用 CMake 构建：

```bash
mkdir build
cd build
cmake ..
make -j4
```

主要 CMake 选项：

| 选项 | 默认值 | 说明 |
|-----|-------|------|
| POLYFEM_WITH_TESTS | ON | 构建测试 |
| POLYFEM_WITH_APP | OFF | 构建 GUI 应用 |
| POLYFEM_WITH_MMG | OFF | 启用 MMG 重划分 |
| POLYFEM_THREADING | TBB | 线程库选择 |

---

## 附录

### A. 常见问题解答

**Q1: 如何选择合适的单元类型？**

A: 对于简单几何，使用拉格朗日单元；对于复杂几何或多边形网格，使用多边形单元；对于高精度需求，使用高阶单元或样条单元。

**Q2: 如何处理网格质量问题？**

A: PolyFEM 支持自适应网格重划分和 p-细化。对于质量很差的网格，建议先使用外部工具进行网格优化。

**Q3: 接触问题的 dhat 如何选择？**

A: dhat 通常取为最小边长的一定比例（如 1%）。太小会导致数值问题，太大会影响精度。

### B. 参考文献

1. Schneider, T., et al. "Poly-Spline Finite-Element Method." ACM TOG 38.3 (2019).
2. Schneider, T., et al. "Decoupling Simulation Accuracy from Mesh Quality." ACM TOG 37.6 (2018).
3. Li, M., et al. "Incremental Potential Contact." ACM TOG 39.4 (2020).

### C. 术语表

| 术语 | 英文 | 解释 |
|-----|------|------|
| 有限元方法 | Finite Element Method (FEM) | 一种数值求解偏微分方程的方法 |
| 基函数 | Basis Function | 用于表示解的插值函数 |
| 弱形式 | Weak Form | 偏微分方程的积分形式 |
| 求积规则 | Quadrature Rule | 数值积分方法 |
| 刚度矩阵 | Stiffness Matrix | 离散化后的系统矩阵 |
| 质量矩阵 | Mass Matrix | 惯性项对应的矩阵 |
| 牛顿法 | Newton's Method | 一种非线性求解方法 |
| IPC | Incremental Potential Contact | 一种接触处理方法 |

---

*文档版本：1.0*
*生成日期：2026-02-12*
*基于 PolyFEM main 分支*
