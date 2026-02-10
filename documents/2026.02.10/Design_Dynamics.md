# Dynamics 模块设计文档

**职责**: 核心仿真逻辑层。管理刚体状态、组装系统能量（惯性、接触、约束），并执行时间积分。

## 1. 目录结构

遵循头文件与实现分离原则：

```
NexDynIPC/
├── include/
│   └── NexDynIPC/
│       └── Dynamics/
│           ├── RigidBody.hpp       // 刚体定义
│           ├── World.hpp           // 物理世界容器
│           ├── TimeIntegrator.hpp  // 时间积分器接口
│           ├── ImplicitEuler.hpp   // 隐式欧拉积分器
│           └── Forms/              // 能量形式 (Form-based Architecture)
│               ├── Form.hpp
│               ├── InertiaForm.hpp
│               ├── BarrierForm.hpp
│               └── FrictionForm.hpp
└── src/
    └── Dynamics/
        ├── RigidBody.cpp
        ├── World.cpp
        ├── ImplicitEuler.cpp
        └── Forms/
            ├── InertiaForm.cpp
            └── BarrierForm.cpp
```

## 2. 接口设计

### 2.1 刚体 (`RigidBody.hpp`)

存储状态量 $(x, q, v, \omega)$ 和物理属性 $(m, I)$。

```cpp
namespace NexDynIPC::Dynamics {

class RigidBody {
public:
    int id;
    
    // 状态量 (State)
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d velocity;
    Eigen::Vector3d angular_velocity;

    // 物理属性 (Properties)
    double mass;
    Eigen::Matrix3d inertia_body; // 局部坐标系下的惯性张量
    bool is_static = false;

    // 关联的几何形状 (来自 Physics 模块)
    std::shared_ptr<Physics::Shape> shape;

    // 获取当前世界坐标系下的惯性张量
    Eigen::Matrix3d getInertiaWorld() const;

    // 将局部点转换到世界坐标系
    Eigen::Vector3d toWorld(const Eigen::Vector3d& p_local) const;
};

} // namespace
```

### 2.2 能量形式 (`Forms/Form.hpp`)

采用 "Form" 架构，统一计算能量值、梯度和 Hessian。这是构建非线性优化问题的基础。

```cpp
namespace NexDynIPC::Dynamics {

class Form {
public:
    virtual ~Form() = default;

    // 计算当前状态下的能量值 Function Value
    // x: 全局状态向量 [x1, q1, ..., xn, qn]
    virtual double value(const Eigen::VectorXd& x) const = 0;

    // 计算梯度 Gradient (力)
    virtual void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const = 0;

    // 计算 Hessian (刚度矩阵)
    // 写入稀疏矩阵的三元组列表 (Triplet List) 以便高效组装
    virtual void hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) const = 0;
    
    // 计算最大步长 (用于 CCD 或 Line Search 限定)
    virtual double maxStep(const Eigen::VectorXd& x, const Eigen::VectorXd& dx) const { return 1.0; }
};

} // namespace
```

### 2.3 物理世界 (`World.hpp`)

容器类，持有所有刚体和能量形式。

```cpp
namespace NexDynIPC::Dynamics {

class World {
public:
    std::vector<std::shared_ptr<RigidBody>> bodies;
    std::vector<std::shared_ptr<Form>> forms;
    
    // 添加刚体
    void addBody(std::shared_ptr<RigidBody> body);
    
    // 初始化系统状态向量
    Eigen::VectorXd getState() const;
    void setState(const Eigen::VectorXd& x);
};

} // namespace
```

### 2.4 时间积分器 (`TimeIntegrator.hpp`)

```cpp
namespace NexDynIPC::Dynamics {

class TimeIntegrator {
public:
    virtual ~TimeIntegrator() = default;

    // 执行单步仿真
    virtual void step(World& world, double dt) = 0;
};

// 隐式欧拉实现
class ImplicitEuler : public TimeIntegrator {
public:
    void step(World& world, double dt) override;
    
private:
    std::unique_ptr<Math::NewtonSolver> solver_; // 依赖 Math 模块
};

} // namespace
```

## 3. 依赖关系

- 依赖 `Math` 模块进行非线性求解。
- 依赖 `Physics` 模块获取几何信息和接触势能计算逻辑。
