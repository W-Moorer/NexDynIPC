# Physics 模块设计文档

**职责**: 负责几何体的表示、符号距离场 (SDF) 查询、宽相碰撞检测，以及 IPC (Incremental Potential Contact) 接触势能的构建。该层即“无状态物理”，只关心瞬时的几何关系和能量计算。

## 1. 目录结构

遵循头文件与实现分离原则：

```
NexDynIPC/
├── include/
│   └── NexDynIPC/
│       └── Physics/
│           ├── Geometry/
│           │   ├── Shape.hpp       // 几何形状基类
│           │   ├── Mesh.hpp        // 三角网格
│           │   └── SDF.hpp         // SDF 接口
│           ├── Contact/
│           │   ├── BroadPhase.hpp  // 宽相检测接口
│           │   ├── Barrier.hpp     // IPC Log-Barrier 势能函数
│           │   └── Friction.hpp    // IPC 摩擦势能函数
│           └── Utils/
│               └── CCD.hpp         // 连续碰撞检测 (Continuous Collision Detection)
└── src/
    └── Physics/
        ├── Geometry/
        │   ├── Mesh.cpp
        │   └── SDF.cpp
        └── Contact/
            ├── BroadPhase.cpp
            └── Barrier.cpp
```

## 2. 接口设计

### 2.1 几何与 SDF (`Geometry/Shape.hpp`)

```cpp
namespace NexDynIPC::Physics {

class Shape {
public:
    virtual ~Shape() = default;

    // 计算点 p 处的符号距离 (SDF Value)
    virtual double sdf(const Eigen::Vector3d& p) const = 0;

    // 计算点 p 处的 SDF 梯度 (法向量)
    virtual Eigen::Vector3d gradient(const Eigen::Vector3d& p) const = 0;

    // 计算轴对齐包围盒 (AABB)
    virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> computeAABB() const = 0;
};

// 三角网格实现
class MeshShape : public Shape {
    // ... 需要加载 .obj 或 .nsm (Nagata Simulation Mesh)
};

} // namespace
```

### 2.2 接触势能 (`Contact/Barrier.hpp`)

IPC 方法的核心在于将无穿透约束转化为光滑的 Log-Barrier 势能。

```cpp
namespace NexDynIPC::Physics {

class BarrierPotential {
public:
    // 计算单个点-面或边-边对的势能值
    // d: 当前距离
    // d_hat: 激活阈值
    // kappa: 刚度系数
    static double value(double d, double d_hat, double kappa);

    // 计算势能对距离的一阶导数 (力的大小)
    static double gradient(double d, double d_hat, double kappa);

    // 计算势能对距离的二阶导数 (Hessian 的一部分)
    static double hessian(double d, double d_hat, double kappa);
};

} // namespace
```

### 2.3 宽相检测 (`Contact/BroadPhase.hpp`)

用于快速剔除不可能相交的物体对，输出潜在的碰撞对。

```cpp
namespace NexDynIPC::Physics {

struct AABB {
    Eigen::Vector3d min, max;
    int id; // 关联的 Body ID 或 Primitive ID
};

class BroadPhase {
public:
    virtual ~BroadPhase() = default;

    // 更新包围盒并进行检测
    // 输入: 所有物体的 AABB
    // 输出: 潜在相交的 ID 对列表
    virtual std::vector<std::pair<int, int>> detect(const std::vector<AABB>& aabbs) = 0;
};

} // namespace
```

## 3. 依赖关系

- 依赖于 Eigen 库进行向量计算。
- 依赖于 `Math` 模块（如果在几何计算中需要用到求解器）。
