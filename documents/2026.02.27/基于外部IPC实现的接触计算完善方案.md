# 基于外部IPC实现的接触计算完善方案分析

**分析日期**: 2026-02-27
**项目**: NexDynIPC
**参考项目**: ipc-toolkit, libuipc, rigid-ipc, stark

---

## 一、四个参考项目的核心架构对比

### 1.1 ipc-toolkit（官方IPC核心库）

ipc-toolkit 是 SIGGRAPH IPC 论文的官方实现，提供了最完整的IPC算法框架。

**核心模块架构**：

| 模块 | 位置 | 功能描述 |
|------|------|----------|
| **CollisionMesh** | `src/ipc/collision_mesh.hpp` | 碰撞网格封装，管理顶点、边、面，支持full mesh到collision mesh的映射 |
| **Candidates** | `src/ipc/candidates/` | 碰撞候选对管理，包含VV/VE/VF/EE/FF五种基元类型 |
| **BroadPhase** | `src/ipc/broad_phase/` | 宽相位碰撞检测，支持BVH、HashGrid、SweepAndPrune等 |
| **CCD** | `src/ipc/ccd/` | 连续碰撞检测，支持Nonlinear CCD、Tight Inclusion等 |
| **Distance** | `src/ipc/distance/` | 精确距离计算，实现所有基元对的距离查询 |
| **Barrier** | `src/ipc/barrier/` | 障碍函数实现，包含Log、Cubic等多种形式 |
| **Potentials** | `src/ipc/potentials/` | 势能封装，Barrier、Friction、Normal、Tangential |
| **Collisions** | `src/ipc/collisions/` | 碰撞响应计算 |

**关键代码流程**：
```
CollisionMesh.build() 
    → Candidates.build() [BroadPhase + 窄相筛选]
        → compute_collision_free_stepsize() [CCD]
            → BarrierPotential / FrictionPotential
                → gradient() / hessian() 参与优化求解
```

---

### 1.2 libuipc（高性能GPU实现）

libuipc 是一个高性能的IPC库，支持CPU和GPU后端。

**核心模块架构**：

| 模块 | 位置 | 功能描述 |
|------|------|----------|
| **BVH** | `src/geometry/bvh/` | 层次包围盒，支持CPU和GPU实现 |
| **Distance** | `src/geometry/distance.cpp` | 距离计算，包含CUDA实现 |
| **CollisionDetection** | `src/backends/cuda/collision_detection/` | CUDA碰撞检测 |
| **ContactSystem** | `src/backends/cuda/contact_system/` | 接触系统，含IPC变体 |

**特点**：
- 完全模块化的后端设计
- 支持CUDA加速
- 丰富的simplex类型支持（顶点、边、面）

---

### 1.3 rigid-ipc（刚体专用）

rigid-ipc 是针对刚体动力学的IPC实现。

**核心模块架构**：

| 模块 | 位置 | 功能描述 |
|------|------|----------|
| **RigidBody** | `src/physics/rigid_body.hpp` | 刚体定义 |
| **DistanceBarrierConstraint** | `src/opt/distance_barrier_constraint.hpp` | 距离屏障约束 |
| **IPCSolver** | `src/solvers/ipc_solver.cpp` | IPC求解器 |
| **CCD** | `src/ccd/` | 刚体专用CCD |

**特点**：
- 刚体位姿使用Quaternion表示
- 专门的刚体距离计算
- 与变分不等式求解器集成

---

### 1.4 stark（高校实验室实现）

stark 是一个整合了IPC的物理仿真框架，使用SymX进行自动微分。

**核心模块架构**：

| 模块 | 位置 | 功能描述 |
|------|------|----------|
| **EnergyFrictionalContact** | `src/models/interactions/` | 接触能量计算（核心） |
| **TriangleMeshCollisionDetection** | `extern/TriangleMeshCollisionDetection/` | 三角网格碰撞检测 |
| **IPCBarrierType** | 枚举 | Log / Cubic 屏障类型 |
| **IPCFrictionType** | 枚举 | C0 / C1 摩擦类型 |

**特点**：
- 使用SymX进行自动微分
- 能量形式的接触建模
- 支持刚体-刚体、刚体-可变形体、静态物体接触

---

## 二、完整接触计算流程对比

### 2.1 各项目接触计算流程

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           ipc-toolkit 完整流程                                │
├─────────────────────────────────────────────────────────────────────────────┤
│  1. 构建CollisionMesh                                                        │
│     └── vertices, edges, faces → 初始化adjacencies                         │
│                                                                             │
│  2. 宽相位检测 (BroadPhase)                                                 │
│     └── AABB重叠 → 候选对列表                                                │
│                                                                             │
│  3. 构建Candidates                                                          │
│     └── 对每个候选对: VV/VE/VF/EE/FF 基元类型判断                           │
│                                                                             │
│  4. 窄相距离计算 (Distance)                                                  │
│     └── 对每个candidate: 精确距离d                                            │
│                                                                             │
│  5. CCD检测 (可选)                                                          │
│     └── compute_collision_free_stepsize() → 时间步裁剪                      │
│                                                                             │
│  6. 势能计算                                                                │
│     ├── BarrierPotential: d < dhat ? b(d) : 0                              │
│     └── FrictionPotential: D(u, fn, mu)                                    │
│                                                                             │
│  7. 梯度/Hessian计算                                                        │
│     └── potential.gradient(d) / potential.hessian(d)                        │
│                                                                             │
│  8. 集成到优化求解器                                                        │
│     └── Newton迭代: minimize E(x) + κ*B(x)                                  │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│                              stark 流程                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│  1. 初始化Mesh                                                              │
│     └── points, triangles/edges → local_vertices                            │
│                                                                             │
│  2. 碰撞检测 (TriangleMeshCollisionDetection)                              │
│     ├── IntersectionDetection: 交叉检测                                     │
│     └── ProximityDetection: 接近检测                                        │
│                                                                             │
│  3. SymX能量注册                                                            │
│     ├── _energies_contact_*() → 接触能量                                    │
│     ├── _energies_friction_*() → 摩擦能量                                  │
│     └── barrier_potential(d, dhat, k)                                       │
│                                                                             │
│  4. 自动微分 (SymX)                                                         │
│     └── 自动计算 gradient + hessian                                         │
│                                                                             │
│  5. 优化求解                                                               │
│     └── 能量最小化                                                          │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 三、NexDynIPC当前差距与完善方案

### 3.1 当前NexDynIPC的问题

根据代码分析，当前NexDynIPC的接触计算存在以下问题：

| 问题 | 当前实现 | 影响 |
|------|----------|------|
| **网格距离计算** | 简化质心距离近似 | 无法获得精确接触点 |
| **Candidates构建** | 仅基于AABB筛选 | 缺少VV/VE/VF/EE细粒度 |
| **Barrier集成** | 未集成到求解器 | 接触排斥力未参与迭代 |
| **CCD** | 旋转积分不完整 | 高速运动可能漏检 |
| **摩擦耦合** | 法向力近似估计 | 摩擦响应不一致 |

---

### 3.2 完善方案：模块化实现路线

以下是基于外部项目分析，为NexDynIPC设计的完整接触计算实现方案。

#### Phase 1: 核心几何模块（必须实现）

**目标**: 实现Mesh级别的精确碰撞检测

##### 1.1 CollisionMesh / RigidBodyCollisionMesh

**参考**: ipc-toolkit `CollisionMesh` + stark `EnergyFrictionalContact`

```cpp
// 新增文件: include/NexDynIPC/Physics/Geometry/RigidBodyCollisionMesh.h

namespace NexDynIPC::Physics {

class RigidBodyCollisionMesh {
public:
    // 构建碰撞网格
    void build(const RigidBody& body);
    
    // 获取变换后的顶点位置
    Eigen::MatrixXd getVertices(const Eigen::VectorXd& positions) const;
    
    // 获取变换后的边顶点
    std::array<Eigen::Vector3d, 2> getEdgeVertices(size_t edge_idx, 
                                                     const Eigen::VectorXd& positions) const;
    
    // 获取变换后的面顶点
    std::array<Eigen::Vector3d, 3> getFaceVertices(size_t face_idx,
                                                    const Eigen::VectorXd& positions) const;
    
    // 基础属性
    const Eigen::MatrixXd& restPositions() const { return rest_positions_; }
    const Eigen::MatrixXi& edges() const { return edges_; }
    const Eigen::MatrixXi& faces() const { return faces_; }
    
    // 邻接关系
    void initAdjacencies();
    
private:
    Eigen::MatrixXd rest_positions_;  // 静止位置
    Eigen::MatrixXi edges_;            // 边索引
    Eigen::MatrixXi faces_;            // 面索引
    
    // 邻接关系
    std::vector<std::vector<size_t>> vertex_to_edges_;
    std::vector<std::vector<size_t>> vertex_to_faces_;
    std::vector<std::vector<size_t>> edge_to_faces_;
};

}
```

**工作量**: 1周

##### 1.2 距离计算模块

**参考**: ipc-toolkit `src/ipc/distance/` + libuipc `src/geometry/distance.cpp`

```cpp
// 新增文件: include/NexDynIPC/Physics/Contact/Distance.h

namespace NexDynIPC::Physics::Contact {

// 基元类型
enum class PrimitiveType { VERTEX, EDGE, FACE };

// 距离计算结果
struct DistanceResult {
    double d;                    // 距离
    Eigen::Vector3d p;           // 接触点A
    Eigen::Vector3d q;           // 接触点B
    Eigen::Vector3d n;           // 接触法向 (A→B)
    size_t type;                 // 接触类型
    
    // 雅可比信息用于梯度计算
    std::array<int, 4> indices;  // 涉及顶点的全局索引
    std::array<double, 4> weights; // 重心坐标权重
};

// 点-点距离
double pointPointDistance(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);

// 点-边距离
double pointEdgeDistance(const Eigen::Vector3d& p, 
                         const Eigen::Vector3d& e0, 
                         const Eigen::Vector3d& e1,
                         Eigen::Vector3d& closest_p,
                         Eigen::Vector3d& closest_e);

// 点-面距离
double pointFaceDistance(const Eigen::Vector3d& p,
                         const Eigen::Vector3d& f0,
                         const Eigen::Vector3d& f1,
                         const Eigen::Vector3d& f2,
                         Eigen::Vector3d& closest_p,
                         Eigen::Vector3d& closest_f);

// 边-边距离
double edgeEdgeDistance(const Eigen::Vector3d& ea0, const Eigen::Vector3d& ea1,
                        const Eigen::Vector3d& eb0, const Eigen::Vector3d& eb1,
                        Eigen::Vector3d& closest_ea,
                        Eigen::Vector3d& closest_eb);

// 距离 + 梯度 + Hessian
struct DistanceDerivatives {
    double distance;
    Eigen::VectorXd gradient;    // n * 6 (2 bodies * 3 dof)
    Eigen::SparseMatrix<double> hessian;
};

DistanceDerivatives computeDistanceWithDerivatives(
    const RigidBodyCollisionMesh& meshA,
    const RigidBodyCollisionMesh& meshB,
    const Eigen::VectorXd& positionsA,
    const Eigen::VectorXd& positionsB,
    size_t primitiveA, size_t primitiveB);

}
```

**工作量**: 2-3周

---

#### Phase 2: 碰撞检测管线

##### 2.1 BroadPhase 增强

**参考**: ipc-toolkit `BroadPhase` + libuipc `BVH`

```cpp
// 扩展: include/NexDynIPC/Physics/Contact/BroadPhase.h

// 已有: AABB, BVH (需增强)
// 新增: 
// - AABB随刚体运动自动更新
// - 与RigidBodyCollisionMesh联动

class RigidBodyBroadPhase {
public:
    // 为刚体集合构建BVH
    void build(const std::vector<RigidBodyCollisionMesh>& meshes,
               const std::vector<Eigen::VectorXd>& positions);
    
    // 检测候选碰撞对
    std::vector<CollisionPair> detectCandidatePairs();
    
private:
    // 刚体AABB列表
    std::vector<AABB> aabbs_;
    // BVH结构
    BVH bvh_;
};
```

**工作量**: 1周

##### 2.2 Candidates 构建

**参考**: ipc-toolkit `Candidates` 类

```cpp
// 新增文件: include/NexDynIPC/Physics/Contact/Candidates.h

namespace NexDynIPC::Physics::Contact {

// 碰撞候选对
struct CollisionCandidate {
    size_t body_a_idx;
    size_t body_b_idx;
    
    // 基元类型对
    enum class Type {
        VERTEX_VERTEX,       // VV
        VERTEX_EDGE,         // VE  
        VERTEX_FACE,         // VF
        EDGE_EDGE,           // EE
        EDGE_FACE,           // EF
        FACE_FACE            // FF
    };
    Type type;
    
    // 基元索引
    size_t primitive_a_idx;
    size_t primitive_b_idx;
};

// 候选对管理器
class CollisionCandidates {
public:
    // 从宽相位结果构建候选对
    void build(const RigidBodyBroadPhase& broad_phase,
               const std::vector<RigidBodyCollisionMesh>& meshes,
               const std::vector<Eigen::VectorXd>& positions);
    
    // 获取候选对列表
    const std::vector<CollisionCandidate>& get() const { return candidates_; }
    
    // 过滤近邻候选对（窄相检测）
    void filterByDistance(double dhat);
    
private:
    std::vector<CollisionCandidate> candidates_;
};

}
```

**工作量**: 1-2周

##### 2.3 窄相距离计算

```cpp
// 扩展: src/Physics/Contact/Distance.cpp

// 实现所有Primitive对之间的距离计算
class NarrowPhaseDistance {
public:
    // 对每个候选对计算精确距离
    std::vector<DistanceResult> computeDistances(
        const std::vector<CollisionCandidate>& candidates,
        const std::vector<RigidBodyCollisionMesh>& meshes,
        const std::vector<Eigen::VectorXd>& positions);
    
private:
    // dispatch到具体实现
    DistanceResult computeVV(const CollisionCandidate& c, ...);
    DistanceResult computeVE(const CollisionCandidate& c, ...);
    DistanceResult computeVF(const CollisionCandidate& c, ...);
    DistanceResult computeEE(const CollisionCandidate& c, ...);
};
```

**工作量**: 1-2周

---

#### Phase 3: 势能集成

##### 3.1 ContactForm 实现

**参考**: ipc-toolkit `BarrierPotential` + stark `EnergyFrictionalContact`

```cpp
// 新增文件: include/NexDynIPC/Dynamics/Forms/ContactForm.h

namespace NexDynIPC::Dynamics {

class ContactForm : public Form {
public:
    ContactForm(const World& world,
                double dhat = 0.01,
                double stiffness = 1e5,
                bool enable_friction = true,
                double mu = 0.5);
    
    // 更新接触状态（每步调用）
    void updateContacts(
        const std::vector<CollisionCandidate>& candidates,
        const std::vector<DistanceResult>& distances);
    
    // Form接口
    double value(const Eigen::VectorXd& x) const override;
    void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const override;
    void hessian(const Eigen::VectorXd& x, 
                  std::vector<Eigen::Triplet<double>>& hess) const override;
    
private:
    // Barrier势能
    double barrierEnergy(double d) const;
    double barrierGradient(double d) const;
    double barrierHessian(double d) const;
    
    // 摩擦势能
    double frictionEnergy(const DistanceResult& contact, 
                          const Eigen::VectorXd& x) const;
    
    // 接触数据
    struct ContactData {
        size_t body_a, body_b;
        size_t prim_a, prim_b;
        DistanceResult::Type type;
        double d;           // 当前距离
        double d0;          // 上一步距离
        Eigen::Vector3d normal;
        Eigen::Vector3d tangent_u;
        Eigen::Vector3d tangent_v;
    };
    std::vector<ContactData> contacts_;
    
    double dhat_;
    double stiffness_;
    bool enable_friction_;
    double mu_;
};

}
```

**IPC Barrier函数**（来自ipc-toolkit）:
```cpp
// 标准Log Barrier
double b(double d, double dhat) {
    if (d >= dhat) return 0.0;
    double t = d / dhat;
    return -dhat * dhat * log(t) / 3.0;  // 因子3对应3D
}

// 梯度
double db(double d, double dhat) {
    if (d >= dhat) return 0.0;
    return -dhat * (dhat / d - 1.0) / 3.0;
}

// Hessian
double ddb(double d, double dhat) {
    if (d >= dhat) return 0.0;
    return dhat * dhat / (d * d) / 3.0;
}
```

**工作量**: 2-3周

##### 3.2 IPCSolver 集成

```cpp
// 修改: src/Dynamics/IPCSolver.cpp

void IPCSolver::step(World& world, double dt) {
    // ... 现有代码 ...
    
    // 1. 构建/更新碰撞网格
    for (auto& body : world.bodies) {
        collision_mesh_[body->id].build(*body);
    }
    
    // 2. 宽相位检测
    RigidBodyBroadPhase broad_phase;
    broad_phase.build(collision_meshes_, positions_);
    auto candidate_pairs = broad_phase.detectCandidatePairs();
    
    // 3. 构建候选对
    CollisionCandidates candidates;
    candidates.build(broad_phase, collision_meshes_, positions_);
    
    // 4. 窄相距离计算
    NarrowPhaseDistance narrow_phase;
    auto distances = narrow_phase.computeDistances(candidates, collision_meshes_, positions_);
    
    // 5. 创建/更新ContactForm
    if (!contact_form_) {
        contact_form_ = std::make_shared<ContactForm>(world);
    }
    contact_form_->updateContacts(candidates, distances);
    
    // 6. 添加到优化问题
    problem.addForm(contact_form_);
    
    // ... 现有Newton迭代 ...
}
```

**工作量**: 1周

---

#### Phase 4: CCD增强

##### 4.1 刚体CCD

**参考**: ipc-toolkit `NonlinearCCD` + rigid-ipc `CCD`

```cpp
// 扩展: src/Physics/Contact/CCD/RigidBodyCCD.h

class RigidBodyCCD {
public:
    // 检测时间步内是否发生碰撞
    bool detectCollision(
        const RigidBody& bodyA,
        const RigidBody& bodyB,
        double dt,
        double& toi);  // 输出: 最早碰撞时间 [0, 1]
    
private:
    // 考虑旋转的轨迹
    Eigen::Vector3d positionAtTime(const RigidBody& body, double t);
    Eigen::Quaterniond rotationAtTime(const RigidBody& body, double t);
};
```

**工作量**: 2周

---

## 四、工作量汇总

### 实现计划总览

| Phase | 模块 | 预估工作量 | 优先级 |
|-------|------|-----------|--------|
| **Phase 1** | | **4-5周** | |
| 1.1 | RigidBodyCollisionMesh | 1周 | P0 |
| 1.2 | 距离计算模块 (VV/VE/VF/EE) | 2-3周 | P0 |
| 1.3 | 梯度/Hessian计算 | 1周 | P0 |
| **Phase 2** | | **3-4周** | |
| 2.1 | RigidBodyBroadPhase增强 | 1周 | P0 |
| 2.2 | CollisionCandidates | 1-2周 | P0 |
| 2.3 | NarrowPhaseDistance | 1-2周 | P0 |
| **Phase 3** | | **3-4周** | |
| 3.1 | ContactForm实现 | 2-3周 | P0 |
| 3.2 | IPCSolver集成 | 1周 | P0 |
| **Phase 4** | | **2周** | |
| 4.1 | RigidBodyCCD | 2周 | P1 |

**总计**: 约 12-15周

---

## 五、与现有代码的整合策略

### 5.1 保持向后兼容

- 现有关节约束系统保持不变
- 现有驱动系统保持不变
- 接触计算作为独立模块添加

### 5.2 文件组织建议

```
include/NexDynIPC/Physics/Contact/
├── Barrier.h              (现有，保留)
├── BroadPhase.h           (现有，增强)
├── Friction.h             (现有，保留)
├── CCD/
│   ├── CCD.h              (现有)
│   ├── RigidBodyCCD.h     (新增)
│   └── TimeOfImpact.h     (现有)
├── Distance.h             (新增)
└── CollisionCandidates.h  (新增)

include/NexDynIPC/Dynamics/Forms/
├── Form.h                 (现有)
├── ContactForm.h          (新增)
└── ... (现有其他Forms)

src/Physics/Contact/
├── Distance.cpp            (新增)
└── ... (现有)

src/Dynamics/
├── Forms/
│   ├── ContactForm.cpp     (新增)
│   └── ... (现有)
└── IPCSolver.cpp          (修改，集成接触)
```

### 5.3 接口设计原则

参考stark的设计，将接触计算设计为可配置的Form：

```cpp
// SceneLoader中的配置示例
{
    "contact": {
        "enabled": true,
        "dhat": 0.01,
        "stiffness": 1e5,
        "friction": {
            "enabled": true,
            "mu": 0.5
        },
        "ccd": {
            "enabled": true,
            "max_iterations": 10
        }
    }
}
```

---

## 六、结论与建议

### 6.1 核心差距总结

通过分析四个外部IPC实现项目，NexDynIPC要实现完整的Mesh接触计算，需要重点实现：

1. **精确几何距离计算** - 当前最大差距
2. **完整的碰撞检测管线** - BroadPhase → Candidates → NarrowPhase
3. **ContactForm集成** - 势能参与Newton迭代
4. **CCD增强** - 考虑旋转的连续碰撞检测

### 6.2 推荐实现优先级

1. **立即开始**: Phase 1 (核心几何) + Phase 3 (势能集成)
2. **随后进行**: Phase 2 (碰撞检测管线)
3. **最后完善**: Phase 4 (CCD增强)

### 6.3 关键参考代码位置

| 功能 | ipc-toolkit | libuipc | stark |
|------|-------------|---------|-------|
| CollisionMesh | `src/ipc/collision_mesh.hpp` | - | - |
| 距离计算 | `src/ipc/distance/` | `src/geometry/distance.cpp` | `TriangleMeshCollisionDetection` |
| Barrier | `src/ipc/barrier/` | - | `EnergyFrictionalContact` |
| Candidates | `src/ipc/candidates/` | - | - |
| 刚体CCD | `src/ipc/ccd/` | - | - |

---

*报告生成时间: 2026-02-27*
*项目路径: e:\workspace\NexDynIPC*
