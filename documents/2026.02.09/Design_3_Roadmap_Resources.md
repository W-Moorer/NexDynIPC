# 刚体接触仿真平台设计文档 (3/3) - 开发路线与参考资源

## 1. 开发路线图 (Development Roadmap)

本项目建议按 12 周 (3 个月) 周期规划，分为 6 个里程碑 (Milestones)。

### Phase 1: 基础设施建设 (Week 1-4)

#### **M1: 基础几何与刚体系统 (Week 1-2)**
- **目标**: 实现刚体状态管理、SDF几何接口、简易显式积分器。
- **输入**: 相关数学库 (Eigen)。
- **输出**: 
  - `RigidBody` 类
  - `SphereSDF`, `BoxSDF`
  - **`Form` 基类定义 (参考 PolyFEM)**
  - 简单的自由落体测试 (无碰撞)
  - 可视化接口 (输出OBJ/VTK序列)

#### **M2: 隐式积分器与牛顿求解器 (Week 3-4)**
- **目标**: 实现 Backward Euler 积分器和无约束的牛顿迭代。
- **输入**: M1 的刚体系统。
- **输出**: 
  - `ImplicitIntegrator`
  - `NewtonSolver`
  - 能够稳定模拟大步长下的自由运动和简单的弹簧阻尼系统。

---

### Phase 2: 核心接触算法 (Week 5-8)

#### **M3: IPC 接触模型 (Week 5-6)**
- **目标**: 实现基于 SDF 的距离查询、Barrier 势能、以及对应的梯度/Hessian。
- **输入**: M2 的积分器框架。
- **输出**: 
  - `IPCContactModel`
  - 点-SDF 距离查询优化
  - 能够模拟球体掉落在固定的 SDF 地面上 (无穿透，无摩擦)。

#### **M4: 摩擦与线性系统求解 (Week 7-8)**
- **目标**: 引入摩擦势能，优化线性求解器效率 (使用 Sparse Cholesky)。
- **输入**: M3 的接触模型。
- **输出**: 
  - `FrictionModel` (基于 Maximum Dissipation Principle 或近似模型)
  - 稀疏矩阵组装器 (`SparseMatrixAssembler`)
  - 斜面滑块测试 (验证摩擦力)。

---

### Phase 3: 约束与高级特性 (Week 9-12)

#### **M5: 关节约束系统 (Week 9-10)**
- **目标**: 实现球铰、旋转副等约束，集成到优化框架中。
- **输入**: M4 的完整动力学系统。
- **输出**: 
  - `ConstraintSystem`
  - `RevoluteJoint`, `SphericalJoint`
  - 双摆、链条测试场景。

#### **M6: 系统优化与验证 (Week 11-12)**
- **目标**: 代码性能优化、参数调优、编写完整文档。
- **输入**: 完整的仿真平台。
- **输出**: 
  - 并行化 SDF 查询
  - 宽相检测优化 (Spatial Hash / BVH)
  - 用户手册与 API 文档
  - 最终演示 Demo (复杂机械结构跌落)。

---

## 2. 风险点与应对策略

| 风险点 | 影响 | 应对策略 |
| :--- | :--- | :--- |
| **SDF 精度问题** | 接触点计算不准确，物体抖动 | 对于简单几何使用解析 SDF；对于 Mesh 使用高质量的 Grid SDF (如 OpenVDB) 或 Discregrid。 |
| **牛顿迭代不收敛** | 仿真爆炸或卡死 | 实现鲁棒的线搜索 (Line Search) 与 CCD；在极难收敛时回退到较小步长或增加阻尼。 |
| **线性求解效率低** | 仿真速度慢 | 确保 Hessian 的稀疏性被利用；使用 SuiteSparse 或 Pardiso 等高性能求解器。 |
| **参数敏感 (Kappa/Hat)** | 穿透或刚度过大 | 提供参数自动调节机制，或遵循 IPC paper 的自适应刚度策略。 |

---

## 3. 参考资源 (References)

### 3.1 核心论文
1. **[IPC]** Li, M., et al. "Incremental Potential Contact: Intersection-and-Inversion-Free, Large-Deformation Dynamics." *SIGGRAPH 2020*.
   - *必读，理解 Barrier Function 和 CCD 集成。*
2. **[Affine Body IPC]** Lan, L., et al. "Affine Body Dynamics: Fast, Stable and Intersection-free Simulation of Stiff Materials." *SIGGRAPH 2022*.
   - *参考其对刚性/仿射物体的处理方式。*
3. **[Rigid IPC]** Ferguson, Z., et al. "Intersection-Free Rigid Body Dynamics." *SIGGRAPH 2021*.
   - *直接相关的刚体 IPC 实现细节。*

### 3.2 开源参考代码
- **ipc-toolkit**: https://github.com/ipc-sim/ipc-toolkit
  - *官方 IPC 核心库，包含 Barrier 和摩擦计算的参考实现。*
- **rigid-ipc**: https://github.com/ipc-sim/rigid-ipc
  - *刚体 IPC 的参考实现，重点关注其对转动自由度 (Rotation) 的处理。*
- **polyfem**: https://github.com/polyfem/polyfem
  - *包含完整的 FEM + IPC 流程。其 `solver/forms` 模块展示了如何优雅地组织不同物理势能 (Elastic, Contact, Friction)，是本项目架构设计的主要参考。*

### 3.3 数学相关
- **Lie Group Integration**: Eade, E. "Lie Groups for 2D and 3D Transformations."
  - *用于刚体姿态的更新和微分。*
- **Optimization**: Nocedal, J., & Wright, S. "Numerical Optimization."
  - *牛顿法与线搜索的理论基础。*
