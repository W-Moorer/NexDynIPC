# NexDynIPC 与 rigid-ipc-main 接触完整复刻差异分析

日期：2026-02-27

## 结论先行

可以做“完整复刻”，但不是小改动：这不是参数调优，而是接触求解架构迁移。

当前 NexDynIPC 的接触链路是“简化近似版”，而 rigid-ipc-main 是“约束集 + barrier 能量 + CCD + 迭代耦合”的完整 IPC 管线。

---

## 1) 接触网格导入差异

### rigid-ipc-main
- 网格导入支持：OBJ、通用三角网格、线元/面元、组件拆分、尺度/旋转/密度/kinematic 轨迹。
- 关键入口：
  - [temps/rigid-ipc-main/rigid-ipc-main/src/io/read_obj.hpp](temps/rigid-ipc-main/rigid-ipc-main/src/io/read_obj.hpp)
  - [temps/rigid-ipc-main/rigid-ipc-main/src/io/read_rb_scene.cpp](temps/rigid-ipc-main/rigid-ipc-main/src/io/read_rb_scene.cpp)

### 当前 NexDynIPC
- 仅在 SceneLoader 中做了轻量 OBJ 读取，并绑定到 `RigidBody::shape`。
- 入口：
  - [nexdynipc/src/App/SceneLoader.cpp](nexdynipc/src/App/SceneLoader.cpp)
  - [nexdynipc/include/Physics/Geometry/MeshShape.h](nexdynipc/include/Physics/Geometry/MeshShape.h)

### 差距
- 缺少“统一场景级刚体装配器（assembler）”与几何拓扑规范化（边/面/连通分量）流程。
- 缺少 kinematic 轨迹、组装参数、多体导入策略的一致入口。

---

## 2) 接触检测差异（Broad-phase / Narrow-phase / CCD）

### rigid-ipc-main
- 使用完整约束集生成：VV/EV/EE/FV（点点、边点、边边、面点）。
- 使用 CCD 与最早碰撞时间（TOI）计算，和求解步耦合。
- 关键入口：
  - [temps/rigid-ipc-main/rigid-ipc-main/src/problems/rigid_body_collision_constraint.hpp](temps/rigid-ipc-main/rigid-ipc-main/src/problems/rigid_body_collision_constraint.hpp)
  - [temps/rigid-ipc-main/rigid-ipc-main/src/ccd/ccd.hpp](temps/rigid-ipc-main/rigid-ipc-main/src/ccd/ccd.hpp)
  - [temps/rigid-ipc-main/rigid-ipc-main/src/problems/distance_barrier_rb_problem.cpp](temps/rigid-ipc-main/rigid-ipc-main/src/problems/distance_barrier_rb_problem.cpp)

### 当前 NexDynIPC
- 已有 BroadPhase 与 BVH 组件，但求解主链仍主要依赖简化中心距离近似；
- 现有 CCD 系统在 `IPCSolver` 中可调用，但与完整约束集并未同层融合。
- 入口：
  - [nexdynipc/include/Physics/Contact/BroadPhase.h](nexdynipc/include/Physics/Contact/BroadPhase.h)
  - [nexdynipc/src/Physics/Contact/BroadPhase.cpp](nexdynipc/src/Physics/Contact/BroadPhase.cpp)
  - [nexdynipc/src/Dynamics/IPCSolver.cpp](nexdynipc/src/Dynamics/IPCSolver.cpp)

### 差距
- 缺失 IPC 级别窄相约束类型与统一候选约束池；
- 缺失“检测结果 -> barrier/friction 约束对象”的结构化转换；
- 缺失与 Newton 步一致的 CCD 对齐逻辑。

---

## 3) 接触计算差异（Barrier / Friction / Hessian）

### rigid-ipc-main
- 目标函数：刚体能量 + barrier 项 + friction 项，统一在优化问题内计算值/梯度/Hessian。
- 自适应刚度（κ）更新有完整理论流程：初始 κ、上界、迭代更新。
- 关键入口：
  - [temps/rigid-ipc-main/rigid-ipc-main/src/problems/distance_barrier_rb_problem.hpp](temps/rigid-ipc-main/rigid-ipc-main/src/problems/distance_barrier_rb_problem.hpp)
  - [temps/rigid-ipc-main/rigid-ipc-main/src/solvers/ipc_solver.cpp](temps/rigid-ipc-main/rigid-ipc-main/src/solvers/ipc_solver.cpp)

### 当前 NexDynIPC
- Barrier 与 Friction 存在，但接触项仍偏“外置近似驱动”而非完整统一优化约束系统。
- 入口：
  - [nexdynipc/include/Physics/Contact/AdaptiveBarrier.h](nexdynipc/include/Physics/Contact/AdaptiveBarrier.h)
  - [nexdynipc/include/Dynamics/Forms/FrictionForm.h](nexdynipc/include/Dynamics/Forms/FrictionForm.h)
  - [nexdynipc/src/Dynamics/IPCSolver.cpp](nexdynipc/src/Dynamics/IPCSolver.cpp)

### 差距
- 缺失完整“距离约束集驱动”的 barrier/friction 导数链；
- 缺失和接触约束一致的 Hessian 结构；
- 缺失 friction 约束迭代（lagged friction）机制。

---

## 4) 接触对积分器的影响差异

### rigid-ipc-main
- 接触与积分不是分离后处理：接触能量直接进入时间步优化问题。
- 支持多种体能量积分法并在问题层切换（implicit Euler / implicit Newmark / stabilized Newmark）。
- 入口：
  - [temps/rigid-ipc-main/rigid-ipc-main/src/time_stepper/time_stepper.hpp](temps/rigid-ipc-main/rigid-ipc-main/src/time_stepper/time_stepper.hpp)
  - [temps/rigid-ipc-main/rigid-ipc-main/src/problems/distance_barrier_rb_problem.hpp](temps/rigid-ipc-main/rigid-ipc-main/src/problems/distance_barrier_rb_problem.hpp)

### 当前 NexDynIPC
- 已有隐式积分器，但接触项与积分器耦合深度不足，更多在求解器外层拼装。
- 入口：
  - [nexdynipc/include/TimeIntegration/ImplicitTimeIntegrator.h](nexdynipc/include/TimeIntegration/ImplicitTimeIntegrator.h)
  - [nexdynipc/src/App/Simulation.cpp](nexdynipc/src/App/Simulation.cpp)
  - [nexdynipc/src/Dynamics/IPCSolver.cpp](nexdynipc/src/Dynamics/IPCSolver.cpp)

### 差距
- 缺失“接触约束对时间离散能量的一等公民化”设计；
- 缺失接触-积分联合收敛判据和同层线搜索策略。

---

## 5) 复刻范围判定（你提出的四项）

你提出的“接触网格导入 + 接触检测 + 接触计算 + 接触对积分器影响”在 rigid-ipc-main 中属于同一闭环。

要达到“完整复刻”，必须至少满足：
1. 约束类型齐全（VV/EV/EE/FV）；
2. barrier/friction 完整导数链纳入统一优化问题；
3. CCD 与 Newton 更新一致；
4. 时间积分与接触项同层耦合。

---

## 6) 建议迁移路线（可落地）

### Phase A：架构对齐（1）
- 新增 `RigidBodyAssembler` 风格中间层，把 `World` 几何拓扑标准化。
- 新增接触约束容器（VV/EV/EE/FV）及转换接口。

### Phase B：检测链路（2）
- 将 Broad-phase 结果映射到窄相约束生成。
- 把 CCD/TOI 接到约束层，而不是只做全局 dt 修正。

### Phase C：能量与求解（3）
- 新建 `DistanceBarrier` 类优化问题对象（值/梯度/Hessian）。
- friction 采用 lagged 迭代并与 barrier 共享约束集。

### Phase D：积分耦合与验收（4）
- 将接触项并入积分步目标函数，统一收敛与线搜索。
- 对齐算例回归（落球、滑移、边边擦碰、多体堆叠）。

---

## 7) 当前能否“完全复刻”？

可以，但建议按上述 Phase 分阶段执行；
如果强行一次性替换，风险是：
- 回归面太大，难以定位数值问题；
- 你现在已有功能（关节、驱动、导出）容易被牵连。

建议先做 Phase A+B（先把检测链路从“中心距离近似”替换成“约束候选集”），这是性能和正确性的共同拐点。
