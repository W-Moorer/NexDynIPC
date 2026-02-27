# NexDynIPC 接触链路 rigid-ipc-main 完整替换实施规划

日期：2026-02-28
负责人：GitHub Copilot（执行规划草案）

## 1. 目标与范围

本次工作目标是：
- 清理当前项目中接触检测相关的近似/分叉算法；
- 完整替换为 rigid-ipc-main 的主流程范式；
- 形成统一闭环：网格导入 → Broad/Narrow Phase → CCD/TOI → Barrier/Friction → 与积分器同层耦合。

本次替换包含以下四条主线（全部必做）：
1) 网格导入与刚体装配；
2) BVH/候选约束构建；
3) 接触计算（Barrier/Friction）导数链；
4) 与时间积分器/求解器一体化。

---

## 2. 当前问题与替换原则

### 2.1 当前问题（需移除/下线）
- 接触链路存在多入口并行：ContactAssembler 候选与 CCD 候选不统一；
- CCD 中存在高复杂度候选展开，且未严格依赖同一候选池；
- 接触项尚未完全作为时间步优化问题的一等项；
- 部分近似路径（中心距离、平面距离直判）与 IPC 约束语义不一致。

### 2.2 替换原则
- 单一候选源：全链路只使用统一 Candidates（VV/EV/EE/FV）；
- 单一问题层：Barrier/Friction 仅通过优化问题计算 Value/Gradient/Hessian；
- 单一时间步入口：CCD 只为时间步和约束激活服务，不另起接触语义；
- 渐进替换：先并行接入与对比，再硬切换，最后删除旧代码。

---

## 3. 目标架构（对齐 rigid-ipc-main）

## 3.1 数据层
- RigidBodyAssembler（新）：
  - 统一管理 global vertices/edges/faces 与 body-local 索引映射；
  - 提供 close_bodies(poses_t0, poses_t1, inflation_radius)；
  - 输出供 Broad/Narrow/CCD 共用的数据视图。

- ContactCandidateSet（升级）：
  - 保留并规范 VV/EV/EE/FV 四类；
  - 增加 body-pair 与 primitive 映射校验；
  - 支持按距离/组别/阈值做筛选。

## 3.2 检测层
- Broad Phase：
  - 使用 swept AABB + BVH 获取 close body pairs；
  - 对 close pairs 做 body-pair 层筛选（allowed pairs/group filtering）。

- Narrow Phase：
  - 基于统一 candidates 进行几何一致性检测；
  - 与 rigid-ipc-main 一致的 primitive 语义（FV/EE/EV/VV）。

- CCD/TOI：
  - 从统一 candidates 做 earliest TOI；
  - earliest TOI 仅影响步长与激活，不绕过约束系统。

## 3.3 物理问题层
- DistanceBarrierProblem（新）：
  - 汇总 inertia + gravity + constraints + barrier + friction；
  - 统一输出 objective / gradient / hessian；
  - 与 Newton/LineSearch 同步。

- Friction（lagged）
  - 使用同一接触候选集和法向响应；
  - 与 barrier 同层迭代更新。

## 3.4 时间步层
- IPCSolver.step() 目标流程：
  1. 组装 bodies/poses；
  2. close_bodies + candidates；
  3. CCD earliest TOI（必要时缩步）；
  4. 构建 DistanceBarrierProblem；
  5. Newton + line search；
  6. 更新状态并导出诊断。

---

## 4. 分阶段实施计划

## Phase A：装配与候选统一（基础替换）
目标：把“数据和候选入口”统一。

工作项：
- 新建 Assembler 数据对象（或升级 ContactAssembler 为 Assembler 语义）；
- 为所有 body 建立全局拓扑映射；
- 将现有接触候选生成迁移到统一 API；
- 引入 close_bodies（swept AABB）并替代当前 CCD 前置体对逻辑。

交付标准：
- 同一帧内，Contact/Friction/CCD 读取同一 candidate 集；
- 输出统计日志一致（body_pairs, candidates）。

## Phase B：CCD/Broad/Narrow 对齐替换
目标：彻底替换当前 CCD 候选生成流程。

工作项：
- 删除或下线 CCD 内部自建候选分支；
- CCD 仅接收统一 candidates；
- 引入 body-pair 早退门控（pairs=0 时 O(1) 返回）；
- 修正 BVH 使用方式，禁止节点索引当作 body 索引。

交付标准：
- t=0 日志可见：close_bodies → candidates → ccd_end 全链路；
- 空候选场景下 CCD 毫秒级返回。

## Phase C：Barrier/Friction 问题层重构
目标：把接触项完全并入优化问题。

工作项：
- 新建 DistanceBarrierProblem（或在 SimulationProblem 内模块化）；
- barrier 值/梯度/Hessian 全部由 candidates 驱动；
- friction 采用 lagged 迭代，与法向响应一致更新；
- 去除中心距离/平面距离等临时近似分支。

交付标准：
- 接触项可独立开关并输出导数一致性检查；
- Newton 每迭代耗时分解中，接触项可追踪。

## Phase D：与积分器耦合与收敛策略
目标：实现接触-积分联合收敛。

工作项：
- 将接触能量完全纳入 time-step objective；
- line search 使用统一目标函数；
- 收敛标准同时检查约束残差与接触残差；
- 对 dt 缩放、TOI、active set 做一致策略。

交付标准：
- 单步结果不再依赖后处理修正；
- 同一场景可稳定运行并复现参考趋势。

## Phase E：旧代码清理与硬切换
目标：删除旧路径，避免回归风险。

工作项：
- 删除历史近似 API 与无效入口；
- 保留一层兼容适配（最多 1 版本周期）；
- 更新文档与调试开关；
- 回归全部接触算例。

交付标准：
- 接触链路无双实现；
- CI/本地回归全通过。

---

## 5. 文件级改造清单（首批）

核心改造文件：
- nexdynipc/include/Physics/Contact/ContactAssembler.h
- nexdynipc/src/Physics/Contact/ContactAssembler.cpp
- nexdynipc/include/Physics/Contact/CCD/CCD.h
- nexdynipc/include/Dynamics/IPCSolver.h
- nexdynipc/src/Dynamics/IPCSolver.cpp
- nexdynipc/src/Dynamics/Forms/FrictionForm.cpp
- nexdynipc/src/Math/NewtonSolver.cpp

新增建议文件：
- nexdynipc/include/Physics/Contact/RigidBodyAssembler.h
- nexdynipc/src/Physics/Contact/RigidBodyAssembler.cpp
- nexdynipc/include/Physics/Contact/CollisionCandidates.h
- nexdynipc/include/Dynamics/Problems/DistanceBarrierProblem.h
- nexdynipc/src/Dynamics/Problems/DistanceBarrierProblem.cpp

---

## 6. 验收与基准

## 6.1 功能验收
- Case-1（空接触）：候选=0 时单步快速返回；
- Case-2（单球落板）：存在稳定接触响应，不穿透；
- Case-3（滑移摩擦）：速度衰减趋势与参考一致；
- Case-4（边边擦碰）：EE/FV 激活正确。

## 6.2 性能验收
- 输出阶段耗时：build_assembler, close_bodies, candidates, ccd, newton；
- t=0 单步耗时较当前版本显著下降（目标：至少 50%）；
- 候选规模与耗时呈线性可解释关系。

## 6.3 数值验收
- 无 NaN/Inf；
- 收敛判据稳定；
- barrier/friction 导数检查通过（抽样）。

---

## 7. 风险与应对

主要风险：
- 一次性替换面过大导致回归困难；
- 旧关节/驱动项与新问题层耦合冲突；
- CCD 与 line search 同时调整引入不稳定。

应对策略：
- 每个 Phase 保留可切换开关；
- 先跑最小场景再扩展资产；
- 每阶段都固化性能与数值基线。

---

## 8. 里程碑建议（可执行）

- M1（2-3天）：Phase A 完成，统一 candidates 与 close_bodies 接口。
- M2（2-3天）：Phase B 完成，CCD 完全改为统一候选输入。
- M3（3-5天）：Phase C 完成，Barrier/Friction 问题层成型。
- M4（2-3天）：Phase D 完成，积分器耦合与收敛统一。
- M5（1-2天）：Phase E 完成，旧代码清理+文档回归。

---

## 9. 本规划的执行顺序（下一步）

建议立刻进入 M1：
1) 新建 RigidBodyAssembler 接口；
2) 将 IPCSolver 的候选来源切换到 Assembler；
3) 把 CCD 改成“仅消费 candidates”；
4) 维持旧流程可开关，做 A/B 对比日志。

本规划文件即为 2026-02-28 实施基线，后续按阶段增量更新。

---

## 10. 执行进展（2026-02-28）

已完成：
- Phase A/B 核心链路已落地：
  - CCD 改为仅消费统一 candidates；
  - 旧 CCD 兼容分支已硬删除；
  - 接入 close_bodies 门控（无近体对或无候选可早退）。
- 测试迁移与稳定性收口完成：
  - CCD 相关测试已迁移到新接口；
  - 回归测试构造脆弱点已修复；
  - 全量测试通过（135/135）。

Phase C 增量完成：
- 法向力计算已下沉到 `DistanceBarrierForm`，删除 `IPCSolver` 内重复 barrier 法向力逻辑；
- 在 ALM 迭代内新增按 `x_new` 重建接触集：
  - 每次迭代刷新 barrier contact set；
  - 每次迭代刷新 friction contact set；
  - friction normal force 使用刷新后的 barrier 响应更新。
- friction 时间步近似已清理：
  - 移除 `FrictionForm` 内部硬编码 `dt=0.01`；
  - 改为由 `IPCSolver` 在每步将 `actual_dt` 注入 `FrictionForm`；
  - 新增测试验证摩擦能量随 `setTimeStep()` 缩放。
- friction 外部状态滞后已进一步清理：
  - `FrictionForm::computeTangentDisplacement()` 不再读取 `world_.bodies[*]->velocity`；
  - 改为基于当前迭代状态 `x` 与上一步位姿推导相对切向位移/速度；
  - 新增测试验证摩擦响应依赖迭代状态而非 world 速度快照。
- friction Jacobian 已升级到含旋转自由度：
  - 梯度/海森从“仅平移块”升级为接触点速度 Jacobian 映射（含角速度项）；
  - 在接触点偏心位置下，旋转 DOF 对摩擦响应的贡献已计入；
  - 新增 `FrictionForm` 旋转分量有限差分测试并通过。
- 姿态恢复语义已与 IPCSolver 完全对齐：
  - `FrictionForm::getBodyState()` 由“绝对旋转向量→四元数”改为 `dq * base_orientation`（与 `IPCSolver` 一致）；
  - 接触点偏移 `r` 改为“base 姿态局部锚点”经当前姿态旋回世界，统一旋转线性化来源；
  - 在非单位初始姿态下的旋转有限差分校验已通过。

Phase D 首步完成：
- 已在 ALM 迭代中引入接触残差指标（由 barrier + friction 接触梯度无穷范数构成）；
- 收敛判据由“约束残差 + 对偶残差”扩展为“约束残差 + 对偶残差 + 接触残差”并行检查；
- 新增 `IPCSolver` 接口：`setALMContactTolerance()` 与 `lastContactResidual()`，用于调参与诊断。

Phase D 第二步完成：
- line search 接受准则已显式纳入接触目标：
  - 在原总目标 Armijo 条件之外，新增接触目标的 Armijo/非增检查；
  - 接触目标与接触梯度由 `OptimizationProblem` 的 contact-only 接口提供（默认零贡献，兼容旧问题类）；
  - `SimulationProblem` 已实现 contact-only value/gradient（仅 barrier + friction），实现“同层收敛”闭环。

Phase D 第三步完成：
- 已新增接触残差时间序列诊断接口（每次 ALM/Newton）：
  - `NewtonSolver` 记录每次 Newton 的接触残差、线搜索步长、接触目标前后值；
  - `IPCSolver` 在每次 ALM 后聚合 Newton 诊断，导出 `lastContactResidualSeries()`；
  - 序列元素包含 `(alm_iteration, newton_iteration, contact_residual, line_search_alpha, contact_value_before, contact_value_after)`。

诊断导出工具已落地：
- `IPCSolver::writeLastContactResidualSeriesCSV()` 支持一步导出 CSV；
- 默认输出路径：`output/contact_residual_series.csv`；
- 支持自定义路径，自动创建上级目录。

待继续（Phase C/D）：
- 将 friction 的切向位移/速度模型从固定 `dt` 近似替换为与积分器一致的离散化；
- 统一接触残差与约束残差的收敛判据，并并入 line search 目标；
- 逐步引入（或模块化到）DistanceBarrierProblem，削减 IPCSolver 中流程拼接代码。