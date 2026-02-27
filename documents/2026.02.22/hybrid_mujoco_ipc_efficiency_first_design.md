# 效率优先的混合路线设计：MuJoCo 主流程 + IPC 局部精化

## 1. 设计目标

本方案目标是：在保持当前项目“能量框架可扩展性”的前提下，采用 **MuJoCo 风格高效率主流程** 做全局推进，仅在局部高风险区域调用 **IPC 精化**，实现“效率优先、精度兜底”。

核心原则：

1. **全局快解**：默认用低成本、可并行、可 warmstart 的约束/接触求解。
2. **局部精化**：仅对少量高风险接触簇启用 IPC 精化。
3. **预算驱动**：每步计算受时间预算约束，超预算自动退化精化。
4. **渐进落地**：不推倒重来，先改流程与调度，再逐步替换子模块。

---

## 2. 为什么采用混合路线

纯 MuJoCo 式路线优势在吞吐与实时性，纯 IPC 式路线优势在强接触鲁棒与能量一致性。混合路线试图拿到两者的共同优势：

1. 大规模时，主流程维持稳定帧时。
2. 高频接触/精细抓取/薄间隙场景，由 IPC 局部精化兜底。
3. 支持未来继续添加控制、可微、接触模型，不被单一框架锁死。

---

## 3. 总体架构（分层）

建议把求解系统拆为三层：

1. **Global Fast Layer（全局快层）**  
   负责刚体状态推进、广义力/约束主求解、主接触响应。  
   目标：低延迟、稳定、可并行。

2. **Local Refinement Layer（局部精化层）**  
   对高风险接触区域建立局部子问题，用 IPC 能量最小化修正接触状态。  
   目标：消除穿透风险，提升局部精度。

3. **Policy & Budget Layer（策略预算层）**  
   决定本步哪些区域精化、精化多少迭代、何时降级。  
   目标：保证实时预算和鲁棒性。

---

## 4. 单步仿真流程（推荐）

以下流程可以直接作为 `step()` 设计蓝图：

1. **Stage A: 运动学/惯性更新**  
   更新 `M, c, J` 所需量，更新 broad-phase 索引。

2. **Stage B: 快速接触候选生成**  
   用 BVH + 过滤生成 contact pairs，并构建约束岛（island）。

3. **Stage C: 全局快解（主求解）**  
   在每个 island 上运行主求解器（可选 PGS/CG/Newton 风格），得到 `qacc / v / x` 预测状态。

4. **Stage D: 风险评估与精化选择**  
   计算每个接触簇风险分数，只选 Top-K 高风险簇进入 IPC 精化。

5. **Stage E: IPC 局部精化**  
   对选中的簇构建局部能量子问题（只含相关 DOF），做有限步牛顿修正。

6. **Stage F: 状态合并与守恒检查**  
   将局部修正写回全局状态，做约束残差、最小距离、能量异常检查。

7. **Stage G: 预算回路**  
   若超出预算，降低下一帧精化强度（K、迭代上限、CCD级别）；若余量充足，可提高局部精化比例。

---

## 5. 关键设计点

## 5.1 “全局快解”如何设计

建议特征：

1. 面向约束求解而非全局大能量最小化。
2. 支持 warmstart（上一帧解作为初值）。
3. 支持 island 并行。
4. 支持 early-stop（改进量或残差达阈值即停）。

你当前项目可先保留现有牛顿框架，但在主路径上引入：

1. 稀疏模式复用（减少重复 `analyzePattern`）。
2. 岛化后分块求解。
3. 低迭代预算 + 统计闭环。

---

## 5.2 “IPC 局部精化”如何设计

局部子问题只包含：

1. 高风险接触涉及的 body DOF（以及必要邻接 body）。
2. 对应接触屏障势、摩擦势、必要关节约束项。
3. 固定边界条件（外部 body 视为冻结或线性化背景）。

精化输出：

1. 接触法向修正（减少穿透/逼近距离阈值）。
2. 切向速度/位移修正（改善摩擦响应）。
3. 必要时回写局部拉格朗日乘子修正。

建议控制：

1. 每簇 IPC 迭代上限（如 3~8 次）。
2. 每步最多精化簇数（Top-K）。
3. 超预算时自动降级（K 减半、迭代减半）。

---

## 5.3 风险评分（触发式精化）

建议定义风险分数：

`risk = w1 * penetration_risk + w2 * relative_normal_speed + w3 * friction_slip_risk + w4 * residual`

可用指标：

1. `penetration_risk`：`d_min < d_hat` 的程度。
2. `relative_normal_speed`：高速逼近优先。
3. `friction_slip_risk`：切向速度大且法向力大。
4. `residual`：快解后约束残差仍大。

策略：

1. 只对 `risk > T_refine` 的簇启用 IPC。
2. 簇按风险排序，取前 K 个。
3. 对连续多帧高风险簇可提升优先级（时间一致性）。

---

## 5.4 预算控制（强烈建议）

定义每步预算：

1. `B_total`：总预算（例如 4 ms）。
2. `B_fast`：快层预算（例如 2.8 ms）。
3. `B_refine`：精化预算（例如 1.0 ms）。
4. `B_guard`：留给导出/日志等（例如 0.2 ms）。

运行中动态调度：

1. 快层超时：减少主迭代上限、降低接触候选上限。
2. 精化超时：立即停止后续精化簇，保留已完成修正。
3. 连续超时 N 帧：临时进入“效率保护模式”。

---

## 6. 数据结构建议

建议新增以下核心结构：

1. `ContactCluster`
   - `cluster_id`
   - `body_ids`
   - `contact_ids`
   - `risk_score`
   - `island_id`

2. `RefinementBudget`
   - `max_clusters`
   - `max_newton_iters_per_cluster`
   - `time_budget_ms`
   - `consumed_ms`

3. `HybridSolverStats`
   - `fast_stage_ms`
   - `refine_stage_ms`
   - `num_clusters_refined`
   - `num_ipc_iters_total`
   - `constraint_residual_before/after`
   - `min_distance_before/after`

---

## 7. 与当前代码的落地映射（NexDynIPC）

建议改造入口与模块：

1. **主调度入口**  
   - `src/Dynamics/IPCSolver.cpp`  
   - 将单一路径 `step()` 拆为多 stage 函数。

2. **接触候选与聚类**  
   - `include/NexDynIPC/Physics/Contact/BroadPhase.h`  
   - `src/Physics/Contact/BroadPhase.cpp`  
   - 新增 contact clustering 模块。

3. **局部精化器**  
   - 新增 `LocalIPCRefiner`（建议 `src/Dynamics/Refinement/`）。
   - 复用现有 `Form`、`NewtonSolver`，但限制到局部 DOF 子空间。

4. **预算与策略**  
   - 新增 `HybridPolicy`、`RefinementBudgetController`。
   - 在每步结束根据统计回写下一步策略参数。

5. **精度关键修复（必须优先）**
   - `src/Dynamics/Forms/FrictionForm.cpp`：移除固定 `dt=0.01`，改用真实 `dt`。
   - `include/NexDynIPC/Physics/Contact/CCD/CCD.h`：补上角速度旋转推进。

---

## 8. 伪代码（可直接实现）

```cpp
StepResult HybridStep(World& world, double dt, HybridConfig cfg) {
    Timer timer_total;
    HybridSolverStats stats;

    // A/B: kinematics + contacts + islands
    updateKinematicsAndInertia(world, dt);
    auto contacts = buildContactsFast(world);
    auto islands = buildConstraintIslands(world, contacts);

    // C: fast global solve
    auto fast_result = solveFastGlobal(world, islands, cfg.fast);
    stats.constraint_residual_before = fast_result.residual;
    stats.min_distance_before = fast_result.min_distance;

    // D: select risky clusters
    auto clusters = buildContactClusters(contacts, islands);
    scoreRisk(clusters, fast_result);
    auto selected = selectTopKClusters(clusters, cfg.refine.max_clusters, cfg.refine.risk_threshold);

    // E: local IPC refinement under budget
    RefinementBudget budget = cfg.refine.budget;
    for (auto& c : selected) {
        if (budget.exceeded()) break;
        refineClusterIPC(world, c, dt, cfg.refine, budget, stats);
    }

    // F: merge + checks
    mergeRefinementToWorld(world);
    auto after = evaluateStateQuality(world);
    stats.constraint_residual_after = after.residual;
    stats.min_distance_after = after.min_distance;

    // G: adapt policy for next step
    adaptPolicyByBudgetAndQuality(cfg, stats, timer_total.elapsedMs());

    return {world, stats};
}
```

---

## 9. 实施路线（建议）

### M1（1-2 周）：流程化与预算化

1. `step()` 拆 stage，接入 `HybridSolverStats`。
2. 加预算控制器和风险评分框架（先用简化评分）。
3. 修复摩擦 `dt` 一致性。

目标：先建立“可控的效率框架”。

### M2（2-4 周）：局部精化最小闭环

1. 实现 `LocalIPCRefiner`（只处理单簇）。
2. 完成“选择 Top-K 簇 + 局部修正 + 回写”。
3. 增加前后质量指标输出。

目标：证明局部精化在有限预算内能降低穿透/残差。

### M3（4-8 周）：规模化优化

1. 引入 island 并行。
2. 增强 warmstart。
3. 完善 CCD 旋转推进与接触聚类质量。

目标：在大场景下保持预算稳定。

---

## 10. 验收标准（效率优先）

必须同时满足两类指标：

1. 效率指标  
   - `P95 step time` 不超过预算。  
   - 超预算帧比例低于目标（如 < 2%）。

2. 精度指标  
   - 穿透事件次数下降。  
   - 最小距离统计改善。  
   - 约束残差下降或不劣化。  
   - 参考轨迹误差（RMSE）不明显恶化。

---

## 11. 最终建议

在你的场景里，建议明确采用：

1. **主流程 MuJoCo 化**：面向吞吐与实时性。
2. **精度 IPC 化**：仅在局部关键接触做高质量修正。
3. **调度预算化**：确保“效率优先”的目标可执行、可验证、可回归。

这条路线不是折中，而是工程上更现实的“分层最优”：  
把高成本精度放到真正需要的地方，把全局计算留给高效率路径。

