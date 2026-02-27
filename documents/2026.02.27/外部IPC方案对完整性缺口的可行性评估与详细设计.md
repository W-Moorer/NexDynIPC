# 外部IPC方案对完整性缺口的可行性评估与详细设计

**日期**: 2026-02-27  
**项目**: NexDynIPC  
**输入文档**:
- `documents/2026.02.27/基于外部IPC实现的接触计算完善方案.md`
- `documents/2026.02.27/动力学仿真完整性缺口总结.md`

---

## 1. 结论（先回答问题）

结论分两层：

1. **若问题是“是否能补齐《缺口总结》的全部剩余部分”**：**不可直接一次性完成**。  
   原因是外部IPC方案主要覆盖接触主链路（几何距离、候选构建、Barrier、CCD），而《缺口总结》中的约束装配层、动力学参数化、场景产品化、系统回归与CI等部分并不由该方案直接覆盖。

2. **若问题是“是否能完成剩余缺口中的核心阻塞项（P0主线）并带动后续闭环”**：**可行**。  
   可直接完成 A/B/C（接触几何、CCD、摩擦耦合）和 G 的一部分（求解鲁棒性中与接触相关的退化处理与诊断），并为 D/F/H 提供稳定底座。

因此本设计采用：**“可行且应实施，但范围按阶段收敛”**。

---

## 2. 缺口 A-H 与外部IPC方案覆盖映射

| 缺口项 | 缺口总结中的定位 | 外部IPC方案覆盖度 | 可行性判定 | 备注 |
|---|---|---|---|---|
| A. 接触几何与距离查询 | 高优先级 | 高 | 可行（直接） | 与 `CollisionMesh + Distance + Candidates` 完全对齐 |
| B. CCD不完整 | 高优先级 | 中高 | 可行（直接） | 需补旋转轨迹与TOI策略 |
| C. 摩擦耦合不完整 | 高优先级 | 中高 | 可行（直接） | ContactForm与法向接触同源耦合可落地 |
| D. 约束与关节工程能力 | 中高优先级 | 低 | 部分可行（间接） | 外部IPC仅提供接触侧，不替代 ConstraintSet/CompositeJoint 设计 |
| E. 状态参数化原型化 | 中优先级 | 低 | 部分可行（间接） | 可借 rigid-ipc 的刚体参数化思路，但需本地动力学重构 |
| F. 场景建模能力不足 | 中优先级 | 低 | 部分可行（间接） | 可扩展 SceneLoader 配置，但非IPC核心 |
| G. 求解器鲁棒性/性能 | 中优先级 | 中 | 可行（部分） | 可引入接触相关 regularization/diagnostic，但线性后端优化需独立推进 |
| H. 产品化配套能力 | 中优先级 | 低 | 不可直接覆盖 | 需额外回归体系、可视化与CI建设 |

**综合判断**：外部IPC方案可以完成《缺口总结》里“剩下部分”的**核心物理阻塞项**，但不能单文档解决“完整产品化”全部尾项。

---

## 3. 详细设计（按“可完成部分优先”）

### 3.1 目标范围

本次设计目标定义为：
- **必须闭环**：A/B/C
- **同步增强**：G（接触相关鲁棒性）
- **给出接口预留**：D/E/F/H

---

### 3.2 模块设计与落点（NexDynIPC）

#### M1. 碰撞网格与几何查询（A）

**新增/扩展模块**
- `include/NexDynIPC/Physics/Geometry/RigidBodyCollisionMesh.h`（新增）
- `include/NexDynIPC/Physics/Contact/Distance.h`（新增）
- `src/Physics/Contact/Distance.cpp`（新增）

**关键能力**
- 刚体网格顶点/边/面缓存与索引映射
- VV/VE/VF/EE 精确距离与接触点法向恢复
- 距离导数（至少一阶）供 Form 梯度使用

**参考仓库对应部分**
- ipc-toolkit: `src/ipc/collision_mesh.hpp`, `src/ipc/distance/`
- libuipc: `src/geometry/distance.cpp`
- stark: `extern/TriangleMeshCollisionDetection/`

---

#### M2. 宽相位 + 候选生成 + 窄相（A/B）

**新增/扩展模块**
- `include/NexDynIPC/Physics/Contact/BroadPhase.h`（增强）
- `include/NexDynIPC/Physics/Contact/CollisionCandidates.h`（新增）
- `src/Physics/Contact/*` 下补 `NarrowPhaseDistance` 实现

**关键能力**
- BVH/AABB 动态更新
- 候选对类型化（VV/VE/VF/EE/可扩展EF/FF）
- 与窄相距离联动过滤（`d < dhat`）

**参考仓库对应部分**
- ipc-toolkit: `src/ipc/broad_phase/`, `src/ipc/candidates/`
- libuipc: `src/geometry/bvh/`, `src/backends/cuda/collision_detection/`

---

#### M3. ContactForm 势能耦合（C）

**新增/扩展模块**
- `include/NexDynIPC/Dynamics/Forms/ContactForm.h`（新增）
- `src/Dynamics/Forms/ContactForm.cpp`（新增）
- `src/Dynamics/IPCSolver.cpp`（改造接入）

**关键能力**
- Barrier 势能、梯度、Hessian 与求解器同迭代
- 摩擦势能与法向接触同源接触集（去除硬编码 `dt`）
- 接触历史（切向基/累积位移）缓存，支持 stick-slip

**参考仓库对应部分**
- ipc-toolkit: `src/ipc/barrier/`, `src/ipc/potentials/`
- stark: `src/models/interactions/EnergyFrictionalContact*`

---

#### M4. 刚体CCD增强（B）

**新增/扩展模块**
- `include/NexDynIPC/Physics/Contact/CCD/RigidBodyCCD.h`（新增）
- `src/Physics/Contact/CCD/*`（扩展实现）

**关键能力**
- 旋转轨迹插值（含角速度）
- TOI 求解 + 安全裕量
- 与步长控制策略联动（substep/rollback）

**参考仓库对应部分**
- ipc-toolkit: `src/ipc/ccd/`
- rigid-ipc: `src/ccd/`

---

#### M5. 求解鲁棒性与诊断（G-部分）

**新增/扩展模块**
- `src/Dynamics/IPCSolver.cpp`（失败回退策略）
- `output/*`（新增诊断指标输出）

**关键能力**
- Newton 失败时阻尼/正则回退
- 接触数量、最小距离、TOI裁剪率、残差曲线导出
- 形成 A/B/C 方案的可观测闭环

**参考仓库对应部分**
- ipc-toolkit: 势能 + 牛顿求解耦合流程
- stark: 接触能量与自动微分流程可作为诊断指标设计参考

---

## 4. 对未完全覆盖项的补齐策略（D/E/F/H）

### D. 约束装配层
- 在现有原语化关节基础上补 `ConstraintSet/CompositeJoint` 显式层。
- 外部仓库参考：stark 的 interaction 注册方式（作为装配思路），NexDynIPC 自主实现接口。

### E. 状态参数化
- 参考 rigid-ipc 的刚体状态表示与旋转更新策略，统一 `x -> pose/velocity/inertia` 映射。
- 与 M4 同步推进，避免CCD与状态解释不一致。

### F. 场景建模
- 扩展 Scene JSON：mesh path、material、collision filter、contact/ccd 参数。
- 与 ContactForm 参数直接映射，减少运行时硬编码。

### H. 产品化能力
- 补系统级回归基准（双摆、堆叠、滑动摩擦、高速旋转碰撞）。
- 加 CI 门禁：能量漂移阈值、最小距离阈值、求解失败率阈值。

---

## 5. 分阶段里程碑（建议）

### P0（6~8周，先打通物理闭环）
1. M1 距离/法向/接触点
2. M2 候选与窄相
3. M3 ContactForm（含摩擦去硬编码 dt）
4. M4 旋转CCD最小可用版本

**验收标准**
- 典型接触场景无明显漏检穿透
- `d_min >= -eps`，且接触力随步长变化不发散
- CSV 输出包含接触诊断字段

### P1（3~4周，稳定化）
1. M4 完整TOI与步长策略
2. M5 失败回退与指标完善
3. D/F 最小工程化接口

### P2（3~6周，工程化）
1. E 状态参数化统一
2. H 回归与CI体系
3. 性能优化（线性后端、并行与缓存）

---

## 6. 实施风险与规避

1. **风险：距离导数实现复杂，影响收敛**  
   规避：先上稳定一阶梯度 + 数值校验，再扩Hessian。

2. **风险：CCD与求解器步长耦合引发振荡**  
   规避：引入安全系数与最小步长钳制，保底 rollback。

3. **风险：摩擦历史状态导致漂移**  
   规避：接触ID稳定匹配 + 历史状态过期回收。

4. **风险：一次性追求“全缺口闭环”导致工期失控**  
   规避：坚持 A/B/C 先闭环，再并行 D/E/F/H。

---

## 7. 最终可行性判断

- **可行部分**：A/B/C + G（部分）可由外部IPC方案直接驱动实现，且与现有 NexDynIPC 架构兼容。  
- **不可直接覆盖部分**：D/E/F/H 需要在 NexDynIPC 内做额外工程化建设。  
- **总体结论**：该方案**可作为“剩余缺口收敛的主路径”**，但应明确为“分阶段完成”，而非单文档一次性补齐全部能力。
