# 以 Stark 为蓝本的约束/关节工程化迁移规划（NexDynIPC）

日期：2026-02-27

## 1) 学习结论（来自 `temps/stark-main`）

本次重点学习了以下实现与测试组织方式：
- 约束能力分层：
  - 原语约束：point / direction / distance / distance_limits / angle_limits
  - 软约束：damped_spring
  - 驱动约束：linear_velocity / angular_velocity（带饱和与 delay）
- 关节通过“原语组合”形成：
  - hinge = point + direction
  - slider = point_on_axis + direction
  - prismatic_slider = slider + direction(额外轴向锁定)
  - motor = hinge + angular_velocity
- 求解稳定性工程：
  - 使用容差检查（m、deg）评估约束违背
  - 违背超阈值时自动提高约束刚度（hardening）
  - 时间步接受后做软增强（preemptive hardening）
- 测试方法：
  - 每类约束都有可解释的“违背量 + 力/力矩”检查
  - 用固定场景验证极限行为（限位、饱和、弹簧阻尼）

对照的核心参考文件：
- `stark/src/models/rigidbodies/RigidBodyConstraints.h`
- `stark/src/models/rigidbodies/EnergyRigidBodyConstraints.h`
- `stark/src/models/rigidbodies/EnergyRigidBodyConstraints.cpp`
- `stark/src/models/rigidbodies/RigidBodies.h`
- `stark/src/models/rigidbodies/RigidBodies.cpp`
- `tests/rb_constraints.cpp`

---

## 2) NexDynIPC 当前短板（约束与关节工程能力）

结合现状代码，主要短板是：
1. 关节“可组合能力”不足：当前以单类 Joint 实现为主，缺少“原语约束组合层”。
2. 缺少关节限位能力：角限位、行程限位尚未形成统一机制。
3. 缺少驱动器工程接口：线速度/角速度驱动缺乏统一、可饱和、可延迟参数化。
4. 缺少软约束闭环：弹簧+阻尼与约束系统耦合不完整。
5. 缺少基于容差的自适应刚度与可观测性：目前调参依赖人工经验。

---

## 3) 迁移目标设计（按“先可用、再工程化”）

### G1. 约束原语统一层（最先落地）
目标：把几何约束统一为可复用原语，而不是分散在关节类里。
- `PointConstraint`
- `DirectionConstraint`
- `PointOnAxisConstraint`
- `DistanceConstraint`
- `DistanceLimitConstraint`
- `AngleLimitConstraint`

验收：同一求解框架下可独立启用/禁用，每个原语可输出违背量与等效力（或力矩）。

### G2. 组合关节层（工程接口）
目标：关节由原语组合产生，形成一致 API。
- `HingeJoint = Point + Direction`
- `SliderJoint = PointOnAxis + Direction`
- `PrismaticJoint = Slider + Direction(roll lock)`
- `HingeWithAngleLimit = Hinge + AngleLimit`

验收：组合关节可在 JSON 中声明并映射到内部约束图。

### G3. 驱动与软约束（性能可控）
目标：引入驱动器和软约束，具备可调稳定性。
- `LinearVelocityDrive(target_v, max_force, delay)`
- `AngularVelocityDrive(target_w, max_torque, delay)`
- `DampedSpring(rest, k, c)`

验收：出现饱和时输出受限于 max_force/max_torque，且无明显数值爆振。

### G4. 容差驱动的自适应刚度（鲁棒性）
目标：复制 Stark 的 hardening 思路，减少“手工调大刚度”的成本。
- 每步统计约束违背（m/deg）
- 超阈值自动倍率提升 stiffness
- 时间步接受后轻量 preemptive hardening

验收：在极限算例中约束误差收敛，并减少失败步。

---

## 4) 建议迁移实施阶段

### M0（1~2周）：结构重构
- 抽象 `ConstraintPrimitive` 基类（value/gradient/hessian/violation）
- Joint 迁移为“组合器”而非独立重复求导逻辑
- JSON 增加 `constraints` 节点（先兼容旧 `joints`）

### M1（1~2周）：功能补齐
- AngleLimit / DistanceLimit 原语
- VelocityDrive（线速度+角速度，含饱和和 delay）
- DampedSpring 原语

### M2（1周）：鲁棒性
- 约束容差统计与日志
- 自动刚度 hardening
- 失败步重试策略（可选）

### M3（持续）：回归体系
- 以 JSON 算例驱动回归
- 输出误差/力/力矩 KPI 到 csv

---

## 5) 本次创建的 JSON 验证算例（assets/{模型名称}）

已创建以下迁移验收算例（可作为后续回归套件）：

1. `assets/hinge_angle_limit_pendulum/hinge_angle_limit_pendulum.json`
   - 基线：fixed + revolute
   - 未来验收：角限位（min/max angle）
   - 指标：角超调、约束误差

2. `assets/prismatic_slider_press/prismatic_slider_press.json`
   - 基线：fixed + prismatic
   - 未来验收：行程限位 + 线速度驱动
   - 指标：侧向漂移、限位超调

3. `assets/motor_torque_saturation/motor_torque_saturation.json`
   - 基线：fixed + revolute
   - 未来验收：角速度驱动 C1 过渡与力矩饱和
   - 指标：最大力矩、稳态速度误差

4. `assets/spherical_attachment_compliance/spherical_attachment_compliance.json`
   - 基线：fixed + spherical
   - 未来验收：弹簧阻尼柔顺约束
   - 指标：附着误差、收敛时间

说明：
- 这些 JSON 兼容当前结构（`settings/bodies/joints`），并附加 `validation_targets` 作为未来功能验收说明。
- 当前 `SceneLoader` 未消费 `validation_targets`，该字段用于迁移期间测试脚本读取与自动验收。

---

## 6) 验证策略（建议）

1. 先运行基线场景，确认现有求解稳定。
2. 引入新约束原语后，对应场景逐个开启未来目标字段的功能映射。
3. 每个场景产出统一 KPI：
   - 几何违背：m / deg
   - 控制误差：m/s / rad/s
   - 饱和约束：N / Nm
4. 与阈值比较，纳入回归门禁。

---

## 7) 一句话总结

迁移关键不是“多写几个关节类”，而是引入 **约束原语化 + 组合关节 + 驱动/限位统一 + 容差驱动自适应刚度** 的工程框架；本次已完成对标设计与可执行 JSON 验证套件准备。

---

## 8) ALM 约束规范（v1，理论定义）

本规范用于 NexDynIPC 在保留 ALM 的前提下，统一约束建模、残差统计、容差判定与参数更新。

### 8.1 约束类型

按求解属性分为四类：

1. 等式几何约束（Equality）
    - 目标：`C_e(x) = 0`
    - 示例：Point、Direction、PointOnAxis、Attachment

2. 不等式限位约束（Inequality）
    - 目标：`g(x) <= 0`
    - 示例：AngleLimit、DistanceLimit、StrokeLimit

3. 驱动约束（Drive）
    - 目标：速度/角速度跟踪
    - 示例：LinearVelocityDrive、AngularVelocityDrive（含饱和）

4. 软约束（Soft）
    - 目标：弹簧阻尼等柔顺行为
    - 示例：DampedSpring

### 8.2 统一能量与残差定义

#### A) 等式约束

- 约束向量：`C_e(x)`
- ALM 能量：

   `E_e(x, lambda_e, mu_e) = lambda_e^T C_e(x) + 0.5 * mu_e * ||C_e(x)||^2`

- 原始残差：

   `r_p_e = ||C_e(x)||_inf`

- 对偶残差（离散形式）：

   `r_d_e = ||lambda_e^{k+1} - lambda_e^k||_inf`

#### B) 不等式约束

- 约束函数：`g(x) <= 0`
- 正部函数：`g^+(x) = max(g(x), 0)`
- ALM（带投影）等价实现可采用活跃集近似：

   `E_i(x, lambda_i, mu_i) = lambda_i^T g^+(x) + 0.5 * mu_i * ||g^+(x)||^2`

- 原始残差：

   `r_p_i = ||g^+(x)||_inf`

- 对偶残差：

   `r_d_i = ||lambda_i^{k+1} - lambda_i^k||_inf`

#### C) 驱动约束

- 线速度：`C_v = (v_rel · a) - v_target`
- 角速度：`C_w = (w_rel · a) - w_target`
- 可按等式约束进入 ALM，或采用 C1 饱和势（推荐与饱和参数一致化）
- 残差：`r_p_v = |C_v|`，`r_p_w = |C_w|`

#### D) 软约束

- 通常不进入乘子更新，仅作为势能项：
   - 弹簧：`E_s = 0.5*k*(l-l0)^2`
   - 阻尼：`E_d = 0.5*c*(dl/dt)^2`
- 监控量：位移误差、速度误差、能量衰减趋势

### 8.3 容差单位与推荐口径

为避免混用，约束容差按物理量固定单位：

- 位置/距离约束：m
- 方向/角度约束：deg（内部可转 rad）
- 线速度约束：mps
- 角速度约束：radps

推荐默认阈值（可按场景覆盖）：

- `eps_pos = 0.002 m`
- `eps_ang = 1.0 deg`
- `eps_v = 0.05 mps`
- `eps_w = 0.2 radps`
- `eps_dual = 1e-3`（乘子更新无量纲监控阈值）

### 8.4 μ 更新策略（按约束组）

对每类约束组 `G` 单独维护 `mu_G`，采用分段倍率更新：

- 若 `r_p_G > eta_high * eps_G`，则 `mu_G <- min(mu_G * gamma_up, mu_max)`
- 若 `r_p_G < eta_low * eps_G` 且 `r_d_G` 也小，则 `mu_G <- max(mu_G / gamma_down, mu_min)`（可选）
- 否则保持不变

推荐参数：

- `eta_high = 2.0`
- `eta_low = 0.3`
- `gamma_up = 2.0`
- `gamma_down = 1.2`
- `mu_min = 1e3`
- `mu_max = 1e10`

说明：
- 几何硬约束（point/direction/limit）可设更高初始 `mu`。
- 驱动约束不宜过大 `mu`，避免与时间积分项刚性冲突。

### 8.5 乘子更新与投影规则

#### 等式约束

- `lambda_e <- lambda_e + mu_e * C_e(x)`

#### 不等式约束

- 采用投影更新：

   `lambda_i <- max(0, lambda_i + mu_i * g(x))`

- 仅当 `g(x) > 0` 或 `lambda_i > 0` 时视为活跃。

#### 外层 ALM 停止准则（建议）

当以下全部满足时停止外循环：

- `r_p_e <= eps_e`
- `r_p_i <= eps_i`
- `r_p_v <= eps_v`（若有驱动）
- `r_p_w <= eps_w`（若有驱动）
- `max(r_d_e, r_d_i) <= eps_dual`

若不满足且达到外层上限，触发：
- 记录未收敛约束组
- 按 8.4 做 `mu` 提升
- 进入下一时间步前保留乘子（warm start）

---

## 9) 四个 JSON 算例到 ALM 规范的映射

### 9.1 算例-约束组映射表

| 算例 | 主约束组 | 次约束组 | 目标容差单位 | 备注 |
|---|---|---|---|---|
| `hinge_angle_limit_pendulum` | Point + Direction（铰链） | AngleLimit（未来） | m / deg | 验证角限位激活与反力 |
| `prismatic_slider_press` | PointOnAxis + Direction（移动副） | DistanceLimit + LinearVelocityDrive（未来） | m / mps | 验证行程限位与线驱动饱和 |
| `motor_torque_saturation` | Point + Direction（铰链） | AngularVelocityDrive（未来） | deg / radps | 验证角速度跟踪与力矩饱和 |
| `spherical_attachment_compliance` | Point（球铰位置） | DampedSpring（未来） | m / mps | 验证柔顺约束稳定收敛 |

### 9.2 每个算例的 ALM 配置建议（理论）

| 算例 | `mu_eq_init` | `mu_ineq_init` | `mu_drive_init` | `mu_soft` | 推荐外层迭代上限 |
|---|---:|---:|---:|---:|---:|
| `hinge_angle_limit_pendulum` | 1e6 | 5e5 | - | 0 | 10 |
| `prismatic_slider_press` | 1e6 | 5e5 | 1e4 | 0 | 12 |
| `motor_torque_saturation` | 1e6 | - | 5e3 | 0 | 12 |
| `spherical_attachment_compliance` | 5e5 | - | - | `k,c` 势能参数 | 8 |

说明：
- `mu_soft` 表示软约束通常不做乘子更新，仅通过势能参数 `k,c` 控制。
- 上表是起始建议，实际以 8.4 的自适应更新为主。

---

## 10) 每类约束的验收指标表（统一门禁）

### 10.1 指标定义

- `V_pos`: 位置/距离约束最大违背（m）
- `V_ang`: 角度约束最大违背（deg）
- `V_v`: 线速度跟踪误差稳态均值（mps）
- `V_w`: 角速度跟踪误差稳态均值（radps）
- `F_sat`: 力饱和超限比例（%）
- `T_sat`: 力矩饱和超限比例（%）
- `R_dual`: 对偶残差峰值（乘子增量无量纲）

### 10.2 约束类别验收门限

| 约束类别 | 主验收指标 | 门限建议 | 次验收指标 | 门限建议 |
|---|---|---|---|---|
| Point / PointOnAxis / Distance | `V_pos` | <= 0.002 m | `R_dual` | <= 1e-3 |
| Direction / AngleLimit | `V_ang` | <= 1.0 deg | `R_dual` | <= 1e-3 |
| LinearVelocityDrive | `V_v` | <= 0.05 mps | `F_sat` | <= 1% 超限 |
| AngularVelocityDrive | `V_w` | <= 0.2 radps | `T_sat` | <= 1% 超限 |
| DampedSpring | 位移衰减时间 | <= 目标设定 | 能量单调衰减段占比 | >= 95% |

### 10.3 四算例验收矩阵（执行口径）

| 算例 | 必过指标 | 建议阈值 |
|---|---|---|
| `hinge_angle_limit_pendulum` | `V_pos`, `V_ang`, `R_dual` | `V_pos<=0.005 m`, `V_ang<=1.0 deg`, `R_dual<=1e-3` |
| `prismatic_slider_press` | `V_pos`, `V_v`, `F_sat` | `V_pos<=0.003 m`, `V_v<=0.05 mps`, `F_sat<=1%` |
| `motor_torque_saturation` | `V_w`, `T_sat` | `V_w<=0.5 radps`, `T_sat<=1%` |
| `spherical_attachment_compliance` | `V_pos`, 衰减时间 | `V_pos<=0.004 m`, `settling_time<=1.2 s` |

---

## 11) 落地备注（仅理论更新）

1. 本文新增内容为理论规范，不涉及代码修改。
2. 推荐先在日志层实现上述指标统计，再做约束接口改造。
3. 当 `constraints` 新 JSON 节点落地后，可直接复用本规范作为 CI 门禁配置来源。

---

## 12) Joint 与约束共存的一体化统一规划

你当前项目已经有 `Joint`（`computeC/computeJ + ALM`）体系，因此不应“替换式重写”，而应采用**同核求解、双层表达、分阶段收敛**策略：

### 12.1 统一目标

统一后系统应满足：

1. 对求解器而言，所有约束都映射为同一抽象：
   - `C(x)=0`（等式）或 `g(x)<=0`（不等式）
   - 支持 ALM 的 `value/gradient/hessian/violation`
2. 对业务接口而言，同时支持：
   - 旧接口：`joints[]`
   - 新接口：`constraints[]`（原语与驱动）
3. 对迁移过程而言，老场景零破坏，新功能可渐进启用。

### 12.2 共存架构（理论）

建议将“表达层”和“求解层”彻底解耦：

- 表达层（Modeling）
  - `LegacyJointAdapter`：把现有 `Joint` 适配为统一约束项。
  - `ConstraintPrimitive`：Point/Direction/Limit/Drive/Spring 等新原语。
  - `CompositeJoint`：由多个原语组合（hinge/slider/motor）。

- 聚合层（Assembly）
  - `ConstraintSet`：收集所有来源约束（legacy + primitive + composite）。
  - 约束分组：`eq / ineq / drive / soft`，用于独立统计残差与 μ。

- 求解层（ALM）
  - 单一外层循环：统一处理乘子更新、投影、μ 自适应。
  - 单一牛顿内层：只关心当前总能量与导数。

核心原则：
- **Joint 不是被抛弃，而是成为一种约束来源（source）**。
- 所有 source 在 ALM 层视同仁，避免两套数学规则并存。

### 12.3 Joint 与约束避免冲突规则

为防止重复约束或过约束，定义以下规则：

1. 同一物理关系只能有一个“主约束源”。
   - 若 `hinge` 已由 `revolute joint` 提供，不再重复添加 `point+direction`。
2. 允许“主约束 + 附加约束”。
   - 例如：`revolute` + `angle_limit` + `angular_velocity_drive`。
3. 检测冲突组合并告警（理论要求）：
   - 完全重复 DOF 锁定
   - 互相矛盾的目标（如同轴上不同固定角目标）

### 12.4 JSON 兼容与合并策略

采用“兼容旧格式 + 新格式并存”的解析策略：

1. 继续支持：`joints[]`
2. 新增支持：`constraints[]`
3. 解析顺序建议：
   - 先加载 `joints[]`（生成 legacy constraints）
   - 再加载 `constraints[]`（补充限位/驱动/软约束）
4. 去重与冲突检查：
   - 依据 `(bodyA, bodyB, type, anchor/axis/signature)` 生成约束签名
   - 重复签名默认拒绝或降级为 warning（按配置）

### 12.5 统一 ALM 执行链（含 legacy Joint）

每个时间步的理论执行顺序：

1. 组装 `ConstraintSet`（legacy + primitive + composite）。
2. 按组计算残差：`r_p_eq, r_p_ineq, r_p_drive, r_d`。
3. 构建总目标：
   - 惯性 + 外力 + 接触 + `E_eq + E_ineq + E_drive + E_soft`。
4. 牛顿内层求解。
5. 统一乘子更新：
   - 等式：`lambda <- lambda + mu*C`
   - 不等式：`lambda <- max(0, lambda + mu*g)`
6. 统一 μ 自适应（按组），并记录 KPI。
7. 若满足停止准则，接受步；否则继续外层迭代。

### 12.6 分阶段迁移（合并版）

#### 阶段 A：共存接入（不改变旧行为）
- 目标：旧 `Joint` 路径输出结果不变。
- 动作：仅新增适配层定义与指标口径，不切断旧链路。

#### 阶段 B：增量启用新约束
- 目标：在旧 joint 基础上叠加 limit/drive/soft。
- 动作：优先用于四个 JSON 验证算例，观测 `V_pos/V_ang/V_v/V_w/R_dual`。

#### 阶段 C：组合关节替代旧关节（可选）
- 目标：新场景优先使用 `constraints[] + composite`。
- 动作：旧 `joints[]` 保留为兼容入口，逐步退为 legacy。

### 12.7 统一验收标准（共存期）

共存期必须额外满足两条“系统级一致性验收”：

1. 回归一致性：
   - 同一场景仅使用 `joints[]` 与使用“等价 `constraints[]`”时，主轨迹偏差不超过容差阈值。
2. 稳定性不退化：
   - 外层 ALM 迭代次数分布不显著恶化；失败步占比不高于基线。

### 12.8 对当前问题的直接回答

“当前项目中的 joint 怎么和这些约束共存？”

答案：
- 将 `joint` 视为**约束来源之一**，通过适配层并入统一 `ConstraintSet`；
- 新增的限位/驱动/软约束通过 `constraints[]` 注入同一 ALM 链路；
- 用冲突检测与签名去重保证不重复约束；
- 用统一残差、统一 μ 策略、统一乘子规则保证数学一致性。

这就是“全部内容合并在一起、统一规划处理”的可执行理论方案。