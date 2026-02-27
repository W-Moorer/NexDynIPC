# RMD 解析认知与 JSON 映射说明

日期：2026-02-27  
参考实现：`temps/pytools/rmd_to_scene.py`、`temps/pytools/tune_complex_contact.py`、`temps/pytools/README.md`

## 1. 我对 RMD 文件结构的认知（基于当前样本与实现）

RMD（在本项目上下文中）可视为一种“分段实体 + 行内键值 + 逗号续行块”的文本格式，核心特点：

1. **实体起始行**：形如 `ENTITY / id`（例如 `PART / 2`、`MARKER / 101`），用于切换解析上下文。  
2. **键值行**：形如 `KEY = value`（例如 `MASS = 1.2`、`REULER = ...`）。  
3. **续行数值块**：以逗号开头（`,`），常用于 patch/node 或惯量积等数组数据。  
4. **注释处理**：`!` 之后视为注释并剔除。  
5. **数值兼容**：科学计数法支持 `D/d` 指数（会转为 `E/e`），并对 `1.23E` 自动补 `0`。

从解析策略看，`rmd_to_scene.py` 采用的是**状态机式单遍扫描**：
- 通过 `_is_entity_start()` 识别实体切换；
- 在不同实体上下文中按字段名提取数据；
- 对 `CSURFACE/GGEOM` 的 `PATCH/NODE` 子块使用独立读入状态；
- 对 `JOINT` 使用 `flush_joint_if_needed()` 在实体切换点提交结构体。

这意味着：当前解析器并不是“泛化 RMD 解释器”，而是“面向本项目仿真导入目标的定向语义提取器”。

## 2. 当前已覆盖的实体与语义

### 2.1 刚体与位姿
- `PART`：`NAME/MASS/IP/QG/REULER`，并支持 `IP` 后续行读取惯量积 `ip_products`。
- `MARKER`：`NAME/PART/QP/REULER`。
- `REULER` 单位支持 `auto|deg|rad`：
  - `auto` 通过幅值阈值判断弧度或角度；
  - 内部统一转为度，供后续几何旋转与关节轴计算使用。

### 2.2 关节与驱动
- `JOINT`：识别 `Fixed` 与 `Revolute`；提取 `I/J` marker 关联。
- `MOTION`：提取 `JOINT/FUNCTION` 及 `rotation/velocity` 标志。
- `EXPRESSION`：当 `MOTION.FUNCTION` 是表达式 ID 时，回查 `EXPRESSION/FUNCTION`。
- 仅当同时满足 `rotation + velocity + 可解析常数函数` 时，映射为 motor 约束；否则退化为 hinge。

### 2.3 接触
- `SOLIDCONTACT/GGEOMCONTACT`：
  - 对象关联：`ICSURFACEID/JCSURFACEID` 或 `IGGEOMID/JGGEOMID`
  - 参数：`BPEN`、`K`、`S_F_C`
- 语义映射：
  - `BPEN -> default_contact_thickness`
  - `K -> min_contact_stiffness`
  - `S_F_C -> pair.friction`

### 2.4 仿真设置
- `accgrav` 段：`IGRAV/JGRAV/KGRAV -> gravity`
- `INTPAR/HMAX -> settings.simulation.max_time_step_size`
- `SOLVEROPTION/NUM_THREAD -> settings.execution.n_threads`
- 终止时间：由用户指定，默认为1.0，也可 CLI `--end-time` 强制指定。
- 时间步长默认为1e-3。

### 2.5 网格与几何
- `CSURFACE/GGEOM`：读取 `NAME/RM/NO_PATCH/NO_NODE/PATCHTYPE/PATCH/NODE`。
- 支持将 patch 三角片转 OBJ，并构建法线：
  - 优先使用面法线或 `EACHNODENORMAL`；
  - 自动剔除退化三角形；
  - 使用 `RM -> world -> CM` 变换导出到刚体局部坐标，避免重复施加位姿。

### 2.6 OBJ 导出是转换链路中的必需产物（补充）
- 在当前工程实践中，RMD 转换不只是输出场景 JSON/XML，还必须包含**对应 OBJ 网格导出**，用于仿真模型几何加载。
- `tune_complex_contact.py` 的流程侧面验证了这一点：其调用转换脚本时显式传入 `--obj-output-dir`，并在后续 XML patch 中处理 mesh 文件路径（可选绝对路径）。
- 因此对“RMD 解析完成”的验收应包含：
  1. scene 文件可解析；
  2. OBJ 文件已生成且路径可被 scene/XML 正确引用；
  3. 几何坐标系与刚体位姿一致（不会发生重复变换导致的错位）。

## 3. RMD -> 可解析 JSON 的映射认知

我认为“可解析 JSON”应满足两层目标：

1. **语义完备到可执行**：足以驱动当前 NexDyn 场景加载与仿真。  
2. **结构稳定可校验**：字段有固定位置、类型明确、缺省行为可预期。

当前实现输出 JSON 已基本满足目标 1；若希望更工程化，建议明确目标 2 的契约。

另外需要强调：**scene JSON 可解析不代表转换闭环完成**。在本项目里，闭环定义应为“scene + OBJ + 路径引用一致性”三者同时成立。

### 3.1 当前场景 JSON 核心结构（已实现）

- `version`
- `metadata`: `name`, `source_rmd`
- `settings`
  - `output`: 输出目录、fps 等
  - `simulation`: `gravity`, `init_frictional_contact`, `max_time_step_size?`
  - `execution`: `end_simulation_time`, `n_threads?`
- `objects.rigidbodies[]`
  - `id/label/mass/inertia_diagonal`
  - `geometry.type=file`, `geometry.path`
  - `transform.translation/rotation`
  - 可选 `inertia_tensor`
- `boundary_conditions[]`
  - `fix|hinge|hinge_to_world|motor|motor_to_world`
- `interactions.contact`
  - `global_params.default_contact_thickness?`
  - `global_params.min_contact_stiffness?`
  - `pairs[]`（含 `friction?`）

### 3.2 我建议补充的“解析中间 JSON（IR）”层

为便于后续扩展、调试和测试，建议把流程拆成：
`RMD -> IR(JSON) -> Scene(JSON)`。

IR（中间 JSON）建议至少包含：
- `entities.part[]/marker[]/joint[]/motion[]/expression[]/surface[]/contact[]`
- `raw_refs`（保留原始 ID 关系，例如 marker-part、joint marker I/J）
- `parse_warnings[]`（例如无法识别的 function、缺失 marker）
- `units`（记录 `reuler_source_unit` 与已归一化策略）
- `provenance`（源文件路径、解析时间、脚本版本）

这样可以把“RMD 语法解析正确性”与“场景语义映射正确性”解耦。

## 4. 当前认知边界与风险点

1. **JOINT 类型覆盖有限**：当前只显式支持 `Fixed/Revolute`。  
2. **MOTION 函数能力有限**：仅稳定支持常数函数或可回查的常数 expression；复杂时变函数未完整表达。  
3. **实体容错依赖样本分布**：若 RMD 写法大幅偏离当前模式（例如非典型换行/注释/字段顺序），可能漏读。  
4. **接触参数策略是“首值/局部映射”**：`BPEN/K` 作为全局参数时采用首次命中，需确认多接触对时的业务语义。  
5. **坐标链假设**：几何导出依赖 RM/CM 命名与关联规则，异常命名会影响几何对齐。

## 5. 我认为“认知足够可进入实现”的验收标准

若你要确认我对 RMD 解析认知是否足够，我建议按以下清单验收：

- 能解释实体级语法（起始行、键值行、续行块、注释）并给出对应解析状态机。  
- 能逐项说明当前脚本中“RMD 字段 -> scene JSON 字段”的映射关系。  
- 能明确指出当前**已支持范围**与**未支持范围**，而不是笼统声称“可全部解析”。  
- 能提出可落地的 IR 设计，支撑后续扩展和测试。  
- 能给出风险点与回归验证要点（特别是关节、驱动、接触、坐标变换）。
- 能把 OBJ 导出与路径引用（相对/绝对）纳入统一验收，而不是只检查 JSON 字段存在。

基于以上，我判断：**当前认知足够进入下一阶段（制定 RMD->IR JSON 规范与实现计划）**。

---

## 6. 下一步建议（若你同意我即可继续）

1. 定义 `rmd_ir.schema.json`（先最小可用版本）。  
2. 在 `rmd_to_scene.py` 中增加 `--emit-ir-json` 输出。  
3. 用 2~3 个现有场景做 IR 与 scene 的双产物回归对比，固化解析契约。
