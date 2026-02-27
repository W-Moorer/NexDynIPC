# RecurDyn RMD 模型数据文件详解与参数规范

## 一、RMD 文件概述

.rmd 文件基于**数据块（Data Blocks）**的结构构建。每个数据块通过关键字（如 `PART`, `MARKER`, `CSURFACE`, `SOLIDCONTACT` 等）开头，内部属性由逗号 `,` 分隔。注释行以叹号 `!` 开头。

在多体动力学中，解耦"物理属性（质量/惯量）"和"几何属性（网格/表面）"是标准做法，RMD 文件完美体现了这一架构。

---

## 二、核心数据块（Data Blocks）参数解析

### 1. 刚体部件 (PART)

PART 块定义了物理世界中的刚体。它承载了物体的质量特性，但不直接包含任何几何形状。

```plaintext
PART / 3
, NAME = 'Body1'
, TYPEENTITY = GENERAL
, MASS = 15616.6385083649
, CM = 4
, IP = 6299.2003, 13146.8749, 9034.9829, -4.68e-13, -5.22e-13, -7.78e-14
, QG = 0 , 0 , 0
, REULER = 0 , 0 , 0
```

**参数说明：**

| 参数 | 说明 |
|------|------|
| `NAME` | 部件在软件中的名称 |
| `TYPEENTITY` | 实体类型（通常为 `GENERAL` 刚体或 `GROUND` 地面） |
| `MASS` | 刚体质量 |
| `CM` | Center of Mass，指定作为该刚体质心坐标系的 MARKER ID（非常关键） |
| `IP` | 质心坐标系下的惯性张量向量，包含 6 个分量：$[I_{xx}, I_{yy}, I_{zz}, I_{xy}, I_{xz}, I_{yz}]$ |
| `QG` / `REULER` | 刚体本身的绝对初始位移和欧拉角（通常设为0，实际位姿由其 CM 的 MARKER 决定） |

---

### 2. 局部坐标系标记点 (MARKER)

MARKER 依附于 PART 之上，是定义位置、连接关节（Joint）和受力点的参考基准。

```plaintext
MARKER / 4
, NAME = 'Body1.CM'
, PART = 3
, QP = 0 , -0.662182624072882 , 0
, REULER = 0 , 0 , 0
```

**参数说明：**

| 参数 | 说明 |
|------|------|
| `PART` | 该 Marker 所依附的母体部件 ID |
| `QP` | 该 Marker 在全局坐标系（Global Coordinate System）下的初始平移位置向量 $(x, y, z)$ |
| `REULER` | 该 Marker 在全局坐标系下的初始姿态（旋转），通常以欧拉角序列（如 3-1-3 或 3-2-1）表示 $(\theta_x, \theta_y, \theta_z)$ |

---

### 3. 接触几何表面 (CSURFACE / GGEOM)

该数据块负责将有限元网格（Nodes）或几何面片组装成可用于接触检测的外壳。

```plaintext
CSURFACE / 2
, NAME = '##SOLID##_Body1.Subtract1'
, RM = 6
, MAX = 3.028 , 2.028 , 3.028
, TYPE = PATCH
, NO_PATCH = 26518 , NO_LINE = 39777 , NO_NODE = 13261
, PATCH = 
, 1, 2, 3, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1
...
```

**参数说明：**

| 参数 | 说明 |
|------|------|
| `RM` | Reference Marker，几何参考标记点 ID。整个表面所有网格节点的坐标都是相对于此 RM 描述的 |
| `MAX` / `BBLENGTH` / `RLENGTH` | 用于碰撞检测宽相（Broad-phase）空间加速结构（如包围盒 Bounding Box）的几何尺寸参数 |
| `NO_NODE` / `NO_PATCH` | 节点总数和网格面片（三角形或四边形）总数 |
| `PATCH` | 详细的网格拓扑矩阵与节点坐标/法向数组（这部分数据量极大） |

---

### 4. 实体接触力定义 (SOLIDCONTACT / CONTACT_GEO_SURFACE)

用于定义两个几何表面之间的法向罚函数接触（Penalty Method）和切向摩擦力。

```plaintext
SOLIDCONTACT / 3
, NAME = 'SolidContact3'
, IFLOAT = 27 , JFLOAT = 28
, ICSURFACEID = 6 , JCSURFACEID = 10
, K = 100000000 , KORDER = 2 , C = 10000 
, S_T_V = 1.e-005 , D_T_V = 1.e-004 
, S_F_C = 0. , D_F_C = 0.
```

**参数说明：**

| 参数 | 说明 |
|------|------|
| `IFLOAT` / `JFLOAT` | 接触的 Action Marker（浮动从动系）和 Base Marker（基准系） |
| `ICSURFACEID` / `JCSURFACEID` | 参与接触的一对几何表面 ID（对应上文的 CSURFACE ID） |
| `K` (Stiffness) | 接触罚函数刚度系数（阻力随穿透深度增加的倍率） |
| `KORDER` | 接触力指数（线性罚函数为1，Hertzian 接触通常为 1.5，此处为 2） |
| `C` (Damping) | 接触阻尼系数，用于耗散碰撞产生的高频能量 |

**摩擦力参数：**

| 参数 | 说明 |
|------|------|
| `S_F_C` / `D_F_C` | 静摩擦系数 $\mu_s$ / 动摩擦系数 $\mu_k$ |
| `S_T_V` / `D_T_V` | 摩擦过渡速度阈值（用于平滑库仑摩擦曲线，避免迭代矩阵奇异性） |

---

### 5. 求解器控制设置

存在于文件的结尾部分，用于指导底层数值积分器（Integrator）。

| 参数块 | 说明 |
|--------|------|
| `INTPAR` | 时间步进控制（如 `ADVHYBRID`, `INITIAL_H` 初始时间步, `HMAX` 最大时间步, `ERROR` 容差） |
| `EQUILIBRIUM` | 静力学平衡计算的迭代上限和误差容限 |
| `SOLVEROPTION` | 包括 `MAXIMUMCONVERGENCECOUNT`（牛顿法最大迭代步数）和冗余约束处理方式 |

---

## 三、深层物理架构：节点(Nodes)、RM与CM的耦合映射关系

在多体动力学仿真中，将"算力的物理属性"与"复杂的几何外形"桥接起来的核心纽带就是 MARKER。在 RMD 文件中，刚体（PART）、质心（CM）、几何参考（RM）和网格节点（Nodes）存在极其严密的层级从属关系：

### 1. 物理计算层（CM 为主导）

- **中心地位**：求解器（Newton-Euler 方程组）只关心质心（CM）
- **方程建立**：系统的动能、外力做功和惯性力（包含 IP 惯性张量）都是在 CM 所对应的 MARKER 局部坐标系下进行积分求解的
- **更新机制**：每一时间步求解完毕后，求解器会输出 CM 的全局新位置（QP）和新姿态（REULER）

### 2. 几何映射层（RM 与 Nodes 为随动）

- **RM的本质**：RM（Reference Marker）与 CM 一样，都是刚体 PART 上的局部坐标系。由于刚体不发生形变，RM 与 CM 之间的相对位置和旋转在整个仿真过程中是绝对固定不变的
- **节点的局部性**：包含成千上万个顶点的网格表面（CSURFACE 的 PATCH 数据），其坐标仅是在 RM 局部坐标系下的常数矩阵，不需要在每步中重新计算其相对值
