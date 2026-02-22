# MuJoCo 物理引擎详细解析

## 文档信息
- **项目路径**: `e:\workspace\NexDynIPC\temps\mujoco-main`
- **分析日期**: 2026-02-21
- **对比项目**: NexDynIPC

---

## 一、项目概述

### 1.1 MuJoCo 简介

MuJoCo（Multi-Joint dynamics with Contact）是一个高性能的物理引擎，专为机器人学、生物力学、图形和机器学习研究设计。2021年被 DeepMind 收购并开源。

**核心特性**:
- 通用刚体动力学仿真
- 软接触模型（基于约束求解）
- 丰富的执行器和传感器模型
- 可微分仿真（支持自动微分）
- 多种编程语言绑定（C/C++/Python）

### 1.2 目录结构

```
mujoco-main/
├── include/mujoco/          # 公共API头文件
│   ├── mujoco.h             # 主API头文件
│   ├── mjmodel.h            # 模型数据结构
│   ├── mjdata.h             # 运行时数据结构
│   └── mjplugin.h           # 插件系统接口
│
├── src/                     # 源代码
│   ├── engine/              # 核心物理引擎
│   │   ├── engine_forward.c # 正向动力学
│   │   ├── engine_solver.c  # 约束求解器
│   │   ├── engine_collision_*.c # 碰撞检测
│   │   └── engine_passive.c # 被动弹簧/阻尼
│   ├── user/                # 用户模型编译
│   ├── xml/                 # XML解析器 (MJCF/URDF)
│   └── render/              # OpenGL渲染
│
├── model/                   # 示例模型
│   ├── humanoid/            # 人形机器人
│   ├── flex/                # 柔性体示例
│   └── gripper.xml          # 夹爪模型
│
├── plugin/                  # 官方插件
│   ├── actuator/            # 执行器插件 (PID)
│   ├── sensor/              # 传感器插件
│   ├── sdf/                 # 有符号距离场
│   └── elasticity/          # 弹性体
│
├── python/                  # Python绑定
├── simulate/                # 交互式模拟器
└── mjx/                     # JAX/GPU加速版本
```

---

## 二、核心架构

### 2.1 架构层次图

```
┌─────────────────────────────────────────────────────────────┐
│                    应用层 (Application)                      │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │ simulate │  │  python  │  │   mjx    │  │  sample  │   │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘   │
├─────────────────────────────────────────────────────────────┤
│                    API层 (C API)                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │                    mujoco.h                          │  │
│  │  mj_step(), mj_forward(), mj_inverse(), ...          │  │
│  └──────────────────────────────────────────────────────┘  │
├─────────────────────────────────────────────────────────────┤
│                  物理引擎核心 (Engine)                       │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │ Forward  │  │  Solver  │  │ Collision│  │  Sensor  │   │
│  │  正向    │  │  求解器  │  │  碰撞    │  │  传感器  │   │
│  │Dynamics  │  │          │  │Detection │  │          │   │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘   │
├─────────────────────────────────────────────────────────────┤
│                  数据结构层 (Data Structures)                │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                  │
│  │ mjModel  │  │  mjData   │  │  mjSpec  │                  │
│  │ 静态模型 │  │ 动态状态  │  │ 模型规格 │                  │
│  └──────────┘  └──────────┘  └──────────┘                  │
├─────────────────────────────────────────────────────────────┤
│                    插件系统 (Plugins)                        │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │ Actuator │  │  Sensor  │  │   SDF    │  │ Passive  │   │
│  │ 执行器   │  │  传感器  │  │ 距离场   │  │  被动    │   │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 核心数据结构

#### mjModel - 静态模型

```c
struct mjModel_ {
  // 尺寸信息
  int nq;           // 广义坐标数
  int nv;           // 自由度数量
  int nu;           // 执行器数量
  int na;           // 激活状态数
  int nbody;        // 刚体数量
  int njnt;         // 关节数量
  int ngeom;        // 几何体数量
  
  // 物理选项
  mjOption opt;     // 物理选项
  
  // 刚体数据
  int* body_parentid;   // 父体ID
  mjtNum* body_pos;     // 相对位置
  mjtNum* body_quat;    // 相对朝向
  mjtNum* body_mass;    // 质量
  mjtNum* body_inertia; // 惯性
  
  // 关节数据
  int* jnt_type;        // 关节类型
  mjtNum* jnt_range;    // 关节范围
  
  // 执行器数据
  int* actuator_trntype;  // 传动类型
  int* actuator_dyntype;  // 动力学类型
  mjtNum* actuator_ctrlrange; // 控制范围
  
  // ... 更多数据
};
```

#### mjData - 运行时状态

```c
struct mjData_ {
  // 状态变量
  mjtNum* qpos;         // 位置 (nq x 1)
  mjtNum* qvel;         // 速度 (nv x 1)
  mjtNum* qacc;         // 加速度 (nv x 1)
  mjtNum* act;          // 激活状态 (na x 1)
  mjtNum* ctrl;         // 控制输入 (nu x 1)
  
  // 动力学
  mjtNum* qfrc_applied;    // 施加的广义力
  mjtNum* xfrc_applied;    // 施加的笛卡尔力
  mjtNum* qfrc_actuator;   // 执行器力
  mjtNum* qfrc_passive;    // 被动力
  mjtNum* qfrc_constraint; // 约束力
  
  // 接触
  mjContact* contact;   // 接触数组
  int ncon;             // 接触数量
  
  // 约束
  int nefc;             // 约束数量
  mjtNum* efc_J;        // 约束雅可比
  mjtNum* efc_force;    // 约束力
};
```

---

## 三、核心功能模块

### 3.1 正向动力学 (engine_forward.c)

**计算流程**:

```
mj_forward()
├── mj_fwdPosition()           # 位置相关计算
│   ├── mj_kinematics()        # 运动学计算
│   ├── mj_comPos()            # 质心位置
│   ├── mj_makeM()             # 惯性矩阵
│   ├── mj_factorM()           # 惯性矩阵分解
│   └── mj_collision()         # 碰撞检测
├── mj_fwdVelocity()           # 速度相关计算
│   ├── mj_comVel()            # 质心速度
│   ├── mj_rne()               # 递归牛顿-欧拉
│   └── mj_passive()           # 被动弹簧/阻尼
├── mj_fwdActuation()          # 执行器力计算
├── mj_fwdAcceleration()       # 加速度计算
└── mj_fwdConstraint()         # 约束求解
```

### 3.2 碰撞检测系统

**几何体类型**:
```c
typedef enum mjtGeom_ {
  mjGEOM_PLANE = 0,     // 平面
  mjGEOM_HFIELD,        // 高度场
  mjGEOM_SPHERE,        // 球体
  mjGEOM_CAPSULE,       // 胶囊体
  mjGEOM_ELLIPSOID,     // 椭球体
  mjGEOM_CYLINDER,      // 圆柱体
  mjGEOM_BOX,           // 盒子
  mjGEOM_MESH,          // 网格
  mjGEOM_SDF,           // 有符号距离场
  mjNGEOMTYPES
} mjtGeom;
```

**碰撞检测流程**:
```c
// 宽阶段 (Broad Phase)
int mj_broadphase(const mjModel* m, mjData* d, int* bfpair, int maxpair);

// 窄阶段 (Narrow Phase) - GJK/EPA
mjtNum mjc_ccd(const mjCCDConfig* config, mjCCDStatus* status, 
               mjCCDObj* obj1, mjCCDObj* obj2);
```

### 3.3 约束求解器

**求解器类型**:
```c
typedef enum mjtSolver_ {
  mjSOL_PGS = 0,    // PGS (Projected Gauss-Seidel)
  mjSOL_CG,         // CG (Conjugate Gradient)
  mjSOL_NEWTON      // Newton
} mjtSolver;
```

**约束类型**:
```c
typedef enum mjtConstraint_ {
  mjCNSTR_EQUALITY = 0,        // 等式约束
  mjCNSTR_FRICTION_DOF,        // 自由度摩擦
  mjCNSTR_FRICTION_TENDON,     // 肌腱摩擦
  mjCNSTR_LIMIT_JOINT,         // 关节限制
  mjCNSTR_LIMIT_TENDON,        // 肌腱限制
  mjCNSTR_CONTACT_FRICTIONLESS,// 无摩擦接触
  mjCNSTR_CONTACT_PYRAMIDAL,   // 金字塔摩擦锥接触
  mjCNSTR_CONTACT_ELLIPTIC     // 椭圆摩擦锥接触
} mjtConstraint;
```

---

## 四、关节与执行器系统

### 4.1 关节类型

```c
typedef enum mjtJoint_ {
  mjJNT_FREE = 0,   // 自由关节 (7 DOF: 3 pos + 4 quat)
  mjJNT_BALL,       // 球关节 (4 DOF: quaternion)
  mjJNT_SLIDE,      // 滑动关节 (1 DOF)
  mjJNT_HINGE       // 旋转关节 (1 DOF)
} mjtJoint;
```

### 4.2 执行器系统

**传动类型**:
```c
typedef enum mjtTrn_ {
  mjTRN_JOINT = 0,       // 关节力
  mjTRN_JOINTINPARENT,   // 父坐标系关节力
  mjTRN_SLIDERCRANK,     // 滑块-曲柄
  mjTRN_TENDON,          // 肌腱力
  mjTRN_SITE,            // 位置力
  mjTRN_BODY             // 吸附力
} mjtTrn;
```

**执行器类型**:

1. **Motor** - 直接力矩控制
   ```xml
   <motor name="joint1" gear="100" joint="joint1"/>
   ```

2. **Position** - PD位置控制
   ```xml
   <position name="pos1" joint="joint1" kp="100" kv="10"/>
   ```

3. **Velocity** - PI速度控制
   ```xml
   <velocity name="vel1" joint="joint1" kv="100"/>
   ```

4. **Muscle** - 肌肉模型
   ```xml
   <muscle name="muscle1" tendon="tendon1" .../>
   ```

---

## 五、夹爪模型分析

### 5.1 夹爪XML模型 (model/flex/gripper.xml)

```xml
<mujoco model="Gripper">
  <worldbody>
    <body name="hand" pos="0 0 .37">
      <joint name="lift" type="slide" range="0 1"/>
      <geom type="box" size=".2 .1 .05"/>
      
      <!-- 右夹爪 -->
      <body name="right_gripper">
        <joint name="right" type="slide" axis="-1 0 0"/>
        <geom type="box" size=".02 .1 .15" pos=".18 0 -.2"/>
        <!-- 柔性夹爪垫 -->
        <flexcomp type="mesh" file="cap.obj" dim="3" .../>
      </body>
      
      <!-- 左夹爪 -->
      <body name="left_gripper">
        <joint name="left" type="slide" axis="1 0 0"/>
        <geom type="box" size=".02 .1 .15" pos="-.18 0 -.2"/>
        <flexcomp type="mesh" file="cap.obj" dim="3" .../>
      </body>
    </body>
  </worldbody>

  <!-- 肌腱连接两侧夹爪 -->
  <tendon>
    <fixed name="grasp">
      <joint joint="right" coef="1"/>
      <joint joint="left" coef="1"/>
    </fixed>
  </tendon>

  <!-- 位置控制执行器 -->
  <actuator>
    <position name="lift" joint="lift" kp="600" dampratio="1"/>
    <position name="grasp" tendon="grasp" kp="200" dampratio="1"/>
  </actuator>
</mujoco>
```

### 5.2 关键设计要点

| 组件 | 功能 | NexDynIPC对应实现 |
|------|------|------------------|
| `slide` 关节 | 夹爪开合 | PrismaticJoint |
| `tendon` | 同步两侧夹爪 | 需要新增 Tendon 类 |
| `flexcomp` | 柔性夹爪垫 | 结合PolyFEM柔性体 |
| `position` 执行器 | PD位置控制 | OpenLoopController + PD |

---

## 六、与 NexDynIPC 对比

### 6.1 相似之处

| 特性 | MuJoCo | NexDynIPC |
|------|--------|-----------|
| 物理基础 | 刚体动力学 + 约束求解 | 刚体动力学 + 约束求解 |
| 碰撞检测 | GJK/EPA + BVH | GJK/EPA + BVH |
| 时间积分 | Euler/RK4/隐式 | 隐式 Euler/Newmark |
| 关节类型 | 自由/球/滑动/旋转 | 固定/旋转/滑动/球/圆柱 |
| 摩擦模型 | 金字塔/椭圆摩擦锥 | 摩擦锥模型 |

### 6.2 关键差异

| 方面 | MuJoCo | NexDynIPC |
|------|--------|-----------|
| **接触处理** | 约束求解器 (PGS/CG/Newton) | IPC 屏障势 + 牛顿求解 |
| **数值方法** | 稀疏线性代数 | 自动微分 + 牛顿法 |
| **可微性** | 有限差分 | 自动微分 (核心特性) |
| **柔性体** | 原生支持 (Flex) | 刚体为主 |
| **传感器** | 丰富内置传感器 | 基础传感器 |
| **执行器** | 多种预定义类型 | 开环控制 + 外部力 |
| **架构** | C API + 插件系统 | C++ 现代设计 |

### 6.3 代码结构对比

**MuJoCo 数据流**:
```
XML/URDF -> mjModel (编译时) -> mjData (运行时) -> 物理步骤
```

**NexDynIPC 数据流**:
```
JSON -> World -> RigidBody/Joint -> IPCSolver -> 时间积分
```

---

## 七、控制系统对比

### 7.1 MuJoCo 控制架构

**控制回调机制**:
```c
// 控制回调函数
MJAPI extern mjfGeneric mjcb_control;

void my_controller(const mjModel* m, mjData* d) {
  // 读取传感器数据
  // 计算控制律
  // 设置 d->ctrl
}

// 注册回调
mjcb_control = my_controller;
```

### 7.2 NexDynIPC 控制架构

**OpenLoopController**:
```cpp
class OpenLoopController {
public:
    void preSolveUpdate(double t, double dt);   // 预求解更新
    void postSolveUpdate();                      // 后求解更新
    void applyTrajectories(double t);            // 应用轨迹
private:
    std::vector<std::shared_ptr<KinematicBody>> kinematic_bodies_;
};
```

### 7.3 控制方式对比

| 特性 | MuJoCo | NexDynIPC |
|------|--------|-----------|
| 控制架构 | 回调函数 + 执行器模型 | 运动学刚体 + 轨迹 |
| 力控制 | 内置多种执行器类型 | ExternalForceForm |
| 位置控制 | PD 伺服 | 目标位姿约束 |
| 闭环控制 | 传感器 + 回调 | 需要扩展 |
| 延迟处理 | 执行器动力学 | 需要扩展 |

---

## 八、NexDynIPC 可借鉴的设计

### 8.1 立即可以借鉴的

1. **肌腱系统 (Tendon)**
   - 实现夹爪同步运动
   - 连接多个关节实现耦合

2. **传感器框架**
   - 触觉传感器 (Touch)
   - 力传感器 (Force)
   - 力矩传感器 (Torque)

3. **执行器模型**
   - PD 控制器参数化
   - 力矩限制和饱和

### 8.2 需要适配的

1. **柔性体 (Flex)**
   - MuJoCo: 原生支持
   - NexDynIPC: 结合 PolyFEM

2. **岛屿分解 (Island)**
   - MuJoCo: 多刚体系统并行求解
   - NexDynIPC: 需要新增

3. **接触过滤**
   - MuJoCo: 父子体碰撞过滤
   - NexDynIPC: 需要新增

---

## 九、机械手夹取实现建议

### 9.1 基于 MuJoCo 设计的 NexDynIPC 实现

```cpp
// 1. 夹爪关节 - 使用 PrismaticJoint
auto left_joint = std::make_shared<PrismaticJoint>(
    hand, left_finger, Vector3d::UnitX(), hand_pos
);
auto right_joint = std::make_shared<PrismaticJoint>(
    hand, right_finger, -Vector3d::UnitX(), hand_pos
);

// 2. 肌腱约束 - 新增 Tendon 类（借鉴 MuJoCo）
auto tendon = std::make_shared<Tendon>();
tendon->addJoint(left_joint, 1.0);
tendon->addJoint(right_joint, 1.0);  // 同步运动
world->addTendon(tendon);

// 3. 位置控制执行器
auto actuator = std::make_shared<PositionActuator>(tendon);
actuator->setKp(200.0);
actuator->setKd(20.0);
actuator->setTargetPosition(0.1);  // 开合宽度

// 4. 触觉传感器
auto touch_sensor = std::make_shared<TouchSensor>(left_finger);
```

### 9.2 夹取控制流程

```
1. 检测物体位置
   └─> 视觉传感器 / 碰撞检测

2. 规划夹取轨迹
   └─> OpenLoopController + KeyframeTrajectory

3. 执行夹取动作
   ├─> 移动夹爪到物体两侧
   ├─> 闭合夹爪（位置控制）
   └─> 检测夹取力（触觉传感器）

4. 提起物体
   └─> 力控制（补偿重力）

5. 放置物体
   └─> 缓慢张开夹爪
```

---

## 十、总结

### 10.1 MuJoCo 的优势

1. **成熟稳定**: 多年开发，广泛应用于机器人研究
2. **性能优化**: 稀疏矩阵、多线程、岛屿分解
3. **功能丰富**: 传感器、执行器、柔性体、肌腱等
4. **生态完善**: Python 绑定、可视化工具、MJCF 格式
5. **可扩展**: 插件系统支持自定义功能

### 10.2 NexDynIPC 的优势

1. **可微分仿真**: 自动微分支持，适合机器学习
2. **现代C++**: 面向对象设计，易于扩展
3. **IPC接触**: 连续碰撞检测，无穿透
4. **自动微分**: 完整的数学函数支持

### 10.3 建议的融合方向

| MuJoCo 组件 | NexDynIPC 实现优先级 |
|------------|---------------------|
| 肌腱系统 | ⭐⭐⭐ 高 |
| 传感器框架 | ⭐⭐⭐ 高 |
| 执行器模型 | ⭐⭐⭐ 高 |
| 柔性体 | ⭐⭐ 中 |
| 岛屿分解 | ⭐⭐ 中 |
| 碰撞过滤 | ⭐ 低 |

---

## 参考资料

- MuJoCo 官方文档: https://mujoco.readthedocs.io/
- MuJoCo GitHub: https://github.com/google-deepmind/mujoco
- MuJoCo 论文: "MuJoCo: A physics engine for model-based control" (Todorov et al., 2012)
