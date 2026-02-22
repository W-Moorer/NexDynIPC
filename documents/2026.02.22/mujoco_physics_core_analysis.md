# MuJoCo 物理引擎计算核心架构深度解析

## 目录
1. [概述](#1-概述)
2. [物理引擎核心架构](#2-物理引擎核心架构)
3. [速度驱动（Velocity-Driven）机制](#3-速度驱动velocity-driven机制)
4. [Motor 执行器组件](#4-motor-执行器组件)
5. [Hinge 关节约束](#5-hinge-关节约束)
6. [总结](#6-总结)

---

## 1. 概述

MuJoCo（Multi-Joint dynamics with Contact）是一个高性能的物理引擎，专为机器人学和生物力学仿真设计。其核心特点是采用**基于约束的物理仿真**（Constraint-based Physics），而非传统的惩罚力方法（Penalty Method）。

### 核心设计理念

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        MuJoCo 物理仿真流程                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐             │
│   │  运动学   │───▶│  动力学   │───▶│  约束求解 │───▶│  积分更新 │             │
│   │ Kinematics│    │ Dynamics │    │  Solver  │    │Integrator│             │
│   └──────────┘    └──────────┘    └──────────┘    └──────────┘             │
│        │               │               │               │                   │
│        ▼               ▼               ▼               ▼                   │
│   ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐             │
│   │ mj_kinematics│  │mj_fwdVelocity│ │mj_solver │    │mj_Euler/RK4│             │
│   │  位置计算  │    │ 速度计算  │    │ 力计算   │    │ 状态更新  │             │
│   └──────────┘    └──────────┘    └──────────┘    └──────────┘             │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. 物理引擎核心架构

### 2.1 前向动力学流程 (Forward Dynamics)

**核心文件**: `src/engine/engine_forward.c`

```c
// 主前向动力学函数
void mj_forward(const mjModel* m, mjData* d) {
    mj_fwdPosition(m, d);      // 位置相关计算
    mj_fwdVelocity(m, d);      // 速度相关计算
    mj_fwdActuation(m, d);     // 执行器力计算
    mj_fwdAcceleration(m, d);  // 加速度计算
    mj_fwdConstraint(m, d);    // 约束求解
}
```

### 2.2 核心计算阶段详解

#### 阶段 1: 位置计算 (mj_fwdPosition)

```c
void mj_fwdPosition(const mjModel* m, mjData* d) {
    // 1. 运动学计算
    mj_fwdKinematics(m, d);     // 计算全局位置和姿态
    
    // 2. 惯性计算
    mj_makeM(m, d);             // 构建质量矩阵 M(q)
    mj_factorM(m, d);           // LDL 分解 M(q)
    
    // 3. 碰撞检测
    mj_collision(m, d);         // 检测几何体碰撞
    
    // 4. 约束构建
    mj_makeConstraint(m, d);    // 构建约束矩阵
    mj_projectConstraint(m, d); // 计算 efc_AR = J * M^-1 * J'
    
    // 5. 传动计算
    mj_transmission(m, d);      // 计算执行器传动长度和力矩臂
}
```

#### 阶段 2: 速度计算 (mj_fwdVelocity)

```c
void mj_fwdVelocity(const mjModel* m, mjData* d) {
    // 1. 柔性边速度
    mju_mulMatVecSparse(d->flexedge_velocity, d->flexedge_J, d->qvel, ...);
    
    // 2. 肌腱速度
    mju_mulMatVecSparse(d->ten_velocity, d->ten_J, d->qvel, ...);
    
    // 3. 执行器速度（关键：速度驱动的基础）
    if (!mjDISABLED(mjDSBL_ACTUATION)) {
        mju_mulMatVecSparse(d->actuator_velocity, d->actuator_moment, d->qvel, 
                            m->nu, d->moment_rownnz, d->moment_rowadr, 
                            d->moment_colind, NULL);
    }
    
    // 4. 质心速度
    mj_comVel(m, d);
    
    // 5. 被动力（阻尼、弹簧）
    mj_passive(m, d);
    
    // 6. 约束参考值
    mj_referenceConstraint(m, d);
    
    // 7. 偏置力（科里奥利力、离心力）
    mj_rne(m, d, 0, d->qfrc_bias);  // Recursive Newton-Euler
}
```

#### 阶段 3: 执行器力计算 (mj_fwdActuation)

这是速度驱动机制的核心实现部分，详见第 4 节。

#### 阶段 4: 约束求解 (mj_solver)

**求解器类型**:
- **PGS** (Projected Gauss-Seidel): 对偶求解器
- **CG** (Conjugate Gradient): 原始求解器
- **Newton**: 牛顿求解器

```c
void mj_solver(const mjModel* m, mjData* d) {
    // 求解约束问题：min 1/2 * f' * A * f - f' * b
    // 其中 A = J * M^-1 * J', b = J * qacc_smooth + aref
    
    switch (m->opt.solver) {
        case mjSOL_PGS:    mj_PGS(m, d); break;
        case mjSOL_CG:     mj_CG(m, d); break;
        case mjSOL_NEWTON: mj_Newton(m, d); break;
    }
}
```

---

## 3. 速度驱动（Velocity-Driven）机制

### 3.1 数学模型

在 MuJoCo 中，速度驱动通过 **PD 控制器**（比例-微分控制器）实现，其数学模型为：

```
F = kp * (q_target - q) + kv * (v_target - v)
```

其中：
- `kp`: 位置增益（刚度）
- `kv`: 速度增益（阻尼）
- `q_target`: 目标位置
- `v_target`: 目标速度
- `q`: 当前位置
- `v`: 当前速度

### 3.2 速度驱动的实现方式

MuJoCo 提供两种实现速度驱动的机制：

#### 方式 1: 使用 mjGAIN_AFFINE 增益类型

```xml
<actuator>
    <motor name="velocity_servo" joint="joint_name" 
           ctrlrange="-100 100"
           gear="1"
           gaintype="affine"
           gainprm="0 0 kv"        <!-- prm = [const, kp, kv] -->
           biastype="affine"
           biasprm="0 kp 0"/>      <!-- prm = [const, kp, kv] -->
</actuator>
```

对应的 C++ 代码逻辑：

```c
// mj_fwdActuation 中的增益计算
switch ((mjtGain) m->actuator_gaintype[i]) {
    case mjGAIN_AFFINE:
        // gain = prm[0] + prm[1]*length + prm[2]*velocity
        gain = prm[0] + prm[1]*d->actuator_length[i] + prm[2]*d->actuator_velocity[i];
        break;
}

// 偏置计算
switch ((mjtBias) m->actuator_biastype[i]) {
    case mjBIAS_AFFINE:
        // bias = prm[0] + prm[1]*length + prm[2]*velocity
        bias = prm[0] + prm[1]*d->actuator_length[i] + prm[2]*d->actuator_velocity[i];
        break;
}

// 最终执行器力
force[i] = gain * ctrl[i] + bias;
```

#### 方式 2: 使用阻尼关节 + 执行器

```xml
<joint name="hinge_joint" type="hinge" damping="0.1" axis="0 0 1"/>

<actuator>
    <motor name="drive" joint="hinge_joint" 
           ctrlrange="-1 1" 
           gear="1500"/>
</actuator>
```

### 3.3 控制流程图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        速度驱动控制流程                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   目标速度 v_target                                                          │
│        │                                                                    │
│        ▼                                                                    │
│   ┌─────────────────────────────────────┐                                  │
│   │         控制器（Control）            │                                  │
│   │  ctrl = kp*(q_target - q) + kv*(v_target - v)                         │
│   └──────────────┬──────────────────────┘                                  │
│                  │                                                          │
│                  ▼                                                          │
│   ┌─────────────────────────────────────┐                                  │
│   │      执行器动力学（Actuation）        │                                  │
│   │  force = gain * ctrl + bias           │                                  │
│   └──────────────┬──────────────────────┘                                  │
│                  │                                                          │
│                  ▼                                                          │
│   ┌─────────────────────────────────────┐                                  │
│   │      传动映射（Transmission）         │                                  │
│   │  qfrc_actuator = moment' * force      │                                  │
│   └──────────────┬──────────────────────┘                                  │
│                  │                                                          │
│                  ▼                                                          │
│   ┌─────────────────────────────────────┐                                  │
│   │      前向动力学（Forward Dynamics）   │                                  │
│   │  M(q)*qacc + C(q,qvel) = qfrc_actuator + qfrc_constraint              │
│   └──────────────┬──────────────────────┘                                  │
│                  │                                                          │
│                  ▼                                                          │
│   ┌─────────────────────────────────────┐                                  │
│   │      积分器（Integrator）             │                                  │
│   │  qvel += qacc * dt                    │                                  │
│   │  qpos += qvel * dt                    │                                  │
│   └─────────────────────────────────────┘                                  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 4. Motor 执行器组件

### 4.1 Motor 组件定义

在 XML 中定义：

```xml
<actuator>
    <motor name="drive" joint="hinge_joint" 
           ctrlrange="-1 1"      <!-- 控制信号范围 -->
           gear="1500"           <!-- 传动比/齿轮比 -->
           ctrllimited="true"/>  <!-- 是否限制控制信号 -->
</actuator>
```

### 4.2 执行器类型系统

**传动类型 (mjtTrn)**:

```c
typedef enum mjtTrn_ {
    mjTRN_JOINT         = 0,    // 关节力
    mjTRN_JOINTINPARENT,        // 关节力（父坐标系）
    mjTRN_SLIDERCRANK,          // 曲柄滑块
    mjTRN_TENDON,               // 肌腱力
    mjTRN_SITE,                 // 站点力
    mjTRN_BODY                  // 物体粘附力
} mjtTrn;
```

**动力学类型 (mjtDyn)**:

```c
typedef enum mjtDyn_ {
    mjDYN_NONE          = 0,    // 无内部动力学，ctrl 直接指定力
    mjDYN_INTEGRATOR,           // 积分器：da/dt = u
    mjDYN_FILTER,               // 线性滤波器：da/dt = (u-a) / tau
    mjDYN_FILTEREXACT,          // 精确积分滤波器
    mjDYN_MUSCLE,               // 肌肉模型
    mjDYN_USER                  // 用户自定义
} mjtDyn;
```

**增益类型 (mjtGain)**:

```c
typedef enum mjtGain_ {
    mjGAIN_FIXED        = 0,    // 固定增益
    mjGAIN_AFFINE,              // 仿射：const + kp*length + kv*velocity
    mjGAIN_MUSCLE,              // 肌肉 FLV 曲线
    mjGAIN_USER                 // 用户自定义
} mjtGain;
```

**偏置类型 (mjtBias)**:

```c
typedef enum mjtBias_ {
    mjBIAS_NONE         = 0,    // 无偏置
    mjBIAS_AFFINE,              // 仿射偏置
    mjBIAS_MUSCLE,              // 肌肉被动力
    mjBIAS_USER                 // 用户自定义
} mjtBias;
```

### 4.3 力/力矩计算方式

#### 计算流程

```c
void mj_fwdActuation(const mjModel* m, mjData* d) {
    // 步骤 1: 计算激活动力学（act_dot）
    for (int i = 0; i < nu; i++) {
        switch ((mjtDyn) m->actuator_dyntype[i]) {
            case mjDYN_NONE:
                // 无内部状态
                break;
            case mjDYN_INTEGRATOR:
                d->act_dot[act_last] = ctrl[i];
                break;
            case mjDYN_FILTER:
                tau = mju_max(mjMINVAL, prm[0]);
                d->act_dot[act_last] = (ctrl[i] - d->act[act_last]) / tau;
                break;
        }
    }
    
    // 步骤 2: 计算增益
    switch ((mjtGain) m->actuator_gaintype[i]) {
        case mjGAIN_FIXED:
            gain = prm[0];
            break;
        case mjGAIN_AFFINE:
            gain = prm[0] + prm[1]*d->actuator_length[i] + prm[2]*d->actuator_velocity[i];
            break;
    }
    
    // 步骤 3: 计算偏置
    switch ((mjtBias) m->actuator_biastype[i]) {
        case mjBIAS_NONE:
            bias = 0.0;
            break;
        case mjBIAS_AFFINE:
            bias = prm[0] + prm[1]*d->actuator_length[i] + prm[2]*d->actuator_velocity[i];
            break;
    }
    
    // 步骤 4: 计算执行器力
    // force = gain * [ctrl/act] + bias
    if (m->actuator_actadr[i] == -1) {
        force[i] = gain * ctrl[i];      // 直接力控制
    } else {
        force[i] = gain * act + bias;   // 激活状态控制
    }
    
    // 步骤 5: 映射到关节空间
    // qfrc_actuator = moment' * force
    mju_mulMatTVecSparse(d->qfrc_actuator, d->actuator_moment, force, nu, nv,
                         d->moment_rownnz, d->moment_rowadr, d->moment_colind);
}
```

### 4.4 与关节的绑定关系

#### 传动计算 (mj_transmission)

```c
void mj_transmission(const mjModel* m, mjData* d) {
    for (int i = 0; i < nu; i++) {
        int id = m->actuator_trnid[2*i];    // 传动目标 ID
        mjtNum* gear = m->actuator_gear+6*i; // 齿轮比
        
        switch ((mjtTrn) m->actuator_trntype[i]) {
            case mjTRN_JOINT:
                if (m->jnt_type[id] == mjJNT_SLIDE || m->jnt_type[id] == mjJNT_HINGE) {
                    // 标量传动
                    rownnz[i] = 1;
                    colind[adr] = m->jnt_dofadr[id];
                    
                    // 传动长度 = 关节位置 * 齿轮比
                    length[i] = d->qpos[m->jnt_qposadr[id]] * gear[0];
                    
                    // 力矩臂 = 齿轮比
                    moment[adr] = gear[0];
                }
                break;
                
            case mjTRN_TENDON:
                // 肌腱传动...
                break;
        }
    }
}
```

#### 绑定关系图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        执行器-关节绑定关系                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌──────────────┐         ┌──────────────┐         ┌──────────────┐       │
│   │   Motor      │         │ Transmission │         │    Joint     │       │
│   │  执行器       │────────▶│    传动      │────────▶│    关节      │       │
│   └──────────────┘         └──────────────┘         └──────────────┘       │
│          │                        │                        │               │
│          │                        │                        │               │
│          ▼                        ▼                        ▼               │
│   ┌──────────────┐         ┌──────────────┐         ┌──────────────┐       │
│   │ ctrl (控制)   │         │ gear (齿轮比) │         │ qpos (位置)  │       │
│   │ force (力)   │         │ length (长度)│         │ qvel (速度)  │       │
│   │ gain (增益)  │         │ moment (力矩)│         │ qacc (加速度)│       │
│   └──────────────┘         └──────────────┘         └──────────────┘       │
│                                                                             │
│   数学关系：                                                                 │
│   • length = gear * qpos                                                    │
│   • moment = gear                                                           │
│   • qfrc_actuator = moment' * force                                         │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 5. Hinge 关节约束

### 5.1 Hinge 关节定义

**关节类型枚举** (`include/mujoco/mjmodel.h`):

```c
typedef enum mjtJoint_ {
    mjJNT_FREE          = 0,    // 自由关节（7 DOF：3位置 + 4四元数）
    mjJNT_BALL,                 // 球关节（4 DOF：四元数）
    mjJNT_SLIDE,                // 滑动关节（1 DOF：沿轴平移）
    mjJNT_HINGE                 // 铰链关节（1 DOF：绕轴旋转）
} mjtJoint;
```

### 5.2 Hinge 关节的物理特性

#### XML 定义示例

```xml
<joint name="hinge_joint" type="hinge" 
       pos="0 0 0"           <!-- 关节位置 -->
       axis="0 0 1"          <!-- 旋转轴 -->
       range="-180 180"      <!-- 角度限制（度）-->
       damping="0.1"         <!-- 阻尼系数 -->
       stiffness="0"         <!-- 刚度系数 -->
       armature="0"          <!-- 电枢惯量 -->
       limited="true"/>      <!-- 是否限制范围 -->
```

#### 物理特性

| 特性 | 说明 | 数学表达 |
|------|------|----------|
| **自由度** | 1 DOF | θ ∈ ℝ |
| **位置变量** | 旋转角度 | qpos = θ (弧度) |
| **速度变量** | 角速度 | qvel = ω (弧度/秒) |
| **旋转轴** | 体坐标系中的固定轴 | axis = (x, y, z) |
| **阻尼** | 与速度成正比的阻力 | τ_damp = -damping * ω |
| **限制** | 角度范围限制 | θ_min ≤ θ ≤ θ_max |

### 5.3 数学表达

#### 运动学

```
位置更新：
    qpos(t+dt) = qpos(t) + qvel(t) * dt

旋转矩阵（从轴角表示）：
    R = I*cos(θ) + (1-cos(θ))*axis*axis' + sin(θ)*[axis]×

其中 [axis]× 是 axis 的叉积矩阵：
    [axis]× = |  0   -az   ay |
              |  az   0   -ax |
              | -ay   ax   0  |
```

#### 动力学

```
铰链关节的动力学方程：

    I * α + C(θ, ω) = τ_actuator + τ_constraint + τ_passive

其中：
    I: 关节惯量（通过 CRB 算法计算）
    α: 角加速度 (qacc)
    C: 科里奥利力和离心力项
    τ_actuator: 执行器力矩
    τ_constraint: 约束力矩
    τ_passive: 被动力矩（阻尼、弹簧）
```

### 5.4 MuJoCo 中的实现细节

#### 正向运动学实现

**文件**: `src/engine/engine_core_smooth.c`

```c
void mj_kinematics1(const mjModel* m, mjData* d) {
    for (int j = 0; j < jntnum; j++) {
        int jid = jntadr + j;
        int qadr = m->jnt_qposadr[jid];
        mjtJoint jtype = m->jnt_type[jid];
        
        // 计算全局坐标系中的轴
        mji_rotVecQuat(xaxis, m->jnt_axis+3*jid, xquat);
        
        // 计算全局坐标系中的锚点
        mji_rotVecQuat(xanchor, m->jnt_pos+3*jid, xquat);
        mji_addTo3(xanchor, xpos);
        
        // 应用关节变换
        switch (jtype) {
            case mjJNT_SLIDE:
                // 滑动：沿轴平移
                mji_addToScl3(xpos, xaxis, d->qpos[qadr] - m->qpos0[qadr]);
                break;
                
            case mjJNT_BALL:
            case mjJNT_HINGE:
                // 球关节/铰链：旋转
                mjtNum qloc[4];
                if (jtype == mjJNT_BALL) {
                    // 球关节：使用四元数
                    mji_copy4(qloc, d->qpos+qadr);
                    mju_normalize4(qloc);
                } else {
                    // 铰链关节：轴角表示转四元数
                    // qloc = [axis*sin(θ/2), cos(θ/2)]
                    mji_axisAngle2Quat(qloc, m->jnt_axis+3*jid, 
                                        d->qpos[qadr] - m->qpos0[qadr]);
                }
                
                // 应用旋转
                mju_mulQuat(xquat, xquat, qloc);
                
                // 修正偏心旋转
                mjtNum vec[3];
                mji_rotVecQuat(vec, m->jnt_pos+3*jid, xquat);
                mji_sub3(xpos, xanchor, vec);
                break;
        }
    }
}
```

#### 约束处理

**文件**: `src/engine/engine_core_constraint.c`

```c
// 关节限制约束
void mj_makeConstraint(const mjModel* m, mjData* d) {
    // 为每个超出限制的关节添加约束
    for (int i = 0; i < m->njnt; i++) {
        if (m->jnt_limited[i]) {
            mjtNum pos = d->qpos[m->jnt_qposadr[i]];
            mjtNum range[2] = {m->jnt_range[2*i], m->jnt_range[2*i+1]};
            
            // 检查是否超出限制
            if (pos < range[0] || pos > range[1]) {
                // 添加等式约束
                // J * qacc = aref
            }
        }
    }
}
```

### 5.5 Hinge 与机械旋转副（Revolute Joint）的对应关系

**是的，MuJoCo 中的 Hinge 关节完全对应机械系统中的旋转副（Revolute Joint）。**

| 特性 | MuJoCo Hinge | 机械旋转副 (Revolute Joint) |
|------|--------------|----------------------------|
| **自由度** | 1 DOF | 1 DOF |
| **运动类型** | 绕固定轴旋转 | 绕固定轴旋转 |
| **约束** | 5 个约束（3位置 + 2方向） | 5 个约束 |
| **变量** | 角度 θ | 角度 θ |
| **限制** | 可选角度范围 | 可选机械限位 |
| **摩擦** | 阻尼系数 | 库仑摩擦/粘滞摩擦 |

#### 约束雅可比矩阵

对于铰链关节，其约束雅可比矩阵将关节空间速度映射到笛卡尔空间：

```
对于连接父体 p 和子体 c 的铰链关节：

    v_c = v_p + ω_p × r + ω_joint × r
    
    ω_c = ω_p + ω_joint * axis

其中：
    r: 从父体到子体的位置向量
    ω_joint: 关节角速度
    axis: 关节旋转轴（全局坐标系）

雅可比矩阵 J 满足：
    |v_c|   |  I   [r]×  0  |   |v_p|     |  0  |
    |   | = |               | * |   | + J *| ω_joint |
    |ω_c|   |  0     I    0  |   |ω_p|     |  1  |
```

---

## 6. 总结

### 6.1 核心架构要点

1. **基于约束的物理**: MuJoCo 使用约束求解器而非惩罚力方法处理接触和关节限制
2. **广义坐标**: 使用关节坐标（qpos, qvel）而非笛卡尔坐标，提高效率
3. **递归算法**: 使用 RNE（Recursive Newton-Euler）和 CRB（Composite Rigid Body）算法
4. **稀疏矩阵**: 利用关节树结构的稀疏性优化计算

### 6.2 速度驱动要点

1. **PD 控制**: 通过 `mjGAIN_AFFINE` 和 `mjBIAS_AFFINE` 实现 PD 控制器
2. **传动映射**: 通过 `gear` 参数将执行器力映射到关节力矩
3. **阻尼**: 可通过关节 `damping` 属性或执行器 `kv` 参数实现

### 6.3 Motor 组件要点

1. **灵活配置**: 支持多种传动类型、动力学类型、增益类型
2. **力矩计算**: `force = gain * ctrl + bias`
3. **关节映射**: `qfrc_actuator = moment' * force`

### 6.4 Hinge 关节要点

1. **1 DOF 旋转**: 绕固定轴的单自由度旋转
2. **对应旋转副**: 完全等价于机械系统中的 Revolute Joint
3. **约束处理**: 通过约束求解器处理关节限制

---

## 参考文件清单

| 文件路径 | 主要功能 |
|---------|---------|
| `include/mujoco/mjmodel.h` | 数据结构和枚举定义 |
| `src/engine/engine_forward.c` | 前向动力学主流程 |
| `src/engine/engine_core_smooth.c` | 运动学、惯性、RNE 计算 |
| `src/engine/engine_core_constraint.c` | 约束构建 |
| `src/engine/engine_solver.c` | 约束求解器 |
| `src/engine/engine_passive.c` | 被动力计算 |
