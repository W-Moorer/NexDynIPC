# 刚体接触仿真平台设计文档 (1/3) - 理论调研与核心算法

## 1. 核心理论调研

### 1.1 IPC (Incremental Potential Contact) 算法原理

IPC 是一种基于优化的接触处理框架，将接触约束转化为光滑的屏障势能函数，从而通过无约束或带约束的优化问题来求解下一时刻的状态。

### 1.5 模块化势能框架 (参考 PolyFEM)

为了提高系统的扩展性，建议采用 **基于形式 (Form-based)** 的架构。即总能量 $E$ 是各项势能的加权和：
$$
E(\mathbf{x}) = \sum_i w_i E_i(\mathbf{x})
$$
每一项 $E_i$ (如惯性势能、接触势能、约束势能) 都需独立实现以下接口：
- `value(x)`: 计算势能值
- `gradient(x)`: 计算一阶导 (力/残差)
- `hessian(x)`: 计算二阶导 (刚度矩阵)
- `max_step(x, dx)`: 计算基于该势能的最大允许步长 (用于 CFL 或 IPC 步长限制)

这种设计允许将刚体动力学、IPC 接触、关节约束解耦，便于独立测试和组合。
$$
B(d, \hat{d}) = \begin{cases} 
-(d - \hat{d})^2 \ln\left(\frac{d}{\hat{d}}\right) & 0 < d < \hat{d} \\
0 & d \ge \hat{d}
\end{cases}
$$
其中 $\hat{d}$ 是接触激活阈值。该函数在 $d=\hat{d}$ 处二阶连续可导 ($C^2$)，这对于基于牛顿法的求解器至关重要。

**梯度与Hessian:**
- $B'(d) = \dots$ (推导用于力计算)
- $B''(d) = \dots$ (推导用于刚度矩阵计算)

#### 1.1.2 接触势能
总接触势能 $E_{contact}$ 是所有接触对势能的加权和：
$$
E_{contact}(\mathbf{x}) = \sum_{k \in \mathcal{C}} \kappa B(d_k(\mathbf{x}), \hat{d})
$$
其中 $\kappa$ 是屏障刚度，$d_k(\mathbf{x})$ 是第 $k$ 个接触对的最小距离函数。

### 1.2 SDF (符号距离场) 与 IPC 的集成

在传统IPC中，距离通常基于点-三角形（PT）或边-边（EE）。而在本项目中，利用 SDF 也可以高效计算距离。

#### 1.2.1 距离计算
假设刚体 A 为 SDF 几何体 (SDF函数为 $\Phi_A(\mathbf{x})$)，刚体 B 为采样点集 $\mathcal{P}_B$。
对于 B 上的一点 $\mathbf{p}_i$，其到 A 的距离近似为：
$$
d_i = \Phi_A(\mathbf{p}_i^{world}) - r_{margin}
$$
其中 $\Phi_A$ 返回的是代数距离（内部为负，外部为正）。如果考虑两刚体表面接触，需减去碰撞余量或将点投影到表面。

#### 1.2.2 接触点的梯度与 Jacobian
需要计算距离 $d_i$ 对刚体状态 $\mathbf{q}$ (位置 $\mathbf{x}$, 旋转 $\mathbf{\theta}$) 的导数。
$$
\frac{\partial d_i}{\partial \mathbf{q}_A} = \nabla \Phi_A(\mathbf{p}_i) \cdot \frac{\partial \mathbf{p}_i^{local}}{\partial \mathbf{q}_A}
$$
这需要 SDF 的梯度 $\nabla \Phi$ 和刚体运动学的 Jacobian。

### 1.3 隐式积分器与刚体动力学

采用隐式欧拉 (Backward Euler) 方法以保证大时间步长下的稳定性。

#### 1.3.3 滞后摩擦模型 (Lagged Friction)
参考 PolyFEM 的 `FrictionForm`，摩擦力通常作为滞后项处理。在第 $k$ 次牛顿迭代中，摩擦力的大小取决于上一次迭代 $k-1$ 的法向力 $\lambda_n^{k-1}$ 和相对切向速度。
由于摩擦势能是非光滑的 (C0)，通常使用最大耗散原理 (Maximum Dissipation Principle) 的平滑近似形式。
势能形式：
$$
D(\mathbf{x}) = \mu \lambda_n^{k-1} f(\|\mathbf{v}_T\|)
$$
其中 $f(\cdot)$ 是平滑后的切向位移范数函数。

#### 1.3.2 刚体状态更新 (李群积分)
对于旋转，使用指数映射或四元数更新：
$$
\mathbf{q}_{rot, n+1} = \mathbf{q}_{rot, n} \otimes \exp(\boldsymbol{\omega}_{n+1} h)
$$
在优化过程中，通常对增量 $\delta \mathbf{x}, \delta \boldsymbol{\theta}$ 进行求解。

### 1.4 约束系统建模

约束通过拉格朗日乘子 (Lagrange Multipliers) 或 罚函数 (Penalty/Soft Constraints) 集成。为了统一框架，建议初期使用软约束势能项 $E_{joint}$，后期引入硬约束求解器。

#### 1.4.1 通用约束方程
$$
\mathbf{C}(\mathbf{q}) = \mathbf{0}
$$
对应的势能项：
$$
E_{joint} = \frac{1}{2} k_{joint} \|\mathbf{C}(\mathbf{q})\|^2
$$

#### 1.4.2 常见关节
- **球铰 (Spherical)**: $\mathbf{x}_A + \mathbf{R}_A \mathbf{r}_A - (\mathbf{x}_B + \mathbf{R}_B \mathbf{r}_B) = \mathbf{0}$
- **旋转副 (Revolute)**: 限制两个正交方向的相对旋转。

---

## 2. 核心算法流程伪代码

### 2.1 仿真主循环 (Simulation Step)

```cpp
function StepSimulation(world, dt):
    // 1. 预测阶段 (Prediction)
    for body in world.bodies:
        body.x_pred = body.x + dt * body.v
        body.q_pred = integrate_rotation(body.q, body.w, dt)
    
    // 2. 宽相碰撞检测 (Broad Phase)
    collision_pairs = BroadPhase(world.bodies)
    
    // 3. 构建接触约束 (Narrow Phase)
    active_contacts = []
    for pair in collision_pairs:
        // 使用SDF查询距离
        contacts = ComputeSDFContacts(pair.body_a, pair.body_b)
        active_contacts.extend(contacts)
        
    // 4. 牛顿迭代求解 (Newton-Raphson)
    x_new = [body.x, body.q] // 初始猜测
    for iter in 0..MAX_ITER:
        // 计算梯度 (力)
        gradient = ComputeGradient(x_new, world, active_contacts, dt)
        
        // 计算 Hessian (系统刚度矩阵)
        hessian = ComputeHessian(x_new, world, active_contacts, dt)
        
        // 求解线性系统 H * delta = -g
        delta = LinearSolve(hessian, -gradient)
        
        // 线搜索 (Line Search) 保证能量下降且无穿透(CCD)
        alpha = LineSearch(x_new, delta, world)
        
        // 更新状态
        x_new += alpha * delta
        
        if converged(gradient, delta):
            break
            
    // 5. 更新速度与最终状态
    for i, body in enumerate(world.bodies):
        body.v = (x_new[i].x - body.x) / dt
        body.w = compute_angular_velocity(body.q, x_new[i].q, dt)
        body.x = x_new[i].x
        body.q = x_new[i].q
```

### 2.2 SDF 接触查询

```cpp
function ComputeSDFContacts(body_a(SDF), body_b(Mesh/Points)):
    contacts = []
    // 将B的点转换到A的局部坐标系
    T_world_to_a = inverse(body_a.transform)
    
    for point_world in body_b.sample_points:
        point_local = T_world_to_a * point_world
        
        // 查询SDF值与梯度
        distance = body_a.sdf.value(point_local)
        normal_local = body_a.sdf.gradient(point_local)
        
        if distance < HAT_DISTANCE:
            contact = ContactPoint()
            contact.distance = distance
            contact.normal = body_a.transform.rotate(normal_local)
            contact.point_on_a = point_world - contact.normal * distance
            contact.point_on_b = point_world
            contacts.append(contact)
            
    return contacts
```

### 2.3 约束 Jacobian (以球铰为例)

```cpp
function SphericalJointJacobian(body_a, body_b, anchor_a, anchor_b):
    // C = (xA + RA*rA) - (xB + RB*rB)
    // dC/dxA = I
    // dC/dthetaA = -skew(RA*rA)
    // dC/dxB = -I
    // dC/dthetaB = skew(RB*rB)
    
    r_a_world = body_a.rotation * anchor_a
    r_b_world = body_b.rotation * anchor_b
    
    J_a_linear = Identity(3)
    J_a_angular = -SkewSymmetric(r_a_world)
    
    J_b_linear = -Identity(3)
    J_b_angular = SkewSymmetric(r_b_world)
    
    return [J_a_linear, J_a_angular, J_b_linear, J_b_angular]
```
