# NexDynIPC 铰链关节 (Hinge Joint) 设计文档
**日期：** 2026-02-10

## 1. 概述
本设计文档旨在规划 **铰链关节 (Hinge Joint)** 的实现逻辑。铰链关节是一种常见的运动学约束，允许两个刚体绕通过连接点的特定轴进行相对旋转，同时限制其他 5 个自由度（3 个平移 + 2 个旋转）。

在 NexDynIPC 基于优化的隐式积分框架中，我们将关节约束建模为 **势能项 (Energy Potential / Soft Constraint)**。

## 2. 数学表述 (Mathematical Formulation)

为了实现 **硬约束 (Hard Constraint)** 的效果，同时保持求解器的数值稳定性，我们采用 **增广拉格朗日法 (Augmented Lagrangian Method, ALM)**。相比单纯的惩罚法，ALM 通过引入拉格朗日乘子 $\lambda$ 来消除稳态误差，而不需要无限大的刚度系数。

### 2.1 约束定义 $C(x)$

我们要满足约束 $C(x) = 0$。对于铰链关节：

1.  **位置约束** ($C_{pos} \in \mathbb{R}^3$)：
    $$ C_{pos}(x) = (p_A + R(q_A) r_A) - (p_B + R(q_B) r_B) = 0 $$
2.  **轴对齐约束** ($C_{align} \in \mathbb{R}^3$)：
    $$ C_{align}(x) = (R(q_A) n_A) \times (R(q_B) n_B) = 0 $$

### 2.2 增广拉格朗日目标函数

ALM 结合了拉格朗日项和罚函数项。对于每个约束 $C_i(x)$，势能项定义为：
$$ E_i(x) = \lambda_i^T C_i(x) + \frac{1}{2} \mu_i \| C_i(x) \|^2 $$

其中：
- $\lambda_i$：拉格朗日乘子 (Lagrange Multiplier)，向量维度同 $C_i$。
- $\mu_i$：罚参数 (Penalty Parameter) / 刚度。

总目标函数 $\mathcal{L}(x)$ 为惯性势能、重力势能与关节势能之和。

### 2.3 求解策略 (ALM Loop)
在一个时间步 $t \to t+1$ 内，我们执行 ALM 迭代：

1.  **初始化**：
    - 继承上一时间步的 $\lambda$（热启动），或者重置为 0。
    - 设定初始罚参数 $\mu$。
2.  **内层循环 (Newton Solve)**：
    - 固定 $\lambda$，求解 $\min_x \mathcal{L}(x, \lambda, \mu)$。
    - 使用现有的 `NewtonSolver`。
3.  **外层更新 (Multiplier Update)**：
    - 求解完成后，更新乘子：$\lambda \leftarrow \lambda + \mu C(x)$。
    - 检查约束违反度 $\|C(x)\|_\infty$。如果满足阈值 (Tolerance)，则退出。
    - (可选) 如果收敛太慢，增大 $\mu$（如 $\mu \leftarrow \min(\rho \mu, \mu_{max})$）。

### 2.5 为什么 ALM 也需要刚度 ($\mu$)?

虽然 ALM 旨在解决单纯罚函数法需要无限大刚度的问题，但它依然包含罚项 $\frac{1}{2} \mu \|C(x)\|^2$。这有两方面原因：

1.  **凸化 (Convexification)**：拉格朗日乘子法本身寻找的是鞍点 (Saddle Point)，Hessian 矩阵是不定的 (Indefinite)，这对牛顿法不友好。引入足够大的 $\mu$ 可以使 Hessian 在约束流形切空间附近变得正定 (Positive Definite)，从而保证牛顿法的收敛性。
2.  **收敛速度**：罚项为优化器提供了明确的梯度方向指向约束面。$\mu$ 越大，单次外层迭代中约束满足得越好，但也可能导致内层牛顿法条件数变差。ALM 的优势在于，即使 $\mu$ 保持在一个有限的、数值稳定的范围内，通过 $\lambda$ 的迭代更新，约束误差最终也能收敛到机器精度，而不需要 $\mu \to \infty$。

**总结**：在 ALM 中，$\mu$ 不需要“无限大”，只需要“足够大”以保证局部凸性即可。通常我们从一个适中的值开始（如 $10^3$），并在必要时动态调整。

### 2.6 $\lambda$ 与 $\mu$ 的物理意义

-   **$\lambda$ (拉格朗日乘子)**：它代表了维持约束所需的 **真实约束力 (Constraint Force)**。
    -   例如，对于铰链，$\lambda$ 对应于防止两个物体分离的 **拉力** 和防止其相对倾斜的 **反扭矩**。
    -   当 ALM 收敛时 ($C(x) \to 0$)，所有的约束力都由 $\lambda$ 提供，罚项 $\frac{1}{2} \mu \|C\|^2$ 的贡献为 0。

-   **$\mu$ (罚参数/刚度)**：它是一个 **数值稳定剂 (Numerical Stabilizer)**。
    -   物理上，你可以把它想象成在关节处附加的一个 **辅助弹簧**。
    -   它的作用是“拉住”解不要跑太远，让牛顿法能在一个凸坑里找到正确的 $\lambda$。
    -   与纯罚函数法不同，这个弹簧不需要无限硬，因为它不是为了消除误差，而是为了辅助求解器工作。最终的误差消除由 $\lambda$ 完成。

---

## 3. 代码架构设计

### 3.1 类结构更新

`Joint` 类不仅需要存储刚度，还需要存储当前的 $\lambda$。

```cpp
// src/Dynamics/Joints/Joint.hpp
class Joint {
public:
    virtual ~Joint() = default;
    
    // 约束维度 (如 Hinge 为 3+3=6)
    virtual int dim() const = 0; 

    // 计算约束向量 C(x)
    virtual void computeC(const Eigen::VectorXd& x, Eigen::VectorXd& C) const = 0;
    
    // 计算约束雅可比 J = dC/dx
    virtual void computeJ(const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& J) const = 0;

    // ALM 相关接口
    // value = lambda^T * C + 0.5 * mu * C^T * C
    virtual double value(const Eigen::VectorXd& x) const;
    virtual void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const;
    virtual void hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) const;
    
    // 更新乘子: lambda += mu * C
    void updateLambda(const Eigen::VectorXd& x);
    
    // 重置乘子 (必要时)
    void resetLambda();

protected:
    Eigen::VectorXd lambda_; // 拉格朗日乘子
    double mu_;              // 罚参数 (刚度)
};
```

### 3.2 求解器流程更新 (`ImplicitEuler.cpp`)

需要在 `step` 函数中增加 ALM 外层循环：

```cpp
void ImplicitEuler::step(World& world, double dt) {
    // ... 预测 x_hat ...
    
    const int max_alm_iters = 10;
    const double tol = 1e-4;

    for (int iter = 0; iter < max_alm_iters; ++iter) {
        // 1. 构建问题 (包含当前的 lambda 和 mu)
        SimulationProblem problem(world, dt);
        problem.setPredictiveState(x_hat);
        
        // 2. 牛顿法求解 (Inner Loop)
        solver_.minimize(problem, x_new);
        
        // 3. 检查约束满足情况
        double max_violation = 0.0;
        for (auto& joint : world.joints) {
             // 计算 C(x_new) 并更新 max_violation
        }
        
        if (max_violation < tol) {
            break; // 收敛
        }
        
        // 4. 更新乘子
        for (auto& joint : world.joints) {
            joint->updateLambda(x_new);
        }
        
        // (可选) 增大 mu
    }
    
    // ... 更新 World 状态 ...
}
```

## 4. 实施计划 (Revised)

1.  **Joint 基类设计**：实现通用的 ALM 能量、梯度、Hessian 计算逻辑（基于 `computeC` 和 `computeJ` 接口）。
2.  **HingeJoint 实现**：
    - 实现 `computeC` (位置差 + 轴叉积)。
    - 实现 `computeJ` (解析导数)。
3.  **ALM 求解循环**：修改 `ImplicitEuler::step` 以支持外层 ALM 迭代。
4.  **Integration**：在 `ConstraintForm` 中调用 Joint 的计算接口。
5.  **验证**：对比纯罚函数法，验证 ALM 能在有限刚度下消除关节间隙。
