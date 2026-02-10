# Math 模块设计文档

**职责**: 提供通用的数学工具，封装 Eigen 和数值优化算法。此模块不依赖于物理或动力学层。

## 1. 目录结构

遵循头文件与实现分离原则：

```
NexDynIPC/
├── include/
│   └── NexDynIPC/
│       └── Math/
│           ├── LinearSolver.hpp        // 稀疏线性求解器接口
│           ├── NewtonSolver.hpp        // 牛顿-拉夫逊非线性求解器接口
│           ├── LineSearch.hpp          // 线搜索策略接口
│           └── SparseUtils.hpp         // 稀疏矩阵辅助工具
└── src/
    └── Math/
        ├── LinearSolver.cpp Implementation
        ├── NewtonSolver.cpp Implementation
        └── LineSearch.cpp Implementation
```

## 2. 接口设计

### 2.1 稀疏线性求解器 (`LinearSolver.hpp`)

封装 SuiteSparse (CHOLMOD) 或 Eigen 的稀疏求解器。

```cpp
namespace NexDynIPC::Math {

class LinearSolver {
public:
    virtual ~LinearSolver() = default;
    
    // 分析模式 (Symbolic Analysis)
    virtual void analyzePattern(const Eigen::SparseMatrix<double>& A) = 0;
    
    // 数值分解 (Numerical Factorization)
    virtual void factorize(const Eigen::SparseMatrix<double>& A) = 0;
    
    // 求解 Ax = b
    virtual void solve(const Eigen::VectorXd& b, Eigen::VectorXd& x) = 0;
};

// 工厂函数
std::unique_ptr<LinearSolver> createCholeskySolver(); // Prefer CHOLMOD

} // namespace
```

### 2.2 非线性求解器 (`NewtonSolver.hpp`)

用于求解 $F(x) = 0$ 问题，支持自定义的 Hessian 和 Gradient 计算。

```cpp
namespace NexDynIPC::Math {

// 问题接口：用户需实现此接口以定义优化问题
class OptimizationProblem {
public:
    virtual ~OptimizationProblem() = default;
    
    // 计算目标函数值 (或残差范数)
    virtual double computeValue(const Eigen::VectorXd& x) = 0;
    
    // 计算梯度 (或残差)
    virtual void computeGradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) = 0;
    
    // 计算 Hessian
    virtual void computeHessian(const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& hess) = 0;

    // 更新回调 (每次迭代后调用)
    virtual void postIteration(const Eigen::VectorXd& x) {}
};

class NewtonSolver {
public:
    struct Options {
        int max_iterations = 100;
        double tolerance = 1e-6;
        double line_search_alpha = 1.0;
    };

    NewtonSolver(const Options& opts = Options());

    // 求解，返回是否收敛
    bool minimize(OptimizationProblem& problem, Eigen::VectorXd& x);

private:
    Options opts_;
};

} // namespace
```

### 2.3 辅助工具 (`SparseUtils.hpp`)

提供 `setFromTriplets` 等辅助函数的增强版或封装。
