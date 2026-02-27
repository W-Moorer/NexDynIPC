// ============================================================================
// NexDynIPC AutoDiff 实用工具
// ============================================================================
// 提供常用的自动微分辅助函数和工具
// ============================================================================

#pragma once

#include "NexDynIPC/Math/AutoDiffExtended.h"
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>
#include <functional>
#include <cmath>

namespace NexDynIPC::Math {

// ============================================================================
// 数值验证工具
// ============================================================================

/**
 * @brief 梯度检查工具
 * 
 * 使用有限差分法验证自动微分计算的正确性
 */
class GradientChecker {
public:
    /**
     * @brief 检查梯度计算的正确性
     * 
     * @tparam Func 函数类型
     * @param f 目标函数
     * @param x 检查点
     * @param epsilon 有限差分步长
     * @param tolerance 误差容限
     * @return std::pair<bool, double> (是否通过, 最大相对误差)
     */
    template<typename Func>
    static std::pair<bool, double> checkGradient(
        Func&& f,
        const Eigen::VectorXd& x,
        double epsilon = 1e-6,
        double tolerance = 1e-4
    ) {
        using namespace autodiff;
        
        // 使用自动微分计算梯度
        VectorXdual<> xd = x.cast<dual>();
        Eigen::VectorXd ad_grad = gradient(f, wrt(xd), at(xd));
        
        // 使用中心差分计算数值梯度
        Eigen::VectorXd fd_grad = numericalGradient(f, x, epsilon);
        
        // 计算相对误差
        Eigen::VectorXd diff = ad_grad - fd_grad;
        double max_rel_error = 0.0;
        
        for (int i = 0; i < x.size(); ++i) {
            double denom = std::max(std::abs(ad_grad[i]), std::abs(fd_grad[i])) + 1e-10;
            double rel_error = std::abs(diff[i]) / denom;
            max_rel_error = std::max(max_rel_error, rel_error);
        }
        
        return { max_rel_error < tolerance, max_rel_error };
    }
    
    /**
     * @brief 检查 Jacobian 计算的正确性
     */
    template<typename Func>
    static std::pair<bool, double> checkJacobian(
        Func&& f,
        const Eigen::VectorXd& x,
        double epsilon = 1e-6,
        double tolerance = 1e-4
    ) {
        using namespace autodiff;
        
        // 使用自动微分计算 Jacobian
        VectorXvar<> xv = x.cast<var>();
        VectorXvar<> y = f(xv);
        Eigen::MatrixXd ad_jac = jacobian(y, xv);
        
        // 使用有限差分计算数值 Jacobian
        Eigen::MatrixXd fd_jac = numericalJacobian(f, x, epsilon);
        
        // 计算相对误差
        double max_rel_error = 0.0;
        for (int i = 0; i < ad_jac.rows(); ++i) {
            for (int j = 0; j < ad_jac.cols(); ++j) {
                double denom = std::max(std::abs(ad_jac(i,j)), std::abs(fd_jac(i,j))) + 1e-10;
                double rel_error = std::abs(ad_jac(i,j) - fd_jac(i,j)) / denom;
                max_rel_error = std::max(max_rel_error, rel_error);
            }
        }
        
        return { max_rel_error < tolerance, max_rel_error };
    }
    
    /**
     * @brief 检查 Hessian 计算的正确性
     */
    template<typename Func>
    static std::pair<bool, double> checkHessian(
        Func&& f,
        const Eigen::VectorXd& x,
        double epsilon = 1e-5,
        double tolerance = 1e-3
    ) {
        using namespace autodiff;
        
        // 使用自动微分计算 Hessian
        VectorXdual2nd<> x2 = x.cast<dual2nd>();
        Eigen::MatrixXd ad_hess = hessian(f, wrt(x2), at(x2));
        
        // 使用有限差分计算数值 Hessian
        Eigen::MatrixXd fd_hess = numericalHessian(f, x, epsilon);
        
        // 计算相对误差
        double max_rel_error = 0.0;
        for (int i = 0; i < ad_hess.rows(); ++i) {
            for (int j = 0; j < ad_hess.cols(); ++j) {
                double denom = std::max(std::abs(ad_hess(i,j)), std::abs(fd_hess(i,j))) + 1e-10;
                double rel_error = std::abs(ad_hess(i,j) - fd_hess(i,j)) / denom;
                max_rel_error = std::max(max_rel_error, rel_error);
            }
        }
        
        return { max_rel_error < tolerance, max_rel_error };
    }

private:
    /**
     * @brief 数值梯度计算（中心差分）
     */
    template<typename Func>
    static Eigen::VectorXd numericalGradient(Func&& f, 
                                              const Eigen::VectorXd& x, 
                                              double epsilon) {
        Eigen::VectorXd grad(x.size());
        
        for (int i = 0; i < x.size(); ++i) {
            Eigen::VectorXd x_plus = x;
            Eigen::VectorXd x_minus = x;
            x_plus[i] += epsilon;
            x_minus[i] -= epsilon;
            
            // 评估函数
            autodiff::VectorXdual<> xp = x_plus.cast<autodiff::dual>();
            autodiff::VectorXdual<> xm = x_minus.cast<autodiff::dual>();
            
            double f_plus = f(xp).val;
            double f_minus = f(xm).val;
            
            grad[i] = (f_plus - f_minus) / (2.0 * epsilon);
        }
        
        return grad;
    }
    
    /**
     * @brief 数值 Jacobian 计算
     */
    template<typename Func>
    static Eigen::MatrixXd numericalJacobian(Func&& f,
                                              const Eigen::VectorXd& x,
                                              double epsilon) {
        // 首先评估函数获取输出维度
        autodiff::VectorXvar<> xv = x.cast<autodiff::var>();
        autodiff::VectorXvar<> y0 = f(xv);
        int m = y0.size();
        int n = x.size();
        
        Eigen::MatrixXd jac(m, n);
        
        for (int j = 0; j < n; ++j) {
            Eigen::VectorXd x_plus = x;
            Eigen::VectorXd x_minus = x;
            x_plus[j] += epsilon;
            x_minus[j] -= epsilon;
            
            autodiff::VectorXvar<> y_plus = f(x_plus.cast<autodiff::var>());
            autodiff::VectorXvar<> y_minus = f(x_minus.cast<autodiff::var>());
            
            for (int i = 0; i < m; ++i) {
                jac(i, j) = (y_plus[i].val - y_minus[i].val) / (2.0 * epsilon);
            }
        }
        
        return jac;
    }
    
    /**
     * @brief 数值 Hessian 计算
     */
    template<typename Func>
    static Eigen::MatrixXd numericalHessian(Func&& f,
                                             const Eigen::VectorXd& x,
                                             double epsilon) {
        int n = x.size();
        Eigen::MatrixXd hess(n, n);
        
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                // 使用中心差分近似二阶导数
                Eigen::VectorXd x_pp = x, x_pm = x, x_mp = x, x_mm = x;
                x_pp[i] += epsilon; x_pp[j] += epsilon;
                x_pm[i] += epsilon; x_pm[j] -= epsilon;
                x_mp[i] -= epsilon; x_mp[j] += epsilon;
                x_mm[i] -= epsilon; x_mm[j] -= epsilon;
                
                autodiff::VectorXdual<> vpp = x_pp.cast<autodiff::dual>();
                autodiff::VectorXdual<> vpm = x_pm.cast<autodiff::dual>();
                autodiff::VectorXdual<> vmp = x_mp.cast<autodiff::dual>();
                autodiff::VectorXdual<> vmm = x_mm.cast<autodiff::dual>();
                
                double f_pp = f(vpp).val;
                double f_pm = f(vpm).val;
                double f_mp = f(vmp).val;
                double f_mm = f(vmm).val;
                
                hess(i, j) = (f_pp - f_pm - f_mp + f_mm) / (4.0 * epsilon * epsilon);
            }
        }
        
        return hess;
    }
};

// ============================================================================
// 优化辅助工具
// ============================================================================

/**
 * @brief 线搜索工具
 * 
 * 提供多种线搜索策略
 */
class LineSearch {
public:
    /**
     * @brief 回溯线搜索
     * 
     * @tparam Func 函数类型
     * @param f 目标函数
     * @param x 当前点
     * @param direction 搜索方向
     * @param initial_step 初始步长
     * @param c 充分下降条件参数 (0 < c < 1)
     * @param rho 步长缩减因子 (0 < rho < 1)
     * @param max_iters 最大迭代次数
     * @return double 最优步长
     */
    template<typename Func>
    static double backtracking(Func&& f,
                                const Eigen::VectorXd& x,
                                const Eigen::VectorXd& direction,
                                double initial_step = 1.0,
                                double c = 1e-4,
                                double rho = 0.5,
                                int max_iters = 20) {
        using namespace autodiff;
        
        double step = initial_step;
        
        // 当前函数值
        VectorXdual<> xd = x.cast<dual>();
        double f0 = f(xd).val;
        
        // 当前梯度
        Eigen::VectorXd grad = gradient(f, wrt(xd), at(xd));
        double grad_dot_dir = grad.dot(direction);
        
        for (int i = 0; i < max_iters; ++i) {
            Eigen::VectorXd x_new = x + step * direction;
            VectorXdual<> xn = x_new.cast<dual>();
            double f_new = f(xn).val;
            
            // Armijo 条件
            if (f_new <= f0 + c * step * grad_dot_dir) {
                return step;
            }
            
            step *= rho;
        }
        
        return step;
    }
    
    /**
     * @brief 精确线搜索（二分法）
     */
    template<typename Func>
    static double exact(Func&& f,
                        const Eigen::VectorXd& x,
                        const Eigen::VectorXd& direction,
                        double min_step = 0.0,
                        double max_step = 1.0,
                        double tolerance = 1e-6,
                        int max_iters = 50) {
        using namespace autodiff;
        
        auto phi = [&](double alpha) -> double {
            Eigen::VectorXd x_new = x + alpha * direction;
            VectorXdual<> xn = x_new.cast<dual>();
            return f(xn).val;
        };
        
        // 黄金分割搜索
        const double phi_ratio = (1.0 + std::sqrt(5.0)) / 2.0;
        double a = min_step, b = max_step;
        
        double c = b - (b - a) / phi_ratio;
        double d = a + (b - a) / phi_ratio;
        
        for (int i = 0; i < max_iters; ++i) {
            if (std::abs(b - a) < tolerance) {
                break;
            }
            
            if (phi(c) < phi(d)) {
                b = d;
            } else {
                a = c;
            }
            
            c = b - (b - a) / phi_ratio;
            d = a + (b - a) / phi_ratio;
        }
        
        return (a + b) / 2.0;
    }
};

// ============================================================================
// 自动微分包装器
// ============================================================================

/**
 * @brief 将普通函数转换为支持自动微分的函数
 */
class AutoDiffFunction {
public:
    /**
     * @brief 包装标量函数
     * 
     * 将 f: R^n -> R 转换为支持 dual 类型的函数
     */
    template<typename Func>
    static auto wrapScalar(Func&& f) {
        return [f](const autodiff::VectorXdual<>& x) -> autodiff::dual {
            // 提取值
            Eigen::VectorXd x_val = x.cast<double>();
            
            // 调用原函数
            double y_val = f(x_val);
            
            return autodiff::dual(y_val);
        };
    }
    
    /**
     * @brief 包装向量函数
     * 
     * 将 f: R^n -> R^m 转换为支持 var 类型的函数
     */
    template<typename Func>
    static auto wrapVector(Func&& f) {
        return [f](const autodiff::VectorXvar<>& x) -> autodiff::VectorXvar<> {
            // 提取值
            Eigen::VectorXd x_val(x.size());
            for (int i = 0; i < x.size(); ++i) {
                x_val[i] = x[i].val;
            }
            
            // 调用原函数
            Eigen::VectorXd y_val = f(x_val);
            
            // 转换回 var
            autodiff::VectorXvar<> y(y_val.size());
            for (int i = 0; i < y_val.size(); ++i) {
                y[i] = autodiff::var(y_val[i]);
            }
            return y;
        };
    }
};

// ============================================================================
// 常用能量函数模板
// ============================================================================

/**
 * @brief 二次能量函数
 * 
 * E(x) = 0.5 * x^T A x - b^T x + c
 */
class QuadraticEnergy {
public:
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    double c;
    
    QuadraticEnergy(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, double c = 0.0)
        : A(A), b(b), c(c) {}
    
    template<typename T>
    T operator()(const Eigen::Matrix<T, Eigen::Dynamic, 1>& x) const {
        return 0.5 * x.dot(A.cast<T>() * x) - b.cast<T>().dot(x) + T(c);
    }
};

/**
 * @brief 高斯能量函数
 * 
 * E(x) = -exp(-||x - mu||^2 / (2*sigma^2))
 */
class GaussianEnergy {
public:
    Eigen::VectorXd mu;
    double sigma;
    
    GaussianEnergy(const Eigen::VectorXd& mu, double sigma)
        : mu(mu), sigma(sigma) {}
    
    template<typename T>
    T operator()(const Eigen::Matrix<T, Eigen::Dynamic, 1>& x) const {
        using std::exp;
        T dist_sq = (x - mu.cast<T>()).squaredNorm();
        return -exp(-dist_sq / (2.0 * sigma * sigma));
    }
};

/**
 * @brief 弹簧能量函数
 * 
 * E(x1, x2) = 0.5 * k * (||x1 - x2|| - rest_length)^2
 */
class SpringEnergy {
public:
    double rest_length;
    double stiffness;
    
    SpringEnergy(double rest_length, double stiffness)
        : rest_length(rest_length), stiffness(stiffness) {}
    
    template<typename T>
    T operator()(const Eigen::Matrix<T, Eigen::Dynamic, 1>& x) const {
        using std::sqrt;
        int n = x.size() / 2;
        
        Eigen::Matrix<T, Eigen::Dynamic, 1> x1 = x.head(n);
        Eigen::Matrix<T, Eigen::Dynamic, 1> x2 = x.tail(n);
        
        T dist = sqrt((x1 - x2).squaredNorm());
        T stretch = dist - T(rest_length);
        
        return 0.5 * T(stiffness) * stretch * stretch;
    }
};

// ============================================================================
// 批量计算工具
// ============================================================================

/**
 * @brief 批量梯度计算
 * 
 * 对多个点同时计算梯度
 */
template<typename Func>
std::vector<Eigen::VectorXd> batchGradient(Func&& f, 
                                           const std::vector<Eigen::VectorXd>& points) {
    std::vector<Eigen::VectorXd> gradients;
    gradients.reserve(points.size());
    
    for (const auto& x : points) {
        gradients.push_back(AutoDiffSelector::gradient(f, x));
    }
    
    return gradients;
}

/**
 * @brief 批量 Hessian 计算
 */
template<typename Func>
std::vector<Eigen::MatrixXd> batchHessian(Func&& f,
                                          const std::vector<Eigen::VectorXd>& points) {
    std::vector<Eigen::MatrixXd> hessians;
    hessians.reserve(points.size());
    
    for (const auto& x : points) {
        hessians.push_back(AutoDiffSelector::hessian(f, x));
    }
    
    return hessians;
}

} // namespace NexDynIPC::Math

// ============================================================================
// 使用示例
// ============================================================================
/*
 * // 示例 1: 梯度检查
 * auto func = [](const auto& x) { return x.dot(x); };
 * Eigen::VectorXd x = ...;
 * auto [passed, error] = GradientChecker::checkGradient(func, x);
 * std::cout << "Gradient check: " << (passed ? "PASSED" : "FAILED") 
 *           << " (error: " << error << ")" << std::endl;
 * 
 * // 示例 2: 线搜索
 * Eigen::VectorXd grad = ...;
 * double step = LineSearch::backtracking(func, x, -grad);
 * 
 * // 示例 3: 使用预定义能量函数
 * QuadraticEnergy energy(A, b);
 * double val = energy(x);
 * Eigen::VectorXd grad = AutoDiffSelector::gradient(energy, x);
 * 
 * // 示例 4: 批量计算
 * std::vector<Eigen::VectorXd> points = {x1, x2, x3};
 * auto grads = batchGradient(func, points);
 */
// ============================================================================
