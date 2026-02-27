// ============================================================================
// NexDynIPC AutoDiff 兼容层
// ============================================================================
// 提供现有 DScalar 与 autodiff 库之间的兼容接口
// ============================================================================

#pragma once

#include "NexDynIPC/Math/AutoDiffExtended.h"
#include "NexDynIPC/Math/AutoDiffTypes.h"

#include <Eigen/Core>
#include <functional>

namespace NexDynIPC::Math {

/**
 * @brief 自动微分选择器
 * 
 * 根据问题规模和类型自动选择最优的自动微分实现：
 * - 输入维度 <= 10：使用现有 DScalar（前向模式）
 * - 输入维度 > 10：使用 autodiff::dual（前向模式）
 * - 多输出函数：使用 autodiff::var（反向模式）
 * - 需要 Hessian：使用 autodiff::dual2nd
 */
class AutoDiffSelector {
public:
    /**
     * @brief 计算标量函数在一点的梯度
     * 
     * @tparam Func 函数类型，接受 dual 或 DScalar 类型参数
     * @param f 目标函数
     * @param x 计算点
     * @return Eigen::VectorXd 梯度向量
     * 
     * @example
     * auto func = [](const auto& x) {
     *     return x[0]*x[0] + x[1]*x[1] + x[2]*x[2];
     * };
     * Eigen::VectorXd x(3); x << 1, 2, 3;
     * Eigen::VectorXd grad = AutoDiffSelector::gradient(func, x);
     * // grad = [2, 4, 6]
     */
    template<typename Func>
    static Eigen::VectorXd gradient(Func&& f, const Eigen::VectorXd& x) {
        if (x.size() <= 10) {
            return gradientForwardLegacy(f, x);
        } else {
            return gradientAutodiff(f, x);
        }
    }

    /**
     * @brief 计算向量值函数的 Jacobian 矩阵
     * 
     * @tparam Func 函数类型，返回向量
     * @param f 目标函数
     * @param x 计算点
     * @return Eigen::MatrixXd Jacobian 矩阵
     * 
     * @note 多输出函数使用反向模式更高效
     */
    template<typename Func>
    static Eigen::MatrixXd jacobian(Func&& f, const Eigen::VectorXd& x) {
        return jacobianReverse(f, x);
    }

    /**
     * @brief 计算标量函数的 Hessian 矩阵
     * 
     * @tparam Func 函数类型
     * @param f 目标函数
     * @param x 计算点
     * @return Eigen::MatrixXd Hessian 矩阵
     * 
     * @example
     * auto func = [](const auto& x) {
     *     return x[0]*x[0] + x[0]*x[1] + x[1]*x[1];
     * };
     * Eigen::VectorXd x(2); x << 1, 2;
     * Eigen::MatrixXd H = AutoDiffSelector::hessian(func, x);
     * // H = [[2, 1], [1, 2]]
     */
    template<typename Func>
    static Eigen::MatrixXd hessian(Func&& f, const Eigen::VectorXd& x) {
        return hessianAutodiff(f, x);
    }

    /**
     * @brief 计算标量函数在一点的值和梯度
     * 
     * @tparam Func 函数类型
     * @param f 目标函数
     * @param x 计算点
     * @return std::pair<double, Eigen::VectorXd> (值, 梯度)
     */
    template<typename Func>
    static std::pair<double, Eigen::VectorXd> valueAndGradient(Func&& f, 
                                                                const Eigen::VectorXd& x) {
        using namespace autodiff;
        
        VectorXdual xd = x.cast<dual>();
        dual y = f(xd);
        
        Eigen::VectorXd grad = gradient(f, wrt(xd), at(xd));
        return { y.val, grad };
    }

    /**
     * @brief 计算标量函数在一点的值、梯度和 Hessian
     * 
     * @tparam Func 函数类型
     * @param f 目标函数
     * @param x 计算点
     * @return std::tuple<double, Eigen::VectorXd, Eigen::MatrixXd> (值, 梯度, Hessian)
     */
    template<typename Func>
    static std::tuple<double, Eigen::VectorXd, Eigen::MatrixXd> valueGradientAndHessian(
        Func&& f, const Eigen::VectorXd& x) {
        using namespace autodiff;
        
        VectorXdual2nd x2 = x.cast<dual2nd>();
        dual2nd y = f(x2);
        
        Eigen::VectorXd grad = gradient(f, wrt(x2), at(x2));
        Eigen::MatrixXd H = hessian(f, wrt(x2), at(x2));
        
        return { y.val, grad, H };
    }

private:
    /**
     * @brief 使用现有 DScalar 实现的前向模式梯度计算
     */
    template<typename Func>
    static Eigen::VectorXd gradientForwardLegacy(Func&& f, const Eigen::VectorXd& x) {
        const int n = static_cast<int>(x.size());
        Eigen::VectorXd grad(n);
        
        // 激活自动微分
        AutodiffType<Dynamic>::activate(n);
        
        // 对每个变量计算偏导数
        for (int i = 0; i < n; ++i) {
            AutodiffType<Dynamic>::DDouble1 xi(i, x[i]);
            
            // 构建输入向量
            std::vector<AutodiffType<Dynamic>::DDouble1> x_ad(n);
            for (int j = 0; j < n; ++j) {
                if (j == i) {
                    x_ad[j] = xi;
                } else {
                    x_ad[j] = AutodiffType<Dynamic>::DDouble1(x[j]);
                }
            }
            
            // 计算函数值
            auto y = f(x_ad);
            grad[i] = y.getGradient()[0];
        }
        
        return grad;
    }

    /**
     * @brief 使用 autodiff 的前向模式梯度计算
     */
    template<typename Func>
    static Eigen::VectorXd gradientAutodiff(Func&& f, const Eigen::VectorXd& x) {
        using namespace autodiff;
        
        VectorXdual xd = x.cast<dual>();
        return gradient(f, wrt(xd), at(xd));
    }

    /**
     * @brief 使用反向模式的 Jacobian 计算
     */
    template<typename Func>
    static Eigen::MatrixXd jacobianReverse(Func&& f, const Eigen::VectorXd& x) {
        using namespace autodiff;
        
        VectorXvar xv = x.cast<var>();
        VectorXvar y = f(xv);
        
        return jacobian(y, xv);
    }

    /**
     * @brief 使用 autodiff 的 Hessian 计算
     */
    template<typename Func>
    static Eigen::MatrixXd hessianAutodiff(Func&& f, const Eigen::VectorXd& x) {
        using namespace autodiff;
        
        VectorXdual2nd x2 = x.cast<dual2nd>();
        return hessian(f, wrt(x2), at(x2));
    }
};

/**
 * @brief 自动微分转换工具
 * 
 * 提供 DScalar 与 autodiff 类型之间的转换
 */
class AutoDiffConverter {
public:
    //===========================================================================
    // DScalar -> autodiff::dual
    //===========================================================================
    
    /**
     * @brief 将 DScalar1 转换为 autodiff::dual
     */
    static autodiff::dual toDual(const DScalar1& ds) {
        const auto& grad = ds.getGradient();
        double g = grad.size() > 0 ? grad[0] : 0.0;
        return autodiff::dual(ds.getValue(), g);
    }

    /**
     * @brief 将 DScalar2 转换为 autodiff::dual2nd
     */
    static autodiff::dual2nd toDual2nd(const DScalar2& ds) {
        // DScalar2 存储的是值、梯度和 Hessian
        // dual2nd 的结构是 dual<dual<double>>
        // 这里简化为只提取值和一阶梯度
        return autodiff::dual2nd(ds.getValue(), ds.getGradient()[0]);
    }

    /**
     * @brief 将 Eigen::VectorXd 转换为 VectorXdual
     */
    static autodiff::VectorXdual<> toVectorDual(const Eigen::VectorXd& v) {
        return v.cast<autodiff::dual>();
    }

    /**
     * @brief 将 Eigen::VectorXd 转换为 VectorXdual2nd
     */
    static autodiff::VectorXdual2nd<> toVectorDual2nd(const Eigen::VectorXd& v) {
        return v.cast<autodiff::dual2nd>();
    }

    /**
     * @brief 将 Eigen::VectorXd 转换为 VectorXvar
     */
    static autodiff::VectorXvar<> toVectorVar(const Eigen::VectorXd& v) {
        autodiff::VectorXvar<> result(v.size());
        for (int i = 0; i < v.size(); ++i) {
            result[i] = autodiff::var(v[i]);
        }
        return result;
    }

    //===========================================================================
    // autodiff::dual -> DScalar
    //===========================================================================
    
    /**
     * @brief 将 autodiff::dual 转换为 DScalar1
     */
    static DScalar1 toDScalar1(const autodiff::dual& d, int nVars = 1) {
        DScalar1 result(d.val);
        if (nVars > 0) {
            result.getGradient().resize(nVars);
            result.getGradient()[0] = d.grad;
        }
        return result;
    }

    /**
     * @brief 将 VectorXdual 转换为 Eigen::VectorXd（提取值）
     */
    static Eigen::VectorXd toVectorXd(const autodiff::VectorXdual<>& vd) {
        return vd.cast<double>();
    }

    /**
     * @brief 从 VectorXdual 提取梯度向量
     */
    static Eigen::VectorXd extractGradient(const autodiff::VectorXdual<>& vd) {
        Eigen::VectorXd grad(vd.size());
        for (int i = 0; i < vd.size(); ++i) {
            grad[i] = vd[i].grad;
        }
        return grad;
    }

    /**
     * @brief 从 VectorXdual2nd 提取 Hessian 矩阵
     */
    static Eigen::MatrixXd extractHessian(const autodiff::VectorXdual2nd<>& vd) {
        int n = static_cast<int>(vd.size());
        Eigen::MatrixXd H(n, n);
        
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                // dual2nd 的 grad 是 dual 类型，其 grad 是二阶导数
                H(i, j) = vd[i].grad.grad;
            }
        }
        return H;
    }
};

/**
 * @brief 自动微分包装器
 * 
 * 提供统一的函数包装接口，自动处理类型转换
 */
class AutoDiffWrapper {
public:
    /**
     * @brief 包装函数以支持自动微分
     * 
     * 将接受 Eigen::VectorXd 的函数转换为接受 VectorXdual 的函数
     */
    template<typename Func>
    static auto wrapForAutodiff(Func&& f) {
        return [f](const autodiff::VectorXdual<>& x) -> autodiff::dual {
            // 转换为普通向量
            Eigen::VectorXd x_plain = x.cast<double>();
            
            // 调用原函数
            double y = f(x_plain);
            
            return autodiff::dual(y);
        };
    }

    /**
     * @brief 包装向量值函数
     */
    template<typename Func>
    static auto wrapVectorForAutodiff(Func&& f) {
        return [f](const autodiff::VectorXvar<>& x) -> autodiff::VectorXvar<> {
            // 转换为普通向量
            Eigen::VectorXd x_plain(x.size());
            for (int i = 0; i < x.size(); ++i) {
                x_plain[i] = x[i].val;
            }
            
            // 调用原函数
            Eigen::VectorXd y = f(x_plain);
            
            // 转换回 var 向量
            autodiff::VectorXvar<> result(y.size());
            for (int i = 0; i < y.size(); ++i) {
                result[i] = autodiff::var(y[i]);
            }
            return result;
        };
    }
};

} // namespace NexDynIPC::Math

// ============================================================================
// 便捷宏定义
// ============================================================================

/**
 * @brief 定义支持自动微分的函数
 * 
 * 使用方式：
 * AUTODIFF_FUNCTION(my_func, (const auto& x), {
 *     return x[0]*x[0] + x[1]*x[1];
 * })
 */
#define AUTODIFF_FUNCTION(name, params, body) \
    template<typename T> \
    auto name params -> T { \
        body \
    }

/**
 * @brief 定义支持自动微分的标量函数
 */
#define AUTODIFF_SCALAR_FUNCTION(name, params, body) \
    template<typename T> \
    T name params { \
        body \
    }

// ============================================================================
// 使用示例
// ============================================================================
/*
 * // 示例 1: 使用 AutoDiffSelector 自动选择最优实现
 * auto func = [](const auto& x) {
 *     return x[0]*x[0] + x[1]*x[1] + x[2]*x[2];
 * };
 * 
 * Eigen::VectorXd x(3); x << 1, 2, 3;
 * Eigen::VectorXd grad = NexDynIPC::Math::AutoDiffSelector::gradient(func, x);
 * 
 * // 示例 2: 类型转换
 * DScalar1 ds(5.0);
 * autodiff::dual d = NexDynIPC::Math::AutoDiffConverter::toDual(ds);
 * 
 * // 示例 3: 计算 Hessian
 * Eigen::MatrixXd H = NexDynIPC::Math::AutoDiffSelector::hessian(func, x);
 * 
 * // 示例 4: 同时获取值、梯度和 Hessian
 * auto [val, grad, hess] = NexDynIPC::Math::AutoDiffSelector::valueGradientAndHessian(func, x);
 */
// ============================================================================
