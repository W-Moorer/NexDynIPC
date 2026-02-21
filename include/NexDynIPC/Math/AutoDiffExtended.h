// ============================================================================
// NexDynIPC Extended AutoDiff - 统一入口头文件
// ============================================================================
// 基于 autodiff 库 (https://github.com/autodiff/autodiff)
// MIT License - Copyright (c) 2018-2024 Allan Leal
// ============================================================================

#pragma once

// ============================================================================
// 引入 autodiff 核心组件
// ============================================================================

// 前向模式 - dual 数
#include "autodiff/forward/dual.hpp"
#include "autodiff/forward/dual/eigen.hpp"

// 前向模式 - real 数
#include "autodiff/forward/real.hpp"
#include "autodiff/forward/real/eigen.hpp"

// 前向模式 - 工具函数
#include "autodiff/forward/utils/derivative.hpp"
#include "autodiff/forward/utils/gradient.hpp"
#include "autodiff/forward/utils/taylorseries.hpp"

// 反向模式 - var 变量
#include "autodiff/reverse/var.hpp"
#include "autodiff/reverse/var/eigen.hpp"

// ============================================================================
// NexDynIPC 命名空间适配
// ============================================================================

namespace NexDynIPC {

/// 引入 autodiff 命名空间
namespace autodiff = ::autodiff;

/// 前向模式 dual 数类型别名
using dual = autodiff::dual;
using dual2nd = autodiff::dual2nd;
using dual3rd = autodiff::dual3rd;
using dual4th = autodiff::dual4th;

/// 前向模式 real 数类型别名
using real = autodiff::real;
using real2nd = autodiff::real2nd;
using real3rd = autodiff::real3rd;
using real4th = autodiff::real4th;

/// 反向模式 var 类型别名
using var = autodiff::var;

/// Eigen 集成类型别名
template<size_t N = Eigen::Dynamic>
using VectorXdual = autodiff::VectorXdual<N>;

template<size_t N = Eigen::Dynamic>
using VectorXvar = autodiff::VectorXvar<N>;

template<size_t N = Eigen::Dynamic, size_t M = Eigen::Dynamic>
using MatrixXdual = autodiff::MatrixXdual<N, M>;

/// 导数计算函数
using autodiff::derivative;
using autodiff::derivatives;
using autodiff::gradient;
using autodiff::jacobian;
using autodiff::hessian;
using autodiff::wrt;
using autodiff::at;

/// 泰勒级数
using autodiff::taylor;

/// 数学函数 - 基本运算
using autodiff::abs;
using autodiff::fabs;
using autodiff::floor;
using autodiff::ceil;
using autodiff::round;

/// 数学函数 - 三角函数
using autodiff::sin;
using autodiff::cos;
using autodiff::tan;
using autodiff::asin;
using autodiff::acos;
using autodiff::atan;
using autodiff::atan2;

/// 数学函数 - 双曲函数
using autodiff::sinh;
using autodiff::cosh;
using autodiff::tanh;
using autodiff::asinh;
using autodiff::acosh;
using autodiff::atanh;

/// 数学函数 - 指数对数
using autodiff::exp;
using autodiff::log;
using autodiff::log10;
using autodiff::log2;
using autodiff::log1p;
using autodiff::sqrt;
using autodiff::cbrt;
using autodiff::pow;

/// 数学函数 - 特殊函数
using autodiff::erf;
using autodiff::erfc;
using autodiff::gamma;
using autodiff::lgamma;
using autodiff::hypot;

/// 数学函数 - 最值函数
using autodiff::min;
using autodiff::max;
using autodiff::fmin;
using autodiff::fmax;

/// 数学函数 - 其他
using autodiff::inverse;
using autodiff::square;
using autodiff::cubic;

} // namespace NexDynIPC

// ============================================================================
// 向后兼容 - NexDynIPC::Math 命名空间
// ============================================================================

namespace NexDynIPC::Math {

/// 引入所有 autodiff 类型和函数
using namespace NexDynIPC;

} // namespace NexDynIPC::Math

// ============================================================================
// 使用示例（文档）
// ============================================================================
/*
 * 基本使用示例:
 *
 * #include <NexDynIPC/Math/AutoDiffExtended.h>
 * using namespace NexDynIPC;
 *
 * // 1. 标量函数导数
 * dual f(dual x) { return sin(x) * exp(x); }
 * double dfdx = derivative(f, wrt(x), at(x));
 *
 * // 2. 多变量梯度
 * dual g(dual x, dual y, dual z) { return x*y + y*z + z*x; }
 * auto [gx, gy, gz] = derivatives(g, wrt(x, y, z), at(x, y, z));
 *
 * // 3. 向量函数 Jacobian
 * VectorXvar F(const VectorXvar& x) {
 *     VectorXvar y(2);
 *     y << x.sum(), x.prod();
 *     return y;
 * }
 * Eigen::MatrixXd J = jacobian(F(x), x);
 *
 * // 4. Hessian 矩阵
 * dual2nd h(dual2nd x, dual2nd y) { return x*x + x*y + y*y; }
 * Eigen::MatrixXd H = hessian(h, wrt(x, y), at(x, y));
 *
 * // 5. 反向模式
 * var u(var x, var y) { return sqrt(x*x + y*y); }
 * var x = 3.0, y = 4.0;
 * var z = u(x, y);
 * auto [zx, zy] = derivatives(z, wrt(x, y));
 */
// ============================================================================
