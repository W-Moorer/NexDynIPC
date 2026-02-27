/**
 * @file Interval.h
 * @brief Interval arithmetic for conservative computations
 * 
 * This module provides interval arithmetic capabilities for computing
 * guaranteed bounds on numerical results. Interval arithmetic is essential
 * for conservative collision detection in CCD (Continuous Collision Detection).
 * 
 * The implementation uses Boost.Interval library as the backend.
 */

#ifndef NEXDYNIPC_MATH_INTERVAL_HPP
#define NEXDYNIPC_MATH_INTERVAL_HPP

#include <Eigen/Core>
#include <boost/numeric/interval.hpp>
#include <string>
#include <cmath>

namespace NexDynIPC::Math {

/**
 * @brief Checking policy for interval arithmetic that catches NaN values
 */
using IntervalCheckingPolicy = boost::numeric::interval_lib::checking_catch_nan<double>;

/**
 * @brief Rounding policy using standard math functions
 * 
 * This provides proper rounding for interval arithmetic operations.
 * On Windows, we use the standard rounding mode approach.
 */
using IntervalRoundingPolicy = boost::numeric::interval_lib::save_state<
    boost::numeric::interval_lib::rounded_transc_std<double>>;

/**
 * @brief Interval type with proper rounding and NaN checking
 * 
 * This interval type guarantees conservative bounds for all arithmetic
 * operations. The lower bound is always rounded down and the upper bound
 * is always rounded up to ensure the true result is always contained.
 */
using Interval = boost::numeric::interval<
    double,
    boost::numeric::interval_lib::policies<
        IntervalRoundingPolicy,
        IntervalCheckingPolicy>>;

// ============================================================================
// Type aliases for interval vectors and matrices
// ============================================================================

using Vector2I = Eigen::Vector2<Interval>;
using Vector3I = Eigen::Vector3<Interval>;
using VectorXI = Eigen::VectorX<Interval>;

using RowVector2I = Eigen::RowVector2<Interval>;
using RowVector3I = Eigen::RowVector3<Interval>;
using RowVectorXI = Eigen::RowVectorX<Interval>;

using Matrix2I = Eigen::Matrix2<Interval>;
using Matrix3I = Eigen::Matrix3<Interval>;
using MatrixXI = Eigen::MatrixX<Interval>;

// ============================================================================
// Interval utility functions
// ============================================================================

/**
 * @brief Get the width (upper - lower) of an interval
 * @param i The interval
 * @return The width of the interval
 */
inline double width(const Interval& i)
{
    return boost::numeric::width(i);
}

/**
 * @brief Get the width of each component in an interval vector
 * @param x The interval vector
 * @return Vector of widths
 */
template <typename Derived>
inline Eigen::VectorXd width(const Eigen::MatrixBase<Derived>& x)
{
    Eigen::VectorXd w(x.size());
    for (int i = 0; i < x.size(); i++) {
        w(i) = width(x(i));
    }
    return w;
}

/**
 * @brief Get the midpoint of an interval
 * @param i The interval
 * @return The midpoint of the interval
 */
inline double midpoint(const Interval& i)
{
    return boost::numeric::median(i);
}

/**
 * @brief Get the lower bound of an interval
 * @param i The interval
 * @return The lower bound
 */
inline double lower(const Interval& i)
{
    return i.lower();
}

/**
 * @brief Get the upper bound of an interval
 * @param i The interval
 * @return The upper bound
 */
inline double upper(const Interval& i)
{
    return i.upper();
}

/**
 * @brief Check if an interval contains zero
 * @param i The interval
 * @return True if zero is in [lower, upper]
 */
inline bool zero_in(const Interval& i)
{
    return boost::numeric::zero_in(i);
}

/**
 * @brief Check if an n-dimensional interval vector contains the origin
 * @param x The interval vector
 * @return True if all components contain zero
 */
template <typename Derived>
inline bool zero_in(const Eigen::MatrixBase<Derived>& x)
{
    for (int i = 0; i < x.size(); i++) {
        if (!zero_in(x(i))) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Check if an interval contains a value
 * @param i The interval
 * @param x The value to check
 * @return True if x is in [lower, upper]
 */
inline bool contains(const Interval& i, double x)
{
    return i.lower() <= x && x <= i.upper();
}

/**
 * @brief Check if an interval is empty
 * @param i The interval
 * @return True if the interval is empty
 */
inline bool is_empty(const Interval& i)
{
    return boost::numeric::empty(i);
}

/**
 * @brief Compute the diagonal width (L2 norm of widths) of an interval vector
 * @param x The interval vector
 * @return The diagonal width
 */
template <typename Derived>
inline double diagonal_width(const Eigen::MatrixBase<Derived>& x)
{
    Eigen::VectorXd widths = width(x);
    double w = 0;
    for (int i = 0; i < widths.size(); i++) {
        w += widths(i) * widths(i);
    }
    return std::sqrt(w);
}

/**
 * @brief Compute the squared L2 norm of an interval vector
 * 
 * This function avoids negative intermediate values that can occur
 * with naive computation using Eigen's squaredNorm().
 * 
 * @param x The interval vector
 * @return The squared L2 norm as an interval
 */
inline Interval squared_norm(const VectorXI& x)
{
    Interval result(0.0, 0.0);
    for (int i = 0; i < x.size(); i++) {
        result += x(i) * x(i);
    }
    return result;
}

/**
 * @brief Compute the L2 norm of an interval vector
 * @param x The interval vector
 * @return The L2 norm as an interval
 */
inline Interval norm(const VectorXI& x)
{
    return sqrt(squared_norm(x));
}

/**
 * @brief Intersect two intervals
 * @param a First interval
 * @param b Second interval
 * @return The intersection of a and b
 */
inline Interval intersect(const Interval& a, const Interval& b)
{
    return boost::numeric::intersect(a, b);
}

/**
 * @brief Compute the hull (union) of two intervals
 * @param a First interval
 * @param b Second interval
 * @return The hull of a and b
 */
inline Interval hull(const Interval& a, const Interval& b)
{
    return boost::numeric::hull(a, b);
}

/**
 * @brief Create an empty interval
 * @return An empty interval
 */
inline Interval empty_interval()
{
    return Interval(std::numeric_limits<double>::infinity(),
                    -std::numeric_limits<double>::infinity());
}

/**
 * @brief Format an interval as a string
 * @param i The interval
 * @param precision Number of decimal places
 * @return Formatted string "[lower, upper]"
 */
inline std::string fmt_interval(const Interval& i, int precision = 16)
{
    std::ostringstream ss;
    ss.precision(precision);
    ss << "[" << i.lower() << ", " << i.upper() << "]";
    return ss.str();
}

/**
 * @brief Format an interval vector as a string
 * @param x The interval vector
 * @param precision Number of decimal places
 * @return Formatted string
 */
inline std::string fmt_eigen_intervals(const VectorXI& x, int precision = 16)
{
    std::ostringstream ss;
    ss.precision(precision);
    ss << "[";
    for (int i = 0; i < x.size(); i++) {
        if (i > 0) ss << ", ";
        ss << "[" << x(i).lower() << ", " << x(i).upper() << "]";
    }
    ss << "]";
    return ss.str();
}

} // namespace NexDynIPC::Math

// ============================================================================
// Eigen traits for interval arithmetic
// ============================================================================

namespace Eigen {

template <typename BinOp>
struct ScalarBinaryOpTraits<NexDynIPC::Math::Interval, double, BinOp> {
    typedef NexDynIPC::Math::Interval ReturnType;
};

template <typename BinOp>
struct ScalarBinaryOpTraits<double, NexDynIPC::Math::Interval, BinOp> {
    typedef NexDynIPC::Math::Interval ReturnType;
};

#if EIGEN_VERSION_AT_LEAST(3, 3, 0)
namespace internal {
    template <typename X, typename S, typename P>
    struct is_convertible<X, boost::numeric::interval<S, P>> {
        enum { value = is_convertible<X, S>::value };
    };

    template <typename S, typename P1, typename P2>
    struct is_convertible<
        boost::numeric::interval<S, P1>,
        boost::numeric::interval<S, P2>> {
        enum { value = true };
    };
} // namespace internal
#endif

} // namespace Eigen

#endif // NEXDYNIPC_MATH_INTERVAL_HPP
