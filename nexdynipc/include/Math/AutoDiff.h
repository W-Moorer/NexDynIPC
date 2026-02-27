/**
 * @file AutoDiff.h
 * @brief Automatic differentiation data type for C++
 * 
 * This library provides automatic differentiation capabilities for computing
 * gradients and Hessians of scalar functions. It depends on the Eigen
 * linear algebra library.
 * 
 * Copyright (c) 2012 by Wenzel Jakob. Based on code by Jon Kaldor
 * and Eitan Grinspun.
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#ifndef NEXDYNIPC_MATH_AUTODIFF_HPP
#define NEXDYNIPC_MATH_AUTODIFF_HPP

#define EIGEN_DONT_PARALLELIZE
#define EIGEN_NO_DEBUG

#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace NexDynIPC::Math {

/**
 * @brief Base class of all automatic differentiation types
 * 
 * This class records the number of independent variables with respect
 * to which derivatives are computed.
 */
struct DiffScalarBase {
    /**
     * @brief Set the independent variable count used by the automatic
     * differentiation layer
     * 
     * This function must be called before doing any computations with
     * DScalar1 or DScalar2. The value will be recorded in thread-local storage.
     * 
     * @param value Number of independent variables
     */
    static inline void setVariableCount(size_t value) {
        m_variableCount = value;
    }

    /**
     * @brief Get the variable count used by the automatic differentiation layer
     * @return Number of independent variables
     */
    static inline size_t getVariableCount() {
        return m_variableCount;
    }

    static thread_local size_t m_variableCount;
};

} // namespace NexDynIPC::Math

/**
 * @brief Macro to declare the thread-local variable for DiffScalarBase
 */
#define DECLARE_DIFFSCALAR_BASE() \
    thread_local size_t NexDynIPC::Math::DiffScalarBase::m_variableCount = 0

namespace NexDynIPC::Math {

/**
 * @brief Automatic differentiation scalar with first-order derivatives
 * 
 * This class provides an instrumented "scalar" value, which may be dependent on
 * a number of independent variables. The implementation keeps tracks of
 * first-order derivatives with respect to these variables using a set
 * of overloaded operations and implementations of special functions (sin,
 * tan, exp, ..).
 * 
 * This is extremely useful for numerical zero-finding, particularly when
 * analytic derivatives from programs like Maple or Mathematica suffer from
 * excessively complicated expressions.
 * 
 * The class relies on templates, which makes it possible to fix the
 * number of independent variables at compile-time so that instances can
 * be allocated on the stack. Otherwise, they will be placed on the heap.
 * 
 * @tparam _Scalar The underlying scalar type (typically double)
 * @tparam _Gradient The gradient vector type
 * 
 * @sa DScalar2
 * @author Wenzel Jakob
 */
template <typename _Scalar, typename _Gradient = Eigen::Matrix<_Scalar, Eigen::Dynamic, 1>>
struct DScalar1 : public DiffScalarBase {
public:
    typedef _Scalar Scalar;
    typedef _Gradient Gradient;
    typedef Eigen::Matrix<DScalar1, 2, 1> DVector2;
    typedef Eigen::Matrix<DScalar1, 3, 1> DVector3;

    explicit DScalar1(Scalar value_ = (Scalar)0) : value(value_) {
        size_t variableCount = getVariableCount();
        grad.resize(variableCount);
        grad.setZero();
    }

    DScalar1(size_t index, const Scalar& value_) : value(value_) {
        size_t variableCount = getVariableCount();
        grad.resize(variableCount);
        grad.setZero();
        grad(index) = 1;
    }

    DScalar1(Scalar value_, const Gradient& grad_) : value(value_), grad(grad_) {}

    DScalar1(const DScalar1& s) : value(s.value), grad(s.grad) {}

    inline const Scalar& getValue() const { return value; }
    inline const Gradient& getGradient() const { return grad; }

    friend DScalar1 operator+(const DScalar1& lhs, const DScalar1& rhs) {
        return DScalar1(lhs.value + rhs.value, lhs.grad + rhs.grad);
    }

    friend DScalar1 operator+(const DScalar1& lhs, const Scalar& rhs) {
        return DScalar1(lhs.value + rhs, lhs.grad);
    }

    friend DScalar1 operator+(const Scalar& lhs, const DScalar1& rhs) {
        return DScalar1(rhs.value + lhs, rhs.grad);
    }

    inline DScalar1& operator+=(const DScalar1& s) {
        value += s.value;
        grad += s.grad;
        return *this;
    }

    inline DScalar1& operator+=(const Scalar& v) {
        value += v;
        return *this;
    }

    friend DScalar1 operator-(const DScalar1& lhs, const DScalar1& rhs) {
        return DScalar1(lhs.value - rhs.value, lhs.grad - rhs.grad);
    }

    friend DScalar1 operator-(const DScalar1& lhs, const Scalar& rhs) {
        return DScalar1(lhs.value - rhs, lhs.grad);
    }

    friend DScalar1 operator-(const Scalar& lhs, const DScalar1& rhs) {
        return DScalar1(lhs - rhs.value, -rhs.grad);
    }

    friend DScalar1 operator-(const DScalar1& s) {
        return DScalar1(-s.value, -s.grad);
    }

    inline DScalar1& operator-=(const DScalar1& s) {
        value -= s.value;
        grad -= s.grad;
        return *this;
    }

    inline DScalar1& operator-=(const Scalar& v) {
        value -= v;
        return *this;
    }

    friend DScalar1 operator/(const DScalar1& lhs, const Scalar& rhs) {
        if (rhs == 0)
            throw std::runtime_error("DScalar1: Division by zero!");
        Scalar inv = 1.0f / rhs;
        return DScalar1(lhs.value * inv, lhs.grad * inv);
    }

    friend DScalar1 operator/(const Scalar& lhs, const DScalar1& rhs) {
        return lhs * inverse(rhs);
    }

    friend DScalar1 operator/(const DScalar1& lhs, const DScalar1& rhs) {
        return lhs * inverse(rhs);
    }

    friend DScalar1 inverse(const DScalar1& s) {
        Scalar valueSqr = s.value * s.value, invValueSqr = (Scalar)1 / valueSqr;
        return DScalar1((Scalar)1 / s.value, s.grad * -invValueSqr);
    }

    inline DScalar1& operator/=(const Scalar& v) {
        value /= v;
        grad /= v;
        return *this;
    }

    inline friend DScalar1 operator*(const DScalar1& lhs, const Scalar& rhs) {
        return DScalar1(lhs.value * rhs, lhs.grad * rhs);
    }

    inline friend DScalar1 operator*(const Scalar& lhs, const DScalar1& rhs) {
        return DScalar1(rhs.value * lhs, rhs.grad * lhs);
    }

    inline friend DScalar1 operator*(const DScalar1& lhs, const DScalar1& rhs) {
        return DScalar1(lhs.value * rhs.value,
                        rhs.grad * lhs.value + lhs.grad * rhs.value);
    }

    inline DScalar1& operator*=(const Scalar& v) {
        value *= v;
        grad *= v;
        return *this;
    }

    friend DScalar1 sqrt(const DScalar1& s) {
        Scalar sqrtVal = std::sqrt(s.value), temp = (Scalar)1 / ((Scalar)2 * sqrtVal);
        return DScalar1(sqrtVal, s.grad * temp);
    }

    friend DScalar1 pow(const DScalar1& s, const Scalar& a) {
        Scalar powVal = std::pow(s.value, a), temp = a * std::pow(s.value, a - 1);
        return DScalar1(powVal, s.grad * temp);
    }

    friend DScalar1 exp(const DScalar1& s) {
        Scalar expVal = std::exp(s.value);
        return DScalar1(expVal, s.grad * expVal);
    }

    friend DScalar1 log(const DScalar1& s) {
        Scalar logVal = std::log(s.value);
        return DScalar1(logVal, s.grad / s.value);
    }

    friend DScalar1 sin(const DScalar1& s) {
        return DScalar1(std::sin(s.value), s.grad * std::cos(s.value));
    }

    friend DScalar1 cos(const DScalar1& s) {
        return DScalar1(std::cos(s.value), s.grad * -std::sin(s.value));
    }

    friend DScalar1 acos(const DScalar1& s) {
        if (std::abs(s.value) >= 1)
            throw std::runtime_error("acos: Expected a value in (-1, 1)");
        Scalar temp = -std::sqrt((Scalar)1 - s.value * s.value);
        return DScalar1(std::acos(s.value), s.grad * ((Scalar)1 / temp));
    }

    friend DScalar1 asin(const DScalar1& s) {
        if (std::abs(s.value) >= 1)
            throw std::runtime_error("asin: Expected a value in (-1, 1)");
        Scalar temp = std::sqrt((Scalar)1 - s.value * s.value);
        return DScalar1(std::asin(s.value), s.grad * ((Scalar)1 / temp));
    }

    friend DScalar1 atan2(const DScalar1& y, const DScalar1& x) {
        Scalar denom = x.value * x.value + y.value * y.value;
        return DScalar1(std::atan2(y.value, x.value),
                        y.grad * (x.value / denom) - x.grad * (y.value / denom));
    }

    inline void operator=(const DScalar1& s) { value = s.value; grad = s.grad; }
    inline void operator=(const Scalar& v) { value = v; grad.setZero(); }
    inline bool operator<(const DScalar1& s) const { return value < s.value; }
    inline bool operator<=(const DScalar1& s) const { return value <= s.value; }
    inline bool operator>(const DScalar1& s) const { return value > s.value; }
    inline bool operator>=(const DScalar1& s) const { return value >= s.value; }
    inline bool operator<(const Scalar& s) const { return value < s; }
    inline bool operator<=(const Scalar& s) const { return value <= s; }
    inline bool operator>(const Scalar& s) const { return value > s; }
    inline bool operator>=(const Scalar& s) const { return value >= s; }
    inline bool operator==(const Scalar& s) const { return value == s; }
    inline bool operator!=(const Scalar& s) const { return value != s; }

    static inline DVector2 vector(const Eigen::Matrix<Scalar, 2, 1>& v) {
        return DVector2(DScalar1(v.x()), DScalar1(v.y()));
    }

    static inline DVector3 vector(const Eigen::Matrix<Scalar, 3, 1>& v) {
        return DVector3(DScalar1(v.x()), DScalar1(v.y()), DScalar1(v.z()));
    }

protected:
    Scalar value;
    Gradient grad;
};

template <typename Scalar, typename VecType>
std::ostream& operator<<(std::ostream& out, const DScalar1<Scalar, VecType>& s) {
    out << "[" << s.getValue()
        << ", grad=" << s.getGradient().format(Eigen::IOFormat(4, 1, ", ", "; ", "", "", "[", "]"))
        << "]";
    return out;
}

/**
 * @brief Automatic differentiation scalar with first- and second-order derivatives
 */
template <typename _Scalar, typename _Gradient = Eigen::Matrix<_Scalar, Eigen::Dynamic, 1>,
          typename _Hessian = Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic>>
struct DScalar2 : public DiffScalarBase {
public:
    typedef _Scalar Scalar;
    typedef _Gradient Gradient;
    typedef _Hessian Hessian;
    typedef Eigen::Matrix<DScalar2, 2, 1> DVector2;
    typedef Eigen::Matrix<DScalar2, 3, 1> DVector3;

    explicit DScalar2(Scalar value_ = (Scalar)0) : value(value_) {
        size_t variableCount = getVariableCount();
        grad.resize(variableCount);
        grad.setZero();
        hess.resize(variableCount, variableCount);
        hess.setZero();
    }

    DScalar2(size_t index, const Scalar& value_) : value(value_) {
        size_t variableCount = getVariableCount();
        grad.resize(variableCount);
        grad.setZero();
        grad(index) = 1;
        hess.resize(variableCount, variableCount);
        hess.setZero();
    }

    DScalar2(Scalar value_, const Gradient& grad_, const Hessian& hess_)
        : value(value_), grad(grad_), hess(hess_) {}

    DScalar2(const DScalar2& s) : value(s.value), grad(s.grad), hess(s.hess) {}

    inline const Scalar& getValue() const { return value; }
    inline const Gradient& getGradient() const { return grad; }
    inline const Hessian& getHessian() const { return hess; }

    friend DScalar2 operator+(const DScalar2& lhs, const DScalar2& rhs) {
        return DScalar2(lhs.value + rhs.value, lhs.grad + rhs.grad, lhs.hess + rhs.hess);
    }

    friend DScalar2 operator+(const DScalar2& lhs, const Scalar& rhs) {
        return DScalar2(lhs.value + rhs, lhs.grad, lhs.hess);
    }

    friend DScalar2 operator+(const Scalar& lhs, const DScalar2& rhs) {
        return DScalar2(rhs.value + lhs, rhs.grad, rhs.hess);
    }

    inline DScalar2& operator+=(const DScalar2& s) {
        value += s.value;
        grad += s.grad;
        hess += s.hess;
        return *this;
    }

    inline DScalar2& operator+=(const Scalar& v) {
        value += v;
        return *this;
    }

    friend DScalar2 operator-(const DScalar2& lhs, const DScalar2& rhs) {
        return DScalar2(lhs.value - rhs.value, lhs.grad - rhs.grad, lhs.hess - rhs.hess);
    }

    friend DScalar2 operator-(const DScalar2& lhs, const Scalar& rhs) {
        return DScalar2(lhs.value - rhs, lhs.grad, lhs.hess);
    }

    friend DScalar2 operator-(const Scalar& lhs, const DScalar2& rhs) {
        return DScalar2(lhs - rhs.value, -rhs.grad, -rhs.hess);
    }

    friend DScalar2 operator-(const DScalar2& s) {
        return DScalar2(-s.value, -s.grad, -s.hess);
    }

    inline DScalar2& operator-=(const DScalar2& s) {
        value -= s.value;
        grad -= s.grad;
        hess -= s.hess;
        return *this;
    }

    inline DScalar2& operator-=(const Scalar& v) {
        value -= v;
        return *this;
    }

    friend DScalar2 operator/(const DScalar2& lhs, const Scalar& rhs) {
        if (rhs == 0)
            throw std::runtime_error("DScalar2: Division by zero!");
        Scalar inv = 1.0f / rhs;
        return DScalar2(lhs.value * inv, lhs.grad * inv, lhs.hess * inv);
    }

    friend DScalar2 operator/(const Scalar& lhs, const DScalar2& rhs) {
        return lhs * inverse(rhs);
    }

    friend DScalar2 operator/(const DScalar2& lhs, const DScalar2& rhs) {
        return lhs * inverse(rhs);
    }

    friend DScalar2 inverse(const DScalar2& s) {
        Scalar valueSqr = s.value * s.value, valueCub = valueSqr * s.value,
               invValueSqr = (Scalar)1 / valueSqr;

        DScalar2 result((Scalar)1 / s.value);
        result.grad = s.grad * -invValueSqr;
        result.hess = s.hess * -invValueSqr;
        result.hess += s.grad * s.grad.transpose() * ((Scalar)2 / valueCub);

        return result;
    }

    inline DScalar2& operator/=(const Scalar& v) {
        value /= v;
        grad /= v;
        hess /= v;
        return *this;
    }

    friend DScalar2 operator*(const DScalar2& lhs, const Scalar& rhs) {
        return DScalar2(lhs.value * rhs, lhs.grad * rhs, lhs.hess * rhs);
    }

    friend DScalar2 operator*(const Scalar& lhs, const DScalar2& rhs) {
        return DScalar2(rhs.value * lhs, rhs.grad * lhs, rhs.hess * lhs);
    }

    friend DScalar2 operator*(const DScalar2& lhs, const DScalar2& rhs) {
        DScalar2 result(lhs.value * rhs.value);
        result.grad = rhs.grad * lhs.value + lhs.grad * rhs.value;
        result.hess = rhs.hess * lhs.value;
        result.hess += lhs.hess * rhs.value;
        result.hess += lhs.grad * rhs.grad.transpose();
        result.hess += rhs.grad * lhs.grad.transpose();
        return result;
    }

    inline DScalar2& operator*=(const Scalar& v) {
        value *= v;
        grad *= v;
        hess *= v;
        return *this;
    }

    friend DScalar2 sqrt(const DScalar2& s) {
        Scalar sqrtVal = std::sqrt(s.value), temp = (Scalar)1 / ((Scalar)2 * sqrtVal);
        DScalar2 result(sqrtVal);
        result.grad = s.grad * temp;
        result.hess = s.hess * temp;
        result.hess += s.grad * s.grad.transpose() * (-(Scalar)1 / ((Scalar)4 * s.value * sqrtVal));
        return result;
    }

    friend DScalar2 pow(const DScalar2& s, const Scalar& a) {
        Scalar powVal = std::pow(s.value, a), temp = a * std::pow(s.value, a - 1);
        DScalar2 result(powVal);
        result.grad = s.grad * temp;
        result.hess = s.hess * temp;
        result.hess += s.grad * s.grad.transpose() * (a * (a - 1) * std::pow(s.value, a - 2));
        return result;
    }

    friend DScalar2 exp(const DScalar2& s) {
        Scalar expVal = std::exp(s.value);
        DScalar2 result(expVal);
        result.grad = s.grad * expVal;
        result.hess = (s.grad * s.grad.transpose() + s.hess) * expVal;
        return result;
    }

    friend DScalar2 log(const DScalar2& s) {
        Scalar logVal = std::log(s.value);
        DScalar2 result(logVal);
        result.grad = s.grad / s.value;
        result.hess = s.hess / s.value - (s.grad * s.grad.transpose() / (s.value * s.value));
        return result;
    }

    friend DScalar2 sin(const DScalar2& s) {
        Scalar sinVal = std::sin(s.value), cosVal = std::cos(s.value);
        DScalar2 result(sinVal);
        result.grad = s.grad * cosVal;
        result.hess = s.hess * cosVal;
        result.hess += s.grad * s.grad.transpose() * -sinVal;
        return result;
    }

    friend DScalar2 cos(const DScalar2& s) {
        Scalar sinVal = std::sin(s.value), cosVal = std::cos(s.value);
        DScalar2 result(cosVal);
        result.grad = s.grad * -sinVal;
        result.hess = s.hess * -sinVal;
        result.hess += s.grad * s.grad.transpose() * -cosVal;
        return result;
    }

    friend DScalar2 acos(const DScalar2& s) {
        if (std::abs(s.value) >= 1)
            throw std::runtime_error("acos: Expected a value in (-1, 1)");
        Scalar temp = -std::sqrt((Scalar)1 - s.value * s.value);
        DScalar2 result(std::acos(s.value));
        result.grad = s.grad * ((Scalar)1 / temp);
        result.hess = s.hess * ((Scalar)1 / temp);
        result.hess += s.grad * s.grad.transpose() * s.value / (temp * temp * temp);
        return result;
    }

    friend DScalar2 asin(const DScalar2& s) {
        if (std::abs(s.value) >= 1)
            throw std::runtime_error("asin: Expected a value in (-1, 1)");
        Scalar temp = std::sqrt((Scalar)1 - s.value * s.value);
        DScalar2 result(std::asin(s.value));
        result.grad = s.grad * ((Scalar)1 / temp);
        result.hess = s.hess * ((Scalar)1 / temp);
        result.hess += s.grad * s.grad.transpose() * s.value / (temp * temp * temp);
        return result;
    }

    friend DScalar2 atan2(const DScalar2& y, const DScalar2& x) {
        DScalar2 result(std::atan2(y.value, x.value));
        Scalar denom = x.value * x.value + y.value * y.value, denomSqr = denom * denom;
        result.grad = y.grad * (x.value / denom) - x.grad * (y.value / denom);
        result.hess = (y.hess * x.value + y.grad * x.grad.transpose() - x.hess * y.value -
                       x.grad * y.grad.transpose()) / denom;
        result.hess -= (y.grad * (x.value / denomSqr) - x.grad * (y.value / denomSqr)) *
                       (x.grad * ((Scalar)2 * x.value) + y.grad * ((Scalar)2 * y.value)).transpose();
        return result;
    }

    inline void operator=(const DScalar2& s) { value = s.value; grad = s.grad; hess = s.hess; }
    inline void operator=(const Scalar& v) { value = v; grad.setZero(); hess.setZero(); }
    inline bool operator<(const DScalar2& s) const { return value < s.value; }
    inline bool operator<=(const DScalar2& s) const { return value <= s.value; }
    inline bool operator>(const DScalar2& s) const { return value > s.value; }
    inline bool operator>=(const DScalar2& s) const { return value >= s.value; }
    inline bool operator<(const Scalar& s) const { return value < s; }
    inline bool operator<=(const Scalar& s) const { return value <= s; }
    inline bool operator>(const Scalar& s) const { return value > s; }
    inline bool operator>=(const Scalar& s) const { return value >= s; }
    inline bool operator==(const Scalar& s) const { return value == s; }
    inline bool operator!=(const Scalar& s) const { return value != s; }

    static inline DVector2 vector(const Eigen::Matrix<Scalar, 2, 1>& v) {
        return DVector2(DScalar2(v.x()), DScalar2(v.y()));
    }

    static inline DVector3 vector(const Eigen::Matrix<Scalar, 3, 1>& v) {
        return DVector3(DScalar2(v.x()), DScalar2(v.y()), DScalar2(v.z()));
    }

protected:
    Scalar value;
    Gradient grad;
    Hessian hess;
};

template <typename Scalar, typename VecType, typename MatType>
std::ostream& operator<<(std::ostream& out, const DScalar2<Scalar, VecType, MatType>& s) {
    out << "[" << s.getValue()
        << ", grad=" << s.getGradient().format(Eigen::IOFormat(4, 1, ", ", "; ", "", "", "[", "]"))
        << ", hess=" << s.getHessian().format(Eigen::IOFormat(4, 0, ", ", "; ", "", "", "[", "]"))
        << "]";
    return out;
}

} // namespace NexDynIPC::Math

namespace Eigen {

template <typename Scalar, typename Gradient, typename BinOp>
struct ScalarBinaryOpTraits<NexDynIPC::Math::DScalar1<Scalar, Gradient>, Scalar, BinOp> {
    typedef NexDynIPC::Math::DScalar1<Scalar, Gradient> ReturnType;
};

template <typename Scalar, typename Gradient, typename BinOp>
struct ScalarBinaryOpTraits<Scalar, NexDynIPC::Math::DScalar1<Scalar, Gradient>, BinOp> {
    typedef NexDynIPC::Math::DScalar1<Scalar, Gradient> ReturnType;
};

template <typename Scalar, typename Gradient, typename Hessian, typename BinOp>
struct ScalarBinaryOpTraits<NexDynIPC::Math::DScalar2<Scalar, Gradient, Hessian>, Scalar, BinOp> {
    typedef NexDynIPC::Math::DScalar2<Scalar, Gradient, Hessian> ReturnType;
};

template <typename Scalar, typename Gradient, typename Hessian, typename BinOp>
struct ScalarBinaryOpTraits<Scalar, NexDynIPC::Math::DScalar2<Scalar, Gradient, Hessian>, BinOp> {
    typedef NexDynIPC::Math::DScalar2<Scalar, Gradient, Hessian> ReturnType;
};

} // namespace Eigen

#endif // NEXDYNIPC_MATH_AUTODIFF_HPP
