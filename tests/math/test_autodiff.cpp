/**
 * @file test_autodiff.cpp
 * @brief Unit tests for automatic differentiation (AutoDiff)
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <NexDynIPC/Math/AutoDiff.h>
#include <cmath>

using namespace NexDynIPC::Math;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

/**
 * @brief Test DiffScalarBase variable count management
 */
TEST_CASE("DiffScalarBase variable count management", "[math][autodiff][base]")
{
    SECTION("Set and get variable count")
    {
        DiffScalarBase::setVariableCount(5);
        REQUIRE(DiffScalarBase::getVariableCount() == 5);
        
        DiffScalarBase::setVariableCount(10);
        REQUIRE(DiffScalarBase::getVariableCount() == 10);
    }
    
    SECTION("Zero variable count")
    {
        DiffScalarBase::setVariableCount(0);
        REQUIRE(DiffScalarBase::getVariableCount() == 0);
    }
}

/**
 * @brief Test DScalar1 basic construction
 */
TEST_CASE("DScalar1 construction", "[math][autodiff][dscalar1]")
{
    DiffScalarBase::setVariableCount(3);
    
    SECTION("Default construction")
    {
        DScalar1<double> s;
        REQUIRE(s.getValue() == 0.0);
        REQUIRE(s.getGradient().size() == 3);
        REQUIRE(s.getGradient().isZero());
    }
    
    SECTION("Value construction")
    {
        DScalar1<double> s(5.0);
        REQUIRE(s.getValue() == 5.0);
        REQUIRE(s.getGradient().isZero());
    }
    
    SECTION("Variable construction")
    {
        DScalar1<double> s(1, 3.0);  // Variable 1 with value 3.0
        REQUIRE(s.getValue() == 3.0);
        REQUIRE(s.getGradient()(0) == 0.0);
        REQUIRE(s.getGradient()(1) == 1.0);
        REQUIRE(s.getGradient()(2) == 0.0);
    }
    
    SECTION("Copy construction")
    {
        DScalar1<double> s1(1, 2.0);
        DScalar1<double> s2(s1);
        REQUIRE(s2.getValue() == 2.0);
        REQUIRE(s2.getGradient().isApprox(s1.getGradient()));
    }
}

/**
 * @brief Test DScalar1 arithmetic operations
 */
TEST_CASE("DScalar1 arithmetic operations", "[math][autodiff][dscalar1]")
{
    DiffScalarBase::setVariableCount(2);
    
    DScalar1<double> x(0, 2.0);  // x = 2, dx/dx = 1
    DScalar1<double> y(1, 3.0);  // y = 3, dy/dy = 1
    
    SECTION("Addition")
    {
        auto sum = x + y;
        REQUIRE(sum.getValue() == 5.0);
        REQUIRE(sum.getGradient()(0) == 1.0);
        REQUIRE(sum.getGradient()(1) == 1.0);
        
        auto sum_scalar = x + 5.0;
        REQUIRE(sum_scalar.getValue() == 7.0);
        REQUIRE(sum_scalar.getGradient()(0) == 1.0);
    }
    
    SECTION("Subtraction")
    {
        auto diff = x - y;
        REQUIRE(diff.getValue() == -1.0);
        REQUIRE(diff.getGradient()(0) == 1.0);
        REQUIRE(diff.getGradient()(1) == -1.0);
        
        auto neg = -x;
        REQUIRE(neg.getValue() == -2.0);
        REQUIRE(neg.getGradient()(0) == -1.0);
    }
    
    SECTION("Multiplication")
    {
        auto prod = x * y;  // 2 * 3 = 6
        REQUIRE(prod.getValue() == 6.0);
        // d(xy)/dx = y = 3, d(xy)/dy = x = 2
        REQUIRE(prod.getGradient()(0) == 3.0);
        REQUIRE(prod.getGradient()(1) == 2.0);
        
        auto prod_scalar = x * 5.0;
        REQUIRE(prod_scalar.getValue() == 10.0);
        REQUIRE(prod_scalar.getGradient()(0) == 5.0);
    }
    
    SECTION("Division")
    {
        auto quot = x / 2.0;
        REQUIRE(quot.getValue() == 1.0);
        REQUIRE(quot.getGradient()(0) == 0.5);
    }
    
    SECTION("Compound assignment")
    {
        DScalar1<double> s(0, 1.0);
        s += x;
        REQUIRE(s.getValue() == 3.0);
        
        s -= y;
        REQUIRE(s.getValue() == 0.0);
        
        s *= 2.0;
        REQUIRE(s.getValue() == 0.0);
    }
}

/**
 * @brief Test DScalar1 comparison operators
 */
TEST_CASE("DScalar1 comparison operators", "[math][autodiff][dscalar1]")
{
    DiffScalarBase::setVariableCount(1);
    
    DScalar1<double> s1(0, 5.0);
    DScalar1<double> s2(0, 3.0);
    
    SECTION("DScalar1 comparisons")
    {
        REQUIRE(s1 > s2);
        REQUIRE(s1 >= s2);
        REQUIRE(s2 < s1);
        REQUIRE(s2 <= s1);
    }
    
    SECTION("Scalar comparisons")
    {
        REQUIRE(s1 > 4.0);
        REQUIRE(s1 >= 5.0);
        REQUIRE(s2 < 5.0);
        REQUIRE(s2 <= 3.0);
        REQUIRE(s1 == 5.0);
        REQUIRE(s1 != 3.0);
    }
}

/**
 * @brief Test DScalar1 mathematical functions
 */
TEST_CASE("DScalar1 mathematical functions", "[math][autodiff][dscalar1]")
{
    DiffScalarBase::setVariableCount(1);
    
    double x_val = GENERATE(0.5, 1.0, 2.0, 3.14);
    DScalar1<double> x(0, x_val);
    
    SECTION("sqrt")
    {
        auto result = sqrt(x);
        REQUIRE_THAT(result.getValue(), WithinAbs(std::sqrt(x_val), 1e-10));
        
        // d(sqrt(x))/dx = 1/(2*sqrt(x))
        double expected_grad = 1.0 / (2.0 * std::sqrt(x_val));
        REQUIRE_THAT(result.getGradient()(0), WithinAbs(expected_grad, 1e-10));
    }
    
    SECTION("pow")
    {
        auto result = pow(x, 2.0);
        REQUIRE_THAT(result.getValue(), WithinAbs(std::pow(x_val, 2.0), 1e-10));
        
        // d(x^2)/dx = 2*x
        double expected_grad = 2.0 * x_val;
        REQUIRE_THAT(result.getGradient()(0), WithinAbs(expected_grad, 1e-10));
    }
    
    SECTION("exp")
    {
        auto result = exp(x);
        REQUIRE_THAT(result.getValue(), WithinAbs(std::exp(x_val), 1e-10));
        REQUIRE_THAT(result.getGradient()(0), WithinAbs(std::exp(x_val), 1e-10));
    }
    
    SECTION("log")
    {
        auto result = log(x);
        REQUIRE_THAT(result.getValue(), WithinAbs(std::log(x_val), 1e-10));
        
        // d(log(x))/dx = 1/x
        double expected_grad = 1.0 / x_val;
        REQUIRE_THAT(result.getGradient()(0), WithinAbs(expected_grad, 1e-10));
    }
    
    SECTION("sin")
    {
        auto result = sin(x);
        REQUIRE_THAT(result.getValue(), WithinAbs(std::sin(x_val), 1e-10));
        
        // d(sin(x))/dx = cos(x)
        double expected_grad = std::cos(x_val);
        REQUIRE_THAT(result.getGradient()(0), WithinAbs(expected_grad, 1e-10));
    }
    
    SECTION("cos")
    {
        auto result = cos(x);
        REQUIRE_THAT(result.getValue(), WithinAbs(std::cos(x_val), 1e-10));
        
        // d(cos(x))/dx = -sin(x)
        double expected_grad = -std::sin(x_val);
        REQUIRE_THAT(result.getGradient()(0), WithinAbs(expected_grad, 1e-10));
    }
}

/**
 * @brief Test DScalar1 inverse and division
 */
TEST_CASE("DScalar1 inverse and division", "[math][autodiff][dscalar1]")
{
    DiffScalarBase::setVariableCount(1);
    
    DScalar1<double> x(0, 2.0);
    
    SECTION("inverse")
    {
        auto inv = inverse(x);
        REQUIRE(inv.getValue() == 0.5);
        
        // d(1/x)/dx = -1/x^2
        double expected_grad = -1.0 / 4.0;
        REQUIRE_THAT(inv.getGradient()(0), WithinAbs(expected_grad, 1e-10));
    }
    
    SECTION("division by zero throws")
    {
        REQUIRE_THROWS_AS(x / 0.0, std::runtime_error);
    }
}

/**
 * @brief Test DScalar1 trigonometric functions with domain checks
 */
TEST_CASE("DScalar1 trigonometric domain checks", "[math][autodiff][dscalar1]")
{
    DiffScalarBase::setVariableCount(1);
    
    SECTION("acos domain error")
    {
        DScalar1<double> x(0, 1.5);  // |x| > 1
        REQUIRE_THROWS_AS(acos(x), std::runtime_error);
        
        DScalar1<double> y(0, -1.5);
        REQUIRE_THROWS_AS(acos(y), std::runtime_error);
    }
    
    SECTION("asin domain error")
    {
        DScalar1<double> x(0, 1.5);
        REQUIRE_THROWS_AS(asin(x), std::runtime_error);
    }
    
    SECTION("acos valid input")
    {
        DScalar1<double> x(0, 0.5);
        auto result = acos(x);
        REQUIRE_THAT(result.getValue(), WithinAbs(std::acos(0.5), 1e-10));
    }
    
    SECTION("asin valid input")
    {
        DScalar1<double> x(0, 0.5);
        auto result = asin(x);
        REQUIRE_THAT(result.getValue(), WithinAbs(std::asin(0.5), 1e-10));
    }
}

/**
 * @brief Test DScalar1 atan2
 */
TEST_CASE("DScalar1 atan2", "[math][autodiff][dscalar1]")
{
    DiffScalarBase::setVariableCount(2);
    
    DScalar1<double> y(0, 1.0);
    DScalar1<double> x(1, 1.0);
    
    auto result = atan2(y, x);
    REQUIRE_THAT(result.getValue(), WithinAbs(std::atan2(1.0, 1.0), 1e-10));
    
    // d(atan2(y,x))/dy = x/(x^2+y^2)
    // d(atan2(y,x))/dx = -y/(x^2+y^2)
    double denom = 2.0;  // x^2 + y^2
    REQUIRE_THAT(result.getGradient()(0), WithinAbs(1.0 / denom, 1e-10));
    REQUIRE_THAT(result.getGradient()(1), WithinAbs(-1.0 / denom, 1e-10));
}

/**
 * @brief Test DScalar1 vector creation
 */
TEST_CASE("DScalar1 vector creation", "[math][autodiff][dscalar1]")
{
    DiffScalarBase::setVariableCount(2);
    
    SECTION("2D vector")
    {
        Eigen::Vector2d v(1.0, 2.0);
        auto dv = DScalar1<double>::vector(v);
        REQUIRE(dv(0).getValue() == 1.0);
        REQUIRE(dv(1).getValue() == 2.0);
    }
    
    SECTION("3D vector")
    {
        Eigen::Vector3d v(1.0, 2.0, 3.0);
        auto dv = DScalar1<double>::vector(v);
        REQUIRE(dv(0).getValue() == 1.0);
        REQUIRE(dv(1).getValue() == 2.0);
        REQUIRE(dv(2).getValue() == 3.0);
    }
}

// =============================================================================
// DScalar2 Tests
// =============================================================================

/**
 * @brief Test DScalar2 basic construction
 */
TEST_CASE("DScalar2 construction", "[math][autodiff][dscalar2]")
{
    DiffScalarBase::setVariableCount(2);
    
    SECTION("Default construction")
    {
        DScalar2<double> s;
        REQUIRE(s.getValue() == 0.0);
        REQUIRE(s.getGradient().size() == 2);
        REQUIRE(s.getGradient().isZero());
        REQUIRE(s.getHessian().rows() == 2);
        REQUIRE(s.getHessian().cols() == 2);
        REQUIRE(s.getHessian().isZero());
    }
    
    SECTION("Variable construction")
    {
        DScalar2<double> s(0, 3.0);
        REQUIRE(s.getValue() == 3.0);
        REQUIRE(s.getGradient()(0) == 1.0);
        REQUIRE(s.getGradient()(1) == 0.0);
        REQUIRE(s.getHessian().isZero());
    }
}

/**
 * @brief Test DScalar2 arithmetic operations
 */
TEST_CASE("DScalar2 arithmetic operations", "[math][autodiff][dscalar2]")
{
    DiffScalarBase::setVariableCount(2);
    
    DScalar2<double> x(0, 2.0);
    DScalar2<double> y(1, 3.0);
    
    SECTION("Addition")
    {
        auto sum = x + y;
        REQUIRE(sum.getValue() == 5.0);
        REQUIRE(sum.getGradient()(0) == 1.0);
        REQUIRE(sum.getGradient()(1) == 1.0);
        REQUIRE(sum.getHessian().isZero());
    }
    
    SECTION("Multiplication")
    {
        auto prod = x * y;
        REQUIRE(prod.getValue() == 6.0);
        // d^2(xy)/dx^2 = 0, d^2(xy)/dy^2 = 0, d^2(xy)/dxdy = 1
        REQUIRE(prod.getHessian()(0, 1) == 1.0);
        REQUIRE(prod.getHessian()(1, 0) == 1.0);
    }
}

/**
 * @brief Test DScalar2 mathematical functions
 */
TEST_CASE("DScalar2 mathematical functions", "[math][autodiff][dscalar2]")
{
    DiffScalarBase::setVariableCount(1);
    
    double x_val = 2.0;
    DScalar2<double> x(0, x_val);
    
    SECTION("sqrt")
    {
        auto result = sqrt(x);
        REQUIRE_THAT(result.getValue(), WithinAbs(std::sqrt(x_val), 1e-10));
        
        // d^2(sqrt(x))/dx^2 = -1/(4*x^(3/2))
        double expected_hessian = -1.0 / (4.0 * std::pow(x_val, 1.5));
        REQUIRE_THAT(result.getHessian()(0, 0), WithinAbs(expected_hessian, 1e-10));
    }
    
    SECTION("exp")
    {
        auto result = exp(x);
        double exp_val = std::exp(x_val);
        REQUIRE_THAT(result.getValue(), WithinAbs(exp_val, 1e-10));
        
        // d^2(exp(x))/dx^2 = exp(x)
        REQUIRE_THAT(result.getHessian()(0, 0), WithinAbs(exp_val, 1e-10));
    }
    
    SECTION("sin")
    {
        auto result = sin(x);
        REQUIRE_THAT(result.getValue(), WithinAbs(std::sin(x_val), 1e-10));
        
        // d^2(sin(x))/dx^2 = -sin(x)
        double expected_hessian = -std::sin(x_val);
        REQUIRE_THAT(result.getHessian()(0, 0), WithinAbs(expected_hessian, 1e-10));
    }
    
    SECTION("cos")
    {
        auto result = cos(x);
        REQUIRE_THAT(result.getValue(), WithinAbs(std::cos(x_val), 1e-10));
        
        // d^2(cos(x))/dx^2 = -cos(x)
        double expected_hessian = -std::cos(x_val);
        REQUIRE_THAT(result.getHessian()(0, 0), WithinAbs(expected_hessian, 1e-10));
    }
}

/**
 * @brief Test DScalar2 inverse
 */
TEST_CASE("DScalar2 inverse", "[math][autodiff][dscalar2]")
{
    DiffScalarBase::setVariableCount(1);
    
    DScalar2<double> x(0, 2.0);
    auto inv = inverse(x);
    
    REQUIRE(inv.getValue() == 0.5);
    
    // d^2(1/x)/dx^2 = 2/x^3
    double expected_hessian = 2.0 / 8.0;
    REQUIRE_THAT(inv.getHessian()(0, 0), WithinAbs(expected_hessian, 1e-10));
}

/**
 * @brief Test DScalar2 pow
 */
TEST_CASE("DScalar2 pow", "[math][autodiff][dscalar2]")
{
    DiffScalarBase::setVariableCount(1);
    
    DScalar2<double> x(0, 2.0);
    auto result = pow(x, 3.0);
    
    REQUIRE(result.getValue() == 8.0);
    
    // d^2(x^3)/dx^2 = 6*x
    double expected_hessian = 6.0 * 2.0;
    REQUIRE_THAT(result.getHessian()(0, 0), WithinAbs(expected_hessian, 1e-10));
}

/**
 * @brief Test complex expression with DScalar1
 */
TEST_CASE("DScalar1 complex expression", "[math][autodiff][dscalar1]")
{
    DiffScalarBase::setVariableCount(2);
    
    // f(x,y) = x^2 + 2xy + y^2 at x=2, y=3
    DScalar1<double> x(0, 2.0);
    DScalar1<double> y(1, 3.0);
    
    auto f = x * x + 2.0 * x * y + y * y;
    
    // f(2,3) = 4 + 12 + 9 = 25
    REQUIRE(f.getValue() == 25.0);
    
    // df/dx = 2x + 2y = 4 + 6 = 10
    REQUIRE(f.getGradient()(0) == 10.0);
    
    // df/dy = 2x + 2y = 4 + 6 = 10
    REQUIRE(f.getGradient()(1) == 10.0);
}

/**
 * @brief Test complex expression with DScalar2
 */
TEST_CASE("DScalar2 complex expression", "[math][autodiff][dscalar2]")
{
    DiffScalarBase::setVariableCount(2);
    
    // f(x,y) = x^2 * y at x=2, y=3
    DScalar2<double> x(0, 2.0);
    DScalar2<double> y(1, 3.0);
    
    auto f = x * x * y;
    
    // f(2,3) = 4 * 3 = 12
    REQUIRE(f.getValue() == 12.0);
    
    // df/dx = 2xy = 12
    REQUIRE(f.getGradient()(0) == 12.0);
    
    // df/dy = x^2 = 4
    REQUIRE(f.getGradient()(1) == 4.0);
    
    // d^2f/dx^2 = 2y = 6
    REQUIRE(f.getHessian()(0, 0) == 6.0);
    
    // d^2f/dy^2 = 0
    REQUIRE(f.getHessian()(1, 1) == 0.0);
    
    // d^2f/dxdy = 2x = 4
    REQUIRE(f.getHessian()(0, 1) == 4.0);
    REQUIRE(f.getHessian()(1, 0) == 4.0);
}
