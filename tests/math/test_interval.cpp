/**
 * @file test_interval.cpp
 * @brief Unit tests for interval arithmetic
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <NexDynIPC/Math/Interval.h>
#include <cmath>

using namespace NexDynIPC::Math;
using Catch::Matchers::WithinAbs;

/**
 * @brief Test basic interval construction
 */
TEST_CASE("Interval construction", "[math][interval]")
{
    SECTION("Default construction")
    {
        Interval i;
        REQUIRE(i.lower() == 0.0);
        REQUIRE(i.upper() == 0.0);
    }
    
    SECTION("Single value construction")
    {
        Interval i(5.0);
        REQUIRE(i.lower() == 5.0);
        REQUIRE(i.upper() == 5.0);
    }
    
    SECTION("Range construction")
    {
        Interval i(1.0, 5.0);
        REQUIRE(i.lower() == 1.0);
        REQUIRE(i.upper() == 5.0);
    }
    
    SECTION("Reversed bounds produces empty interval")
    {
        Interval i(5.0, 1.0);
        REQUIRE(is_empty(i));
    }
}

/**
 * @brief Test interval arithmetic operations
 */
TEST_CASE("Interval arithmetic operations", "[math][interval]")
{
    Interval i(0, 1);
    Interval j(4, 5);
    
    SECTION("Addition")
    {
        Interval result = i + j;
        REQUIRE(result.lower() == 4.0);
        REQUIRE(result.upper() == 6.0);
        
        result = i + 10.0;
        REQUIRE(result.lower() == 10.0);
        REQUIRE(result.upper() == 11.0);
    }
    
    SECTION("Subtraction")
    {
        Interval result = i - j;
        REQUIRE(result.lower() == -5.0);
        REQUIRE(result.upper() == -3.0);
        
        result = j - i;
        REQUIRE(result.lower() == 3.0);
        REQUIRE(result.upper() == 5.0);
    }
    
    SECTION("Multiplication")
    {
        Interval result = i * j;
        REQUIRE(result.lower() == 0.0);
        REQUIRE(result.upper() == 5.0);
        
        Interval a(-1, 1);
        Interval b(2, 3);
        result = a * b;
        REQUIRE(result.lower() == -3.0);
        REQUIRE(result.upper() == 3.0);
    }
    
    SECTION("Division")
    {
        Interval result = 1.0 / j;
        REQUIRE_THAT(result.lower(), WithinAbs(0.2, 1e-10));
        REQUIRE_THAT(result.upper(), WithinAbs(0.25, 1e-10));
        
        result = i / j;
        REQUIRE(result.lower() >= 0.0);
        REQUIRE(result.upper() <= 0.25);
    }
    
    SECTION("Division by zero throws or produces empty interval")
    {
        Interval zero(0, 0);
        Interval result = i / zero;
        REQUIRE(is_empty(result));
    }
}

/**
 * @brief Test interval utility functions
 */
TEST_CASE("Interval utility functions", "[math][interval]")
{
    Interval i(1.0, 5.0);
    
    SECTION("Width")
    {
        REQUIRE(width(i) == 4.0);
    }
    
    SECTION("Midpoint")
    {
        REQUIRE(midpoint(i) == 3.0);
    }
    
    SECTION("Lower and upper")
    {
        REQUIRE(lower(i) == 1.0);
        REQUIRE(upper(i) == 5.0);
    }
    
    SECTION("Contains")
    {
        REQUIRE(contains(i, 1.0));
        REQUIRE(contains(i, 3.0));
        REQUIRE(contains(i, 5.0));
        REQUIRE_FALSE(contains(i, 0.0));
        REQUIRE_FALSE(contains(i, 6.0));
    }
    
    SECTION("Zero in")
    {
        REQUIRE_FALSE(zero_in(i));
        
        Interval j(-1, 1);
        REQUIRE(zero_in(j));
        
        Interval k(0, 5);
        REQUIRE(zero_in(k));
    }
}

/**
 * @brief Test interval mathematical functions
 */
TEST_CASE("Interval mathematical functions", "[math][interval]")
{
    SECTION("Square root")
    {
        Interval i(1, 4);
        Interval result = sqrt(i);
        REQUIRE(result.lower() == 1.0);
        REQUIRE(result.upper() == 2.0);
    }
    
    SECTION("Square root of zero")
    {
        Interval i(0, 0);
        Interval result = sqrt(i);
        REQUIRE(result.lower() == 0.0);
        REQUIRE(result.upper() == 0.0);
    }
    
    SECTION("Exponential")
    {
        Interval i(0, 1);
        Interval result = exp(i);
        REQUIRE_THAT(result.lower(), WithinAbs(1.0, 1e-10));
        REQUIRE_THAT(result.upper(), WithinAbs(std::exp(1.0), 1e-10));
    }
    
    SECTION("Logarithm")
    {
        Interval i(1, std::exp(1.0));
        Interval result = log(i);
        REQUIRE_THAT(result.lower(), WithinAbs(0.0, 1e-10));
        REQUIRE_THAT(result.upper(), WithinAbs(1.0, 1e-10));
    }
    
    SECTION("Power")
    {
        Interval i(1, 2);
        Interval result = pow(i, 2);  // Use integer exponent
        REQUIRE(result.lower() == 1.0);
        REQUIRE(result.upper() == 4.0);
    }
}

/**
 * @brief Test trigonometric interval functions
 */
TEST_CASE("Interval trigonometric functions", "[math][interval]")
{
    const double PI = 3.14159265358979323846;
    
    SECTION("Cosine - full range")
    {
        Interval i(-1, 7);
        Interval result = cos(i);
        REQUIRE_THAT(result.lower(), WithinAbs(-1.0, 1e-10));
        REQUIRE_THAT(result.upper(), WithinAbs(1.0, 1e-10));
    }
    
    SECTION("Cosine - positive range")
    {
        Interval i(0, 1);
        Interval result = cos(i);
        REQUIRE(result.lower() > 0);
        REQUIRE(result.upper() <= 1.0);
    }
    
    SECTION("Cosine - negative range")
    {
        Interval i(2, 4);
        Interval result = cos(i);
        REQUIRE(result.upper() < 0);
    }
    
    SECTION("Sine - full range")
    {
        Interval i(-1, 7);
        Interval result = sin(i);
        REQUIRE_THAT(result.lower(), WithinAbs(-1.0, 1e-10));
        REQUIRE_THAT(result.upper(), WithinAbs(1.0, 1e-10));
    }
    
    SECTION("Sine - shifted by PI/2")
    {
        Interval i(-1 + PI/2, 7 + PI/2);
        Interval result = sin(i);
        REQUIRE_THAT(result.lower(), WithinAbs(-1.0, 1e-10));
        REQUIRE_THAT(result.upper(), WithinAbs(1.0, 1e-10));
    }
}

/**
 * @brief Test interval intersection and hull
 */
TEST_CASE("Interval intersection and hull", "[math][interval]")
{
    Interval a(1, 5);
    Interval b(3, 7);
    
    SECTION("Intersection")
    {
        Interval result = intersect(a, b);
        REQUIRE(result.lower() == 3.0);
        REQUIRE(result.upper() == 5.0);
    }
    
    SECTION("Non-overlapping intersection")
    {
        Interval c(10, 15);
        Interval result = intersect(a, c);
        REQUIRE(is_empty(result));
    }
    
    SECTION("Hull")
    {
        Interval result = hull(a, b);
        REQUIRE(result.lower() == 1.0);
        REQUIRE(result.upper() == 7.0);
    }
}

/**
 * @brief Test interval vector operations
 */
TEST_CASE("Interval vector operations", "[math][interval][vector]")
{
    SECTION("Vector width")
    {
        VectorXI v(3);
        v << Interval(0, 1), Interval(2, 4), Interval(5, 10);
        
        Eigen::VectorXd w = width(v);
        REQUIRE(w(0) == 1.0);
        REQUIRE(w(1) == 2.0);
        REQUIRE(w(2) == 5.0);
    }
    
    SECTION("Vector zero_in")
    {
        VectorXI v1(2);
        v1 << Interval(-1, 1), Interval(-2, 2);
        REQUIRE(zero_in(v1));
        
        VectorXI v2(2);
        v2 << Interval(1, 2), Interval(-1, 1);
        REQUIRE_FALSE(zero_in(v2));
    }
    
    SECTION("Diagonal width")
    {
        VectorXI v(2);
        v << Interval(0, 3), Interval(0, 4);
        
        double dw = diagonal_width(v);
        REQUIRE_THAT(dw, WithinAbs(5.0, 1e-10));  // sqrt(9 + 16) = 5
    }
    
    SECTION("Squared norm")
    {
        VectorXI v(2);
        v << Interval(1, 1), Interval(2, 2);
        
        Interval sn = squared_norm(v);
        REQUIRE_THAT(sn.lower(), WithinAbs(5.0, 1e-10));
        REQUIRE_THAT(sn.upper(), WithinAbs(5.0, 1e-10));
    }
    
    SECTION("Norm")
    {
        VectorXI v(2);
        v << Interval(3, 3), Interval(4, 4);
        
        Interval n = norm(v);
        REQUIRE_THAT(n.lower(), WithinAbs(5.0, 1e-10));
        REQUIRE_THAT(n.upper(), WithinAbs(5.0, 1e-10));
    }
}

/**
 * @brief Test interval matrix operations
 */
TEST_CASE("Interval matrix operations", "[math][interval][matrix]")
{
    SECTION("Matrix-vector multiplication")
    {
        Matrix2I A;
        A << Interval(1, 1), Interval(2, 2),
             Interval(3, 3), Interval(4, 4);
        
        Vector2I x(Interval(1, 1), Interval(1, 1));
        
        Vector2I result = A * x;
        REQUIRE_THAT(result(0).lower(), WithinAbs(3.0, 1e-10));
        REQUIRE_THAT(result(0).upper(), WithinAbs(3.0, 1e-10));
        REQUIRE_THAT(result(1).lower(), WithinAbs(7.0, 1e-10));
        REQUIRE_THAT(result(1).upper(), WithinAbs(7.0, 1e-10));
    }
    
    SECTION("Matrix with interval entries")
    {
        Matrix2I A;
        A << Interval(1, 2), Interval(3, 4),
             Interval(5, 6), Interval(7, 8);
        
        Vector2I x(Interval(1, 1), Interval(1, 1));
        
        Vector2I result = A * x;
        REQUIRE_THAT(result(0).lower(), WithinAbs(4.0, 1e-10));  // 1+3
        REQUIRE_THAT(result(0).upper(), WithinAbs(6.0, 1e-10));  // 2+4
        REQUIRE_THAT(result(1).lower(), WithinAbs(12.0, 1e-10)); // 5+7
        REQUIRE_THAT(result(1).upper(), WithinAbs(14.0, 1e-10)); // 6+8
    }
}

/**
 * @brief Test conservative property of interval arithmetic
 */
TEST_CASE("Interval conservative property", "[math][interval]")
{
    SECTION("Addition is conservative")
    {
        double a = 0.1;
        double b = 0.2;
        
        Interval ia(a);
        Interval ib(b);
        Interval result = ia + ib;
        
        double exact = a + b;
        REQUIRE(result.lower() <= exact);
        REQUIRE(result.upper() >= exact);
    }
    
    SECTION("Multiplication is conservative")
    {
        double a = 1.5;
        double b = 2.5;
        
        Interval ia(a);
        Interval ib(b);
        Interval result = ia * ib;
        
        double exact = a * b;
        REQUIRE(result.lower() <= exact);
        REQUIRE(result.upper() >= exact);
    }
    
    SECTION("Complex expression is conservative")
    {
        Interval x(1, 2);
        Interval y(3, 4);
        
        // f(x,y) = x^2 + 2xy + y^2 = (x+y)^2
        Interval result = x * x + 2.0 * x * y + y * y;
        
        // Minimum: (1+3)^2 = 16
        // Maximum: (2+4)^2 = 36
        REQUIRE(result.lower() <= 16.0);
        REQUIRE(result.upper() >= 36.0);
    }
}

/**
 * @brief Test empty interval handling
 */
TEST_CASE("Empty interval handling", "[math][interval]")
{
    SECTION("Create empty interval")
    {
        Interval e = empty_interval();
        REQUIRE(is_empty(e));
    }
    
    SECTION("Empty interval in operations")
    {
        Interval e = empty_interval();
        Interval i(1, 2);
        
        Interval result = e + i;
        REQUIRE(is_empty(result));
    }
}

/**
 * @brief Test formatting functions
 */
TEST_CASE("Interval formatting", "[math][interval]")
{
    SECTION("Format single interval")
    {
        Interval i(1.5, 2.5);
        std::string s = fmt_interval(i);
        REQUIRE(s.find("1.5") != std::string::npos);
        REQUIRE(s.find("2.5") != std::string::npos);
    }
    
    SECTION("Format interval vector")
    {
        VectorXI v(2);
        v << Interval(1, 2), Interval(3, 4);
        
        std::string s = fmt_eigen_intervals(v);
        REQUIRE(s.find("1") != std::string::npos);
        REQUIRE(s.find("4") != std::string::npos);
    }
}
