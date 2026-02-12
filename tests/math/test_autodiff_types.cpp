/**
 * @file test_autodiff_types.cpp
 * @brief Unit tests for AutoDiffTypes convenience types and helpers
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <NexDynIPC/Math/AutoDiffTypes.h>

using namespace NexDynIPC::Math;
using Catch::Matchers::WithinAbs;

/**
 * @brief Test AutodiffType type aliases
 */
TEST_CASE("AutodiffType type aliases", "[math][autodiff][types]")
{
    SECTION("Fixed-size 2D types")
    {
        AutodiffType<2>::activate();
        
        AutodiffType<2>::DDouble1 x(0, 1.0);
        AutodiffType<2>::DDouble2 y(1, 2.0);
        
        REQUIRE(x.getValue() == 1.0);
        REQUIRE(y.getValue() == 2.0);
    }
    
    SECTION("Fixed-size 3D types")
    {
        AutodiffType<3>::activate();
        
        AutodiffType<3>::VectorNd v;
        v << 1.0, 2.0, 3.0;
        
        REQUIRE(v(0) == 1.0);
        REQUIRE(v(1) == 2.0);
        REQUIRE(v(2) == 3.0);
    }
    
    SECTION("Dynamic-size types")
    {
        DynamicAutodiff::activate(5);
        
        REQUIRE(DiffScalarBase::getVariableCount() == 5);
        
        DynamicAutodiff::DDouble1 x(0, 1.0);
        REQUIRE(x.getGradient().size() == 5);
    }
}

/**
 * @brief Test AutodiffType::activate
 */
TEST_CASE("AutodiffType activate", "[math][autodiff][types]")
{
    SECTION("Fixed-size activate")
    {
        AutodiffType<4>::activate();
        REQUIRE(DiffScalarBase::getVariableCount() == 4);
    }
    
    SECTION("Dynamic-size activate with parameter")
    {
        DynamicAutodiff::activate(10);
        REQUIRE(DiffScalarBase::getVariableCount() == 10);
    }
}

/**
 * @brief Test d1vars and d2vars helpers
 */
TEST_CASE("AutodiffType dvars helpers", "[math][autodiff][types]")
{
    SECTION("d1vars")
    {
        DynamicAutodiff::activate(5);
        
        Eigen::VectorXd v(3);
        v << 1.0, 2.0, 3.0;
        
        auto d1v = DynamicAutodiff::d1vars(0, v);
        
        REQUIRE(d1v.rows() == 3);
        REQUIRE(d1v(0).getValue() == 1.0);
        REQUIRE(d1v(1).getValue() == 2.0);
        REQUIRE(d1v(2).getValue() == 3.0);
        
        // Check gradient assignment
        REQUIRE(d1v(0).getGradient()(0) == 1.0);
        REQUIRE(d1v(1).getGradient()(1) == 1.0);
        REQUIRE(d1v(2).getGradient()(2) == 1.0);
    }
    
    SECTION("d2vars")
    {
        DynamicAutodiff::activate(5);
        
        Eigen::VectorXd v(2);
        v << 5.0, 10.0;
        
        auto d2v = DynamicAutodiff::d2vars(0, v);
        
        REQUIRE(d2v.rows() == 2);
        REQUIRE(d2v(0).getValue() == 5.0);
        REQUIRE(d2v(1).getValue() == 10.0);
        
        // Check Hessian is zero-initialized
        REQUIRE(d2v(0).getHessian().isZero());
    }
}

/**
 * @brief Test get_value helper functions
 */
TEST_CASE("get_value helpers", "[math][autodiff][types]")
{
    AutodiffType<3>::activate();
    
    SECTION("Single DScalar value")
    {
        AutodiffType<3>::DDouble1 x(0, 5.0);
        REQUIRE(get_value(x) == 5.0);
    }
    
    SECTION("Vector of DScalars")
    {
        AutodiffType<3>::D1VectorXd vec(2);
        vec(0) = AutodiffType<3>::DDouble1(0, 1.0);
        vec(1) = AutodiffType<3>::DDouble1(1, 2.0);
        
        auto values = AutodiffType<3>::get_value(vec);
        
        REQUIRE(values.rows() == 2);
        REQUIRE(values(0) == 1.0);
        REQUIRE(values(1) == 2.0);
    }
}

/**
 * @brief Test get_gradient helper functions
 */
TEST_CASE("get_gradient helpers", "[math][autodiff][types]")
{
    AutodiffType<3>::activate();
    
    SECTION("Single DScalar gradient")
    {
        AutodiffType<3>::DDouble1 x(1, 5.0);  // Variable 1
        const auto& grad = get_gradient(x);
        
        REQUIRE(grad(0) == 0.0);
        REQUIRE(grad(1) == 1.0);
        REQUIRE(grad(2) == 0.0);
    }
    
    SECTION("Vector of DScalars gradient matrix")
    {
        AutodiffType<3>::D1VectorXd vec(2);
        vec(0) = AutodiffType<3>::DDouble1(0, 1.0);
        vec(1) = AutodiffType<3>::DDouble1(1, 2.0);
        
        auto grad_matrix = AutodiffType<3>::get_gradient(vec);
        
        REQUIRE(grad_matrix.rows() == 2);
        REQUIRE(grad_matrix.cols() == 3);
        
        // First row: gradient of first element
        REQUIRE(grad_matrix(0, 0) == 1.0);
        REQUIRE(grad_matrix(0, 1) == 0.0);
        
        // Second row: gradient of second element
        REQUIRE(grad_matrix(1, 0) == 0.0);
        REQUIRE(grad_matrix(1, 1) == 1.0);
    }
}

/**
 * @brief Test get_hessian helper functions
 */
TEST_CASE("get_hessian helpers", "[math][autodiff][types]")
{
    AutodiffType<2>::activate();
    
    SECTION("DScalar2 hessian")
    {
        AutodiffType<2>::DDouble2 x(0, 2.0);
        AutodiffType<2>::DDouble2 y(1, 3.0);
        
        auto f = x * x * y;  // f = x^2 * y
        
        const auto& hess = get_hessian(f);
        
        // d^2f/dx^2 = 2y = 6
        REQUIRE(hess(0, 0) == 6.0);
        
        // d^2f/dxdy = 2x = 4
        REQUIRE(hess(0, 1) == 4.0);
    }
    
    SECTION("DScalar1 hessian throws")
    {
        AutodiffType<2>::DDouble1 x(0, 1.0);
        REQUIRE_THROWS_AS(get_hessian(x), std::runtime_error);
    }
    
    SECTION("Vector of DScalars hessian")
    {
        AutodiffType<2>::D2VectorXd vec(2);
        vec(0) = AutodiffType<2>::DDouble2(0, 1.0);
        vec(1) = AutodiffType<2>::DDouble2(1, 2.0);
        
        auto hessians = AutodiffType<2>::get_hessian(vec);
        
        REQUIRE(hessians.size() == 2);
        REQUIRE(hessians[0].rows() == 2);
        REQUIRE(hessians[0].cols() == 2);
    }
}

/**
 * @brief Test predefined type aliases
 */
TEST_CASE("Predefined type aliases", "[math][autodiff][types]")
{
    SECTION("Autodiff2D")
    {
        Autodiff2D::activate();
        REQUIRE(DiffScalarBase::getVariableCount() == 3);
        
        Autodiff2D::D1Vector3d v;
        v << Autodiff2D::DDouble1(0, 1.0),
             Autodiff2D::DDouble1(1, 2.0),
             Autodiff2D::DDouble1(2, 3.0);
        
        REQUIRE(v(0).getValue() == 1.0);
    }
    
    SECTION("Autodiff3D")
    {
        Autodiff3D::activate();
        REQUIRE(DiffScalarBase::getVariableCount() == 6);
    }
    
    SECTION("AutodiffRigidBody")
    {
        AutodiffRigidBody::activate();
        REQUIRE(DiffScalarBase::getVariableCount() == 6);
    }
}

/**
 * @brief Test practical usage example with AutodiffType
 */
TEST_CASE("AutodiffType practical example", "[math][autodiff][types]")
{
    // Example: Compute gradient and Hessian of f(x,y,z) = x^2 + y^2 + z^2
    AutodiffType<3>::activate();
    
    using AD = AutodiffType<3>;
    
    AD::DDouble2 x(0, 2.0);
    AD::DDouble2 y(1, 3.0);
    AD::DDouble2 z(2, 4.0);
    
    auto f = x * x + y * y + z * z;
    
    // f = 4 + 9 + 16 = 29
    REQUIRE(f.getValue() == 29.0);
    
    // Gradient: [2x, 2y, 2z] = [4, 6, 8]
    REQUIRE(f.getGradient()(0) == 4.0);
    REQUIRE(f.getGradient()(1) == 6.0);
    REQUIRE(f.getGradient()(2) == 8.0);
    
    // Hessian: diagonal matrix with 2s
    REQUIRE(f.getHessian()(0, 0) == 2.0);
    REQUIRE(f.getHessian()(1, 1) == 2.0);
    REQUIRE(f.getHessian()(2, 2) == 2.0);
    REQUIRE(f.getHessian()(0, 1) == 0.0);
}

/**
 * @brief Test Eigen matrix operations with DScalars
 */
TEST_CASE("Eigen matrix operations with DScalars", "[math][autodiff][types]")
{
    AutodiffType<2>::activate();
    
    using AD = AutodiffType<2>;
    
    SECTION("Matrix-vector multiplication")
    {
        AD::D1MatrixXd A(2, 2);
        A(0, 0) = AD::DDouble1(0, 1.0);
        A(0, 1) = AD::DDouble1(1, 2.0);
        A(1, 0) = AD::DDouble1(0, 3.0);
        A(1, 1) = AD::DDouble1(1, 4.0);
        
        AD::D1VectorXd b(2);
        b(0) = AD::DDouble1(0, 1.0);
        b(1) = AD::DDouble1(1, 1.0);
        
        // A * b = [1*1 + 2*1, 3*1 + 4*1] = [3, 7]
        auto c = A * b;
        
        REQUIRE(c(0).getValue() == 3.0);
        REQUIRE(c(1).getValue() == 7.0);
    }
    
    SECTION("Dot product")
    {
        AD::D1VectorXd a(2);
        a(0) = AD::DDouble1(0, 2.0);
        a(1) = AD::DDouble1(1, 3.0);
        
        AD::D1VectorXd b(2);
        b(0) = AD::DDouble1(0, 1.0);
        b(1) = AD::DDouble1(1, 1.0);
        
        // a · b = 2*1 + 3*1 = 5
        auto dot = a.dot(b);
        REQUIRE(dot.getValue() == 5.0);
    }
}
