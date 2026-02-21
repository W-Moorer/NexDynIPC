// ============================================================================
// NexDynIPC AutoDiff 集成测试
// ============================================================================

#include <catch2/catch.hpp>
#include <NexDynIPC/Math/AutoDiffExtended.h>
#include <NexDynIPC/Math/AutoDiffCompatibility.h>
#include <NexDynIPC/Dynamics/Forms/AutoDiffBarrierForm.h>

#include <Eigen/Core>
#include <cmath>

using namespace NexDynIPC;
using namespace NexDynIPC::Math;
using namespace NexDynIPC::Dynamics;

// ============================================================================
// 基本自动微分测试
// ============================================================================

TEST_CASE("AutoDiff basic operations", "[autodiff]") {
    SECTION("Dual scalar operations") {
        autodiff::dual x = 2.0;
        autodiff::dual y = 3.0;
        
        auto z = x + y;
        REQUIRE(z.val == Approx(5.0));
        
        auto w = x * y;
        REQUIRE(w.val == Approx(6.0));
    }
    
    SECTION("Derivative calculation") {
        auto func = [](autodiff::dual x) -> autodiff::dual {
            return x * x + 2.0 * x + 1.0;
        };
        
        autodiff::dual x = 3.0;
        double dfdx = autodiff::derivative(func, autodiff::wrt(x), autodiff::at(x));
        
        // f(x) = x² + 2x + 1, f'(x) = 2x + 2
        // f'(3) = 2*3 + 2 = 8
        REQUIRE(dfdx == Approx(8.0));
    }
    
    SECTION("Gradient calculation") {
        auto func = [](autodiff::dual x, autodiff::dual y, autodiff::dual z) -> autodiff::dual {
            return x * x + y * y + z * z;
        };
        
        autodiff::dual x = 1.0, y = 2.0, z = 3.0;
        auto [gx, gy, gz] = autodiff::derivatives(func, autodiff::wrt(x, y, z), autodiff::at(x, y, z));
        
        // ∇f = [2x, 2y, 2z] = [2, 4, 6]
        REQUIRE(gx == Approx(2.0));
        REQUIRE(gy == Approx(4.0));
        REQUIRE(gz == Approx(6.0));
    }
}

TEST_CASE("AutoDiff vector operations", "[autodiff]") {
    SECTION("Vector gradient") {
        Eigen::VectorXd x(3);
        x << 1.0, 2.0, 3.0;
        
        auto func = [](const autodiff::VectorXdual<>& x) -> autodiff::dual {
            autodiff::dual sum = 0.0;
            for (int i = 0; i < x.size(); ++i) {
                sum += x[i] * x[i];
            }
            return sum;
        };
        
        autodiff::VectorXdual<> xd = x.cast<autodiff::dual>();
        Eigen::VectorXd grad = autodiff::gradient(func, autodiff::wrt(xd), autodiff::at(xd));
        
        // ∇f = [2x₁, 2x₂, 2x₃] = [2, 4, 6]
        REQUIRE(grad[0] == Approx(2.0));
        REQUIRE(grad[1] == Approx(4.0));
        REQUIRE(grad[2] == Approx(6.0));
    }
    
    SECTION("Hessian calculation") {
        Eigen::VectorXd x(2);
        x << 1.0, 2.0;
        
        auto func = [](const autodiff::VectorXdual2nd<>& x) -> autodiff::dual2nd {
            return x[0] * x[0] + x[0] * x[1] + x[1] * x[1];
        };
        
        autodiff::VectorXdual2nd<> x2 = x.cast<autodiff::dual2nd>();
        Eigen::MatrixXd H = autodiff::hessian(func, autodiff::wrt(x2), autodiff::at(x2));
        
        // H = [[2, 1], [1, 2]]
        REQUIRE(H(0, 0) == Approx(2.0));
        REQUIRE(H(0, 1) == Approx(1.0));
        REQUIRE(H(1, 0) == Approx(1.0));
        REQUIRE(H(1, 1) == Approx(2.0));
    }
}

TEST_CASE("AutoDiff reverse mode", "[autodiff]") {
    SECTION("Reverse mode gradient") {
        auto func = [](autodiff::var x, autodiff::var y) -> autodiff::var {
            return x * x + y * y;
        };
        
        autodiff::var x = 3.0, y = 4.0;
        autodiff::var z = func(x, y);
        
        auto [zx, zy] = autodiff::derivatives(z, autodiff::wrt(x, y));
        
        // ∂z/∂x = 2x = 6, ∂z/∂y = 2y = 8
        REQUIRE(zx == Approx(6.0));
        REQUIRE(zy == Approx(8.0));
    }
    
    SECTION("Jacobian calculation") {
        Eigen::VectorXd x(3);
        x << 1.0, 2.0, 3.0;
        
        auto func = [](const autodiff::VectorXvar<>& x) -> autodiff::VectorXvar<> {
            autodiff::VectorXvar<> y(2);
            y << x.sum(), x.prod();
            return y;
        };
        
        autodiff::VectorXvar<> xv = x.cast<autodiff::var>();
        autodiff::VectorXvar<> yv = func(xv);
        
        Eigen::MatrixXd J = autodiff::jacobian(yv, xv);
        
        // J = [[1, 1, 1],    (sum 的导数)
        //      [6, 3, 2]]    (prod 的导数: 2*3, 1*3, 1*2)
        REQUIRE(J(0, 0) == Approx(1.0));
        REQUIRE(J(0, 1) == Approx(1.0));
        REQUIRE(J(0, 2) == Approx(1.0));
        REQUIRE(J(1, 0) == Approx(6.0));
        REQUIRE(J(1, 1) == Approx(3.0));
        REQUIRE(J(1, 2) == Approx(2.0));
    }
}

// ============================================================================
// 兼容层测试
// ============================================================================

TEST_CASE("AutoDiffSelector gradient", "[autodiff][compatibility]") {
    auto func = [](const auto& x) -> auto {
        autodiff::dual sum = 0.0;
        for (int i = 0; i < x.size(); ++i) {
            sum += x[i] * x[i];
        }
        return sum;
    };
    
    SECTION("Small dimension (<= 10)") {
        Eigen::VectorXd x(3);
        x << 1.0, 2.0, 3.0;
        
        Eigen::VectorXd grad = AutoDiffSelector::gradient(func, x);
        
        REQUIRE(grad[0] == Approx(2.0));
        REQUIRE(grad[1] == Approx(4.0));
        REQUIRE(grad[2] == Approx(6.0));
    }
    
    SECTION("Large dimension (> 10)") {
        Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(20, 1.0, 20.0);
        
        Eigen::VectorXd grad = AutoDiffSelector::gradient(func, x);
        
        // ∇f = 2x
        for (int i = 0; i < x.size(); ++i) {
            REQUIRE(grad[i] == Approx(2.0 * x[i]));
        }
    }
}

TEST_CASE("AutoDiffSelector hessian", "[autodiff][compatibility]") {
    auto func = [](const auto& x) -> auto {
        autodiff::dual2nd sum = 0.0;
        for (int i = 0; i < x.size(); ++i) {
            for (int j = 0; j < x.size(); ++j) {
                sum += x[i] * x[j];
            }
        }
        return sum;
    };
    
    Eigen::VectorXd x(2);
    x << 1.0, 2.0;
    
    Eigen::MatrixXd H = AutoDiffSelector::hessian(func, x);
    
    // f = (x₁ + x₂)² = x₁² + 2x₁x₂ + x₂²
    // H = [[2, 2], [2, 2]]
    REQUIRE(H(0, 0) == Approx(2.0));
    REQUIRE(H(0, 1) == Approx(2.0));
    REQUIRE(H(1, 0) == Approx(2.0));
    REQUIRE(H(1, 1) == Approx(2.0));
}

TEST_CASE("AutoDiffConverter", "[autodiff][compatibility]") {
    SECTION("Vector to dual conversion") {
        Eigen::VectorXd x(3);
        x << 1.0, 2.0, 3.0;
        
        autodiff::VectorXdual<> xd = AutoDiffConverter::toVectorDual(x);
        
        REQUIRE(xd[0].val == Approx(1.0));
        REQUIRE(xd[1].val == Approx(2.0));
        REQUIRE(xd[2].val == Approx(3.0));
    }
    
    SECTION("Dual to vector conversion") {
        autodiff::VectorXdual<> xd(3);
        xd[0] = autodiff::dual(1.0, 0.1);
        xd[1] = autodiff::dual(2.0, 0.2);
        xd[2] = autodiff::dual(3.0, 0.3);
        
        Eigen::VectorXd x = AutoDiffConverter::toVectorXd(xd);
        Eigen::VectorXd grad = AutoDiffConverter::extractGradient(xd);
        
        REQUIRE(x[0] == Approx(1.0));
        REQUIRE(x[1] == Approx(2.0));
        REQUIRE(x[2] == Approx(3.0));
        
        REQUIRE(grad[0] == Approx(0.1));
        REQUIRE(grad[1] == Approx(0.2));
        REQUIRE(grad[2] == Approx(0.3));
    }
}

// ============================================================================
// 屏障形式测试
// ============================================================================

TEST_CASE("AutoDiffBarrierForm", "[autodiff][barrier]") {
    SECTION("Quadratic barrier") {
        auto barrier = [](const autodiff::VectorXdual<>& x) -> autodiff::dual {
            autodiff::dual energy = 0.0;
            for (int i = 0; i < x.size(); ++i) {
                energy += x[i] * x[i];
            }
            return energy;
        };
        
        AutoDiffBarrierForm form(barrier, 1.0);
        
        Eigen::VectorXd x(3);
        x << 1.0, 2.0, 3.0;
        
        // 测试值
        double val = form.value(x);
        REQUIRE(val == Approx(14.0));  // 1 + 4 + 9 = 14
        
        // 测试梯度
        Eigen::VectorXd grad(3);
        form.gradient(x, grad);
        REQUIRE(grad[0] == Approx(2.0));
        REQUIRE(grad[1] == Approx(4.0));
        REQUIRE(grad[2] == Approx(6.0));
        
        // 测试 Hessian
        std::vector<Eigen::Triplet<double>> triplets;
        form.hessian(x, triplets);
        
        // Hessian 应该是 2I
        REQUIRE(triplets.size() == 3);
        for (const auto& t : triplets) {
            REQUIRE(t.row() == t.col());  // 对角矩阵
            REQUIRE(t.value() == Approx(2.0));
        }
    }
    
    SECTION("IPC barrier form") {
        IPCBarrierForm ipc_form(0.1, 1000.0);
        
        // 测试距离小于 dhat 的情况
        Eigen::VectorXd x(1);
        x << 0.05;  // d < dhat (0.1)
        
        double val = ipc_form.value(x);
        REQUIRE(val > 0.0);  // 屏障应该产生正的能量
        
        // 测试距离大于 dhat 的情况
        x << 0.2;  // d > dhat
        val = ipc_form.value(x);
        REQUIRE(val == Approx(0.0));  // 屏障应该为零
    }
    
    SECTION("Spring barrier form") {
        SpringBarrierForm spring_form(1.0, 100.0);
        
        // 两个点，距离为 1.5（静息长度为 1.0）
        Eigen::VectorXd x(6);
        x << 0.0, 0.0, 0.0,   // 第一个点
             1.5, 0.0, 0.0;   // 第二个点（距离 1.5）
        
        double val = spring_form.value(x);
        // E = 0.5 * k * (d - d₀)² = 0.5 * 100 * (1.5 - 1.0)² = 12.5
        REQUIRE(val == Approx(12.5));
    }
}

// ============================================================================
// 数学函数测试
// ============================================================================

TEST_CASE("AutoDiff mathematical functions", "[autodiff][math]") {
    SECTION("Trigonometric functions") {
        autodiff::dual x = M_PI / 4.0;  // 45 degrees
        
        auto func = [](autodiff::dual x) -> autodiff::dual {
            return autodiff::sin(x) + autodiff::cos(x);
        };
        
        double dfdx = autodiff::derivative(func, autodiff::wrt(x), autodiff::at(x));
        
        // f(x) = sin(x) + cos(x)
        // f'(x) = cos(x) - sin(x)
        // f'(π/4) = √2/2 - √2/2 = 0
        REQUIRE(dfdx == Approx(0.0).margin(1e-10));
    }
    
    SECTION("Exponential and logarithm") {
        autodiff::dual x = 2.0;
        
        auto func = [](autodiff::dual x) -> autodiff::dual {
            return autodiff::exp(x) * autodiff::log(x);
        };
        
        double val = func(x).val;
        double dfdx = autodiff::derivative(func, autodiff::wrt(x), autodiff::at(x));
        
        // f(x) = e^x * ln(x)
        // f'(x) = e^x * ln(x) + e^x / x = e^x * (ln(x) + 1/x)
        double expected = std::exp(2.0) * (std::log(2.0) + 0.5);
        REQUIRE(dfdx == Approx(expected));
    }
    
    SECTION("Power function") {
        autodiff::dual x = 3.0;
        autodiff::dual y = 2.0;
        
        auto func = [](autodiff::dual x, autodiff::dual y) -> autodiff::dual {
            return autodiff::pow(x, y);  // x^y
        };
        
        auto [dfdx, dfdy] = autodiff::derivatives(func, autodiff::wrt(x, y), autodiff::at(x, y));
        
        // f(x,y) = x^y
        // ∂f/∂x = y * x^(y-1) = 2 * 3^1 = 6
        // ∂f/∂y = x^y * ln(x) = 9 * ln(3)
        REQUIRE(dfdx == Approx(6.0));
        REQUIRE(dfdy == Approx(9.0 * std::log(3.0)));
    }
}

// ============================================================================
// 性能对比测试
// ============================================================================

TEST_CASE("AutoDiff performance comparison", "[autodiff][performance]") {
    // 测试不同维度下的性能
    std::vector<int> dimensions = {5, 10, 20, 50};
    
    for (int n : dimensions) {
        Eigen::VectorXd x = Eigen::VectorXd::Random(n);
        
        auto func = [](const auto& x) -> auto {
            autodiff::dual sum = 0.0;
            for (int i = 0; i < x.size(); ++i) {
                sum += x[i] * x[i];
            }
            return sum;
        };
        
        // 使用 AutoDiffSelector（自动选择）
        auto start = std::chrono::high_resolution_clock::now();
        Eigen::VectorXd grad = AutoDiffSelector::gradient(func, x);
        auto end = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        
        INFO("Dimension " << n << ": " << duration.count() << " μs");
        
        // 验证结果正确性
        for (int i = 0; i < n; ++i) {
            REQUIRE(grad[i] == Approx(2.0 * x[i]));
        }
    }
}
