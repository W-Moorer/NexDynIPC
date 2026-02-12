---
name: "utest-generator"
description: "Generates comprehensive unit tests for C++ code using Catch2 framework. Invoke when user asks to create unit tests, utest, or test cases for a module/class/function."
---

# 单元测试生成器 (Unit Test Generator)

## 概述

本SKILL用于为NexDynIPC项目生成高质量的C++单元测试。项目使用 **Catch2 v3** 作为测试框架，并遵循特定的项目结构和命名约定。

## 项目测试规范

### 测试框架
- **框架**: Catch2 v3
- **主入口**: `tests/main.cpp`
- **测试文件位置**: `tests/<module>/test_<feature>.cpp`

### 测试文件命名规范
```
tests/
├── main.cpp                    # Catch2 主入口
├── CMakeLists.txt              # 测试构建配置
└── <module>/                   # 模块目录
    └── test_<feature>.cpp      # 测试文件
```

### 测试用例命名规范
```cpp
TEST_CASE("<测试描述>", "[<模块>][<子模块>]")
{
    // 测试代码
}

// 示例
TEST_CASE("Test barriers and their derivatives", "[opt][barrier]")
TEST_CASE("Test AutoDiff DScalar1 gradient computation", "[math][autodiff]")
```

### 常用测试模式

#### 1. 基础功能测试
```cpp
TEST_CASE("<功能描述>", "[<模块>][<子模块>]")
{
    // Arrange
    double input = 5.0;
    
    // Act
    double result = function_to_test(input);
    
    // Assert
    REQUIRE(result == Approx(expected_value));
}
```

#### 2. 参数化测试 (使用GENERATE)
```cpp
TEST_CASE("<功能描述> - parameterized", "[<模块>][<子模块>]")
{
    double param = GENERATE(range(1, 10));
    
    REQUIRE(function_to_test(param) > 0);
}
```

#### 3. 梯度/导数测试 (使用有限差分)
```cpp
#include <finitediff.hpp>

TEST_CASE("<功能> gradient test", "[<模块>][<sub>]")
{
    Eigen::VectorXd x = ...;
    
    // 数值梯度
    Eigen::VectorXd fgrad;
    fd::finite_gradient(x, [&](const Eigen::VectorXd& x) {
        return function(x);
    }, fgrad);
    
    // 解析梯度
    Eigen::VectorXd grad = function_gradient(x);
    
    CHECK(fd::compare_gradient(fgrad, grad));
}
```

#### 4. 边界条件测试
```cpp
TEST_CASE("<功能> boundary conditions", "[<module>][<sub>]")
{
    SECTION("Empty input")
    {
        REQUIRE_THROWS(function({}));
    }
    
    SECTION("Single element")
    {
        REQUIRE(function({1.0}) == Approx(1.0));
    }
    
    SECTION("Large values")
    {
        REQUIRE(function({1e6}) < std::numeric_limits<double>::infinity());
    }
}
```

## 测试生成流程

当用户请求生成单元测试时，按以下步骤执行：

### 步骤1: 分析被测代码
1. 读取目标头文件和实现文件
2. 识别公共接口（类、函数、模板）
3. 分析输入参数和返回值
4. 识别关键算法和边界条件

### 步骤2: 设计测试用例
根据被测代码特性，设计以下类型的测试：

| 测试类型 | 说明 | 适用场景 |
|---------|------|---------|
| **功能测试** | 验证基本功能正确性 | 所有函数/类 |
| **边界测试** | 测试边界条件和异常情况 | 数值计算、容器操作 |
| **梯度测试** | 使用有限差分验证导数 | 自动微分、优化相关 |
| **性能测试** | 验证性能指标 | 算法、数据结构 |
| **随机测试** | 使用随机数据验证鲁棒性 | 数值算法 |

### 步骤3: 生成测试代码
1. 创建测试文件目录结构
2. 生成包含必要头文件的测试代码
3. 实现测试用例（遵循AAA模式：Arrange-Act-Assert）
4. 添加详细的注释说明

### 步骤4: 更新构建配置
更新 `tests/CMakeLists.txt` 添加新的测试文件。

## 测试模板

### 头文件测试模板
```cpp
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <NexDynIPC/<Module>/<Header>.hpp>

using namespace NexDynIPC::<Module>;

TEST_CASE("<ClassName> basic construction", "[<module>][<sub>]")
{
    // 测试默认构造
    ClassName obj;
    REQUIRE(obj.isValid());
    
    // 测试参数化构造
    ClassName obj2(params);
    REQUIRE(obj2.getValue() == expected);
}

TEST_CASE("<ClassName> core functionality", "[<module>][<sub>]")
{
    ClassName obj;
    
    SECTION("Normal case")
    {
        auto result = obj.compute(input);
        REQUIRE(result == Approx(expected));
    }
    
    SECTION("Edge case")
    {
        REQUIRE_THROWS(obj.compute(invalid_input));
    }
}
```

### 数值计算测试模板
```cpp
TEST_CASE("<Function> numerical accuracy", "[<module>][<sub>]")
{
    using Catch::Matchers::WithinAbs;
    using Catch::Matchers::WithinRel;
    
    // 使用参数化测试
    double input = GENERATE(take(10, random(-10.0, 10.0)));
    
    double result = function(input);
    double expected = reference_function(input);
    
    REQUIRE_THAT(result, WithinRel(expected, 1e-10));
}
```

### 自动微分测试模板
```cpp
#include <finitediff.hpp>
#include <NexDynIPC/Math/AutoDiff.hpp>

using namespace NexDynIPC::Math;

TEST_CASE("AutoDiff gradient computation", "[math][autodiff]")
{
    // 设置变量数
    DiffScalarBase::setVariableCount(3);
    
    // 创建自变量
    DScalar1d x(0, 1.0);  // 变量0，值1.0
    DScalar1d y(1, 2.0);  // 变量1，值2.0
    DScalar1d z(2, 3.0);  // 变量2，值3.0
    
    // 计算函数
    auto f = x * y + z * z;
    
    // 验证值
    REQUIRE(f.getValue() == Approx(11.0));
    
    // 验证梯度
    Eigen::VectorXd expected_grad(3);
    expected_grad << 2.0, 1.0, 6.0;
    REQUIRE(f.getGradient().isApprox(expected_grad));
}

TEST_CASE("AutoDiff Hessian computation", "[math][autodiff]")
{
    DiffScalarBase::setVariableCount(2);
    
    DScalar2d x(0, 1.0);
    DScalar2d y(1, 2.0);
    
    auto f = x * x * y;
    
    // 验证Hessian
    Eigen::MatrixXd expected_hessian(2, 2);
    expected_hessian << 4.0, 2.0,
                        2.0, 0.0;
    REQUIRE(f.getHessian().isApprox(expected_hessian));
}
```

## CMakeLists.txt 配置

```cmake
# tests/CMakeLists.txt
add_executable(nexdynipc_tests
    main.cpp
    
    # Math模块测试
    math/test_autodiff.cpp
    math/test_linesearch.cpp
    math/test_newtonsolver.cpp
    
    # 其他模块测试...
)

target_link_libraries(nexdynipc_tests PRIVATE
    NexDynIPC
    Catch2::Catch2WithMain
)

# 注册测试
catch_discover_tests(nexdynipc_tests)
```

## 最佳实践

1. **独立性**: 每个测试用例应该独立运行，不依赖其他测试
2. **可读性**: 测试名称应该清晰描述测试目的
3. **覆盖率**: 覆盖正常路径、边界条件和异常情况
4. **数值精度**: 使用 `Approx` 或 `WithinRel` 进行浮点数比较
5. **参数化**: 使用 `GENERATE` 进行参数化测试，提高覆盖率
6. **有限差分**: 对于导数计算，使用有限差分验证解析结果

## 示例：为AutoDiff生成测试

当用户说"为AutoDiff生成单元测试"时：

1. 读取 `include/NexDynIPC/Math/AutoDiff.hpp` 和 `AutoDiffTypes.hpp`
2. 识别关键类：`DScalar1`, `DScalar2`, `DiffScalarBase`
3. 设计测试用例：
   - 构造函数测试
   - 基本运算测试（加减乘除）
   - 梯度计算测试（使用有限差分验证）
   - Hessian计算测试
   - 边界条件测试（零值、大数值）
4. 生成测试文件 `tests/math/test_autodiff.cpp`
5. 更新 `tests/CMakeLists.txt`
