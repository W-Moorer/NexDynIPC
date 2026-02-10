# 刚体接触仿真平台设计文档 (4/4) - 外部库管理 (vcpkg)

## 1. 外部库概览

本项目利用 `vcpkg` 的 **Manifest Mode** (`vcpkg.json`) 进行依赖管理，确保跨平台构建的一致性。为了解决重新下载和编译耗时的问题，我们将启用 **Binary Caching (二进制缓存)**。

### 1.1 核心依赖库列表

| 库名称 | 用途 | 对应模块 | 说明 |
| :--- | :--- | :--- | :--- |
| **eigen3** | 线性代数 | Math Foundation | 作为 header-only 库，必须启用 MKL 或类似后端以加速求解。 |
| **spdlog** | 日志系统 | Application | 高性能 C++ 日志库。 |
| **nlohmann-json** | 序列化/反序列化 | Application | 用于读取场景配置文件和导出结果。 |
| **fmt** | 字符串格式化 | Application | spdlog 的依赖，也用于通用格式化。 |
| **catch2** | 单元测试 | Testing | 现代化的 C++ 测试框架。 |
| **tbb** | 并行计算 | Physics Core | Intel Threading Building Blocks，用于加速宽相检测和 SDF 查询。 |
| **suitesparse** | 稀疏矩阵求解 | Solver | 包含 Cholmod 和 Umfpack，用于求解线性系统 $\mathbf{A}\mathbf{x}=\mathbf{b}$。 |
| **openvdb** | SDF 存储与操作 | Physics Core | 工业级稀疏体积数据结构 (可选，视复杂度而定)。 |
| **libigl** | 几何处理 | Geometry | (可选) 提供这几何算法的便利封装，但在生产中建议仅提取核心算法以减少依赖。 |

---

## 2. vcpkg 清单文件 (vcpkg.json)

在项目根目录下创建 `vcpkg.json`：

```json
{
  "$schema": "https://raw.githubusercontent.com/microsoft/vcpkg-tool/main/docs/vcpkg.schema.json",
  "name": "nexdyn-ipc",
  "version-string": "0.1.0",
  "description": "Rigid Body Contact Simulation Platform",
  "dependencies": [
    {
      "name": "eigen3",
      "features": ["mkl"] 
    },
    "spdlog",
    "nlohmann-json",
    "fmt",
    "catch2",
    "tbb",
    "suitesparse",
    {
      "name": "openvdb",
      "platform": "windows | linux"
    }
  ],
  "builtin-baseline": "38bb87c4db7eb0b5375c3db7da9d873836364c9d", 
  "overrides": [
    { "name": "fmt", "version": "10.1.1" }
  ]
}
```

*注意：`builtin-baseline` 将锁定依赖版本，确保所有开发者使用完全一致的库版本。*

---

## 3. 二进制缓存 (Binary Caching) 配置

vcpkg 的二进制缓存可以存储编译好的包，当在其他机器或新环境中构建时，如果配置相同，则直接下载预编译的二进制文件，无需重新编译。

### 3.1 本地安装目录

vcpkg 在 Manifest Mode 下会自动在构建目录的同级或父级查找配置。通过在 CMake 配置时指定，我们将依赖库统一安装在项目根目录下的 `vcpkg_installed` 文件夹：

`e:\workspace\NexDynIPC\vcpkg_installed`

这确保了依赖库跟随项目，且不污染全局环境。

### 3.2 NuGet 缓存 (进阶)

如果团队有 Azure DevOps 或 GitHub Packages，可以使用 NuGet feed。

```powershell
$env:VCPKG_BINARY_SOURCES = "clear;nuget,https://pkgs.dev.azure.com/my-org/_packaging/my-feed/nuget/v3/index.json,readwrite"
```

### 3.3 配置文件 (`vcpkg-configuration.json`)

也可以在项目根目录添加 `vcpkg-configuration.json` (注意：这主要用于配置 overlay ports 和 registries，**官方建议环境变量配置 Binary Caching**，但部分版本支持在此配置)。

建议做法是保持 `vcpkg-configuration.json` 用于版本控制的注册表配置，而使用环境变量控制缓存位置。

---

## 4. 构建集成 (CMake workflow)

使用 CMake Presets 或直接传递 toolchain file。

### 4.1 CMake Command

```bash
cmake -B build -S . \
    -DCMAKE_TOOLCHAIN_FILE=C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake
```

vcpkg 会自动读取 `vcpkg.json`，检查 `VCPKG_BINARY_SOURCES`，如果有命中缓存则直接下载解压，否则编译并写入缓存。

### 4.2 CMakeLists.txt 集成

```cmake
cmake_minimum_required(VERSION 3.20)
project(NexDynIPC)

# 查找包 (vcpkg 会自动提供)
find_package(Eigen3 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(TBB CONFIG REQUIRED)
find_package(SuiteSparse CONFIG REQUIRED) # 可能需要自定义 Find 模块或 Config

# 链接
add_executable(app main.cpp)
target_link_libraries(app PRIVATE 
    Eigen3::Eigen 
    spdlog::spdlog 
    nlohmann_json::nlohmann_json
    TBB::tbb
)
```

## 5. 避免重新下载的策略

1.  **统一 Baseline**: 确保所有分支的 `vcpkg.json` 中 `builtin-baseline` 一致。
2.  **持久化下载缓存**: vcpkg 默认将源码包缓存在 `downloads` 文件夹。建议将其设置为共享或持久化目录。
    - 环境变量: `VCPKG_DOWNLOADS` 指向固定目录。
3.  **持久化二进制缓存**: 如 3.1 所述，这是最关键的。

---

## 6. 初始化脚本 (setup.ps1)

配置项目以将依赖库安装在本地 `vcpkg_installed` 目录。

```powershell
# setup.ps1
echo "Configuring vcpkg to install packages locally in the project root."
# ... (set install root logic)
```
