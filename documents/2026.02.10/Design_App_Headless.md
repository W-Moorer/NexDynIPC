# App (Headless) 模块设计文档

**职责**: 程序的入口点，负责加载场景、驱动仿真循环、以及输出仿真结果。本模块不包含任何图形界面或渲染代码。

## 1. 目录结构

遵循头文件与实现分离原则：

```
NexDynIPC/
├── include/
│   └── NexDynIPC/
│       └── App/
│           ├── Simulation.hpp      // 仿真控制器
│           ├── SceneLoader.hpp     // 场景加载接口 (JSON/XML)
│           └── StateExporter.hpp   // 状态导出接口 (CSV/NPZ/JSON)
└── src/
    └── App/
        ├── Simulation.cpp
        ├── SceneLoader.cpp
        ├── StateExporter.cpp
        └── main.cpp                // 程序入口
```

## 2. 接口设计

### 2.1 仿真控制器 (`Simulation.hpp`)

驱动整个仿真流程。

```cpp
namespace NexDynIPC::App {

class Simulation {
public:
    struct Config {
        double dt = 0.01;
        double total_time = 10.0;
        std::string output_dir = "./output";
    };

    Simulation(const Config& config);

    // 加载场景 (通过 SceneLoader)
    void loadScene(const std::string& scene_file);

    // 运行仿真循环
    void run();

private:
    Config config_;
    std::unique_ptr<Dynamics::World> world_; // 持有物理世界
    std::unique_ptr<Dynamics::TimeIntegrator> integrator_; // 持有积分器
    std::unique_ptr<StateExporter> exporter_; // 持有导出器

    // 单步执行
    void step();
};

} // namespace
```

### 2.2 场景加载 (`SceneLoader.hpp`)

解析配置文件并构建 `World`。

```cpp
namespace NexDynIPC::App {

class SceneLoader {
public:
    // 解析 JSON 文件并填充 World
    static void load(const std::string& filename, Dynamics::World& world);
};

} // namespace
```

### 2.3 状态导出 (`StateExporter.hpp`)

将每一帧的刚体状态保存到文件。

```cpp
namespace NexDynIPC::App {

class StateExporter {
public:
    explicit StateExporter(const std::string& output_dir);

    // 导出当前帧状态
    // x: 全局状态向量
    void exportFrame(int frame_idx, const Eigen::VectorXd& x);
};

} // namespace
```

## 3. 主程序逻辑 (`main.cpp`)

```cpp
int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: NexDynIPC <scene_file.json>" << std::endl;
        return 1;
    }

    NexDynIPC::App::Simulation::Config config;
    // ... 解析命令行参数覆盖配置 ...

    NexDynIPC::App::Simulation sim(config);
    sim.loadScene(argv[1]);
    sim.run();

    return 0;
}
```

## 4. 依赖关系

- 依赖 `Dynamics` 模块进行物理仿真。
- 依赖 `nlohmann/json` 进行文件 IO。
- 依赖 `spdlog` 进行日志输出。
