# MuJoCo SDF Gear 插件模块分析报告

## 1. 概述

`gear.xml` 文件展示了如何在 MuJoCo 中使用 SDF (Signed Distance Field，有符号距离场) 插件创建齿轮几何体并进行物理仿真。本报告分析实现这一功能所需的 MuJoCo 代码模块及其设计架构。

---

## 2. gear.xml 文件结构分析

```xml
<mujoco>
  <!-- 1. 插件扩展定义 -->
  <extension>
    <plugin plugin="mujoco.sdf.gear">
      <instance name="gear1">
        <config key="alpha" value="0"/>
      </instance>
    </plugin>
  </extension>

  <!-- 2. 资源定义 -->
  <asset>
    <mesh name="gear1">
      <plugin instance="gear1"/>
    </mesh>
  </asset>

  <!-- 3. SDF 特定选项 -->
  <option sdf_iterations="5" sdf_initpoints="20"/>

  <!-- 4. 几何体使用 -->
  <worldbody>
    <geom type="sdf" name="gear1" mesh="gear1" rgba="0.4 0.4 0.4 1">
      <plugin instance="gear1"/>
    </geom>
  </worldbody>
</mujoco>
```

### 关键元素说明：
- **`<extension>`**: 声明使用的插件及其配置实例
- **`<asset><mesh>`**: 将插件实例绑定为可复用的网格资源
- **`<option sdf_iterations>`**: SDF 碰撞检测的迭代次数
- **`<geom type="sdf">`**: 使用 SDF 类型的几何体

---

## 3. 所需模块依赖关系

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                            gear.xml 应用场景                                 │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                        模块依赖层次结构                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  Layer 4: 具体 SDF 插件实现 (plugin/sdf/)                            │   │
│  │  ├── gear.h / gear.cc    ← 齿轮 SDF 插件实现                         │   │
│  │  ├── bolt.h / bolt.cc    ← 螺栓 SDF 插件                             │   │
│  │  ├── nut.h / nut.cc      ← 螺母 SDF 插件                             │   │
│  │  ├── bowl.h / bowl.cc    ← 碗状几何体 SDF 插件                        │   │
│  │  ├── torus.h / torus.cc  ← 圆环体 SDF 插件                           │   │
│  │  ├── sdf.h / sdf.cc      ← SDF 工具类（布尔操作、可视化）              │   │
│  │  └── register.cc         ← 插件注册入口                              │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                      │                                      │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  Layer 3: 插件系统核心 (src/engine/)                                 │   │
│  │  ├── engine_plugin.h / engine_plugin.cc  ← 插件注册、加载、管理       │   │
│  │  └── engine_collision_sdf.h / engine_collision_sdf.c ← SDF 碰撞检测   │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                      │                                      │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  Layer 2: 插件接口定义 (include/mujoco/)                             │   │
│  │  └── mjplugin.h  ← mjpPlugin 结构体、SDF 回调函数定义                 │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                      │                                      │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  Layer 1: 核心数据结构与类型定义                                      │   │
│  │  ├── mjmodel.h / mjdata.h  ← 模型和数据结构                          │   │
│  │  ├── mjtnum.h              ← 数值类型定义                            │   │
│  │  └── mjvisualize.h         ← 可视化结构                              │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 4. 各模块详细设计分析

### 4.1 插件接口层 (mjplugin.h)

**文件路径**: `include/mujoco/mjplugin.h`

#### 核心设计：

```cpp
// 插件能力标志位
typedef enum mjtPluginCapabilityBit_ {
  mjPLUGIN_ACTUATOR = 1<<0,       // 执行器力
  mjPLUGIN_SENSOR   = 1<<1,       // 传感器测量
  mjPLUGIN_PASSIVE  = 1<<2,       // 被动力
  mjPLUGIN_SDF      = 1<<3,       // 有符号距离场 ← Gear 使用
} mjtPluginCapabilityBit;

// 插件结构体定义
struct mjpPlugin_ {
  const char* name;               // 全局唯一名称
  int nattribute;                 // 配置属性数量
  const char* const* attributes;  // 属性名称数组
  int capabilityflags;            // 能力标志位

  // SDF 特定回调函数
  mjtNum (*sdf_distance)(const mjtNum point[3], const mjData* d, int instance);
  void (*sdf_gradient)(mjtNum gradient[3], const mjtNum point[3], const mjData* d, int instance);
  mjtNum (*sdf_staticdistance)(const mjtNum point[3], const mjtNum* attributes);
  void (*sdf_attribute)(mjtNum attribute[], const char* name[], const char* value[]);
  void (*sdf_aabb)(mjtNum aabb[6], const mjtNum* attributes);
};

// SDF 运行时数据结构
struct mjSDF_ {
  const mjpPlugin** plugin;       // 指向插件的指针
  int* id;                        // 实例 ID
  mjtSDFType type;                // SDF 类型
  mjtNum* relpos;                 // 相对位置
  mjtNum* relmat;                 // 相对旋转矩阵
  mjtGeom* geomtype;              // 几何体类型
};
```

#### 设计要点：
1. **能力标志位**: 通过 `mjPLUGIN_SDF` 标识 SDF 插件
2. **回调函数**: SDF 插件必须实现距离计算、梯度计算、AABB 计算等函数
3. **属性系统**: 通过字符串名称数组定义可配置参数
4. **自动注册宏**: `mjPLUGIN_LIB_INIT` 支持跨平台的构造函数注册

---

### 4.2 Gear SDF 插件实现 (plugin/sdf/gear.h, gear.cc)

**文件路径**: `plugin/sdf/gear.h`, `plugin/sdf/gear.cc`

#### 属性定义：

```cpp
struct GearAttribute {
  static constexpr int nattribute = 5;
  static constexpr char const* names[nattribute] = {
    "alpha",        // 初始旋转角度（默认 0）
    "diameter",     // 齿轮直径（默认 2.8）
    "teeth",        // 齿数（默认 25）
    "thickness",    // 厚度（默认 0.2）
    "innerdiameter" // 内径（默认 -1，自动计算）
  };
  static constexpr mjtNum defaults[nattribute] = {0, 2.8, 25, .2, -1};
};
```

#### Gear 类设计：

```cpp
class Gear {
 public:
  static std::optional<Gear> Create(const mjModel* m, mjData* d, int instance);
  mjtNum Distance(const mjtNum point[3]) const;    // 距离计算
  void Gradient(mjtNum grad[3], const mjtNum point[3]) const;  // 梯度计算
  static void RegisterPlugin();  // 插件注册

 private:
  mjtNum attribute[GearAttribute::nattribute];  // 实例属性值
  SdfVisualizer visualizer_;  // 可视化器
};
```

#### 数学原理（渐开线齿轮）：

```cpp
// 2D 齿轮距离场计算
static mjtNum distance2D(const mjtNum p[3], const mjtNum attributes[]) {
  // 基于 Shadertoy 的 2D 齿轮 SDF 实现
  // https://www.shadertoy.com/view/3lG3WR

  mjtNum D = attributes[1];   // 直径
  mjtNum N = attributes[2];   // 齿数
  mjtNum psi = 3.096e-5 * N * N - 6.557e-3 * N + 0.551;  // 压力角

  mjtNum R = D / 2.0;         // 节圆半径
  mjtNum Pd = N / D;          // 径节
  mjtNum P = mjPI / Pd;       // 周节
  mjtNum a = 1.0 / Pd;        // 齿顶高
  mjtNum Do = D + 2.0 * a;    // 外径
  mjtNum Ro = Do / 2.0;       // 外半径
  mjtNum Db = D * mju_cos(psi);  // 基圆直径
  mjtNum Rb = Db / 2.0;       // 基圆半径

  // 渐开线数学计算齿形...
}

// 3D 挤压成型
static mjtNum extrusion(const mjtNum p[3], mjtNum sdf_2d, mjtNum h) {
  mjtNum w[2] = {sdf_2d, abs(p[2]) - h};
  mjtNum w_abs[2] = {mju_max(w[0], 0), mju_max(w[1], 0)};
  return mju_min(mju_max(w[0], w[1]), 0.) + mju_norm(w_abs, 2);
}
```

#### 插件注册实现：

```cpp
void Gear::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.sdf.gear";
  plugin.capabilityflags |= mjPLUGIN_SDF;
  plugin.nattribute = GearAttribute::nattribute;
  plugin.attributes = GearAttribute::names;

  // 生命周期回调
  plugin.init = +[](const mjModel* m, mjData* d, int instance) { ... };
  plugin.destroy = +[](mjData* d, int instance) { ... };
  plugin.reset = +[](...) { ... };

  // SDF 特定回调
  plugin.sdf_distance = +[](const mjtNum point[3], const mjData* d, int instance) {
    auto* sdf = reinterpret_cast<Gear*>(d->plugin_data[instance]);
    return sdf->Distance(point);
  };
  plugin.sdf_gradient = +[](mjtNum gradient[3], const mjtNum point[3], ...) {
    auto* sdf = reinterpret_cast<Gear*>(d->plugin_data[instance]);
    sdf->Gradient(gradient, point);
  };
  plugin.sdf_staticdistance = +[](const mjtNum point[3], const mjtNum* attributes) {
    return distance(point, attributes);  // 编译时调用
  };
  plugin.sdf_aabb = +[](mjtNum aabb[6], const mjtNum* attributes) {
    // 计算包围盒
    aabb[0] = aabb[1] = aabb[2] = 0;
    aabb[3] = aabb[4] = attributes[1] / 2. * 1.25;
    aabb[5] = attributes[3] / 2. * 1.1;
  };

  mjp_registerPlugin(&plugin);
}
```

---

### 4.3 SDF 工具类 (plugin/sdf/sdf.h, sdf.cc)

**文件路径**: `plugin/sdf/sdf.h`, `plugin/sdf/sdf.cc`

#### SDF 布尔操作：

```cpp
// 并集：取最小距离
inline mjtNum Union(mjtNum a, mjtNum b) {
  return mju_min(a, b);
}

// 交集：取最大距离
inline mjtNum Intersection(mjtNum a, mjtNum b) {
  return mju_max(a, b);
}

// 差集：a 减去 b
inline mjtNum Subtraction(mjtNum a, mjtNum b) {
  return mju_max(a, -b);
}
```

#### 属性默认值模板：

```cpp
template <typename T>
class SdfDefault {
 public:
  SdfDefault() {
    for (int i = 0; i < T::nattribute; i++) {
      default_[T::names[i]] = T::defaults[i];
    }
  }

  // 获取单个默认值
  mjtNum GetDefault(const char* name, const char* value) {
    if (std::string(value).empty()) {
      return default_[name];
    }
    try {
      return std::stod(value);
    } catch (...) {
      mju_error("invalid attribute value for '%s'", name);
      return 0;
    }
  }

 private:
  std::map<std::string, mjtNum> default_;
};
```

#### 可视化器：

```cpp
class SdfVisualizer {
 public:
  void Visualize(const mjModel* m, const mjData* d, const mjvOption* opt,
                 mjvScene* scn, int instance);
  void AddPoint(const mjtNum point[3]);  // 添加梯度下降轨迹点
  void Reset();
  void Next();  // 开始新的轨迹

 private:
  std::vector<mjtNum> points_;  // 查询点历史
  std::vector<int> npoints_;    // 每次迭代的点数
};
```

---

### 4.4 SDF 碰撞检测引擎 (src/engine/engine_collision_sdf.h, engine_collision_sdf.c)

**文件路径**: `src/engine/engine_collision_sdf.h`, `src/engine/engine_collision_sdf.c`

#### 核心 API：

```c
// 获取几何体对应的 SDF 插件
MJAPI const mjpPlugin* mjc_getSDF(const mjModel* m, int id);

// 计算点到 SDF 表面的有符号距离
MJAPI mjtNum mjc_distance(const mjModel* m, const mjData* d, const mjSDF* s, const mjtNum x[3]);

// 计算 SDF 在指定点的梯度
MJAPI void mjc_gradient(const mjModel* m, const mjData* d, const mjSDF* s,
                        mjtNum gradient[3], const mjtNum x[3]);

// 碰撞检测函数
int mjc_HFieldSDF(const mjModel* m, const mjData* d, mjContact* con, int g1, int g2, mjtNum margin);
int mjc_MeshSDF(const mjModel* m, const mjData* d, mjContact* con, int g1, int g2, mjtNum margin);
int mjc_SDF(const mjModel* m, const mjData* d, mjContact* con, int g1, int g2, mjtNum margin);
```

#### 八叉树加速结构：

```c
// 八叉树距离查询（用于预计算 SDF）
mjtNum oct_distance(const mjModel* m, const mjtNum p[3], int meshid) {
  int octadr = m->mesh_octadr[meshid];
  int* oct_child = m->oct_child + 8*octadr;
  mjtNum* oct_aabb = m->oct_aabb + 6*octadr;
  mjtNum* oct_coeff = m->oct_coeff + 8*octadr;

  mjtNum w[8];
  mjtNum point[3] = {p[0], p[1], p[2]};
  mjtNum boxDist = boxProjection(point, oct_aabb);
  int node = findOct(w, NULL, oct_aabb, oct_child, point);

  // 三线性插值
  mjtNum sdf = 0;
  for (int i = 0; i < 8; ++i) {
    sdf += w[i] * oct_coeff[8*node + i];
  }
  return boxDist > 0 ? sdf + boxDist : sdf;
}
```

---

### 4.5 插件管理系统 (src/engine/engine_plugin.h, engine_plugin.cc)

**文件路径**: `src/engine/engine_plugin.h`, `src/engine/engine_plugin.cc`

#### 核心功能：

```cpp
// 注册插件
int mjp_registerPlugin(const mjpPlugin* plugin);

// 查找插件
const mjpPlugin* mjp_getPlugin(const char* name);

// 加载插件库
int mj_loadPluginLibrary(const char* path);
void mj_loadAllPluginLibraries(const char* dir, mjfPluginLibraryLoadCallback callback);
```

#### 注册入口 (register.cc)：

```cpp
namespace mujoco::plugin::sdf {

mjPLUGIN_LIB_INIT {
  Bolt::RegisterPlugin();
  Bowl::RegisterPlugin();
  Gear::RegisterPlugin();   // ← 注册 Gear 插件
  Nut::RegisterPlugin();
  Torus::RegisterPlugin();
}

}
```

---

## 5. 数据流与调用链

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           运行时调用流程                                     │
└─────────────────────────────────────────────────────────────────────────────┘

1. 模型加载阶段
   ┌──────────────┐
   │ 加载 gear.xml │
   └──────┬───────┘
          │
          ▼
   ┌─────────────────────────────┐
   │ 解析 <extension> 中的插件声明 │
   │ 调用 sdf_staticdistance 生成  │
   │ Marching Cubes 网格           │
   └──────────────┬──────────────┘
                  │
                  ▼
   ┌─────────────────────────────┐
   │ 调用 sdf_aabb 计算包围盒      │
   │ 存储到 mjModel                │
   └─────────────────────────────┘

2. 仿真运行时
   ┌─────────────────────────────┐
   │ 碰撞检测触发                  │
   │ (mjc_SDF / mjc_MeshSDF)      │
   └──────┬──────────────────────┘
          │
          ▼
   ┌─────────────────────────────┐
   │ 梯度下降优化寻找接触点         │
   │ 迭代调用 sdf_distance         │
   └──────┬──────────────────────┘
          │
          ▼
   ┌─────────────────────────────┐
   │ 计算法线方向                  │
   │ 调用 sdf_gradient             │
   └──────┬──────────────────────┘
          │
          ▼
   ┌─────────────────────────────┐
   │ 生成 mjContact 接触信息       │
   └─────────────────────────────┘

3. 可视化阶段
   ┌─────────────────────────────┐
   │ mjv_updateScene 调用         │
   │ plugin.visualize 回调         │
   └──────┬──────────────────────┘
          │
          ▼
   ┌─────────────────────────────┐
   │ SdfVisualizer 渲染梯度下降轨迹 │
   └─────────────────────────────┘
```

---

## 6. 关键设计模式

### 6.1 插件架构模式

| 模式 | 应用 |
|------|------|
| **策略模式** | 通过回调函数切换不同的 SDF 实现 |
| **工厂模式** | `Gear::Create()` 创建插件实例 |
| **模板方法** | `SdfDefault<T>` 提供属性默认值 |
| **RAII** | 插件生命周期管理（init/destroy） |

### 6.2 SDF 数学工具

| 操作 | 函数 | 数学表达 |
|------|------|----------|
| 并集 | `Union(a, b)` | `min(a, b)` |
| 交集 | `Intersection(a, b)` | `max(a, b)` |
| 差集 | `Subtraction(a, b)` | `max(a, -b)` |
| 平滑并集 | `smoothUnion(a, b, k)` | 多项式插值 |
| 挤压 | `extrusion(p, d2, h)` | 2D→3D 扩展 |

---

## 7. 文件清单

### 核心接口：
- `include/mujoco/mjplugin.h` - 插件系统接口定义
- `include/mujoco/mjmodel.h` - 模型数据结构
- `include/mujoco/mjdata.h` - 运行时数据结构

### 引擎实现：
- `src/engine/engine_plugin.h/cc` - 插件管理
- `src/engine/engine_collision_sdf.h/c` - SDF 碰撞检测

### SDF 插件实现：
- `plugin/sdf/sdf.h/cc` - SDF 工具类
- `plugin/sdf/gear.h/cc` - 齿轮插件
- `plugin/sdf/register.cc` - 插件注册入口

### 示例模型：
- `model/plugin/sdf/gear.xml` - 齿轮仿真示例

---

## 8. 总结

`gear.xml` 的实现依赖于 MuJoCo 的 **SDF 插件系统**，该系统采用分层架构设计：

1. **接口层** (`mjplugin.h`): 定义标准插件接口和 SDF 回调函数
2. **引擎层** (`engine_collision_sdf.c`): 提供碰撞检测和梯度下降算法
3. **插件层** (`plugin/sdf/`): 实现具体几何体的距离场计算
4. **工具层** (`sdf.h`): 提供布尔操作和可视化工具

Gear 插件的核心是**渐开线齿轮数学**，通过 2D SDF 挤压成 3D 几何体，支持平滑并集/交集操作。整个系统通过回调函数机制实现了高度的可扩展性，允许用户自定义任意 SDF 几何体。
