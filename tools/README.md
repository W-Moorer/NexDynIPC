# NexDynIPC 仿真工具集

本目录包含用于分析和对比仿真结果的实用脚本。

## 脚本列表

### 1. `compare_parameters.py`
**用途**：对比两个不同仿真设置（如不同的刚度 `mu` 或数值阻尼 `gamma`）生成的 CSV 结果文件，用于灵敏度分析。
**更新时间**：2026-02-10

#### 用法
```bash
python tools/compare_parameters.py <文件1.csv> <文件2.csv> [--output 输出图片.png]
```

#### 示例
对比不同刚度值的仿真结果：
```bash
python tools/compare_parameters.py output/sim_mu_1000.csv output/sim_mu_10000.csv --output output/mu_comparison.png
```

---

### 2. `compare_with_reference.py`
**用途**：将单个仿真结果 CSV 文件与一组参考数据文件（如 RecurDyn 或 Adam 导出的数据）进行对比。参考数据通常每个刚体对应多个独立的 CSV 文件（分别记录位移、速度、加速度）。
**更新时间**：2026-02-10

#### 用法
```bash
python tools/compare_with_reference.py <仿真结果.csv> <参考数据目录> [--output 输出图片.png]
```

#### 示例
将仿真结果与位于 `assets/reference_data/double_pendulum/` 的参考数据进行对比：
```bash
python tools/compare_with_reference.py output/simulation_results.csv assets/reference_data/double_pendulum/ --output output/reference_comparison.png
```

#### 输出图表颜色含义
- **蓝色 (Blue)**: 位移误差 (Position Divergence)
- **绿色 (Green)**: 速度误差 (Velocity Divergence)
- **橙色 (Orange)**: 加速度误差 (Acceleration Divergence)

---

### 3. `csv_visualizer.py`
**用途**：通用的 CSV 数据可视化工具，可以绘制 CSV 文件中任意列的数据曲线。支持多子图显示。
**更新时间**：2026-02-10

#### 用法
```bash
python tools/csv_visualizer.py <csv文件> [--columns 列名1 列名2 ...]
```

#### 示例
绘制 CSV 文件中的所有列：
```bash
python tools/csv_visualizer.py output/simulation_results.csv
```

#### 指定精确输出目录
当希望图像直接输出到 `figures/<case_name>/`（不再追加CSV文件名子目录）时：
```bash
python tools/csv_visualizer.py output/motor_torque_saturation/motor_torque_saturation.csv -o figures/motor_torque_saturation --exact-output
```

---

### 4. `run_asset_suite.py`
**用途**：批量运行 `assets/*/*.json` 场景，自动生成：
- 仿真 CSV：`output/<case_name>/`
- 可视化图：`figures/<case_name>/`
- 汇总结果：`output/asset_suite_summary.csv`

#### 用法
```bash
python tools/run_asset_suite.py --exe build/Debug/NexDynIPC.exe
```

---

### 5. `rmd_export_to_project.py`
**用途**：从 `.rmd` 导出网格 `OBJ` 与当前项目可解析的 JSON（`bodies/joints/settings` 结构）。

- OBJ 输出目录固定为：`models/obj/{rmd文件名}/`
- JSON 默认输出为：`{rmd同目录}/{rmd文件名}.json`
- 关键参数映射：
	- `PART.MASS -> bodies[].mass`
	- `PART.CM + MARKER.QP -> bodies[].position`
	- `PART.IP(6分量) -> bodies[].inertia(3x3)`
	- `JOINT.Fixed/Revolute -> joints[].fixed/hinge`

#### 用法
```bash
python tools/rmd_export_to_project.py <rmd文件路径>
```

#### 示例
```bash
python tools/rmd_export_to_project.py assets/examples/example_basic_contact/basic_contact.rmd
```

#### 可选参数
```bash
python tools/rmd_export_to_project.py <rmd文件路径> \
	--json-output assets/examples/example_basic_contact/basic_contact.generated.json \
	--reuler-unit auto
```

---

## 依赖库
这些脚本依赖以下 Python 库：
- `pandas`: 用于读取和处理 CSV 数据。
- `numpy`: 用于数值运算（如计算均方根误差 RMSE、插值）。
- `matplotlib`: 用于绘制对比图表。

安装依赖：
```bash
pip install pandas numpy matplotlib
```
