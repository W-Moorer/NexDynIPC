# IPC Toolkit 深度解析与移植指南 (Deep Dive & Porting Guide)

本文档基于 `temp/ipc-toolkit-main` 源码，深度剖析了 IPC (Incremental Potential Contact) 算法的数学核心、关键实现细节及集成策略。

## 1. 核心数学模型 (Mathematical Model)

IPC Toolkit 旨在将所有硬约束转化为光滑的势能函数，从而允许使用无约束优化求解。

### 1.1 势垒函数 (Barrier Potential)
*   **文件**: `src/ipc/barrier/barrier.cpp`
*   **公式**: 标准 Clamped Log Barrier (Li et al. 2020)
    $$ b(d) = -(d - \hat{d})^2 \ln\left(\frac{d}{\hat{d}}\right), \quad 0 < d < \hat{d} $$
    $$ b(d) = 0, \quad d \ge \hat{d} $$
    $$ b(d) = \infty, \quad d \le 0 $$
*   **关键导数实现**:
    *   **一阶导数 ($b'(d)$)**:
        $$ b'(d) = (\hat{d} - d) \left( 2\ln\left(\frac{d}{\hat{d}}\right) - \frac{\hat{d}}{d} + 1 \right) $$
    *   **二阶导数 ($b''(d)$)**: (用于 Hessian 构造)
        $$ b''(d) = \left(\frac{\hat{d}}{d} + 2\right)\frac{\hat{d}}{d} - 2\ln\left(\frac{d}{\hat{d}}\right) - 3 $$
    *   **代码参考**: `ipc::barrier_first_derivative`, `ipc::barrier_second_derivative`
*   **移植注意**:
    *   必须正确处理 $d \le 0$ 的情况（虽然理论上不会发生，但数值误差可能导致）。
    *   建议直接复制 `barrier.cpp` 中的 `CubicBarrier` 和 `ClampedLogSqBarrier` 实现，以备不时之需。

### 1.2 平滑摩擦模型 (Smooth Friction)
*   **文件**: `src/ipc/friction/smooth_mu.cpp`
*   **目的**: 消除标准库伦摩擦在相对速度为 0 处的不连续性，使优化问题变得光滑。
*   **模型**:
    $$ \mu(v_{rel}) = \mu_k + (\mu_s - \mu_k) \cdot m\left(\frac{\|v_{rel}\|}{\epsilon_v}\right) $$
    其中 $m(x)$ 是 $C^2$ 平滑过渡函数 (Mollifier)。
*   **核心函数**: `smooth_mu`, `smooth_mu_derivative`。
*   **参数**:
    *   $\epsilon_v$: 速度阈值，小于此速度被视为静摩擦状态。通常取 $10^{-3}$ 到 $10^{-4}$。

---

## 2. 连续碰撞检测 (CCD)

CCD 是 IPC 的基石，确保每一次迭代的线搜索步长都是无穿透的。

### 2.1 Tight Inclusion CCD
*   **文件**: `src/ipc/ccd/tight_inclusion_ccd.cpp`
*   **原理**: 使用区间算术 (Interval Arithmetic) 求解包含根的最小区间，具有极高的鲁棒性。
*   **策略 (`ccd_strategy`)**:
    1.  **初始检查**: `check_initial_distance`。
    2.  **保守缩放 (Conservative Rescaling)**:
        *   为了数值稳定性，实际步长会缩放为 $t_{safe} = (1 - \eta) \cdot t_{impact}$。
        *   参数 `conservative_rescaling` 通常取 $0.8$ 或 $0.9$。
    3.  **距离公差**: 根据初始距离动态调整公差 `adjusted_tolerance`，避免在远处进行过高精度的检测。

### 2.2 接口复用
*   **核心接口**: `NarrowPhaseCCD` 类。
*   **推荐移植**:
    *   `point_point_ccd`
    *   `point_edge_ccd`
    *   `edge_edge_ccd`
    *   `point_triangle_ccd`
    *   这些函数可以直接作为静态工具函数使用，不依赖于复杂的类层次结构。

---

## 3. 宽相碰撞检测 (Broad Phase)

### 3.1 Hash Grid
*   **文件**: `src/ipc/broad_phase/hash_grid.cpp`
*   **原理**: 将空间划分为大小为 $l$ 的格子，将所有图元映射到格子中。
*   **参数**: $l = \text{max\_edge\_length} + \text{inflation\_radius}$。
*   **优势**: 构建速度快，适合大规模自碰撞检测。

### 3.2 BVH (Bounding Volume Hierarchy)
*   **文件**: `src/ipc/broad_phase/bvh.cpp` (或类似)
*   **优势**: 对非均匀分布的场景更优，查询效率高。
*   **实现细节**:
    *   使用了 `AABB` (Axis-Aligned Bounding Box) 作为包围盒。
    *   支持动态更新 (Refitting) 或重建 (Rebuilding)。

---

## 4. 集成与移植建议

### 4.1 最小化集成路径 (Minimal Path)
若只是想在该物理引擎中使用 IPC 及其势能计算，无需引入整个 Toolkit，只需复制以下核心文件：

1.  **Potentials**:
    *   `ipc/barrier/barrier.hpp` & `.cpp`
    *   `ipc/friction/smooth_mu.hpp` & `.cpp`
    *   `ipc/friction/smooth_friction_mollifier.hpp` & `.cpp`

2.  **CCD**:
    *   `ipc/ccd/tight_inclusion_ccd.hpp` & `.cpp`
    *   (需引入 `tight_inclusion` 第三方库依赖，或使用简单的 `additive_ccd` 作为替代)

3.  **Utils**:
    *   `ipc/distance/` 下的所有距离计算函数 (Point-Triangle, Edge-Edge 等)。这是计算梯度和 Hessian 的基础。

### 4.2 依赖管理
*   **Eigen**: 必须。
*   **Tight Inclusion**: 这是一个独立的库，若难以集成，可以暂时使用 `libigl` 的 CCD 或简单的二分法 CCD 替代，但鲁棒性会下降。

### 4.3 关键参数调优
在移植后，需要重点调试以下参数以保证稳定性：
*   **Barrier Stiffness ($\kappa$)**: 初始值通常设为 $10^5 \sim 10^7$。需要实现自适应调整逻辑（如 `adaptive_stiffness.cpp` 中的逻辑）。
*   **$\hat{d}$ (dhat)**: 势垒激活距离。通常取场景包围盒对角线的 $10^{-3}$。
*   **Newton Tolerance**: 牛顿法收敛容差。能量容差 $10^{-5}$，速度容差 $10^{-2}$。
