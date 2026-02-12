# IPC Toolkit 项目深度分析

## 项目概述
`ipc-toolkit` 是实现 Incremental Potential Contact (IPC) 算法的核心数学库。它提供了接触势能计算、碰撞检测优化策略和连续碰撞检测 (CCD) 的基础组件，旨在被其他物理引擎集成。

## 目录结构分析 (`src/ipc/`)

### 1. 势能计算 (`src/ipc/barrier`, `src/ipc/friction`)
这是 IPC 的核心数学公式实现。

*   **势垒函数 (`src/ipc/barrier/barrier.hpp`)**:
    *   **核心功能**: 实现了多种势垒函数及其一阶、二阶导数。这直接用于构建无穿透约束的能量项。
    *   **关键类**:
        *   `Barrier`: 抽象基类。
        *   `ClampedLogBarrier`: 标准 IPC (Li et al. 2020) 使用的对数势垒。公式：$b(d) = -(d-\hat{d})^2 \ln(d/\hat{d})$。
        *   `CubicBarrier`: 三次多项式势垒 (Ando 2024)，在某些情况下更平滑。
        *   `NormalizedBarrier`: 用于无量纲化处理。
    *   **复用价值**: 极高。如果不想手写复杂的导数公式，直接使用这里的代码是最稳健的选择。

*   **摩擦模型 (`src/ipc/friction/smooth_friction_mollifier.hpp`)**:
    *   **核心功能**: 实现了平滑化的库伦摩擦力模型。
    *   **亮点**: 提供了 $C^2$ 连续的摩擦力过渡函数，这对牛顿法收敛至关重要，避免了因摩擦力非光滑导致的震荡。
    *   **关键函数**: `smooth_mu`, `smooth_mu_derivative` 等。

### 2. 连续碰撞检测 (`src/ipc/ccd`)
处理运动物体间的碰撞时刻计算。

*   **接口 (`src/ipc/ccd/narrow_phase_ccd.hpp`)**:
    *   定义了 `point_point_ccd`, `point_edge_ccd`, `point_triangle_ccd`, `edge_edge_ccd` 四种基本图元的检测接口。
*   **实现 (`src/ipc/ccd/tight_inclusion_ccd.cpp`)**:
    *   集成了 Tight Inclusion CCD 算法，这是目前鲁棒性最高的 CCD 算法之一。
    *   **复用价值**: 高。CCD 是物理引擎中最容易出错的环节，复用这里的成熟实现可以避免大量数值稳定性问题。

### 3. 宽相碰撞检测 (`src/ipc/broad_phase`)
通过空间划分加速碰撞检测。

*   **策略 (`src/ipc/broad_phase/broad_phase.hpp`)**:
    *   提供了统一的 `BroadPhase` 接口。
    *   `HashGrid`: 简单的空间哈希网格，适合均匀分布的场景。
    *   `SpatialHash`: 更高级的空间哈希实现。
    *   `SweepAndPrune`: 扫掠修剪算法。
    *   `BVH` (Bounding Volume Hierarchy): 层次包围盒树，通常是最通用的选择。
    *   **关键方法**: `detect_collision_candidates()` 返回潜在的碰撞对列表 (`Candidates`)。

### 4. 碰撞候选管理 (`src/ipc/candidates`)
管理宽相检测出的潜在碰撞对。

*   **核心类**:
    *   `Candidates`: 存储所有类型的碰撞候选 (VV, EV, EE, FV, FF)。
    *   `EdgeEdgeCandidate`, `FaceVertexCandidate`: 具体的碰撞对结构，存储了图元索引。
    *   **复用价值**: 如果你在构建自己的物理引擎，可以直接复用这套数据结构来管理碰撞对。

## 关键代码复用建议

1.  **Barrier Implementation**: `ClampedLogBarrier` 的实现非常精细，处理了 $d \to 0$ 和 $d \to \hat{d}$ 的边缘情况，建议直接复制 `barrier.hpp` 和 `barrier.cpp` 到项目中。
2.  **CCD Integration**: 如果现有的 CCD 不够稳定（例如漏测或误报），可以考虑将 `NarrowPhaseCCD` 接口及其 Tight Inclusion 实现移植过来。
3.  **Adaptive Stiffness**: `src/ipc/potentials/adaptive_stiffness.cpp` 实现了自适应调整势垒刚度 $\kappa$ 的逻辑，这是 IPC 算法鲁棒性的关键，可以作为高级功能引入。
