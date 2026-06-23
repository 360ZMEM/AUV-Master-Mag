# AUV-Master-Mag 极简重构方案 (Spec v0.1)

> **目标读者**：项目作者本人 + 后续 AI 协作者
> **生成日期**：2026-06-22
> **依据**：`极简重构法则.md`、`tools/health_report_case1.md`、`原理与代码详解.md`、当前代码库静态分析
> **状态**：Draft — 待用户 Review 后进入 Phase 0 执行

---

## 0. TL;DR

当前仓库分层骨架仍然成立（`environment / sensor / perception / controller / main_viz`），但中间层 `perception.py` (2218 行) 与 `behavior_tree.py` 在反复迭代后形成了「**标准 + 部署 + safe-lock + 矢量 + 梯度 + 盲启动**」六套并行兜底逻辑。case1 实测的核心反差是：

> *表面达标 (mean_heading_error 2.91°)，但 80% 的航向输出来自 BOOTSTRAP_OVERRIDE，磁导航实际只贡献 20%；safe_lock_frames=0；整局只捕到 2 个磁峰。*

按照 `极简重构法则.md` 的奥卡姆剃刀原则，重构终态：

- **三态 FSM**：`SEARCH_ZIGZAG → LOCK_ALIGN → TRACK_ACTIVE`，替代当前 5 态 + 6 分支 SearchNode + 多段消歧。
- **极简先验**：3-4 个 waypoints (`±10–30 m` 公差带) 取代「盲启动 / deployment_*」整套兜底机制。
- **perception 拆包**：`perception.py` 拆为 7 个 ≤ 300 行的文件。
- **契约收敛**：`PerceptionState` 一拆为二，`PerceptionResult` (~19 字段) 是控制器硬契约，`PerceptionDiagnostics` 仅供 UI/log。
- **新增能力**：`MagneticBurialInverter` 把"埋深反演"从伪观测升级为真磁法 (peak_amplitude + 电流 → 距离 → 埋深)。

> **总纲 — 高度可维护**：每个模块设计都要"有智慧"，算法简单而精妙；拒绝徒增内容、无效兜底与未触发的死分支；优先让结构便于人工修改（职责单一、参数集中、命名自解释）。这是贯穿所有 Phase 的硬约束，而非某个阶段的任务。

---

## 1. 当前架构（病灶视图）

```mermaid
flowchart TB
    subgraph Entry["main_demo.py"]
        A1[parse_args]:::ok --> A2[build_default_scenarios]:::ok --> A3[AuvCableTrackingSimulation.run]:::ok
    end

    A3 --> SIM[main_viz.py 748 行]:::ok

    SIM --> ENV[environment.py]:::ok
    SIM --> MAG[sensor_model.py]:::warn
    SIM --> DRV[perception_driver.py 477 行]:::warn
    SIM --> P[perception.py<br/>2218 行 / 11 类]:::bad
    SIM --> CTL[controller.py 283 行]:::warn
    CTL --> BT[behavior_tree.py 463 行]:::warn

    subgraph Perception_Inner["perception.py 内部"]
        F1[StreamingBandpassFilter<br/>RMSExtractor]:::dup
        F2[PeakDetector + 3 dataclass]:::ok
        F3[CableRouteFitter — DEAD]:::dead
        F4[WeightedSlidingWindowFitter]:::ok
        F5[MagneticVectorAnalyzer<br/>StreamingVectorPCAFitter]:::dead
        F6[EnvelopeGradientTracker]:::warn
        F7[ConfidenceEstimator]:::ok
        F8[_update_deployment_cable_heading<br/>3× ±90° 消歧 / 4 个 Step]:::bad
        F9[Safe-Lock Criteria A/B — 整段注释]:::dead
    end

    subgraph BT_Inner["behavior_tree.py SearchNode"]
        B1[BOOTSTRAP_SPIRAL]:::dup
        B2[REACQUIRE_SPIRAL]:::dup
        B3[DEPLOYMENT_SPIRAL]:::dup
        B4[RECOVERY_SPIRAL]:::dup
        B5[BLIND_INERTIA]:::dup
        B6[BLIND_RECOVERY]:::dup
    end

    subgraph Dup["重复构造（Code Duplication）"]
        D1[StreamingBandpassFilter perception]:::dup
        D2[ScalarStreamingBandpassFilter driver]:::dup
        D3[RMSExtractor perception]:::dup
        D4[SlidingWindowRMS driver]:::dup
        D5[_build_nominal_route_xy perception]:::dup
        D6[_build_nominal_route_xy controller]:::dup
    end

    subgraph Orphan["与目标正交（可弃）"]
        O1[simulator_connector.py 占位 stub]:::dead
        O2[HighFidelityMagnetometer + case_hf_*]:::warn
        O3[tools/phyphox_fft_demo.py]:::warn
        O4[支线/磁正交校准/]:::warn
        O5[tools/{debug_traj,test_override,trace_*}.py 4 个一次性 artifact]:::dead
    end

    classDef ok fill:#cfe9c8,stroke:#3d8b37
    classDef warn fill:#fbe6a2,stroke:#a07a25
    classDef bad fill:#f5b8b8,stroke:#a83232
    classDef dead fill:#cfcfcf,stroke:#666,stroke-dasharray: 3 3,color:#666
    classDef dup fill:#e7c7f0,stroke:#783793
```

**色彩约定**：绿 = 健康；黄 = 局部冗余；红 = 严重技术债；灰 = 死代码；紫 = 重复构造。

### 1.1 health_report_case1.md 的事实数字

| 维度 | 数值 | 设计预期 | 偏离 |
|---|---|---|---|
| 总步数 / 时长 | 4000 / 199.95 s | — | — |
| 捕到的磁峰 | **2** | 至少 5+ | ❌ 核心机制几乎没触发 |
| 平均置信度 | 0.388 | ≥ 0.6 | ❌ 长期低 |
| safe_lock_frames | **0** | 应在稳态进入 | ❌ 永不锁定 |
| 模式切换 | 4 次 | < 10 | ✅ 稳定 |
| 平均/最终航向误差 | 2.91° / 0.48° | < 15° / < 5° | ✅ 达标 |
| 横向偏差 mean / max | 6.54 / 21.39 m | max ≈ 初始偏置 | ⚠️ max 是初始 10 m 的 2 倍 |
| 航向来源占比 | BOOTSTRAP 80% / MAGNETIC 20% / MAGNETIC_PEAK 0% | MAGNETIC 主导 | ❌ 磁导航被旁路 |

---

## 2. 目标架构（极简重构后）

```mermaid
flowchart TB
    subgraph EntryV2["main_demo.py（不变）"]
        E1[parse_args] --> E2[build_default_scenarios] --> E3[Simulation.run]
    end

    E3 --> SIMV2[simulation/runner.py<br/>≤ 200 行]

    SIMV2 --> ENVV2[environment.py]:::ok
    SIMV2 --> SENS[sensors/<br/>magnetometer + sonar + imu + burial]:::ok
    SIMV2 --> PERC[perception/<br/>7 个文件]:::new
    SIMV2 --> MGR[mission_manager.py<br/>三态 FSM, ~120 行]:::new
    MGR --> CTLV2[controller.py<br/>纯运动学, ~150 行]:::ok

    subgraph PercPkg["perception/ 包"]
        PP1[filters.py<br/>低通+中值+带通+RMS]
        PP2[peaks.py<br/>PeakDetector + dataclasses]
        PP3[fitter.py<br/>WeightedSlidingWindowFitter]
        PP4[heading.py<br/>HeadingFusion 单实现]
        PP5[confidence.py<br/>ConfidenceEstimator + zigzag_width 反映射]
        PP6[burial_inversion.py<br/>★ 新增 MagneticBurialInverter]:::new
        PP7[state.py<br/>PerceptionResult + Diagnostics]
        PP8[orchestrator.py<br/>MagneticCablePerception.update]
    end

    subgraph FSM["mission_manager.py 三态 FSM"]
        S1[SEARCH_ZIGZAG<br/>沿 prior_waypoints 横摆扫描]:::new
        S2[LOCK_ALIGN<br/>降速 0.5×, EKF 收敛]:::new
        S3[TRACK_ACTIVE<br/>声磁协同闭环]:::new
        S1 -->|连续 5 帧 mag_strength>thr<br/>or sonar_conf>thr| S2
        S2 -->|EKF P_y < 1.0<br/>and yaw_err < 5°| S3
        S2 -->|信号丢失| S1
        S3 -->|system_conf < 0.1| EM[EMERGENCY_SURFACE]
        S3 -->|信号退化| S2
    end

    subgraph Removed["Phase 0 删除项"]
        R1[CableRouteFitter 死类]:::dead
        R2[Safe-Lock A/B 注释段]:::dead
        R3[vector_cable_heading 死分支]:::dead
        R4[simulator_connector.py]:::dead
        R5[tools/{debug_traj,test_override,trace_*}.py]:::dead
        R6[deployment_*  整套子树]:::dead
        R7[6 分支 SearchNode]:::dead
    end

    classDef ok fill:#cfe9c8,stroke:#3d8b37
    classDef new fill:#bcd9f0,stroke:#2766a6,stroke-width:2px
    classDef dead fill:#cfcfcf,stroke:#666,stroke-dasharray: 3 3,color:#666
```

**关键变化**：
1. `behavior_tree.py` 删除（463 行 → 0），由 [mission_manager.py](file:///Users/bytedance/coding/AUV-Master-Mag/src/auv_mag_tracking/mission_manager.py) 三态 FSM 取代（~120 行）。
2. `perception.py` (2218 行) → `perception/` (≤ 300 行 × 8) ≈ **总行数减半**。
3. `controller.py` 退回纯运动学层，所有"模式策略 / 启动概念 / 35° 强制角"上提到 `mission_manager`。
4. 新增 `perception/burial_inversion.py`，落地真正的磁法埋深反演。

---

## 3. 三态 FSM 详细规范

### 3.1 状态定义（与 `极简重构法则.md` 对齐）

```python
class MissionState(Enum):
    SEARCH_ZIGZAG = "search"   # 沿 prior_waypoints 做横摆扫描
    LOCK_ALIGN    = "align"    # 检到信号，降速对齐
    TRACK_ACTIVE  = "track"    # 声磁协同稳态闭环
```

### 3.2 输入接口（极简）

```python
@dataclass
class PriorWaypointsRoute:
    """工业先验路由：3–4 个 waypoints + 公差带。"""
    waypoints_xy_m: np.ndarray  # shape (N, 2), N ∈ {3, 4, 5}
    tolerance_band_m: float = 30.0  # ±30 m 误差宽度（默认）
```

### 3.3 状态转移判据（数据驱动，无补丁）

| From → To | 触发条件 | 物理含义 |
|---|---|---|
| `SEARCH_ZIGZAG → LOCK_ALIGN` | `mag_strength > MAG_LOCK_THRESHOLD` 或 `sonar_conf > SONAR_CONF_THRESHOLD` 连续 ≥ 5 帧 | 横摆穿越电缆产生单峰脉冲 |
| `LOCK_ALIGN → TRACK_ACTIVE` | EKF `P_yy < 1.0 m²` 且 `yaw_err < 5°` | 滤波器收敛到稳态 |
| `LOCK_ALIGN → SEARCH_ZIGZAG` | `mag_strength < MAG_LOCK_THRESHOLD` 且 `sonar_conf < SONAR_CONF_THRESHOLD` 连续 ≥ 3 帧 | 信号丢失 |
| `TRACK_ACTIVE → LOCK_ALIGN` | 退化：单一传感器失效 | 优雅降级 |
| `TRACK_ACTIVE → EMERGENCY_SURFACE` | `system_confidence < 0.1` ≥ 5 s | 双盲，紧急上浮 |

### 3.4 阈值集中（写到 `TrackingConfig` 顶部）

```python
@dataclass
class MissionThresholds:
    mag_lock_threshold_nT: float       = 50.0    # 磁场增量阈值
    sonar_confidence_threshold: float  = 0.6     # 声呐置信度阈值
    lock_streak_required: int          = 5       # 锁定计数 (帧)
    loss_streak_required: int          = 3       # 丢失计数 (帧)
    align_speed_factor: float          = 0.5     # LOCK_ALIGN 阶段降速比
    ekf_pyy_converged_m2: float        = 1.0
    ekf_yaw_err_converged_deg: float   = 5.0
    system_confidence_floor: float     = 0.1
```

> 所有可调阈值置于配置顶部 — 符合用户偏好"用户可配置变量集中"。

---

## 4. perception/ 包拆分接口

### 4.1 文件清单

| 文件 | 职责 | 行数预算 | 来源 |
|---|---|---|---|
| `perception/__init__.py` | 公开导出 `MagneticCablePerception, PerceptionResult, PerceptionDiagnostics` | ≤ 30 | 新建 |
| `perception/filters.py` | `LowPassFilter / MedianWindowFilter / StreamingBandpassFilter / RMSExtractor`，**与 driver 副本合并** | ≤ 250 | 抽自 perception.py + driver |
| `perception/peaks.py` | `PeakDetector / PeakEvent / PeakObservation / PeakZoneSample` | ≤ 220 | 抽自 perception.py |
| `perception/fitter.py` | `WeightedSlidingWindowFitter / FitResult / weighted_pca_line_fit()` | ≤ 200 | 抽自 perception.py（删 `CableRouteFitter`） |
| `perception/heading.py` | `HeadingFusion`（单实现，**删除部署/标准双轨**） | ≤ 180 | 重写 |
| `perception/confidence.py` | `ConfidenceEstimator + inverse_confidence_zigzag_width()` | ≤ 150 | 抽自 perception.py |
| `perception/burial_inversion.py` | ★ **新增** `MagneticBurialInverter` | ≤ 200 | 新建 |
| `perception/state.py` | `PerceptionResult` (硬契约) + `PerceptionDiagnostics` (UI/log) | ≤ 120 | 抽自 perception.py |
| `perception/orchestrator.py` | `MagneticCablePerception.update()`，仅做编排 | ≤ 250 | 重写 |

总计：**约 1600 行 → 比当前 2218 行 + 477 行 driver 减少 ≈30%**，且每个文件职责单一。

### 4.2 契约一拆为二

```python
# perception/state.py

@dataclass
class PerceptionResult:
    """控制器与 mission_manager 消费的硬契约（19 字段）。"""
    time_s: float
    mag_strength_nT: float           # ← 替代 tracking_strength_nt
    sonar_confidence: float          # ← 来自 sonar_status 量化
    confidence: float                # 综合置信度
    fused_heading_deg: Optional[float]
    estimated_cable_xy_m: Optional[np.ndarray]
    fit_residual_m: float
    fit_direction_xy: Optional[np.ndarray]
    peak_detected: bool
    peak_count_total: int
    last_detection_age_s: float
    estimated_burial_depth_m: Optional[float]
    burial_uncertainty_m: Optional[float]
    snr_db: float
    weak_signal_flag: bool
    zigzag_width_m: float
    sonar_status: str
    last_peak_xy_m: Optional[np.ndarray]
    estimated_path_xy_m: np.ndarray  # for visualization line


@dataclass
class PerceptionDiagnostics:
    """仅 main_viz / log 使用，可随便加。"""
    sensor_field_nT: np.ndarray
    body_field_nT: np.ndarray
    ned_field_nT: np.ndarray
    rms_strength_nT: float
    noise_floor_nT: float
    is_ac_detected: bool
    dominant_frequency_hz: float
    signal_reliable: bool
    line_heading_deg: Optional[float]
    estimated_path_covariance_xy_m2: Optional[np.ndarray]
    true_burial_depth_m: float
    burial_measurement_valid: bool
    fit_update_rejected: bool
    # …其余 UI 用字段
```

> `MagneticCablePerception.update()` 返回 `Tuple[PerceptionResult, PerceptionDiagnostics]`。
> Controller / mission_manager **只接收** `PerceptionResult`。

---

## 5. 磁法埋深反演设计

### 5.1 物理基础

无限长直导线的横向磁场：

$$B_\perp(d) = \frac{\mu_0 I}{2\pi d}$$

其中 `d = sqrt(lateral_offset² + burial_depth²)`。在 zig-zag 穿越时记录峰值幅度 `B_peak`，结合电流 `I` 与 AUV 高度（已知）即可反演 `d`，进一步分离出 `burial_depth`。

### 5.2 接口

```python
# perception/burial_inversion.py

@dataclass
class BurialEstimate:
    depth_m: float
    sigma_m: float                   # 1σ 不确定度
    fit_quality: float               # [0,1]


class MagneticBurialInverter:
    """基于峰值幅度的磁法埋深反演器。"""

    def __init__(self, current_A: float, auv_altitude_m: float,
                 mu0_over_4pi_nT_per_A_per_m: float = 1e-7):
        ...

    def update(self, peak_event: PeakEvent,
               lateral_offset_m: float) -> BurialEstimate:
        """单次峰值更新。"""
        ...

    def reset(self) -> None:
        ...
```

### 5.3 与 BurialDepthObserver 的关系

- **保留** `BurialDepthObserver` 作为仿真"真值通道"（GT），仅用于评估 inverter 误差。
- `PerceptionResult.estimated_burial_depth_m` 改为 inverter 输出。
- 在 `health_report` 中新增 `burial_inversion_error_mean_m` 指标。

---

## 6. Phase 0 安全删除清单（零风险，1-2h）

> **判定**：删除后 `case1/2/3/4/5` 行为不变（数值一致，单元测试通过）。

### 6.1 死代码

| 项 | 位置 | 原因 |
|---|---|---|
| `CableRouteFitter` 类 | [perception.py:~800-844](file:///Users/bytedance/coding/AUV-Master-Mag/src/auv_mag_tracking/perception.py#L800-L844) | 从未实例化，被 `WeightedSlidingWindowFitter` 取代 |
| Safe-Lock A/B 整段（注释 + 强制 False） | [perception.py:~2061-2090](file:///Users/bytedance/coding/AUV-Master-Mag/src/auv_mag_tracking/perception.py#L2061-L2090) | 永远 False，下游 penalty 不可达 |
| `vector_cable_heading_deg = None` (TEMPORARY ISOLATION) 死分支 | [perception.py:1828-1852](file:///Users/bytedance/coding/AUV-Master-Mag/src/auv_mag_tracking/perception.py#L1828-L1852) | 强制置 None 后下游已死 |
| `weighted_ransac_iterations / _inlier_threshold_m / _min_inlier_ratio` 配置项 | `config/__init__.py` | 代码未实现 RANSAC，纯死配置 |
| `tracking.zigzag_width_gain_m_per_nt` | `config/__init__.py` | 已被 inverse-confidence mapping 取代 |
| `magnetic_takeover_strength_nt` | `config/__init__.py` | grep 无任何调用 |

### 6.2 一次性调试 artifact（不影响功能）

| 文件 | 删除 |
|---|---|
| [tools/debug_traj.py](file:///Users/bytedance/coding/AUV-Master-Mag/tools/debug_traj.py) | ✅ |
| [tools/test_override.py](file:///Users/bytedance/coding/AUV-Master-Mag/tools/test_override.py) | ✅ |
| [tools/trace_deploy_update.py](file:///Users/bytedance/coding/AUV-Master-Mag/tools/trace_deploy_update.py) | ✅ |
| [tools/trace_heading_dist.py](file:///Users/bytedance/coding/AUV-Master-Mag/tools/trace_heading_dist.py) | ✅ |
| `test_deployment_debug.py`（根目录） | ✅（手动 smoke 工具，不在 pytest 集合中） |

### 6.3 与目标正交（Phase 0 不删，Phase 5 再说）

| 项 | 处理 |
|---|---|
| `simulator_connector.py` | 标记 `# DEPRECATED` 并从 `__init__.py` 移出导出，文件保留至 Phase 5 |
| `HighFidelityMagnetometer` + `case_hf_phone/_industrial` | 移到 `experimental/`，不删 |
| `tools/phyphox*` + `phyphox_adapter.py` | 移到 `experimental/`，与硬件演示一起保留 |
| `支线/磁正交校准/` | 完全独立子项目，不动 |

### 6.4 保留的 tools

- [tools/diagnose_heading_error.py](file:///Users/bytedance/coding/AUV-Master-Mag/tools/diagnose_heading_error.py) ✅ — 长期 health-report 工具
- [tools/sweep_tracking_params.py](file:///Users/bytedance/coding/AUV-Master-Mag/tools/sweep_tracking_params.py) ✅ — 扫参工具

---

## 7. Phase 1-4 路线（顺序执行）

### Phase 1：拆分 perception（0.5–1 d）
- **纯机械拆分**（verbatim）：把 `perception.py` 的 16 个类/dataclass 逐字抽到 `perception/` 包，`__init__.py` 重导出全部公开符号，5 个外部 importer 零改动。Phase 1 不做任何逻辑重写（`HeadingFusion` 单实现、`MagneticBurialInverter` 新增均属 Phase 2/4）。
- 实际文件落地（与 §4.1 接口对齐，但本阶段不删/不改逻辑）：
  - `state.py` — `FitResult / PeakEvent / PeakObservation / PeakZoneSample / PerceptionState`
  - `filters.py` — `LowPassFilter / MedianWindowFilter / StreamingBandpassFilter / RMSExtractor`
  - `peaks.py` — `PeakDetector`
  - `vector.py` — `EnvelopeGradientTracker / StreamingVectorPCAFitter / MagneticVectorAnalyzer`
  - `fitter.py` — `WeightedSlidingWindowFitter`
  - `confidence.py` — `ConfidenceEstimator`
  - `orchestrator.py` — `MagneticCablePerception`
  - `__init__.py` — 重导出全部公开符号
- **决议（2026-06-23，用户确认）**：driver 的 `ScalarStreamingBandpassFilter / SlidingWindowRMS` **本阶段不合并**。它们与 perception 滤波器 API 不同（标量块 vs 三轴向量、增量 RMS vs `np.mean`），强行合并会引入 `orchestrator → perception_driver → perception.filters` 循环导入且无法保证 byte-for-byte。该整合推迟到 Phase 3 契约收敛一并处理。
- **验收**：`python main_demo.py --case case1..5 --no-viz` 输出与重构前 byte-for-byte 一致；`pytest tests/` 失败集不新增（基线 12 failed / 41 passed，均属 Phase 2 将重写部分）。

### Phase 2：替换 behavior_tree → mission_manager（1–2 d）
- 新建 [mission_manager.py](file:///Users/bytedance/coding/AUV-Master-Mag/src/auv_mag_tracking/mission_manager.py) 三态 FSM（按 §3 实现）。
- 新建 `routes/prior_waypoints.py` 数据结构。
- 删除 [behavior_tree.py](file:///Users/bytedance/coding/AUV-Master-Mag/src/auv_mag_tracking/behavior_tree.py) 与 `perception._update_deployment_cable_heading` 整套部署分支。
- `controller.py` 退回纯运动学 + 单一航向 PID。
- **验收**：case1 健康报告 `MAGNETIC_PEAK` 占比 > 5%；模式切换 ≤ 6 次；mean_heading_error ≤ 5°。

### Phase 3：契约收敛（0.5 d）
- 把 controller 中所有 magic numbers (35°、`expected_cross_time = max(w*2.5/v, 10)`、`lookahead = max(2*r_min, 10)`、`heading low-pass = 0.1`) 提到 `TrackingConfig`。
- `BehaviorContext` 完全删除。
- 统一 nominal route：`CableEnvironment` 持有 `NominalRouteCache`，perception 与 controller 共享。
- **验收**：`grep -nE "(35\.0|0\.1\b)" src/` 不应再出现裸数。

### Phase 4：埋深反演落地（1 d，与 Phase 1 并行）
- 实现 `MagneticBurialInverter`（§5）。
- `health_report` 新增 `burial_inversion_error_mean_m`。
- **验收**：case1 反演埋深与真值的 MAE < 0.5 m。

### Phase 5（可选，未来）：实验性模块下沉
- `experimental/{high_fidelity_mag, phyphox, simulator_connector}` 隔离。

### Phase 6（TODO，代码全绿后执行）：docs/ 全面重构
> **触发条件**：Phase 1-4 完成、代码定型后再启动，避免文档反复返工。

**重构目标（呼应"高度可维护"总纲）**：

- **单一信息源**：当前根目录散落 `README.md`、`原理与代码详解.md`、`极简重构法则.md`、`docs/REFACTOR_PLAN.md`、`tools/health_report_case1.md`，存在重复与漂移。统一收敛到 `docs/`，建立清晰目录树。
- **建议目录结构**：
  - `docs/README.md`（项目入口，3 分钟跑通）
  - `docs/architecture.md`（洋葱分层 + 三态 FSM 数据流，含 mermaid 源 + 导出 PNG/PDF）
  - `docs/perception.md`（信号链：带通→RMS→峰值→拟合→置信度→埋深反演，每步配公式）
  - `docs/mission_fsm.md`（三态 FSM 状态机、转移判据、阈值表）
  - `docs/burial_inversion.md`（磁法反演物理推导 + 验证结果）
  - `docs/config_reference.md`（所有可配置参数集中说明，对应 `TrackingConfig`/`MissionThresholds`）
  - `docs/dev_log/`（历史决策与 health-report 归档，与现行文档分离）
- **删冗原则**：`极简重构法则.md` 的设计精神并入 `architecture.md`，原文件移入 `docs/dev_log/`；过期 health-report 归档。
- **文档即契约**：每个 `perception/` 模块文件头部 docstring 与对应 `docs/*.md` 章节一一对应，便于人工修改时同步定位。
- **验收**：根目录除 `README.md` 外无散落设计文档；`docs/` 目录树自洽；每个公开模块都能在 docs 找到对应章节。

---

## 8. 验收标准（每阶段都要通过）

```bash
# 1. 数值回归
python main_demo.py --case case1 --no-viz
python main_demo.py --case case2 --no-viz
python main_demo.py --case case3 --no-viz
python main_demo.py --case case4 --no-viz

# 2. 单元测试
python -m unittest discover -s tests

# 3. 健康报告对比
python tools/diagnose_heading_error.py --case case1
diff <(prev_report) <(new_report)  # 关键指标在 ±10% 内
```

**Phase 2 之后的额外指标改善目标**：

| 指标 | 当前 (case1) | 目标 |
|---|---|---|
| MAGNETIC_PEAK 占比 | 0% | ≥ 5% |
| 总磁峰数 | 2 | ≥ 5 |
| safe_lock 等价 (TRACK_ACTIVE 帧占比) | 0% | ≥ 30% |
| mean_heading_error | 2.91° | ≤ 5° (维持) |
| max_lateral_dev_m | 21.39 m | ≤ 12 m |
| perception.py 单文件行数 | 2218 | 0 (拆包后) |
| 文件总数（src/auv_mag_tracking/） | 10 | 12-15 |
| BehaviorContext 字段数 | 30 | 0（删除） |
| PerceptionState 字段数 | 55 | 拆为 19 + 25 |

---

## 9. 风险与回滚

| 风险 | 缓解 |
|---|---|
| Phase 1 拆包过程中行为意外漂移 | 每阶段开 git 分支；保留旧 perception.py 直至全绿 |
| Phase 2 三态 FSM 在 case2 折线上不收敛 | 预留 `MissionThresholds` scenario-level 覆盖；最坏情况引入 `TURN_PEAK` 第四态 |
| 埋深反演公式在折线段失真 | 反演只在 fit_residual_m < 1.5 m 时启用；其余采用上一次有效估计 |
| 测试覆盖不全 | Phase 0 之前先补充 `tests/test_perception_orchestration.py` |

---

## 10. 决议清单（待用户确认）

| # | 决议项 | 默认 | 备选 |
|---|---|---|---|
| D1 | 启动 Phase 0 删除 | ✅ 等本文件确认后立即开 `refactor/phase0` 分支 | 先做 D2-D3 |
| D2 | 三态 FSM 命名 | `SEARCH_ZIGZAG / LOCK_ALIGN / TRACK_ACTIVE`（与 `极简重构法则.md` 一致） | 沿用 `SEARCH/APPROACH/HOLD` |
| D3 | 是否保留 `experimental/` 子模块 | ✅ 保留，不删除 HF / phyphox | 直接 rm |
| D4 | 埋深反演与 Phase 1 并行 | ✅ 同步推进 | 串行（先重构再反演） |
| D5 | 是否引入 EKF 模块 | 🟡 LOCK_ALIGN 状态判据需要 `P_yy`，若现无 EKF，可先用 `fit_residual_m` 代理 | 立即引入 EKF |

---

**附录 A：mermaid 图渲染说明**

本文件中的 `mermaid` 图块需要在支持 mermaid 的 Markdown 渲染器中查看（VSCode + Markdown Preview Enhanced，或 GitHub）。如需导出 PNG/PDF（IEEE/HKU 风格），可在 Phase 1 完成后用 `mmdc -t neutral -i REFACTOR_PLAN.md -o refactor_plan.pdf` 一并生成。
