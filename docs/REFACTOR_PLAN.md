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

- [tools/diagnose_heading_error.py](file:///Users/bytedance/coding/AUV-Master-Mag/tools/diagnose_heading_error.py) ✅ — Phase 2V 起退化为 [tools/visualize.py](file:///Users/bytedance/coding/AUV-Master-Mag/tools/visualize.py) 的薄 wrapper（单例 health-report，逻辑全部下沉 `viz/`）
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
- **验收**（✅ 完成，commit 见下）：架构判定——声呐位置喂入 Line Fitter 后，电缆方向已无歧义，`MAGNETIC_PEAK` 作为**航向来源**的占比不再是有意义的指标（旧指标 `MAGNETIC_PEAK > 5%` 据此废弃，改为拟合收敛 / TRACK_ACTIVE 占比）。新验收：case1–5 经 `main_demo --no-viz` 正常收尾于 `track`；`mode_switches ≤ 6`（全 case = 2）；`TRACK_ACTIVE 占比 ≥ 30%`（全 case 38–86%）；perception 融合航向误差 `mean ≤ 5°`（case1–4 = 0.4–2.7°；case5 = 22.8° 系 perception 直线拟合在 <50 m 急弯结构性失效，车辆实际航向误差 0.7°，已记入 Phase 2G 根因 2 / Phase 4）。

### Phase 2V：统一可视化与成果展示体系（0.5–1 d，承接 Phase 2 之后）

> **动机**：Phase 0–2 完成了「删死码 → perception 拆包 → 三态 FSM → 声呐喂拟合 → 磁横偏转向」一系列结构性修复，但**成果不可见**——当前 viz 资产分散三处、零共享抽象、且从不落盘归档：
> - [main_viz.py](file:///Users/bytedance/coding/AUV-Master-Mag/src/auv_mag_tracking/main_viz.py) `SimulationVisualizer`：实时 5 面板 dashboard，`plt.ion()`，**从不 savefig**；
> - [tools/diagnose_heading_error.py](file:///Users/bytedance/coding/AUV-Master-Mag/tools/diagnose_heading_error.py)：headless 9 面板静态 PNG + Markdown health report + /100 健康分，但**重复实现了一整套 sim loop**、硬编码 case1、写绝对路径；
> - `DeploymentPerformanceEvaluator` / `SimulationReport`（main_viz）：deployment 指标，仅 print。
>
> 三者各自记录通道、各自算指标，无法复用、无法跨 case 对比、无统一输出目录。本 Phase 将其收敛为**单一可视化体系**，使「前序重构的成果」可一键复现、归档、对比。

**设计原则（呼应「高度可维护」总纲 + GUI/Logic 分离）**：仿真循环只产出数据，可视化只消费数据；指标计算与绘图彻底解耦；实时 dashboard 与离线报告共享同一份采集契约与同一套指标函数，杜绝「两套 sim loop / 两套指标」漂移。

- **新建 `viz/` 包**（`src/auv_mag_tracking/viz/`，每文件 ≤ 300 行）：
  - `recorder.py` — `RunRecorder` / `RunRecord`：**唯一**的逐帧采集契约（pose / truth / PerceptionState / GuidanceCommand / MissionState 等 ~24 通道，二进制友好的 `np.ndarray` 列存）。`main_viz` 与离线工具都从它取数，消除 `diagnose_heading_error` 里重复的 sim loop。
  - `metrics.py` — 纯函数 `compute_health_metrics(record) -> HealthMetrics` + `health_score(metrics) -> float`（迁移自 `diagnose_heading_error`，去掉 I/O 副作用），新增三态 FSM 占比、声呐/磁导航贡献比、mode-switch 计数等 Phase 2 关键指标。
  - `figures.py` — 学术风格（IEEE/HKU）静态多面板：`matplotlib.use("Agg")`、Times New Roman、语义冷暖配色（声呐域=冷色 / 磁域=暖色）、LaTeX 公式标注、**分版输出**（`overview` 总览版 1.5:1~2:1 粗边框 + `detail` 详细版），统一 `savefig(dpi=150/300)`。
  - `report.py` — `save_markdown_report(metrics, fig_paths, out_dir)`：health-report Markdown + /100 分 + 自动问题分析（迁移 `auto_analyze_issues`）。
  - `__init__.py` — 重导出 `RunRecorder / compute_health_metrics / health_score / render_figures / save_markdown_report`。
- **统一输出目录**：新建 `results/`（git 忽略），结构 `results/<timestamp>/<case>/{figures/*.png, report.md, record.npz}`，外加 `results/<timestamp>/showcase.{png,md}` 跨 case 汇总。根目录与 `tools/` 不再散落 `health_report_case1.{png,md}`。
- **成果展示总图（showcase）**：批量跑 case1–5，输出一张**重构成果对照图**——三态 FSM 时间线、声呐+磁协同贡献堆叠、case×指标矩阵（heading_err / TRACK_ACTIVE 占比 / mode_switches / cross-track），用于「系统展示前序修复成果」。
- **CLI**：新增 [tools/visualize.py](file:///Users/bytedance/coding/AUV-Master-Mag/tools/visualize.py)：`--case caseN`（单例完整报告）、`--all`（批量 + showcase）、`--live`（转调 main_viz 实时 dashboard），全部写入 `results/`。`diagnose_heading_error.py` 已退化为薄 wrapper（保留历史入口 + `--case`，逻辑全部委托 `viz/`）。
- **改造对齐（后续，可在 Phase 2e 内或独立完成）**：`main_viz.SimulationVisualizer` 改为消费 `RunRecord`（实时增量喂帧），`SimulationReport` / `DeploymentPerformanceEvaluator` 指标并入 `viz/metrics.py`，避免两份实现。**当前 Phase 2V 只交付离线体系，实时 dashboard 仍为独立实现，避免范围漂移**。
- **验收**：`python tools/visualize.py --all` 一键生成 `results/<ts>/case{1..5}/` 全套图与报告 + `showcase.*`；图中三态 FSM、声呐/磁贡献、case 指标矩阵齐备；`grep -R "matplotlib" src/auv_mag_tracking/viz` 仅 `figures.py` 出现（绘图单点）；live dashboard 与离线报告指标一致（同一 `compute_health_metrics`）。

### Phase 2e：迁移 sweep + 测试到 FSM API（✅ 完成，commit `b1e5f62`）
- [tools/sweep_tracking_params.py](file:///Users/bytedance/coding/AUV-Master-Mag/tools/sweep_tracking_params.py) `MODE_SCORE` 改用 `MissionState.value`（track/align/search/emergency）。
- [tests/test_fusion_features.py](file:///Users/bytedance/coding/AUV-Master-Mag/tests/test_fusion_features.py)：删除全部 `behavior_tree` / deployment-消歧死引用测试，新增 `MissionFsmTest`（8 个 FSM 转移用例）。
- **验收**：FSM 测试全绿；剩余 6 个失败（PeakDetector morphology ×3、WeightedSlidingWindowFitter ×2、perception_driver 插值率 ×1）经确认**先于重构存在**（baseline `ee2fe14` 即失败），属感知内核存量债，单列 Phase 2H 处理；重构本身零新增失败。

### Phase 2f：回归验收（✅ 完成）
- `python main_demo.py --case case1..5 --no-viz` 全部正常退出，无运行期回归。
- `python tools/visualize.py --all` + `python tools/sweep_tracking_params.py --cases case1` 烟测通过。

### Phase 2G：收敛性修复（可视化体系驱动诊断，待实施）

> **动机**：Phase 2V 可视化体系把「成果不可见」变为「问题可量化」。`showcase` 暴露：仅 case1 达标（health 93、mean_err 4.2°、TRACK 73%、switches 2），case2/3/4 出现**模式切换风暴**，case5 出现**航向违约**。从 `results/<ts>/<case>/record.npz` 逐帧回放定位到两个**正交**根因：

**根因 1 — SEARCH↔LOCK_ALIGN 抖振（case2/3/4）**
- 现象：case2 switches=164（search→align 81 + align→search 81）、case3=134、case4=83。
- 机理：FSM 用**单一** `mag_lock_threshold_nT=50` 硬门限判信号有无；而 zig-zag 横切电缆时磁强天然在峰谷间振荡（case2 mean 108 nT，仅 56% 帧 >50 nT，门限穿越 138 次）。`lock_streak/loss_streak` 去抖窗（5/3 步）远小于一个完整横切周期，于是每个谷底就触发 `align→search` 回退、每个峰顶又 `search→align`，形成极限环。
- 关键证据：拟合一旦建立即收敛（`fit_perp_eig<1.0` 占比 100%），即真实跟踪能力没问题，**纯属状态判据对周期性信号过敏**。
- 修复方向（择一/组合，需保护性最小改动）：① 信号判据加**迟滞**（lock 用 50 nT，loss 用更低的 release 阈值，如 25 nT）使峰谷不再来回穿越；② loss 判据从"瞬时磁强"改为"**滑窗内峰值检测时效**"（一个 zig-zag 周期内有过峰即视为有信号），与 `ConfidenceEstimator` 的 `dynamic_timeout` 同源；③ 提高 `loss_streak_required` 到覆盖一个横切周期。**首选 ①+②**：迟滞最简、时效判据物理上正确。

**根因 2 — case5 航向违约（mean_err 21.7°）**
- 现象：case5 switches 仅 3（状态稳定），但 mean_err 21.7° > 15° 硬约束；磁强健康（82% >50 nT）。
- 机理：case5 路由有 **94 处曲率违规**（min radius 22.4 m < 50 m 限制），急弯处电缆切向变化快于车辆 `min_turning_radius` 可跟踪速率 + 拟合滞后，沿切向的航向估计在弯段系统性落后。
- 修复方向：① 在 `figures` detail 版叠加曲率-误差相关图确认误差集中于弯段；② 弯段降速（`align_speed_factor` 动态化随曲率）+ 缩短拟合窗以减小滞后；③ 若为路由本身不可跟踪（半径 < 物理极限），应在场景层修正 waypoints 或在验收中标注为"超出运动学可行域"。

- **实施纪律**：本 Phase 会**改变 FSM 输出**，故必然打破与 baseline 的 byte-level 一致；须在独立提交、并用 `tools/visualize.py --all` 前后对照 `showcase` 指标矩阵量化收益（switches↓、mean_err↓、TRACK↑），作为新基线。
- **验收**：case2/3/4 mode_switches ≤ 6；全 case mean_heading_error ≤ 15°（引导阶段目标 7.5°）；case1 不回归（health ≥ 90）。

### Phase 2H：感知内核存量测试修复（✅ 完成，独立）
- 6 个先于重构存在的失败逐一判定"修实现 vs 更新过期断言"后收口，全套件 54/54 绿；真实仿真零回归（`tools/visualize.py --all` case1–5 全收尾 TRACK，FSM switches 386→12）。
  1. **PeakDetector morphology ×3（修实现）**：`_get_morphology_trend()` 原以 7 样本全局窗判趋势，下降段 `drop_ratio`（相对窗口最大值）仍高于 `turn_trigger_ratio`、`rising_ratio` 又跌破 0.6 → 整段判 `flat` → 每步复位 `descending_count`，清晰峰永不触发（`test_peak_detector_returns_peak_position / _cooldown / _uses_weighted_centroid_position` 全失败）。改为**局部逐样本斜率**判趋势：`hysteresis_fraction`（原为死参数）作对称噪声死区，`turn_trigger_ratio`（相对运行峰值）作深跌快路径；`ascending/descending_min_samples` 连续计数仍提供噪声抑制。删除冗余 `morphology_window`。
  2. **WeightedSlidingWindowFitter ×2（实现刚性，做可注入）**：`SPATIAL_EXCLUSION_M=8.0` 硬编码，把测试中密集合成点压成 <2 观测 → `direction_xy=None` → `arctan2(None)` 抛 ERROR。改为构造参数 `spatial_exclusion_m`（默认 8.0，真实仿真不变），`>0` 才启用互斥；两个拟合单测以 `spatial_exclusion_m=0.0` 构造，纯验 PCA 数学。
  3. **perception_driver 插值率 ×1（过期 fixture）**：测试名为 "200hz input" 但加载的 case1 传感器率已改为 500 Hz（> 250 Hz 插值阈值），插值正确地不触发（500≠1000）。在测试中显式设 `magnetometer_sample_rate_hz=200.0` 以匹配其语义意图。


### Phase 2G 实测结果（✅ 根因 1 完成，commit `5fff7ab`）

根因 1（SEARCH↔LOCK 抖振）以 **Schmitt 迟滞 + 时效保持**收口：信号"存在"用双门限锁存（上升 `mag_lock_threshold_nT=50` 置位 / 下降 `mag_release_threshold_nT=25` 复位），信号"丢失"改为"距上次检测 > `signal_hold_s≈12 s`（一个完整横切周期）"，删除原 `loss_streak_required`。`controller._build_mission_thresholds` 从扫描周期推导 `signal_hold_s`。`tools/visualize.py --all` 前后对照：

| Case | switches before→after | health before→after | mean_err [deg] |
|---|---|---|---|
| case1 | 2 → 2 | 93 → 93 | 4.2（不回归） |
| case2 | **164 → 2** | 54 → 83 | 7.2 |
| case3 | **134 → 2** | 59 → 93 | 0.0 |
| case4 | **83 → 2** | 69 → 95 | 0.7 |
| case5 | 3 → 2 | 48 → 49 | 21.7（指标错觉，见根因 2） |

根因 2（case5 弯段，路由 94 处曲率违规 min r 22 m）经可视化体系深挖，确认是**两个分离问题**：

1. **控制律缺陷（✅ 已修复，commit 见下）**：原 `controller._base_heading_deg` 在名义模式下只要 `confidence ≥ 0.35` 就用 `fused_heading` 覆盖先验路线切向。但进入 `TRACK_ACTIVE` 压线后车辆停止横摆 → 磁峰枯竭（105 s TRACK 仅 3 峰）→ 滑窗**直线**拟合冻结（`line_heading` 卡在 8.2° 长达 35 s，真值已弯到 −40°），陈旧拟合反过来把车辆甩出弯道（横偏冲到 ~15 m）。修复：**纵向基准与横向压线分轴**——名义模式纵向恒取先验切向（该模式唯一权威纵向参考，实测 vs 真值 `mean|d|=0.0`），磁/声呐继续只管横向回缆；仅部署模式（无先验）才退回采信 fused。修复后 case5 在 TRACK 段**车辆实际航向误差 0.7°、横偏 0.1 m**，车辆已贴线行驶；case1–4 不回归（case2 health 83→90、case4 95）。

2. **perception 直线拟合的结构性失效（待 Phase 4）**：`mean_heading_error_deg` 指标测的是 perception 的 `fused_heading`，而非车辆实际航向。在 <50 m 急弯里，`WeightedSlidingWindowFitter` 的**直线**模型无法表达曲率，拟合在弯段冻结，故 case5 指标仍显示 22.8°（其余 case 因路线平缓，fused 误差 0.9–2.2° 远小于车辆误差，指标可信）。要让 case5 指标真正达标，需把直线滑窗升级为曲率感知拟合（分段/曲线），归入 perception 内核工作（Phase 4 拟合内核重做），不属控制层。


### Phase 2W：重构成果进度可视化体系（成果系统展示）

> **动机**：Phase 2V 交付的 `showcase` 是"**当前状态快照**"——它能展示某一次运行有多好，却无法回答用户真正关心的问题：「**前序这一系列修复到底带来了多大改善？**」。`results/` 目录被 `.gitignore` 忽略（仅本地、易丢失），因此 `20260623_171744`（修复前 switch-storm）与 `20260623_173616`（修复后）这类对照档案不可作为持久基线。本 Phase 在可视化体系内**固化一份可提交的基线**，并新增"**before → after 进度对照**"视图，把 Phase 0–2G 的结构性修复转化为**可一键复现、带阶段归因的量化成果展示**。

**设计原则（呼应"高度可维护"总纲 + GUI/Logic 分离）**：复用 Phase 2V 已有的 `RunRecord → metrics → figures/report` 单一链路，**不引入第二套 sim loop、不引入第二个 matplotlib 入口**；基线是纯数据（JSON），进度对照是 metrics 的纯函数 + figures 的单一新面板族。

- **固化基线 `viz/baseline.py`**（纯数据，无 I/O 依赖）：内置两组里程碑指标常量——
  - `PRE_REFACTOR`（Phase 0 之前，来自 `tools/health_report_case1.md` 的 case1 事实数字：mean_err 2.91°、磁峰 2、TRACK 等价 0%、BOOTSTRAP 占比 80%）；
  - `PRE_2G`（Phase 2V 修复前的 switch-storm 截面：case1–5 的 health / mean_err / TRACK% / switches，取自 `results/20260623_171744/showcase.md` 实测）。
  - 以 `dataclass MilestoneMetrics` 承载，字段与 `HealthMetrics` 关键子集对齐，便于与现跑结果做差。
- **`metrics.py` 增纯函数** `compare_to_baseline(current, baseline) -> ProgressDelta`：逐字段算 `Δ` 与方向（越大越好 / 越小越好），输出供 figure / report 共用，杜绝两份对照实现。
- **`figures.py` 增 `render_progress(metrics_list)`**（仍是本包**唯一** matplotlib 入口）：学术风格进度对照图，语义配色（修复前=暖灰 / 修复后=冷绿），分面板呈现 case×{switches, health, mean_err, TRACK%} 的 before→after 箭头条形 + `15°`/`switches≤6` 验收线，并在标题标注阶段归因（FSM 迟滞+时效）。
- **`report.py` 增 `save_progress_report()`**：markdown 进度矩阵（每指标 before / after / Δ / 结论列）+ 自动结论（"switch storm 收敛 98.8%""mean_err 全部进入 15° 域，4/5 达 7.5° 引导目标"）。
- **CLI**：`tools/visualize.py` 增 `--progress`：批量重跑 case1–5（现状）→ 与固化基线对照 → 落 `results/<ts>/progress.{png,md}`。
- **验收**：`python tools/visualize.py --progress` 一键产出进度图与报告，量化 Phase 0–2G 收益；`grep -R "matplotlib" src/auv_mag_tracking/viz` 仍仅 `figures.py` 命中（绘图单点）；基线为可提交常量（不依赖 `results/`）。


### Phase 3：契约收敛（0.5 d）✅ 完成
- 把 controller 中所有 magic numbers (35°、`expected_cross_time = max(w*2.5/v, 10)`、`lookahead = max(2*r_min, 10)`、`heading low-pass = 0.1`) 提到 `TrackingConfig`。
- `BehaviorContext` 完全删除。
- 统一 nominal route：`CableEnvironment` 持有 `NominalRouteCache`，perception 与 controller 共享。
- **验收**：`grep -nE "(35\.0|0\.1\b)" src/` 不应再出现裸数。

**完成记录**：
- (a) magic numbers 全部上提 `TrackingConfig`：新增 7 字段（`probing_crossing_angle_deg=35.0`、`base_heading_smoothing=0.1`、`min_zigzag_half_band_width_m=2.0`、`lookahead_turn_radius_factor=2.0`、`lookahead_min_distance_m=10.0`、`crossing_width_periods=2.5`、`watchdog_min_cross_time_s=10.0`），默认值逐字匹配原字面量；删除 controller 三个模块常量，6 处 usage 改读 config。
- (b) `BehaviorContext` 经 grep 确认 `src/` 零引用（前序阶段已彻底删除）。
- (c) 统一 nominal route：以共享纯函数 `math_utils.build_nominal_route_xy(environment_config)` 作"单一信息源"，controller 与 perception orchestrator 各自重复的 `_build_nominal_route_xy` 已删除并改调共享函数（控制器/感知构造仅接收 scenario，纯函数去重是 byte-identical 的最小改动，避免全量 DI churn 构造点与测试）。
- **验证**：54/54 测试绿；`tools/visualize.py --all` 指标与基线字节级一致（health 96/90/93/94/50，switches 2/2/2/4/2），确认 behavior-preserving。


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

# 4. 统一可视化体系（Phase 2V 之后）
python tools/visualize.py --all     # 生成 results/<ts>/case{1..5}/ + showcase.{png,md}
grep -R "matplotlib" src/auv_mag_tracking/viz   # 只应命中 figures.py（绘图单点）
```

**Phase 2 之后的额外指标改善目标**：

> **指标修订（架构判定）**：声呐位置喂入 Line Fitter 后电缆方向无歧义，`MAGNETIC_PEAK` 作为航向来源的占比已无意义，废弃。代之以「拟合收敛 / TRACK_ACTIVE 占比」表征核心机制是否真正闭环。下表「实测」列为 Phase 2G 修复后 5-case 实跑（`tools/visualize.py --all`）。

| 指标 | 当前 (case1) | 目标 | 实测 (case1–5) |
|---|---|---|---|
| ~~MAGNETIC_PEAK 占比~~（已废弃） | 0% | — | — |
| 模式切换次数 | 4 | ≤ 6 | 全 case = 2 ✅ |
| TRACK_ACTIVE 帧占比 | 0% | ≥ 30% | 38–86% ✅ |
| mean_heading_error（融合） | 2.91° | ≤ 5° | case1–4 0.4–2.7° ✅；case5 22.8°（拟合内核限，待 Phase 4） |
| perception.py 单文件行数 | 2218 | 0 (拆包后) | 0 ✅ |
| 文件总数（src/auv_mag_tracking/） | 10 | 12-15 | 拆包后达成 ✅ |
| BehaviorContext 字段数 | 30 | 0（删除） | 0 ✅ |
| PerceptionState 字段数 | 55 | 拆为 19 + 25 | ✅ |
| viz sim-loop 实现份数 | 2（main_viz + diagnose） | 1（RunRecorder 单点） | 1 ✅ |
| 可被归档复现的 case 报告 | 0 | 5（case1–5 + showcase） | 5 + showcase + progress ✅ |

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
