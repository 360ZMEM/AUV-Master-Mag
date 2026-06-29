# 图片样例与诊断总览

> **生成时间**：2026-06-27 15:21
> **数据来源**：`python tools/visualize.py --variants` + `python tools/visualize.py --maze`

---

## 0. 命令速查

### 基本命令

```bash
# 单个场景（overview + detail + selector_sync + report.md + record.npz）
python tools/visualize.py --case case1

# 全部核心场景 case1-6 + showcase
python tools/visualize.py --all

# 下游转弯变种 case1v-6v + showcase
python tools/visualize.py --variants

# 迷宫压力测试（4 个子场景）+ showcase
python tools/visualize.py --maze

# 部署模式（禁用名义路由先验）
python tools/visualize.py --all --deployment

# 限制步数（快速冒烟）
python tools/visualize.py --case case1 --max-steps 500

# 指定输出目录
python tools/visualize.py --all --outdir results/my_run
```

### zig-zag 探针开关

通过 `--zigzag-probe` 标志启用小幅 TRACK zig-zag 磁探针。该标志可与 `--case`、`--all`、`--variants`、`--maze` 任意组合：

```bash
# 单 case 开启 zig-zag 探针
python tools/visualize.py --case case1 --zigzag-probe

# 全部 case 开启 zig-zag 探针
python tools/visualize.py --all --zigzag-probe

# 变种开启 zig-zag 探针
python tools/visualize.py --variants --zigzag-probe

# maze 开启 zig-zag 探针
python tools/visualize.py --maze --zigzag-probe
```

`--zigzag-probe` 具体做了什么（见 `tools/visualize.py` 第 51-64 行）：

```python
def _apply_zigzag_probe(scenario):
    scenario.tracking.track_active_zigzag_angle_deg = max(..., 3.0)
    scenario.tracking.curve_track_crossing_angle_deg = max(..., 3.0)
    scenario.tracking.magnetic_path_observation_enabled = True
    scenario.tracking.magnetic_path_min_horizontal_field_nt = 5.0
    scenario.tracking.magnetic_path_max_cross_track_m = 25.0
```

**重要**：`ProbeBurstManager` 和 reacquire-safe window 是**独立的配置开关**，不走 `--zigzag-probe` CLI 标志。它们的开关在 `config/__init__.py` 中：

```python
# 默认关闭，不污染 p36 基线
probe_burst_manager_enabled: bool = False
probe_burst_manager_reacquire_safe_window_enabled: bool = False
```

### maze 命令契约

`--maze` 当前硬编码运行 4 个子场景：

| 子场景 | 声呐状态 | 含义 |
| --- | --- | --- |
| `case_maze_sonar` | 全程开启 | 声呐辅助迷宫跟踪 |
| `case_maze_sonar_dropout` | TRACK 后离线 | 声呐 dropout 压力测试 |
| `case_maze_sparse_sonar` | 低频稀疏锚点 | 稀疏声呐锚点 + 更长 local path 记忆 |
| `case_maze_no_sonar` | 全程关闭 | 纯磁可观测性边界 |

**简化评估**：当前 `--maze` 是一个干净的单一入口，不需要额外参数。4 个子场景覆盖了从"有声呐闭环"到"纯磁不可行"的完整可观测性谱系，契约合理。唯一可考虑的未来简化：如果 `case_maze_no_sonar` 确认永远不可行，可从默认集合移除，改为单独 `--case case_maze_no_sonar` 调用。

---

## 1. case1v-6v（下游转弯变种）

> 命令：`python tools/visualize.py --variants`
> 输出目录：`results/20260627_152131/`

### 文本日志

```text
[viz] simulating case1v (nominal) ...
       health 91/100  mean_err 10.7deg  TRACK 91%  switches 2
       track_xt 2.7m  track_vehicle_err 4.3deg  final_xt 0.9m
       route 62.6%  stop=duration
[viz] simulating case2v (nominal) ...
       health 89/100  mean_err 8.0deg  TRACK 72%  switches 2
       track_xt 3.9m  track_vehicle_err 5.2deg  final_xt 0.9m
       route 46.2%  stop=duration
[viz] simulating case3v (nominal) ...
       health 98/100  mean_err 3.0deg  TRACK 56%  switches 2
       track_xt 1.0m  track_vehicle_err 3.7deg  final_xt 0.1m
       route 56.3%  stop=duration
[viz] simulating case4v (nominal) ...
       health 97/100  mean_err 5.4deg  TRACK 61%  switches 6
       track_xt 0.4m  track_vehicle_err 1.3deg  final_xt 0.1m
       route 58.4%  stop=duration
[viz] simulating case5v (nominal) ...
       health 96/100  mean_err 10.5deg  TRACK 65%  switches 2
       track_xt 0.3m  track_vehicle_err 0.6deg  final_xt 0.0m
       route 58.4%  stop=duration
[viz] simulating case6v (nominal) ...
       health 82/100  mean_err 19.9deg  TRACK 66%  switches 6
       track_xt 3.4m  track_vehicle_err 6.6deg  final_xt 1.7m
       route 54.3%  stop=duration
```

### 指标矩阵

| case | health | mean_err | TRACK | switches | track_xt | vehicle_err | final_xt | route | stop |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| case1v | 91 | 10.7° | 91% | 2 | 2.7m | 4.3° | 0.9m | 62.6% | duration |
| case2v | 89 | 8.0° | 72% | 2 | 3.9m | 5.2° | 0.9m | 46.2% | duration |
| case3v | 98 | 3.0° | 56% | 2 | 1.0m | 3.7° | 0.1m | 56.3% | duration |
| case4v | 97 | 5.4° | 61% | 6 | 0.4m | 1.3° | 0.1m | 58.4% | duration |
| case5v | 96 | 10.5° | 65% | 2 | 0.3m | 0.6° | 0.0m | 58.4% | duration |
| case6v | 82 | 19.9° | 66% | 6 | 3.4m | 6.6° | 1.7m | 54.3% | duration |

> case1v-5v 全部满足 15° 硬约束；case6v 是连续强弯表达边界。

### case1v

![case1v_overview](../results/20260627_152131/case1v/figures/case1v_overview.png)

![case1v_detail](../results/20260627_152131/case1v/figures/case1v_detail.png)

![case1v_selector_sync](../results/20260627_152131/case1v/figures/case1v_selector_sync.png)

### case2v

![case2v_overview](../results/20260627_152131/case2v/figures/case2v_overview.png)

![case2v_detail](../results/20260627_152131/case2v/figures/case2v_detail.png)

![case2v_selector_sync](../results/20260627_152131/case2v/figures/case2v_selector_sync.png)

### case3v

![case3v_overview](../results/20260627_152131/case3v/figures/case3v_overview.png)

![case3v_detail](../results/20260627_152131/case3v/figures/case3v_detail.png)

![case3v_selector_sync](../results/20260627_152131/case3v/figures/case3v_selector_sync.png)

### case4v

![case4v_overview](../results/20260627_152131/case4v/figures/case4v_overview.png)

![case4v_detail](../results/20260627_152131/case4v/figures/case4v_detail.png)

![case4v_selector_sync](../results/20260627_152131/case4v/figures/case4v_selector_sync.png)

### case5v

![case5v_overview](../results/20260627_152131/case5v/figures/case5v_overview.png)

![case5v_detail](../results/20260627_152131/case5v/figures/case5v_detail.png)

![case5v_selector_sync](../results/20260627_152131/case5v/figures/case5v_selector_sync.png)

### case6v

![case6v_overview](../results/20260627_152131/case6v/figures/case6v_overview.png)

![case6v_detail](../results/20260627_152131/case6v/figures/case6v_detail.png)

![case6v_selector_sync](../results/20260627_152131/case6v/figures/case6v_selector_sync.png)

### showcase（跨变种汇总）

![showcase_variants](../results/20260627_152131/showcase.png)

---

## 2. maze（迷宫压力测试）

> 命令：`python tools/visualize.py --maze`
> 输出目录：`results/20260627_151722/`

### 文本日志

```text
[viz] simulating case_maze_sonar (nominal) ...
       health 93/100  mean_err 9.3deg  TRACK 98%  switches 2
       track_xt 0.8m  track_vehicle_err 8.8deg  final_xt 0.4m
       route 99.5%  stop=endpoint
[viz] simulating case_maze_sonar_dropout (nominal) ...
       health 27/100  mean_err 3.8deg  TRACK 52%  switches 22
       track_xt 24.3m  track_vehicle_err 89.7deg  final_xt 21.3m
       route 1.5%  stop=duration
[viz] simulating case_maze_sparse_sonar (nominal) ...
       health 93/100  mean_err 10.1deg  TRACK 98%  switches 2
       track_xt 0.9m  track_vehicle_err 8.5deg  final_xt 0.0m
       route 99.5%  stop=endpoint
[viz] simulating case_maze_no_sonar (nominal) ...
       health 12/100  mean_err nandeg  TRACK 0%  switches 2
       track_xt nanm  track_vehicle_err nandeg  final_xt 563.9m
       route 0.0%  stop=duration
```

### 指标矩阵

| case | health | mean_err | TRACK | switches | track_xt | vehicle_err | route | max jump | large jumps | shortcut | geometry | stop |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- | --- |
| maze_sonar | 92.6 | 9.3° | 98% | 2 | 0.8m | 8.8° | 99.5% | 0.1m | 0 | no | yes | endpoint |
| maze_sonar_dropout | 27.4 | 3.8° | 52% | 22 | 24.3m | 89.7° | 1.5% | 0.0m | 0 | no | no | duration |
| maze_sparse_sonar | 92.6 | 10.1° | 98% | 2 | 0.9m | 8.5° | 99.5% | 0.1m | 0 | no | yes | endpoint |
| maze_no_sonar | 11.7 | nan | 0% | 2 | nan | nan | 0.0% | 0.0m | 0 | no | no | duration |

> 新口径下，maze_sonar 和 maze_sparse_sonar 不只是 endpoint 成功，也满足 no-shortcut 几何验收；maze_sonar_dropout 仍是任务推进失败，核心信号是 route 只有 1.5% 且 TRACK vehicle heading error 约 90°；maze_no_sonar 是纯磁可观测性边界。

### report.md 关键自动分析

```text
case_maze_sonar:
- MAZE GEOMETRY PASS: no large route-progress shortcut detected.

case_maze_sonar_dropout:
- MAZE GEOMETRY FAIL: task progress or vehicle heading failed maze acceptance.

case_maze_sparse_sonar:
- MAZE GEOMETRY PASS: no large route-progress shortcut detected.

case_maze_no_sonar:
- MAZE GEOMETRY FAIL: task progress or vehicle heading failed maze acceptance.
```

### case_maze_sonar

![maze_sonar_overview](../results/20260627_151722/case_maze_sonar/figures/case_maze_sonar_overview.png)

![maze_sonar_detail](../results/20260627_151722/case_maze_sonar/figures/case_maze_sonar_detail.png)

![maze_sonar_selector_sync](../results/20260627_151722/case_maze_sonar/figures/case_maze_sonar_selector_sync.png)

### case_maze_sonar_dropout

![maze_sonar_dropout_overview](../results/20260627_151722/case_maze_sonar_dropout/figures/case_maze_sonar_dropout_overview.png)

![maze_sonar_dropout_detail](../results/20260627_151722/case_maze_sonar_dropout/figures/case_maze_sonar_dropout_detail.png)

![maze_sonar_dropout_selector_sync](../results/20260627_151722/case_maze_sonar_dropout/figures/case_maze_sonar_dropout_selector_sync.png)

### case_maze_sparse_sonar

![maze_sparse_sonar_overview](../results/20260627_151722/case_maze_sparse_sonar/figures/case_maze_sparse_sonar_overview.png)

![maze_sparse_sonar_detail](../results/20260627_151722/case_maze_sparse_sonar/figures/case_maze_sparse_sonar_detail.png)

![maze_sparse_sonar_selector_sync](../results/20260627_151722/case_maze_sparse_sonar/figures/case_maze_sparse_sonar_selector_sync.png)

### case_maze_no_sonar

![maze_no_sonar_overview](../results/20260627_151722/case_maze_no_sonar/figures/case_maze_no_sonar_overview.png)

![maze_no_sonar_detail](../results/20260627_151722/case_maze_no_sonar/figures/case_maze_no_sonar_detail.png)

![maze_no_sonar_selector_sync](../results/20260627_151722/case_maze_no_sonar/figures/case_maze_no_sonar_selector_sync.png)

### showcase（跨 maze 汇总）

![showcase_maze](../results/20260627_151722/showcase.png)

---

## 3. 图片类型说明

| 图片 | 面板数 | 用途 |
| --- | --- | --- |
| `overview.png` | 4 面板 | 轨迹俯视图 + 航向误差 + FSM 状态 + 关键指标 |
| `detail.png` | 9 面板 | 诊断仪表盘：SNR、置信度、拟合残差、横偏、磁峰事件、导航源等 |
| `selector_sync.png` | 多面板 | shadow 候选选择器与 oracle 一致性时间序列 |
| `showcase.png` | 跨 case | 横向对比 |

---

## 4. 关键诊断通道与指标（永久化记录）

> 以下内容与 [10_可视化体系.md](10_可视化体系.md) §8-10 同步，此处作为实验结果永久化快照。

### 4.1 RunRecord 诊断通道（recorder.py）

**shadow 候选选择器**（~50 通道）：
- `shadow_axis_hypothesis_valid` / `_count` — 候选假设有效性
- `shadow_axis_selected_sign` / `_score` — 选中候选方向与得分
- `shadow_axis_progress_aligned_candidate_valid` / `_sign` / `_score` / `_task_score` / `_combined_score` — progress-aligned 候选
- `shadow_axis_progress_proxy_valid` / `_source_code` / `_age_s` / `_confidence` — progress proxy 状态
- `shadow_axis_route_bound_proxy_valid` / `_progress_m` / `_distance_m` — route-bound proxy

**decoupled lateral shadow**（~11 通道）：
- `shadow_decoupled_lateral_valid` / `_feasible` / `_heading_deg`
- `shadow_decoupled_lateral_forward_dot` / `_targeting_dot`
- `shadow_decoupled_lateral_completed_leg_route_delta_m` / `_sweep_m`

**ProbeBurstManager**（~11 通道）：
- `probe_burst_manager_state_code` — 0=IDLE, 1=BURST, 2=RECOVER, 3=COOLDOWN
- `probe_burst_manager_burst_active` / `_recovery_active`
- `probe_burst_manager_control_allowed` / `_reacquire_safe_control_allowed`
- `probe_burst_manager_entry_abs_cross_track_m` — 进入 burst 时冻结的横偏

**zig-zag probe cycle**（~10 通道）：
- `zigzag_probe_cycle_id` / `_age_s` / `_peak_abs_cross_track_m`
- `zigzag_probe_cycle_burial_valid` / `_depth_m` / `_sigma_m` / `_quality`

### 4.2 HealthMetrics 新增指标（metrics.py）

**shadow axis hypothesis**（~40 字段）：
- `shadow_axis_hypothesis_fraction` — 候选假设帧占比
- `shadow_axis_progress_oracle_consistency_fraction` — oracle 一致性
- `shadow_axis_progress_candidate_forward_fraction` / `_backward_fraction` — 前/后向候选占比
- `shadow_axis_progress_proxy_valid_fraction` / `_held_fraction` / `_local_path_fraction` / `_sonar_fraction`

**ProbeBurstManager**（~15 字段）：
- `probe_burst_manager_active_fraction` / `_idle_fraction` / `_burst_fraction` / `_recovery_fraction` / `_cooldown_fraction`
- `probe_burst_manager_transition_count` / `_recovery_timeout_count`
- `probe_burst_manager_control_allowed_fraction` / `_reacquire_safe_control_allowed_fraction`
- `probe_burst_manager_mean_entry_abs_cross_track_m` / `_entry_xt_le4_fraction` / `_entry_xt_le20_fraction`

**zig-zag probe cycle**（~5 字段）：
- `zigzag_probe_cycle_count` / `_burial_coverage` / `_burial_mae_m`

**maze geometry**（maze 专用验收字段）：
- `p99_cross_track_m` — 99 分位横偏，暴露少量极端脱轨帧
- `route_progress_backward_fraction` — route progress 后退占比
- `route_progress_max_jump_m` — 相邻帧 route progress 最大正向跳变
- `route_progress_large_jump_count` — route progress 单步跳变超过 25m 的次数
- `lane_shortcut_indicator` — 是否存在 lane shortcut / projection jump 风险
- `maze_geometry_passed` — maze 最终几何验收口径

### 4.3 report.md 中 ProbeBurstManager 输出行

```text
| ProbeBurstManager active | X% |
| ProbeBurstManager idle/burst/recovery/cooldown | X% / X% / X% / X% |
| ProbeBurstManager transitions | N |
| ProbeBurstManager recovery timeouts | N |
| ProbeBurstManager control allowed | X% |
| ProbeBurstManager reacquire-safe allowed | X% |
| ProbeBurstManager entry XT mean | X.XX m |
| ProbeBurstManager entry XT <=4/20m | X% / X% |
```

### 4.4 report.md 中 maze geometry 输出行

```text
| P99 cross-track | X.X m |
| Route progress backward fraction | X.X% |
| Route progress max jump | X.X m |
| Route progress large jumps | N |
| Lane shortcut indicator | yes/no |
| Maze geometry passed | yes/no |
```

自动分析中还会给出：

```text
- MAZE GEOMETRY PASS: no large route-progress shortcut detected.
- MAZE GEOMETRY FAIL: route progress jumped by X.X m, indicating a lane shortcut / projection jump.
- MAZE GEOMETRY FAIL: task progress or vehicle heading failed maze acceptance.
```
