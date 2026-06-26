# Lookahead Feed 诊断

## 范围

本文记录 `case_maze_sonar_dropout` 中为 `magnetic_lookahead -> local_path`
feed 门控新增的诊断体系。它是 `docs/17_zigzag纯磁探针方案.md` 的配套说明。

## 逐帧埋点

新增记录通道：

- `magnetic_lookahead_feed_allowed`
- `magnetic_lookahead_feed_reason_code`
- `magnetic_lookahead_feed_phase_age_s`
- `magnetic_lookahead_feed_innovation_m`
- `magnetic_lookahead_feed_axis_delta_deg`
- `magnetic_lookahead_feed_local_residual_m`

原因码映射：

| code | 含义 |
| --- | --- |
| `1` | 允许 feed |
| `2` | 无 lookahead target |
| `3` | feed 未启用 |
| `4` | 置信度过低 |
| `5` | lookahead age 过大 |
| `6` | phase age 过大 |
| `7` | local residual 过大 |
| `8` | heading delta 过大 |
| `9` | innovation 过大 |

## 指标和报告字段

`compute_health_metrics()` 和报告现在包含：

- `lookahead_feed_pct`
- `feed_reject_age`
- `feed_reject_phase_age`
- `feed_reject_residual`
- `feed_reject_heading`
- `feed_reject_innovation`
- `feed_phase_age`
- `feed_innovation`
- `feed_axis_delta`
- `feed_local_residual`

`render_detail()` 现在包含：

- Lookahead feed 门控原因随时间变化。
- Lookahead feed 门控裕度随时间变化：axis delta、innovation、phase age。

## 关键长测结果

命令：

```bash
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p25_probe10_feedlocal_gate60 --name p28_probe10_gate45_conservative --max-steps 24000
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p31_probe10_gate60_heading30 --name p32_probe10_gate60_heading40 --max-steps 24000
```

| variant | max steps | health | TRACK XT | TRACK vehicle err | route | final dist | feed allowed | reject heading | 结论 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `p25_probe10_feedlocal_gate60` | 24000 | 14.5 | 27.0m | 138.8deg | 37.6% | 16.2m | 25.0% | 60.1% | 能维持 route，但 heading 很差。 |
| `p28_probe10_gate45_conservative` | 24000 | 43.9 | 8.2m | 63.9deg | 9.0% | 0.9m | 34.3% | 47.1% | 估计更干净，但有效供给太少。 |
| `p31_probe10_gate60_heading30` | 24000 | 17.0 | 7.7m | 41.2deg | 0.0% | 58.7m | 32.7% | 52.1% | 30deg heading gate 不是有效折中点。 |
| `p32_probe10_gate60_heading40` | 24000 | 14.5 | 27.0m | 138.8deg | 37.6% | 16.2m | 25.0% | 60.1% | 行为与 p25 相同，说明不是简单阈值问题。 |

## 当前解释

主导拒绝原因是 `heading delta too large`。在这些代表性测试中，innovation 和 local
residual 不是主要阻塞项。简单调整 heading 阈值无法形成稳定折中：`30deg` 会阻断有效推进，
而 `35-40deg` 虽能保留推进，但会允许 heading 不稳定。

## 分层 Feed 后续测试

命令：

```bash
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p33_probe10_tiered_anchor_low --name p34_probe10_anchor_only --name p35_probe10_tiered_anchor_mid --max-steps 24000
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p36_probe10_extrapolated_low --name p37_probe10_extrapolated_mid --max-steps 24000
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p36_probe10_extrapolated_low
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p39_probe10_extrapolated_low_pursuit
```

| variant | run | health | TRACK XT | TRACK vehicle err | route | final dist | lookahead pos err | feed allowed | reject heading | 结论 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `p33_probe10_tiered_anchor_low` | 24000 | 7.2 | 23.0m | 70.5deg | 0.0% | 40.5m | 17.3m | 84.8% | 0.0% | 高置信 phase anchor 会破坏推进稳定性。 |
| `p34_probe10_anchor_only` | 24000 | 16.7 | 7.7m | 41.2deg | 0.0% | 49.0m | 11.0m | 67.1% | 14.9% | 仅靠 anchor 对 route 推进而言太稀疏。 |
| `p35_probe10_tiered_anchor_mid` | 24000 | 4.1 | 21.8m | 63.2deg | 0.0% | 46.0m | 18.1m | 81.4% | 5.0% | Anchor + 中等权重外推仍然失败。 |
| `p36_probe10_extrapolated_low` | 24000 | 23.8 | 26.9m | 140.7deg | 40.0% | 0.2m | 13.1m | 25.0% | 56.9% | 低权重外推在不使用 anchor 的情况下改善 route。 |
| `p37_probe10_extrapolated_mid` | 24000 | 14.5 | 27.0m | 138.8deg | 37.6% | 16.2m | 14.1m | 25.0% | 60.1% | 中等权重表现类似 p25。 |
| `p36_probe10_extrapolated_low` | full | 15.3 | 30.1m | 115.1deg | 58.9% | 53.8m | 7.4m | 7.5% | 69.4% | 完整时长确认 route 有提升，但 heading 仍然很差。 |
| `p39_probe10_extrapolated_low_pursuit` | full | 15.3 | 30.1m | 115.1deg | 58.9% | 53.8m | 7.4m | 7.5% | 69.4% | Pure-pursuit 仍无可测收益。 |

解释：

- 高置信 phase anchor 并不是无代价改进。稀疏点会主导 local estimator，并导致 route 推进崩溃。
- 当前最有用的方向是不带 phase anchor 的低权重 lookahead 外推 feed。它能提升 route completion，
  但尚未解决 vehicle heading error 过高的问题。
- `lookahead_pursuit` 不改变结果，因此下一问题仍在估计器侧 heading consistency，而不是控制器 pursuit 强度。

## 下一步计划

1. 将低权重外推 feed 保留为当前最佳候选。
2. 增加同步分析图：
   route progress vs feed allowed、heading error vs reject heading，以及 lookahead
   position error vs local residual。
3. 如果策略稳定，使用新增通道作为论文图表材料，展示可观测性、门控拒绝原因、估计器就绪程度和控制收益。

## Heading 平滑后续测试

实现：

- 新增默认关闭的 feed-heading smoothing：
  - `magnetic_lookahead_feed_heading_smoothing_enabled`
  - `magnetic_lookahead_feed_heading_max_step_deg`
- smoother 只应用于写入 `local_path` 的 heading。
  原始 `MagneticLookaheadTarget` 诊断保持不变。
- 代表性 variants：
  - `p40_probe10_extrapolated_low_smooth12`
  - `p41_probe10_extrapolated_low_smooth6`
  - `p42_probe10_extrapolated_low_smooth9`

命令：

```bash
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p40_probe10_extrapolated_low_smooth12 --name p41_probe10_extrapolated_low_smooth6 --max-steps 24000
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p42_probe10_extrapolated_low_smooth9 --max-steps 24000
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p40_probe10_extrapolated_low_smooth12
```

| variant | run | health | TRACK XT | TRACK vehicle err | route | final dist | lookahead pos err | feed allowed | reject heading | 结论 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `p40_probe10_extrapolated_low_smooth12` | 24000 | 23.8 | 26.9m | 140.7deg | 40.0% | 0.2m | 13.1m | 25.0% | 56.9% | 与 p36 相同；12deg/step 太宽，无法改变行为。 |
| `p41_probe10_extrapolated_low_smooth6` | 24000 | 17.2 | 18.5m | 49.7deg | 3.5% | 52.3m | 26.7m | 45.0% | 36.0% | 强平滑降低了 heading error，但也移除了有用转弯信息。 |
| `p42_probe10_extrapolated_low_smooth9` | 24000 | 26.3 | 10.0m | 81.2deg | 0.0% | 5.6m | 15.0m | 49.3% | 39.3% | 中等平滑仍会导致 route progress 崩溃。 |
| `p40_probe10_extrapolated_low_smooth12` | full | 15.3 | 30.1m | 115.1deg | 58.9% | 53.8m | 7.4m | 7.5% | 69.4% | 完整运行与 p36 一致：安全但不能修正问题。 |

解释：

- 简单的逐 feed heading step 限幅不是缺失的主要机制。
- lookahead heading 同时包含有害的 alias jump 和有用的转弯更新。
  过度平滑会阻断后者，并导致 route progress 崩溃。
- 下一步算法方向应从标量 heading smoothing 转向状态化 axis selection / 多假设消歧，
  最好由 phase event 和 route-progress consistency 共同驱动。

## Axis-selection 后续测试

实现：

- 新增默认关闭的 lookahead axis selection：
  - `magnetic_lookahead_axis_selection_enabled`
  - `magnetic_lookahead_axis_selection_min_progress_m`
- selector 根据 phase-to-phase anchor progress 选择 `+axis/-axis` 符号；
  当 anchor displacement 太小时，退回使用 phase-to-phase vehicle progress。
- 新增默认关闭的控制器实验：
  - `local_path_curve_track_flip_to_vehicle_enabled`
- 该控制器实验在 `curve_track` fused heading 背离当前车辆航向时将其翻转 180deg，
  与非 curve TRACK 行为保持一致。
- 代表性 variants：
  - `p43_probe10_extrapolated_low_axis`
  - `p44_probe10_extrapolated_low_curveflip`

命令：

```bash
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p43_probe10_extrapolated_low_axis --max-steps 24000
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p36_probe10_extrapolated_low --name p44_probe10_extrapolated_low_curveflip --max-steps 24000
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p44_probe10_extrapolated_low_curveflip
```

| variant | run | health | TRACK XT | TRACK vehicle err | route | final dist | lookahead pos err | feed allowed | reject heading | 结论 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `p36_probe10_extrapolated_low` | 24000 | 23.8 | 26.9m | 140.7deg | 40.0% | 0.2m | 13.1m | 25.0% | 56.9% | 当前 route-progress 基线。 |
| `p43_probe10_extrapolated_low_axis` | 24000 | 12.1 | 26.7m | 102.1deg | 1.8% | 42.3m | 21.6m | 43.1% | 36.8% | Phase-anchor progress 作为直接 axis sign selector 仍然太噪。 |
| `p44_probe10_extrapolated_low_curveflip` | 24000 | 23.3 | 7.7m | 41.2deg | 38.4% | 66.1m | 23.3m | 43.9% | 16.1% | 降低 heading/cross-track error，但牺牲 route progress。 |
| `p44_probe10_extrapolated_low_curveflip` | full | 22.9 | 7.7m | 41.2deg | 40.8% | 127.9m | 40.7m | 31.7% | 11.6% | 确认 curve flip 不能作为默认修复；任务推进差于旧 p36 full route 58.9%。 |

解释：

- 显式 axis sign selection 作为单元测试级 primitive 仍有价值，但在 dropout maze 中直接使用
  phase-anchor progress 还不够鲁棒。
- 控制器侧 heading flip 证明：高 heading error 的一部分来自方向歧义，而不是纯粹跟踪失败；
  但它也改变了控制行为，并降低 route completion。
- 下一条可行路径是真正的多假设 selector：同时保留两个 axis sign，在短窗内根据 route progress、
  lookahead innovation 和 controller progress 打分，再通过 hysteresis 决定是否提交切换。

## Hysteresis 多假设尝试

实现：

- 新增默认关闭的 lookahead axis hysteresis：
  - `magnetic_lookahead_axis_hysteresis_enabled`
  - `magnetic_lookahead_axis_hysteresis_threshold`
  - `magnetic_lookahead_axis_score_decay`
- 与 `p43` 不同，`p45` 不再让单次 phase-anchor progress 直接决定 axis sign。
  它将 `+axis/-axis` 证据累积分数，只有分数越过 hysteresis 阈值才提交方向切换。
- 该实现仍然位于 `MagneticLookaheadTargetBuilder` 内部，因此最终仍会向下游输出一个单一方向。

命令：

```bash
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p45_probe10_extrapolated_low_axis_hyst --max-steps 24000
```

| variant | run | health | TRACK XT | TRACK vehicle err | route | final dist | lookahead pos err | feed allowed | reject heading | 结论 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `p45_probe10_extrapolated_low_axis_hyst` | 24000 | 17.2 | 7.7m | 41.2deg | 0.0% | 43.3m | 10.7m | 46.3% | 36.9% | Hysteresis 降低 heading/cross-track error，但仍使 route progress 崩溃。 |

解释：

- `p45` 说明“在 lookahead builder 内部选择并提交单一方向”仍然过早。
- 低 heading error 不等于任务推进健康。该问题必须继续以 route completion、final dist、
  endpoint status 为主指标，而不是只看 `TRACK vehicle err`。
- 下一阶段需要把 `+axis/-axis` 多假设保留到任务/控制评估窗口中，而不是在感知层提前压成一个方向。

## 后续 Zig-zag 整合总计划

### 总目标

目标不是单独让 zig-zag 看起来“更平滑”，而是构建一条可闭环的纯磁/稀疏声呐跟踪链路：

1. 小幅 zig-zag 负责提供横向激励，使磁场几何可观测。
2. 磁观测层输出局部轴线、横偏、埋深和置信度，但不急于承诺全局方向。
3. 多假设层保留 `+axis/-axis`、相邻 branch、lookahead 候选。
4. 任务层用短窗推进质量选择假设，而不是让单帧感知直接驱动控制。
5. 控制层只消费“已通过任务层确认”的局部目标，并保持常规声呐场景不退化。

### 评价口径

所有阶段默认以任务级指标排序：

- 第一优先级：`route_completion_ratio`、`final_route_distance_m`、`endpoint_completed`。
- 第二优先级：`track_mean_cross_track_m`、`track_mean_vehicle_heading_error_deg`。
- 第三优先级：lookahead position error、feed allowed/reject reason、burial MAE。
- 禁止只因 `mean_heading_error` 或局部 heading 降低就接受方案；`p44/p45` 已证明这会误导。

### D0：冻结基线和诊断口径

目标：确保后续所有实验可比较。

当前落地状态：

- `tools/evaluate_dropout_variants.py --phase d0` 已提供固定对照入口。
- 当前 D0 对照包含：
  - `d0_baseline`
  - `d0_p36_route_baseline`
  - `d0_p44_curveflip_counterexample`
  - `d0_sparse_sonar_anchor`
- CSV 输出已加入 probe cycle 字段：
  - `probe_active_pct`
  - `probe_cycles`
  - `probe_flips`
  - `probe_cycle_s`
  - `probe_peak_xt`
  - `probe_phase_per_cycle`

验证命令：

```bash
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase d0 --max-steps 2000
```

步骤：

1. 固定三个对照基线：
   - `p36_probe10_extrapolated_low`：当前 route-progress 最优纯磁外推候选。
   - `p44_probe10_extrapolated_low_curveflip`：低 heading error 但 route 退化的反例。
   - `case_maze_sparse_sonar`：稀疏声呐可成功推进的正例。
2. 输出统一报告表：
   - `health`
   - `track_xt`
   - `track_vehicle_err`
   - `route`
   - `final_dist`
   - `lookahead_pos_err`
   - `feed_allowed`
   - `reject_heading`
   - `burial_cov`
   - `burial_mae`
3. 给 `render_detail()` 增加同步图：
   - route progress vs mode/task state
   - feed allowed vs reject reason
   - lookahead pos err vs local residual
   - vehicle heading err vs hypothesis id
4. 验收：
   - 重跑同一 variant 结果误差在可接受范围内。
   - 文档表格和 CSV 字段一致。

### D1：把 zig-zag probe 从控制动作提升为“观测周期”

目标：明确每个 zig-zag 周期何时开始、何时跨线、何时形成有效 phase observation。

当前落地状态：

- `RunRecord` 已新增逐帧 probe cycle 通道：
  - `zigzag_probe_active`
  - `zigzag_probe_cycle_id`
  - `zigzag_probe_leg_sign`
  - `zigzag_probe_cycle_age_s`
  - `zigzag_probe_leg_flip_event`
  - `zigzag_probe_signed_cross_track_m`
  - `zigzag_probe_cycle_peak_abs_cross_track_m`
  - `zigzag_probe_phase_count`
  - `zigzag_probe_last_cycle_duration_s`
- `compute_health_metrics()` 已汇总：
  - `zigzag_probe_active_fraction`
  - `zigzag_probe_cycle_count`
  - `zigzag_probe_leg_flip_count`
  - `zigzag_probe_mean_cycle_duration_s`
  - `zigzag_probe_mean_peak_abs_cross_track_m`
  - `zigzag_probe_phase_events_per_cycle`
- `render_detail()` 已增加：
  - zig-zag probe cycle 面板。
  - route progress / route distance 面板。
- 单测已覆盖：开启 probe 后 recorder 必须记录 probe cycle 通道且存在 active frame。

验证命令：

```bash
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p36_probe10_extrapolated_low --max-steps 6000
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p36_probe10_extrapolated_low --max-steps 12000
```

步骤：

1. 新增 `ZigzagProbeCycleState` 或等价诊断结构：
   - `cycle_id`
   - `leg_sign`
   - `cycle_start_time_s`
   - `left_extreme_xy`
   - `right_extreme_xy`
   - `crossing_detected`
   - `amplitude_m`
   - `duration_s`
   - `axis_delta_deg`
2. 将 `MagneticZigzagPhaseDetector` 的输出从单点 observation 扩展为 cycle summary。
3. 将 cycle summary 记录到 recorder 和 metrics：
   - phase 成功率
   - phase 平均周期
   - phase 平均振幅
   - phase axis consistency
4. 验收：
   - `case1-6` 和 `case1v-5v` 开启 probe 不应明显降低 route/endpoint。
   - dropout 中 phase event 的时间位置可解释，不再只是稀疏黑箱点。

### D2：多假设感知层，不提前提交方向

目标：从“输出一个 lookahead target”改为“输出候选集合”。

当前测试状态：

- 本轮按既有 D2 代表点复测，没有新增闭环默认行为。
- 命令：

```bash
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase d2 --max-steps 12000
```

| variant | run | health | TRACK XT | TRACK vehicle err | route | final dist | probe active | probe burial MAE | 结论 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `d0_p36_route_baseline` | 12000 | 25.3 | 7.7m | 41.2deg | 37.0% | 31.1m | 1.4% | 1.839m | 当前 12k route-progress 对照。 |
| `d2_local_age180` | 12000 | 26.1 | 18.0m | 88.2deg | 1.7% | 23.5m | 21.9% | 0.423m | probe/埋深窗口变多，但任务推进崩溃。 |
| `d2_local_capacity36` | 12000 | 37.4 | 1.5m | 24.7deg | 1.2% | 27.2m | 1.5% | 0.410m | 局部误差好看，但 route 无收益。 |
| `d2_spacing2m` | 12000 | 25.5 | 20.8m | 90.3deg | 2.4% | 23.5m | 19.6% | 0.859m | 未形成有效推进。 |

解释：

- D2 参数类调整仍无法替代真正的候选集合。
- 多假设不应再只通过 local path 参数间接实现；需要显式保留候选，并把选择权上移到任务层。

步骤：

1. 新增 `MagneticLookaheadHypothesis`：
   - `hypothesis_id`
   - `axis_sign`
   - `anchor_xy_m`
   - `direction_xy`
   - `cable_point_xy_m`
   - `lookahead_xy_m`
   - `confidence`
   - `age_s`
   - `innovation_m`
   - `local_residual_m`
2. `MagneticLookaheadTargetBuilder` 保留两个方向：
   - `+axis`
   - `-axis`
3. 对每个方向分别计算：
   - 到 local_path 的 innovation
   - 与上一 accepted hypothesis 的连续性
   - 与车辆短窗位移方向的一致性
   - 与 route/progress 代理指标的一致性
4. 不在 builder 内做最终选择，只输出候选集合。
5. 验收：
   - 离线单元测试中，方向歧义场景能同时保留两个候选。
   - 单个错误 phase anchor 不会删除另一个假设。

### D3：任务层短窗打分和 hysteresis 提交

目标：让任务推进质量决定选择哪一个假设。

当前测试状态：

- 命令：

```bash
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase d3 --max-steps 12000
```

| variant | run | health | TRACK XT | TRACK vehicle err | route | final dist | probe active | probe burial MAE | 结论 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `d3_progressive_gate` | 12000 | 26.5 | 18.7m | 89.5deg | 1.6% | 28.4m | 23.3% | 0.596m | progressive gate 增加 probe 窗口，但没有任务推进收益。 |

解释：

- 仅靠现有 progressive gate 不能构成 D3 所需的任务层短窗 selector。
- 后续 D3 必须新增显式 `LookaheadHypothesisSelector`，并以 route/control progress 作为选择依据。

步骤：

1. 新增 `LookaheadHypothesisSelector`，位于 perception/task 边界。
2. 每个短窗计算候选分数：
   - `progress_score`：沿当前任务目标方向的净推进。
   - `stability_score`：控制指令变化幅度、mode switch 次数。
   - `innovation_score`：lookahead 与 local_path 的残差。
   - `feed_score`：feed allowed 比例和 reject reason。
   - `safety_score`：横偏、转弯半径、yaw rate 是否接近限制。
3. 使用 hysteresis：
   - 当前假设保持一定惯性。
   - 新假设必须连续多个窗口胜出才切换。
   - 切换后进入短暂 cooldown。
4. 输出诊断：
   - `active_hypothesis_id`
   - `hypothesis_score_plus`
   - `hypothesis_score_minus`
   - `hypothesis_switch_reason`
   - `hypothesis_margin`
5. 验收：
   - 不要求一次到 endpoint，但必须超过 `p36` 的 58.9% full route 或显著降低 final dist。
   - 不能出现 `p44/p45` 那种 heading 改善但 route 崩溃的情况。

### D4：控制层消费“确认后的局部目标”

目标：控制层不再直接响应未确认的磁方向跳变。

当前测试状态：

- 12k 窗口命令：

```bash
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase d4 --max-steps 12000
```

- 完整长测命令：

```bash
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase d4 --name d4_sparse035 --name d4_sparse050
```

| variant | run | health | TRACK XT | TRACK vehicle err | route | final dist | probe active | probe burial MAE | 结论 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `d4_sparse035` | 12000 | 39.1 | 4.0m | 13.2deg | 19.4% | 30.6m | 50.4% | 1.120m | 局部控制质量好，但 12k route 仍低于 p36。 |
| `d4_sparse050` | 12000 | 37.5 | 4.7m | 17.1deg | 20.9% | 23.7m | 56.3% | 2.590m | 声呐概率提高不等于推进提升。 |
| `d4_sparse035` | full | 44.1 | 4.1m | 29.9deg | 55.9% | 4.7m | 44.8% | 0.508m | 接近 p36 full route 58.9%，但未超过；横偏显著更好。 |
| `d4_sparse050` | full | 41.6 | 4.1m | 26.1deg | 35.3% | 5.2m | 34.1% | 1.956m | route 明显退化，说明锚点策略比命中率更关键。 |

解释：

- D4 说明稀疏声呐仍是最可靠锚点来源，但不能简单提高检测概率。
- 控制层应只消费任务层确认后的目标；未确认候选继续 shadow，不直接改主控制。

步骤：

1. 控制输入分层：
   - confirmed target：任务层已确认，可用于主控制。
   - candidate target：仅用于诊断或低权重辅助。
   - probe command：用于维持观测激励，不承担全局寻路。
2. `TRACK_ACTIVE` 中保持小幅 zig-zag：
   - probe angle 默认 6-10deg。
   - curve segment 可动态降低 crossing angle。
   - yaw rate 和曲率约束必须低于控制上限。
3. 若 hypothesis selector 未确认：
   - 保持当前稳定 heading。
   - 不把候选 heading 写入主控制。
   - 可以继续采样，等待下一 phase cycle。
4. 验收：
   - route 不低于 p36。
   - `TRACK vehicle err` 相比 p36 明显下降。
   - mode switch 不显著增加。

### D5：埋深估计和磁场几何联合接入

目标：让 zig-zag 的价值不仅是找方向，也用于估计 cable position 和 burial depth。

当前落地状态：

- D5 已以 shadow 诊断形式接入，不反馈控制层。
- `RunRecord` 新增 probe active 帧磁场几何与埋深通道：
  - `zigzag_probe_b_down_nt`
  - `zigzag_probe_b_perp_nt`
  - `zigzag_probe_field_ratio`
  - `zigzag_probe_burial_valid`
  - `zigzag_probe_burial_error_m`
- `compute_health_metrics()` 已汇总：
  - `zigzag_probe_mean_abs_field_ratio`
  - `zigzag_probe_mean_abs_b_perp_nt`
  - `zigzag_probe_burial_coverage`
  - `zigzag_probe_burial_mae_m`
- `tools/evaluate_dropout_variants.py` CSV 已加入：
  - `probe_field_ratio`
  - `probe_bperp`
  - `probe_burial_cov`
  - `probe_burial_mae`

验证命令：

```bash
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p36_probe10_extrapolated_low --max-steps 12000
```

| variant | run | probe active | probe field ratio | probe B_perp | probe burial cov | probe burial MAE | global burial cov | global burial MAE | 结论 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `p36_probe10_extrapolated_low` | 12000 | 1.4% | 4.17 | 90.7nT | 100.0% | 1.839m | 87.0% | 0.203m | probe 窗口可观测，但当前 probe 段 burial 误差较高，只能 shadow。 |

解释：

- 当前 probe 周期能提供稳定的磁场几何诊断通道。
- 但 probe active 窗口太短，且 route-normal 下的 `B_down/B_perp` 仍受姿态、横偏和分支影响。
- D5 暂不进入闭环；后续需要 cycle-level posterior，而不是逐帧直接使用 burial 修正 offset scale。

步骤：

1. 在每个完整 zig-zag cycle 内估计：
   - 横向磁场梯度
   - `B_down / B_perp` 比值
   - burial depth posterior
   - cable center crossing point
2. 将 burial 估计作为独立诊断，不先强行反馈控制。
3. 当 burial 置信度稳定后，才用于修正 magnetic path offset scale。
4. 验收：
   - burial MAE 在有足够 phase coverage 时下降。
   - offset 估计变稳，而 route 不退化。

### D6：稀疏声呐协同

目标：把 sparse sonar 当作锚点，而不是让纯磁在无全局约束下独自寻路。

步骤：

1. sparse sonar 命中时重置或校准假设集合：
   - 更新 anchor。
   - 校正 branch。
   - 降低错误假设权重。
2. sonar dropout 时由 zig-zag/magnetic 维持短窗局部推进。
3. sonar 恢复时做一致性检查：
   - 若 sonar 与 active hypothesis 一致，提高置信度。
   - 若不一致，进入候选切换评估，而不是立刻跳变。
4. 验收：
   - `case_maze_sparse_sonar` 保持高成功率。
   - `case_maze_sonar_dropout` 至少超过 p36 full route。

### D7：回归和合入门槛

目标：避免为了 dropout 破坏主线。

步骤：

1. 仅当 dropout 代表点有效后，运行：
   - `case1-6`
   - `case1v-5v`
   - `case_maze_sonar`
   - `case_maze_sparse_sonar`
2. 合入门槛：
   - 常规 case 不降低 endpoint。
   - `case6` 继续以 `track_vehicle_err` 和 route/endpoint 为主，不被 `mean_heading_error` 误导。
   - maze sonar 不低于当前主线。
3. 所有新功能默认关闭，只有证明稳定的路径进入默认配置。

### 多方向备选路线

如果 D2-D4 仍失败，按以下优先级转向：

1. 任务层可观测区域重捕获 + zig-zag 观测窗口：
   - 复用 Phase D 的 `REACQUIRE_REGION` 思路。
   - 不追求纯磁连续跟踪，而是让 zig-zag 提供局部可观测区域。
2. 局部拓扑图而非单线估计：
   - 对迷宫分叉维护多个 branch candidate。
   - 用 sonar/phase event 逐步淘汰。
3. 延迟控制接入：
   - 纯磁只生成 shadow target。
   - 控制继续依赖已有稳定 local_path。
   - 先证明 shadow target 对 route 方向有预测能力。
4. 更强主动探针：
   - 仅在 dropout/reacquire 状态提高 zig-zag 幅度或周期。
   - 常规 TRACK 仍保持小幅 probe，避免破坏 case1-6。

### 当前结论

目前已经证伪的方向：

- 直接 magnetic path feed。
- 高置信 phase anchor feed。
- 单纯调 heading gate。
- feed heading smoothing。
- phase-progress 直接 axis selection。
- builder 内 hysteresis axis selection。
- controller curve flip 作为默认修复。

仍值得继续的方向：

- 多假设保留到任务层，由短窗 route/control progress 选择。
- zig-zag cycle 级观测，而不是单帧或单 phase 点。
- sparse sonar 锚点与纯磁局部推进的协同。
- burial/offset 作为观测质量诊断，先 shadow，后闭环。
