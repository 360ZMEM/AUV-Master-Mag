# 受控 zig-zag 调试复盘与调参小白指南

> **定位**：本文是 19 号计划的阶段性收尾版。写给第一次接手这个系统的人——不需要先看完 p36-p77 全部实验记录，也能理解"为什么要这么调""怎么判断调对了""后续先动哪些参数"。
>
> **前置阅读**：建议先看 [15_场景调参小白指南](15_场景调参小白指南.md) 了解基本场景分类和指标口径；再看 [17_zigzag纯磁探针方案](17_zigzag纯磁探针方案.md) 了解小幅 zig-zag 作为探针的背景；最后看 [19_声呐磁传感长期闭环剪枝计划](19_声呐磁传感长期闭环剪枝计划.md) 了解完整路线图。

## 背景：我们面对的是什么问题

### 电缆跟踪的基本矛盾

AUV 沿海底电缆跟踪时，有两个互相矛盾的需求：

1. **路径推进**：AUV 需要沿着电缆方向前进，完成整条电缆的巡检。
2. **磁观测**：磁传感器需要 AUV 横向切过电缆，才能产生可解释的过线证据（峰值、相位、横偏）。

如果 AUV 一直平行于电缆前进，磁场变化不明显，磁传感器几乎"看不见"电缆。但如果 AUV 大幅横切，虽然磁观测变强了，车却会偏离路线。

这就是整个调试的核心矛盾：

> 如何在"不把车带丢"的前提下，给磁传感器提供足够的横向激励？

### 为什么这个问题在 dropout 场景特别严重

在 `case_maze_sonar_dropout` 中：

- 初期有声呐，AUV 可以正常进入 TRACK。
- TRACK 后声呐被强制离线。
- 此时系统只剩磁传感器。
- 但磁传感器缺少横向激励，无法产生足够的过线证据。
- 结果：局部航向看起来不差，但任务整体不前进，最终 `stop=duration`。

这说明系统缺的不是"再平滑一点的航向"，而是三个层次的问题：

1. **观测供给**：磁观测是否足够制造过线证据。
2. **候选选择**：这些证据是否能选出任务需要的前向轴线。
3. **控制消费**：控制器是否能消费这些证据，而不把车带离路线。

一句话总结：

> 这一路不是在找更大的 zig-zag，而是在找一个不会破坏路线推进、但能在必要时补充磁观测证据的受控探测窗口。

## 0. 先记住当前结论

当前最可靠的判断如下：

| 问题 | 当前答案 |
| --- | --- |
| 基线路径推进用谁？ | 仍以 p36 为稳定控制基线。 |
| 是否默认打开 ProbeBurstManager？ | 不打开，`probe_burst_manager_enabled=False`。 |
| 是否默认打开 reacquire-safe window？ | 不打开，`probe_burst_manager_reacquire_safe_window_enabled=False`。 |
| 如果显式打开 manager，推荐 entry/safe XT？ | 当前阶段使用 `80m`，不是 `20m`，也不是继续放大到 `100m`。 |
| `case1v-6v` 基本盘有没有被改坏？ | 没有。默认关闭时 manager 完全 inactive；小幅 zig-zag 开关 A/B 也不破坏基本盘。 |
| 目前最重要的实现修正是什么？ | `entry_abs_cross_track_m` 必须在进入 burst 时冻结，不能每帧用当前横偏覆盖。 |

不要把 `route completion` 单独当作成功。最终判断至少同时看：

| 指标 | 怎么读 |
| --- | --- |
| `stop_reason` | `endpoint` 才是到终点；`duration` 只是时间耗尽。 |
| `route_completion_ratio` | 可作为任务进度参考，但曲线路径中可能有投影假象。 |
| `route_progress_max_jump_m` / `route_progress_large_jump_count` | maze 专用检查：如果 route progress 单步大跳，说明可能跨 lane 抄近路。 |
| `maze_geometry_passed` | maze 最终几何口径；endpoint 成功但该项为 no 时，仍然判失败。 |
| `probe_burst_manager_active_fraction` | 默认关闭回归中必须是 `0.0%`。 |
| `probe_burst_manager_control_allowed_fraction` | manager 真正能输出控制的窗口比例。 |
| `probe_burst_manager_reacquire_safe_control_allowed_fraction` | 被 safe-window 从 deployment reacquire 中放行的比例。 |
| `track_mean_cross_track_m` | TRACK 段是否贴线。 |
| `track_mean_vehicle_heading_error_deg` | 车辆实际走向是否沿电缆切向。 |

特别注意 maze 场景：

- `endpoint=yes` 不等于几何成功。AUV 可以通过最近点投影跳到远端 lane，看起来 route 到了终点，但图上实际是抄近路。
- `fused_heading` 是局部感知方向，不等于车辆真的沿任务路线推进。dropout 中更要优先看 `track_mean_vehicle_heading_error_deg` 和 route 是否增长。
- `case_maze_sonar_dropout` 当前默认仍失败：route 约 `1.5%`，TRACK vehicle heading error 约 `90°`。这不是“再平滑一点 heading”能解决的问题。

## 1. 我们最开始为什么要调 zig-zag

`case_maze_sonar_dropout` 的问题不是局部角度不好，而是任务没有前进：

- 初期有声呐，进入 TRACK 后声呐被强制离线。
- 局部 fused heading 看起来不差。
- 但 route 进度长期上不去，最终 `stop=duration`。

这说明系统缺的不是“再平滑一点的航向”，而是：

1. 磁观测是否足够制造过线证据。
2. 这些证据是否能选出任务需要的前向轴线。
3. 控制器是否能消费这些证据，而不把车带离路线。

因此后续调试分成三层：

| 层级 | 要回答的问题 | 典型诊断 |
| --- | --- | --- |
| 观测供给 | 有没有 crossing / phase / lookahead 证据？ | `probe_cycles`、`probe_peak_xt`、forward crossing count |
| 候选选择 | 选的是前进候选还是后退候选？ | shadow hypothesis、progress-aligned candidate |
| 控制消费 | 证据能否变成真实控制窗口？ | manager state、control_allowed、safe_allowed |

## 2. 为什么不能直接把 zig-zag 调大

早期尝试过直接增大 zig-zag 角度和宽度。结果很清楚：

| 路线 | 得到什么 | 代价 |
| --- | --- | --- |
| 小幅 p36 | route 能推进，基线稳定 | 磁 crossing 证据不足 |
| 大角度/大宽度 p48-p56 | 磁观测可能增加 | route completion 直接崩到很低 |
| 纯 decoupled lateral p57-p66 | forward evidence 很强 | 车辆严重脱轨 |

这就是为什么后面不继续做“角度网格搜索”。单纯调大激励会把 AUV 从“跟踪电缆”变成“为了观测而横向乱扫”。

正确方向是：把激进探测包进短时状态机里，并且显式恢复路线。

## 3. p36 为什么还保留为基线

p36 的价值不是磁观测最强，而是路径推进稳定。后续所有新策略都要满足：

1. 默认关闭时不改变 p36。
2. 打开时必须证明 route/endpoint 不退化。
3. 如果只提高了局部磁证据，却破坏 route completion，就不能闭环。

所以本阶段所有 manager 改动都遵守：

```text
默认配置:
probe_burst_manager_enabled = False
probe_burst_manager_reacquire_safe_window_enabled = False
```

这意味着普通 `case1v-6v`、小幅 zig-zag probe、常规可视化流程都不会被 ProbeBurstManager 改写控制行为。

## 4. ProbeBurstManager 是怎么来的

直接用定时器做 burst/recovery 很难解释，也很容易在错误任务阶段触发。最后采用显式状态机：

```text
IDLE_BASELINE
  -> BURST_COLLECT_EVIDENCE
  -> RECOVER_ROUTE
  -> COOLDOWN
  -> IDLE_BASELINE
```

四个状态的直觉：

| 状态 | 用白话解释 | 允许什么 |
| --- | --- | --- |
| `IDLE_BASELINE` | 正常跟踪，观察是否具备安全探测条件 | 累计健康 TRACK 时间和 route delta |
| `BURST_COLLECT_EVIDENCE` | 短时激进探测，尽量拿到磁证据 | decoupled lateral heading override |
| `RECOVER_ROUTE` | 探测后把车拉回 route | route centerline recovery |
| `COOLDOWN` | 冷却，避免频繁探测 | 不再发 burst/recovery 控制 |

关键点：

- manager 只在 `TRACK_ACTIVE` 中累计和执行。
- `LOCK_ALIGN` 不允许触发 burst，否则会破坏 track 建立。
- `enabled` 和 `control_allowed` 是两回事：
  - `enabled=True`：可以累计健康 TRACK baseline。
  - `control_allowed=True`：当前帧真的允许输出 burst/recovery 控制。
- 如果已经进入 pending burst，但 deployment reacquire 把控制压住，状态可以保持 pending，直到 safe-window 允许执行。

## 5. 这一路最容易误判的坑

### 5.1 只看 route 会误判

在曲线路径中，`route_completion_ratio` 是投影进度，可能非单调。截断仿真时尤其容易看到 route 忽高忽低。

正确做法：

- 看 `stop_reason` 是否为 `endpoint`。
- 看 endpoint 距离。
- 看事件序列，例如 `enter_burst`、`burst_timeout`、`recovery_complete`、`recovery_timeout`。

### 5.2 只看 state 不够

状态机进入 `BURST_COLLECT_EVIDENCE` 不等于控制真的生效。必须同时看：

| 通道 | 含义 |
| --- | --- |
| `probe_burst_manager_state_code` | 当前状态 |
| `probe_burst_manager_burst_active` | 当前帧是否真的输出 burst 控制 |
| `probe_burst_manager_recovery_active` | 当前帧是否真的输出 recovery 控制 |
| `probe_burst_manager_control_allowed` | 当前帧是否允许普通控制 |
| `probe_burst_manager_reacquire_safe_control_allowed` | 当前帧是否被 safe-window 放行 |

### 5.3 entry XT 必须冻结

这是本轮最关键的实现教训。

错误版本：

```text
decision.entry_abs_cross_track_m = 当前帧 obs.abs_cross_track_m
```

后果：

- 进入 burst 时横偏满足 80m。
- recovery 过程中瞬时横偏变大。
- safe-window 又被当前横偏关掉。
- 状态机看起来有设计，但控制窗口被重新压死。

正确版本：

```text
进入 BURST_COLLECT_EVIDENCE 时冻结 entry_abs_cross_track_m
burst / recovery / cooldown 期间一直返回冻结值
回到 IDLE_BASELINE 时清空
```

修正后，`entry/safe XT=80m` 从 `82.6%` 提升到 `99.5% endpoint`。

## 6. 当前推荐参数从哪里来

当前 manager 相关默认值：

| 参数 | 当前值 | 解释 |
| --- | ---: | --- |
| `probe_burst_manager_enabled` | `False` | 默认不接管控制 |
| `probe_burst_manager_reacquire_safe_window_enabled` | `False` | 默认不覆盖 deployment reacquire |
| `probe_burst_manager_entry_max_abs_cross_track_m` | `80.0` | 显式打开 manager 后的宽 entry recovery 门限 |
| `probe_burst_manager_reacquire_safe_max_abs_cross_track_m` | `80.0` | safe-window 使用同一 entry XT 口径 |
| `probe_burst_manager_burst_min_duration_s` | `4.0` | 至少采集 4s |
| `probe_burst_manager_burst_max_duration_s` | `12.0` | 无证据时最多 burst 12s |
| `probe_burst_manager_recovery_min_duration_s` | `20.0` | recovery 至少执行 20s |
| `probe_burst_manager_recovery_target_route_delta_m` | `8.0` | recovery 需恢复一定 route progress |
| `probe_burst_manager_recovery_timeout_s` | `120.0` | 防止 recovery 卡死 |
| `probe_burst_manager_cooldown_duration_s` | `120.0` | 两次 burst 之间冷却 |

为什么是 80m？

| 口径 | 结果 | 判断 |
| --- | --- | --- |
| `20m` | 基本不放行 | 太保守，解决不了 deployment reacquire 互斥 |
| `60m` | 仍基本不放行 | 低于首次有效 entry 窗口 |
| `80m` | 冻结 entry XT 后到 `99.5% endpoint` | 当前有界有效点 |
| `100m` | 触发 recovery timeout，route 退化 | 过宽，会把 recovery 带入失控区 |
| `inf` | 可到 endpoint | 只能证明机制有效，不能作为安全门限 |

## 7. 后续调参优先顺序

不要从大网格开始。建议按以下顺序：

### 第一步：确认默认关闭不污染

命令口径：

```bash
python tools/visualize.py --variants
python tools/visualize.py --variants --zigzag-probe
```

检查：

- `case1v-5v` mean error 仍 `<15deg`。
- `case6v` 仍按强弯边界解释，不要求突然收口。
- `probe_burst_manager_active_fraction = 0.0%`。

### 第二步：只在 dropout 代表点打开 manager

不要先全局打开。只在 `case_maze_sonar_dropout` 代表点上打开：

```python
tracking.probe_burst_manager_enabled = True
tracking.probe_burst_manager_reacquire_safe_window_enabled = True
tracking.decoupled_lateral_target_control_enabled = True
```

然后看：

| 指标 | 合理现象 |
| --- | --- |
| `manager_active` | 不再是 0 |
| `safe_allowed` | 应出现非零窗口 |
| `transition_count` | 有 enter/recovery/cooldown 序列 |
| `recovery_timeout_count` | 不能频繁超时 |
| `stop_reason` | 优先追求 endpoint |

### 第三步：只动一类参数

如果没有进入 burst：

| 现象 | 先动什么 |
| --- | --- |
| `state=IDLE`，无 transition | 放宽 entry XT 或检查 idle baseline |
| `entry_xt` 总是大于 80m | 不要立刻放到 100m，先看为什么 TRACK 时横偏过大 |
| `control_allowed=0` | 看 deployment reacquire 是否持续压制 |

如果进入 burst 但没有 recovery：

| 现象 | 先动什么 |
| --- | --- |
| `burst_timeout` 多 | 看 evidence 是否供应不足 |
| `evidence_target` 少 | 看 progress-aligned candidate / dual gate |

如果 recovery 退化：

| 现象 | 先动什么 |
| --- | --- |
| `recovery_timeout` 多 | 优先看 recovery heading 是否真的指向 route |
| route delta 不涨 | 看 route-bound progress proxy 和 nominal route progress |
| 横偏回不来 | 不要先增大 entry XT；先检查 centerline recovery |

## 8. 什么时候可以考虑合入默认

目前不能默认打开 manager。要考虑默认启用，至少需要：

1. `case1v-6v` off/on A/B 不回归。
2. `case_maze_sonar_dropout` 在 manager 开启后稳定 endpoint。
3. `case_maze_sonar` 和 `case_maze_sparse_sonar` 不退化。
4. `recovery_timeout_count` 不依赖偶然参数。
5. 文档中能解释每个状态转换为什么发生。

当前只满足：

- 默认关闭不污染。
- 小幅 zig-zag 不破坏 `case1v-6v` 基本盘。
- `case_maze_sonar` / `case_maze_sparse_sonar` 已恢复为 endpoint + no large route jump 的 sonar baseline。
- `case_maze_sonar_dropout` 默认仍失败；shadow-only 能产生部分候选，但未形成 route-bound 控制推进；激进 probe/manager 会引入严重 projection jump，不能进入默认。

所以当前阶段的正确定位是：

> manager 是一个已验证的可选受控激进组件，不是默认控制主线。

## 8.1 maze/dropout 调参的正确读图顺序

看 maze 图时，先按任务级指标判断，再看局部 heading：

1. `stop_reason` / `endpoint_completed`：是否真的结束在终点。
2. `route_progress_max_jump_m` / `route_progress_large_jump_count`：有没有跨 lane projection jump。
3. `maze_geometry_passed`：最终几何口径是否通过。
4. `track_mean_vehicle_heading_error_deg`：车辆实际运动方向是否沿缆。
5. `track_mean_cross_track_m` / `p99_cross_track_m`：贴线是否稳定，是否存在少量极端脱轨帧。
6. `fused_heading` / `mean heading error`：最后再看局部感知角度。

dropout 调参尤其容易误判。若 `fused_heading` 很小，但 `track_mean_vehicle_heading_error_deg≈90°`、route 不增长，那么系统只是“局部估计看起来对”，AUV 并没有沿任务路线走。此时不要继续调 heading 平滑或扩大 zig-zag，而应先问：

| 问题 | 应看指标 |
| --- | --- |
| 是否有磁观测供给？ | magnetic path / phase / crossing / shadow fraction |
| 候选是否与任务前进方向一致？ | progress-aligned candidate、route-bound proxy |
| 控制器是否真的消费了候选？ | manager state、control_allowed、safe_allowed |
| 是否出现抄近路？ | route progress max jump、large jumps、maze geometry |

当前建议：把 `case_maze_sonar` 当作稳定 baseline 门槛，把 `case_maze_sonar_dropout` 当作可观测性探索场景。不要为了让 dropout endpoint 而打开 route prior；那会掩盖“磁候选是否能闭环推进”的核心问题。

### 8.2 无 route 先验自适应大角度的第一轮结论

本轮开始尝试 dropout 专用大角度策略，但仍不打开名义 route 先验：

```text
基础 TRACK zig-zag 角度 = 15°
目标横向有效距离 = 3m
角度自适应范围 = 15° ± 5°
ProbeBurstManager = off
route guard = off
```

先只开自适应角度，结果 route 只有 `2.1%`，TRACK vehicle heading error 仍约 `89°`。这说明“加角度”本身不够；如果 magnetic path 供给链路仍关闭，控制器没有更可靠的局部电缆目标可以消费。

随后打开 `magnetic_path_observation_enabled=True`，结果变为：

| 指标 | 结果 |
| --- | ---: |
| route completion | `16.9%` |
| TRACK vehicle heading error | `15.4°` |
| TRACK XT | `6.9m` |
| magnetic path observation | `28%` |
| magnetic crossings | `2` |
| route progress max jump | `0.0m` |

这个结果说明方向是有价值的：大角度激励配合 magnetic path 供给后，车辆实际航向明显变好，也没有抄近路。但它仍不是成功：endpoint 没到，最终横偏很大，磁 path 误差也很大。

调参建议：

- 不要继续单独把角度从 15° 往上推。
- 先看 `magnetic_path_axis_error`、`magnetic_path_position_error`、forward phase coverage。
- 如果 magnetic path 有供给但误差大，优先改观测门控和候选选择；如果供给少，再考虑短时 burst，而不是全程放大 zig-zag。
- 每次都必须同时检查 `route_progress_max_jump_m=0`，避免把抄近路误认为推进。

### 8.3 降低要求：允许低精度 route 先验后，dropout 会怎样？

如果工程上允许准备一条低精度 route prior，可以把问题难度降一档。这里的 route prior 不是完美真值，而是带平移误差的名义路线：

```text
nominal_route_prior_translation_xy_m = (0.0, 7.5m)
代表 5-10m route 先验误差带的中点
只平移控制器/感知层使用的先验路线
不平移真实电缆环境和 metrics 路线
```

本轮结果：

| 场景 | route | endpoint | geometry | TRACK XT | 说明 |
| --- | ---: | ---: | ---: | ---: | --- |
| sonar + 干净 route prior | `99.5%` | yes | yes | `0.8m` | 稳定基线 |
| sonar + `7.5m` noisy route prior | `99.5%` | yes | yes | `5.4m` | 可工作，但横偏随先验误差上升 |
| sparse sonar + `7.5m` noisy route prior | `99.5%` | yes | yes | `5.5m` | 可工作 |
| dropout + 无 route prior | `16.9%` | no | no | `6.9m` | 仍未闭环 |
| dropout + 干净 route prior | `99.5%` | yes | yes | `0.9m` | 可工作 |
| dropout + `7.5m` noisy route prior | `99.5%` | yes | yes | `6.8m` | 可工作，但更像低精度地图辅助 |

这说明一个很重要的调参判断：

- 如果目标是工程可用，低精度 route prior 很有价值，`5-10m` 误差仍能让 maze/dropout 跑通。
- 如果目标是证明“无 route prior 的磁闭环”，那这个结果不能算成功，因为 route prior 已经提供了全局拓扑和 lane 消歧。
- 后续论文可以把两者分开写：主线采用“低精度名义路线辅助的声呐-磁跟踪”，无 route prior dropout 作为更高难度消融实验。

### 8.4 更严重的 route 先验误差：平移 + 3° 旋转

只加 `7.5m` 平移不够代表真实地图误差，因为工程 route 还可能存在整体航向偏差。本轮加入：

```text
translation = (0.0, 7.5m)
rotation = 3.0°
```

未修复时，AUV 会明显跟随偏置先验。连续 sonar 和 sparse sonar 都只能到约 `62% route`，最终横偏接近 `450m`。这说明 `3°` 在长距离 maze 中已经非常严重，不能只靠“route prior + 原控制器”硬走。

修复后的做法：

- 有新鲜声呐时，控制器优先使用声呐电缆点和声呐 heading，而不是盲信 prior heading。
- route prior cache 支持在线平移/旋转校正，但 sparse sonar 下不持续改写整条 prior cache，避免稀疏锚点把回折迷宫投影拉错 lane。
- pose-noise 场景提高 TRACK 横偏修正能力，专门抵消角度偏差造成的慢性横漂。

修复后结果：

| 场景 | route | endpoint | geometry | max jump | TRACK XT | 说明 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| sonar + `7.5m + 3°` prior | `99.5%` | yes | yes | `0.9m` | `3.9m` | 可工作 |
| sparse sonar + `7.5m + 3°` prior | `99.5%` | yes | yes | `0.8m` | `11.7m` | 可工作但裕度较低 |
| dropout + `7.5m + 3°` prior | `99.5%` | yes | yes | `0.2m` | `1.7m` | 可工作 |

调参判断：

- `3°` 旋转误差不是小扰动；长距离后会累积成跨 lane 风险。
- 看到 route 99% 仍要看 `route_progress_max_jump_m`，因为未修复版本会出现 projection jump。
- sparse sonar 的健康分较低，说明“低精度地图 + 稀疏声呐”是可用但接近边界的场景，后续应优先看 TRACK XT 和 no-shortcut，而不是只看 endpoint。

### 8.5 DR/INS 慢漂：不要把真值坐标和导航坐标混用

DR/INS 接入后，最容易犯的错误是：车辆控制用漂移后的导航位姿，但声呐 wrapper 仍然输出真值 NED 坐标。这样会把两个坐标系混在一起，在线 prior correction 会被错误地拉偏。

当前正确数据流是：

```text
真实 pose -> 真实动力学 / 真实磁场 / 真实相对声呐
DR/INS NavigationSimulator -> navigation pose
声呐相对观测 + navigation pose -> navigation-frame cable point
perception + controller -> 使用 navigation pose
metrics -> 仍用真值 route 验收
```

新增的慢漂模型包含两部分：

- 低通白噪声：模拟 DR/INS 输出不会每帧乱跳，而是缓慢抖动。
- 随机游走漂移：模拟位置和航向误差随时间积累，但有最大漂移上限。

本轮还加入了轻微地图扭曲：

```text
translation = 7.5m
rotation = 3.0°
scale_xy = (0.99, 1.0)
```

也就是：地图整体有平移、航向偏差，并且 x 方向长度有 1% 缩放误差。

结果：

| 场景 | route | endpoint | geometry | max jump | TRACK XT |
| --- | ---: | ---: | ---: | ---: | ---: |
| sonar + DR/INS + 扭曲 prior | `99.5%` | yes | yes | `2.9m` | `6.8m` |
| sparse sonar + DR/INS + 扭曲 prior | `99.5%` | yes | yes | `0.5m` | `4.8m` |
| dropout + DR/INS + 扭曲 prior | `99.5%` | yes | yes | `0.2m` | `2.6m` |

调参建议：

- `nominal_route_prior_correction_gain` 在 DR/INS 慢漂下要小，本轮用 `0.01`。太大时会把迷宫回折处的 prior 投影拉错 lane。
- sparse sonar 要单独看。因为锚点少，强 DR/INS 漂移 + 稀疏声呐会变成两个困难问题叠加，不能只靠一个 gain 同时解决。
- 每次改 DR/INS 噪声，必须同时看 `route_progress_max_jump_m` 和 `Maze geometry passed`，不要只看 endpoint。

### 8.1 慢漂估计器门限怎么调

如果假设“只有最初点基本可靠”，后续就不能让每个稀疏声呐点都全局重配 route。正确做法是：

1. 先用上一帧 `nominal_route_progress_m` 约束投影窗口。
2. 再用实际电缆观测做小步 residual correction。
3. residual 太大或 heading error 太大时拒绝更新，而不是强行修正。

本轮新增的关键门限：

```text
nominal_route_progress_guard_enabled
nominal_route_progress_guard_lookback_m
nominal_route_progress_guard_lookahead_m
nominal_route_prior_correction_max_residual_m
nominal_route_prior_correction_max_step_m
nominal_route_prior_correction_max_heading_error_deg
```

当前经验值：

- sparse DR/INS：打开 progress guard，`correction_gain=0.01`，`max_step=0.35m`。
- continuous sonar：不要打开 progress guard；连续声呐本身足够强，额外 progress 窗口反而可能限制纠偏。
- dropout-prior：可以打开 progress guard，用来避免 route projection 跳到空间近但拓扑错的 lane。

边界复测结果：

| 场景 | route | endpoint | geometry | max jump | 结论 |
| --- | ---: | ---: | ---: | ---: | --- |
| sparse + mild DR/INS + `prob=0.15` | `99.5%` | yes | no | `60.6m` | 锚点不足，仍失败 |
| sparse + mild DR/INS + `prob=0.20` | `99.5%` | yes | yes | `0.5m` | 通过 |
| sparse + mid DR/INS + `prob=0.20` | `99.5%` | yes | yes | `0.4m` | 通过 |
| sonar + strong DR/INS | `99.5%` | yes | yes | `2.9m` | 通过 |
| dropout-prior + strong DR/INS | `99.5%` | yes | yes | `0.3m` | 通过 |

因此，当前 sparse sonar 的可用下限是 `prob_detection >= 0.20`。`prob=0.15` 不是“gain 再调一点”的问题，而是实际电缆锚点太少，route 拓扑仍会偶发跳变。

## 9. 最小复现命令

常规变种回归：

```bash
python tools/visualize.py --variants
python tools/visualize.py --variants --zigzag-probe
```

目标单测：

```bash
python -m pytest tests/test_probe_burst_manager.py tests/test_viz_progress.py -q
```

目标编译：

```bash
python -m py_compile \
  src/auv_mag_tracking/config/__init__.py \
  src/auv_mag_tracking/controller.py \
  src/auv_mag_tracking/probe_burst_manager.py \
  src/auv_mag_tracking/viz/recorder.py \
  src/auv_mag_tracking/viz/metrics.py \
  src/auv_mag_tracking/viz/report.py \
  tests/test_probe_burst_manager.py \
  tests/test_viz_progress.py
```

## 10. 调参禁忌

- 不要把 `XT=inf` 当作可上线方案。
- 不要只因为 route 变高就认为成功。
- 不要在 `LOCK_ALIGN` 阶段允许 burst。
- 不要让 entry XT 每帧刷新。
- 不要为了制造磁证据牺牲 route completion。
- 不要把 `case6v` 的 fused mean error 当成所有闭环失败的证据；还要看 TRACK vehicle error 和横偏。

当前最稳的后续路线：

1. 保留 `80m` 有界 entry/safe 门限。
2. 保持 manager 默认关闭。
3. 扩大 dropout 代表点回归。
4. 再考虑是否让 manager 成为某些部署场景的显式选项。

## 关键可视化

调参与复盘建议从 [23_论文图清单](23_论文图清单.md) 中按需调阅：

| slug | 用途 |
| --- | --- |
| `overview_heading_error` | 一眼看出 case 是否进入 15° 目标带，并通过 FSM 着色定位 entry 段 |
| `overview_cross_track` | 横偏均值是不是被 burst recovery 牺牲了 |
| `detail_fsm_timeline` | IDLE → BURST → RECOVER → COOLDOWN 是否符合期望节奏 |
| `detail_probe_cycle` | 调参时确认 leg sign 翻转、forward dot、X-fwd / X-bwd 行为 |
| `selector_progress_rate` | Progress rate 与 readiness 阈值的关系，决定 entry XT 冻结点 |
| `progress_health` / `progress_track_pct` | before/after 调参对比，写小白指南附录 |
