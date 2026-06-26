# 受控 zig-zag 调试复盘与调参小白指南

本文是 19 号计划的阶段性收尾版。它不假设读者已经看过全部 p36-p77 实验记录，而是从“为什么要这么调”“怎么判断调对了”“后续先动哪些参数”三个问题入手。

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
| `probe_burst_manager_active_fraction` | 默认关闭回归中必须是 `0.0%`。 |
| `probe_burst_manager_control_allowed_fraction` | manager 真正能输出控制的窗口比例。 |
| `probe_burst_manager_reacquire_safe_control_allowed_fraction` | 被 safe-window 从 deployment reacquire 中放行的比例。 |
| `track_mean_cross_track_m` | TRACK 段是否贴线。 |
| `track_mean_vehicle_heading_error_deg` | 车辆实际走向是否沿电缆切向。 |

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
- `case_maze_sonar_dropout` 单点证明 `80m + 冻结 entry XT` 可到 endpoint。

所以当前阶段的正确定位是：

> manager 是一个已验证的可选受控激进组件，不是默认控制主线。

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
