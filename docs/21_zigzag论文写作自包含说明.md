# zig-zag 机制论文写作自包含说明

本文面向论文写作，不按开发流水账展开，而是把当前代码中已经实施的 zig-zag 相关机制整理成可复述的技术方案。读者不需要先理解所有历史实验编号，也能看懂系统为什么需要 zig-zag、如何约束 zig-zag、以及当前实现的关键设计。

## 1. 问题设定

目标任务是 AUV 沿海底电缆跟踪，并在可行时利用磁传感数据辅助估计电缆几何和埋深。系统可使用：

- 声呐 wrapper 提供的电缆相对位置或局部路径观测。
- 磁传感器提供的磁场强度和磁异常特征。
- IMU / 航向 / 车辆状态。

困难在于，单帧磁强度并不直接给出“电缆中心线在哪里”。磁异常更像一个随横向距离变化的场强曲线：

- AUV 如果一直沿电缆平行前进，横向激励不足，磁场变化可能不明显。
- AUV 横切电缆时，更容易产生峰值、过线、相位和横偏证据。
- 但横切过强会破坏任务推进，让车辆偏离路线。

因此，本系统中的 zig-zag 不是传统意义上无约束搜索，而是一种受控观测激励：

> 在不显著损害路径推进的前提下，向磁传感链路注入足够横向运动，使系统获得可解释的电缆过线和局部几何证据。

## 2. 总体架构

当前实现可分为四层：

```text
传感层
  -> 感知层
  -> 任务/候选选择层
  -> 控制层
```

| 层级 | 输入 | 输出 | zig-zag 相关职责 |
| --- | --- | --- | --- |
| 传感层 | 磁场、声呐、车辆状态 | 原始观测包 | 提供磁异常与声呐相对位置 |
| 感知层 | 原始观测包 | local path、磁候选、shadow 诊断 | 从横向运动中提取 crossing / candidate |
| 任务/候选选择层 | 多个轴线候选、route progress proxy | progress-aligned candidate | 判断哪个候选符合任务前进方向 |
| 控制层 | 任务状态、候选、状态机决策 | heading command | 决定是否正常跟踪、短时 probe、或 recovery |

代码上，关键模块包括：

| 模块 | 作用 |
| --- | --- |
| `controller.py` | 任务控制、heading override、deployment reacquire 与 probe/recovery 仲裁 |
| `probe_burst_manager.py` | 受控激进 zig-zag 状态机 |
| `perception/orchestrator.py` | 感知编排，输出 local path、shadow candidate 和 deployment reacquire 信号 |
| `perception/hypotheses.py` | 多假设候选与 shadow selection |
| `perception/magnetic_path.py` | 磁路径观测与候选构造 |
| `viz/recorder.py` | 记录状态机、候选、控制窗口和指标通道 |
| `viz/metrics.py` | 计算 route、TRACK、manager 活跃度、safe-window 等指标 |

## 3. 基线跟踪与小幅 zig-zag

系统保留稳定的 baseline controller。其核心目标不是最大化磁证据，而是保持路径推进和基本跟踪稳定。

在普通 TRACK 阶段，小幅 zig-zag 的角色是“探针”：

- 角度通常在小范围，例如 `3deg` 级。
- 它不应该改变任务主线。
- 它可以提高磁路径观测比例。
- 它必须通过 `case1v-6v` 回归证明不破坏基本盘。

当前 `case1v-6v` A/B 结果显示：

| 结论 | 说明 |
| --- | --- |
| `case1v-5v` 保持 `<15deg` mean error | 小幅 zig-zag 没有破坏常规场景 |
| `case6v` 仍为强弯边界 | 未要求该指标突然收口 |
| route / stop 不退化 | 任务推进基本盘保持 |
| ProbeBurstManager 未参与 | A/B 只验证小幅 zig-zag probe，不混入 burst 状态机 |

这说明基础 zig-zag 可作为低风险观测激励，但不能解决所有 dropout 场景。

## 4. 为什么需要受控激进 probe

在 `case_maze_sonar_dropout` 中，声呐在 TRACK 后离线。普通小幅 zig-zag 难以持续提供足够磁 crossing 证据；但直接增大 zig-zag 又会破坏 route。

历史实验形成了一个关键矛盾：

| 方案 | 观测证据 | 任务推进 |
| --- | --- | --- |
| 稳定基线 | 弱 | 稳定 |
| 大幅 zig-zag | 增强 | 明显退化 |
| 纯 decoupled lateral | 很强 | 严重脱轨 |

因此，论文中可以把核心问题表述为：

> 需要在“观测激励强度”和“路径推进稳定性”之间引入时序约束，而不是单纯调整 zig-zag 幅度。

对应实现就是 `ProbeBurstManager`。

## 5. ProbeBurstManager 状态机

`ProbeBurstManager` 将激进横向探测约束为有限状态机：

```text
IDLE_BASELINE
  -> BURST_COLLECT_EVIDENCE
  -> RECOVER_ROUTE
  -> COOLDOWN
  -> IDLE_BASELINE
```

每个状态的论文级解释：

| 状态 | 含义 | 退出条件 |
| --- | --- | --- |
| `IDLE_BASELINE` | 正常跟踪并积累健康前进证据 | TRACK 时间、route delta、entry XT 同时满足 |
| `BURST_COLLECT_EVIDENCE` | 短时增加横向激励，采集磁 crossing / candidate | 达到最小时长且获得证据，或超过最大时长 |
| `RECOVER_ROUTE` | 停止激进探测，恢复到路线推进 | route delta、横偏或超时条件满足 |
| `COOLDOWN` | 防止频繁重复探测 | 冷却时间结束 |

简化伪代码：

```text
if state == IDLE_BASELINE:
    accumulate healthy TRACK time and forward route delta
    if enough baseline and entry XT is acceptable:
        freeze entry XT
        enter BURST_COLLECT_EVIDENCE

if state == BURST_COLLECT_EVIDENCE:
    if control_allowed:
        output burst heading override
        accumulate control-active time
    if evidence collected after min duration:
        enter RECOVER_ROUTE
    elif max duration reached:
        enter RECOVER_ROUTE

if state == RECOVER_ROUTE:
    if control_allowed:
        output route recovery heading
        accumulate recovery progress
    if route recovered:
        enter COOLDOWN
    elif timeout:
        enter COOLDOWN

if state == COOLDOWN:
    wait before allowing another burst
```

关键工程约束：

- 只在 `TRACK_ACTIVE` 阶段允许 manager 累计和执行。
- 不允许在 `LOCK_ALIGN` 阶段触发 burst。
- `enabled` 与 `control_allowed` 分离，使状态机能进入 pending 状态，但不在安全门关闭时强行控制。
- active 状态下如果 manager 被 disable，则 reset，避免半截控制残留。

## 6. Entry XT 冻结机制

论文中建议明确写出 entry gate 的语义，因为这是本阶段最关键的实现修正。

在进入 `BURST_COLLECT_EVIDENCE` 时，系统记录：

```text
entry_abs_cross_track_m = abs(cross_track_offset_m at burst entry)
```

之后在 burst、recovery、cooldown 期间，该值保持不变。它代表“本次 probe 的进入条件”，而不是车辆当前横偏。

这样做的理由：

1. entry gate 应描述启动 probe 时是否安全。
2. recovery 过程中当前横偏可能临时变大，如果每帧刷新 entry XT，会把已经批准的 recovery 窗口重新关掉。
3. 冻结 entry XT 后，状态机决策具有可解释性：一次 probe 使用一次 entry 证据。

修正前后差异：

| 实现 | 结果 |
| --- | --- |
| entry XT 每帧刷新 | `80m` safe-window 被 recovery 中的瞬时横偏关掉，route 只能到约 `82.6%` |
| entry XT 冻结 | `80m` 有界门限可达到 `99.5% endpoint` |

这部分适合作为论文中“工程化稳定机制”的一个小节。

## 7. Deployment reacquire 与 safe-window

在 dropout 场景中，`deployment_reacquire_required` 可能长期为真。它的本意是安全：当系统认为局部路径需要重捕获时，由 deployment reacquire 控制逻辑接管。

但这会带来一个冲突：

- ProbeBurstManager 已经进入 pending burst/recovery。
- deployment reacquire 持续压制 heading override。
- manager 状态存在，但 `control_allowed` 几乎为 0。

因此引入默认关闭的 reacquire-safe window：

```text
probe_burst_manager_reacquire_safe_window_enabled = False by default
```

显式打开后，它只在以下条件同时满足时放行：

1. 任务处于 `TRACK_ACTIVE`。
2. manager 已经处于 `BURST_COLLECT_EVIDENCE` 或 `RECOVER_ROUTE`。
3. 已有冻结的 entry XT。
4. entry XT 不超过配置门限，例如 `80m`。

直觉上，这不是取消 deployment reacquire，而是在“已经批准的一次 bounded probe/recovery”内部，给控制器一个短时可执行窗口。

## 8. 磁证据与候选选择

zig-zag 的感知收益主要体现在多帧证据上，而不是单帧磁强度。

当前系统关心的证据包括：

| 证据 | 含义 |
| --- | --- |
| crossing event | 横切电缆附近的过线证据 |
| peak cross-track | 磁峰对应的横向位置 |
| shadow axis candidate | 不直接闭环的候选轴线 |
| progress-aligned candidate | 与任务前进方向一致的候选 |
| route-bound progress proxy | 用路线进度约束候选选择 |

为什么要有 shadow 模式？

- 早期不能直接让磁候选接管控制。
- 先旁路记录候选，比较它与任务前进方向是否一致。
- 只有候选选择和 route 推进都可信，才考虑进入控制消费。

这为论文提供了一个比较稳的叙述：

> 系统采用 shadow candidate evaluation 将感知假设与控制执行解耦，避免未验证的磁候选直接破坏任务闭环。

## 9. 控制消费机制

控制器最终只消费两个 manager 输出：

| 输出 | 含义 |
| --- | --- |
| `burst_active` | 当前帧执行激进 probe heading |
| `recovery_active` | 当前帧执行 route recovery heading |

两者都受 `control_allowed` 约束。也就是说：

```text
state == BURST_COLLECT_EVIDENCE
```

并不等价于：

```text
burst_active == True
```

这个区分很重要。论文中可以把它解释为“状态许可”和“执行许可”的分离：

- 状态许可：系统认为一次 probe/recovery 在逻辑上成立。
- 执行许可：当前帧环境允许把该逻辑变成 heading override。

这种分离避免了安全门、deployment reacquire、任务状态之间互相覆盖时产生不可解释行为。

## 10. 当前诊断指标

为了让论文实验可复现，建议报告以下指标：

| 指标 | 建议用途 |
| --- | --- |
| `route_completion_ratio` | 路线推进比例 |
| `stop_reason` | endpoint / duration 区分是否真正完成 |
| `track_active_fraction` | TRACK 阶段占比 |
| `track_mean_cross_track_m` | TRACK 段贴线能力 |
| `track_mean_vehicle_heading_error_deg` | 车辆实际航向跟踪质量 |
| `probe_burst_manager_active_fraction` | manager 参与程度 |
| `probe_burst_manager_control_allowed_fraction` | manager 真实执行窗口 |
| `probe_burst_manager_reacquire_safe_control_allowed_fraction` | safe-window 放行比例 |
| `probe_burst_manager_transition_count` | 状态转换次数 |
| `probe_burst_manager_recovery_timeout_count` | recovery 失败/超时风险 |
| `probe_burst_manager_mean_entry_abs_cross_track_m` | probe 启动时横偏分布 |

建议不要只报告全程 fused heading error。对于曲线路径和 dropout 场景，任务级指标更关键：

- 是否到 endpoint。
- 是否在 TRACK 段贴线。
- 是否靠异常大的横偏换取 route 投影。
- 是否有可解释的 probe/recovery 序列。

## 11. 当前实验结论

论文中可以把当前结论组织为三组。

### 11.1 基本盘不回归

`case1v-6v` 的 zig-zag 开关 A/B 表明：

- 小幅 zig-zag 不破坏常规场景。
- manager 默认关闭时完全 inactive。
- `case1v-5v` 保持 `<15deg` mean error。
- `case6v` 仍按强弯边界解释。

### 11.2 直接增大激励不可取

直接增大 zig-zag 或使用纯 decoupled lateral control 会增加磁证据，但破坏任务推进。因此系统必须引入 bounded state machine。

### 11.3 受控 probe 可解决 dropout 代表点

在 `case_maze_sonar_dropout` 代表点中：

- `20m/60m` safe-window 太保守，无法有效放行。
- `80m` entry/safe 门限在 entry XT 冻结后达到 `99.5% endpoint`。
- `100m` 会引入 recovery timeout 和退化风险。

这说明当前策略不是“越宽越好”，而是存在一个与任务几何和 recovery 能力相关的有效窗口。

## 12. 可写进论文的方法段落

下面是一段可作为论文方法描述的草稿：

```text
To improve magnetic observability during cable tracking, we introduce a
bounded zig-zag probing mechanism. Instead of continuously increasing the
tracking oscillation amplitude, the controller schedules short probe bursts
through a finite-state manager. The manager remains idle during nominal
tracking and only enters a burst after sufficient healthy TRACK duration,
forward route progress, and an entry cross-track bound are satisfied. During
the burst, lateral excitation is temporarily increased to collect magnetic
crossing evidence. The controller then enters a recovery phase that prioritizes
route progress before cooling down.

A key implementation detail is the separation between logical state and
control permission. The manager may enter a pending burst or recovery state,
but heading override is emitted only when control_allowed is true. In dropout
deployment scenarios, a bounded reacquire-safe window can temporarily allow the
approved probe/recovery action to execute despite a reacquire request. The
cross-track condition used by this window is frozen at burst entry, ensuring
that the recovery phase is not disabled by transient cross-track excursions
created by the probe itself.
```

中文对应说明：

```text
为提高磁观测可见性，系统引入受限 zig-zag 探测机制。控制器不是持续增大横向振荡幅度，而是通过有限状态机安排短时探测脉冲。状态机在正常跟踪时保持 idle，只有当 TRACK 健康时间、前向 route 进度和进入横偏门限同时满足时才进入 burst。burst 阶段临时增加横向激励以采集磁 crossing 证据，随后进入 route recovery 阶段恢复任务推进，并在 cooldown 后才允许下一次探测。

关键实现是将逻辑状态与控制执行许可分离。状态机可以进入 pending burst/recovery，但只有 control_allowed 为真时才输出 heading override。在 dropout 部署场景中，默认关闭的 reacquire-safe window 可以在有界条件下临时放行一次已批准的 probe/recovery。该窗口使用进入 burst 时冻结的横偏，而不是当前帧横偏，从而避免 recovery 被 probe 造成的瞬时横偏反复关闭。
```

## 13. 局限性

当前实现仍有边界：

1. ProbeBurstManager 默认关闭，尚不是全局默认控制策略。
2. `80m` 是当前代表点有效门限，不应直接声称适用于所有海缆几何。
3. 论文中需要说明 `case6v` 的强弯 fused heading error 仍未完全收口。
4. dropout endpoint 成功依赖 bounded safe-window 和 entry XT 冻结，不能简化为“打开 zig-zag 即成功”。
5. 埋深反演仍应绑定 crossing / cycle 质量，不能用低 coverage 后验直接闭环。

## 14. 后续论文实验建议

建议补充三类实验：

| 实验 | 目的 |
| --- | --- |
| zig-zag off/on A/B | 证明小幅 probe 不破坏基本盘 |
| p36 vs pure decoupled vs ProbeBurstManager | 证明状态机优于简单加大激励 |
| safe-window XT sweep | 说明 80m 有界窗口的必要性，展示 20/60 太保守、100 太宽 |

图表建议：

- 状态机时间轴：`IDLE -> BURST -> RECOVER -> COOLDOWN`。
- route completion 与 stop reason 对比。
- manager active / control_allowed / safe_allowed 时间序列。
- entry XT 分布。
- TRACK 横偏与车辆航向误差箱线图。

这样论文叙述会更清楚：

> zig-zag 的价值不是让轨迹看起来更复杂，而是作为一种受控主动感知激励，在状态机约束下提高磁观测可见性，同时保留任务推进稳定性。
