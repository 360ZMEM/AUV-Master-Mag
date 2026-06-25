# case_maze_sonar_dropout 调优计划

## 1. 当前结论

`case_maze_sonar_dropout` 用来验证一个更接近部署边界的假设：声呐只负责初始锁定，首次进入 `TRACK_ACTIVE` 后强制离线，后续依赖磁感知、局部路径记忆和任务级重捕获继续走完整 maze。

当前实测：

| 场景 | mean fused error | TRACK | route | stop | 关键诊断 |
| --- | --- | --- | --- | --- | --- |
| `case_maze_sonar` | `32.3deg` | `48%` | `99.5%` | `endpoint` | 持续声呐 + Phase D 区域重捕获可闭环。 |
| `case_maze_sonar_dropout` | `3.8deg` | `52%` | `1.5%` | `duration` | 局部角度看起来好，但全局路线几乎不前进。 |
| `case_maze_no_sonar` | `nan` | `0%` | `0.0%` | `duration` | 初始捕获仍不可用。 |

结论：dropout 不是“角度估计坏”，而是“失去持续声呐后缺少全局可观测推进”。因此不能靠单纯降低 `mean heading error` 调好，必须围绕 route progress、最终横向距离、TRACK 段横偏和重捕获区域有效性调。

## 2. 后续验收指标

每次实验至少记录这些指标：

| 指标 | 通过方向 | 原因 |
| --- | --- | --- |
| `route_completion_ratio` | 单调提升，第一阶段目标 `>20%` | 判断是否真的沿 maze 前进。 |
| `final_route_distance_m` | 越小越好 | 防止 route 投影假阳性。 |
| `track_mean_cross_track_m` | 目标 `<8m` | 判断 TRACK 段是否贴线。 |
| `track_mean_vehicle_heading_error_deg` | 目标 `<15deg` | 判断控制航向是否可用。 |
| `deployment_reacquire_required` 占比 | 先观察，不盲目压低 | dropout 下高占比说明缺观测，不一定是状态机坏。 |
| `reacquire_region_reason` 分布 | 应出现有效前向/转弯候选 | 判断 selector 是否还在给可执行区域。 |

不再把全程 `mean_heading_error_deg` 作为唯一健康指标。dropout 当前就是反例：`3.8deg` 很好，但任务失败。

## 3. 调优阶段

### Phase D0：诊断基线固化

目标：先确认失败发生在哪个环节，不改参数。

执行：

```bash
/Users/bytedance/miniconda3/bin/python tools/visualize.py --case case_maze_sonar_dropout
```

检查：

- 声呐是否在首次 TRACK 后确实 `FORCED_OFFLINE`。
- route 是否长期卡在初始段。
- `reacquire_region_confidence` 是否持续有值。
- `REACQUIRE_REGION` 是否只是把 AUV 拉回局部区域，而没有沿路线推进。

预期结论：如果 `track_mean_cross_track_m` 高、route 低，优先处理控制/重捕获目标选择；如果 TRACK 横偏低但 route 低，优先处理“局部绕圈/方向推进”。

### Phase D1：弱声呐递减，而不是一步归零

目标：找出 maze 对声呐持续观测的最低依赖程度。

代表点，不做网格：

| 方案 | 改动 | 目的 |
| --- | --- | --- |
| D1-a | `fail_after_track_delay_s=60` | 看是否需要通过 lane1 初段后再断声呐。 |
| D1-b | `fail_after_track_delay_s=180` | 看持续声呐是否主要用于早期去偏。 |
| D1-c | 不强制离线，但把 `prob_detection` 降到 `0.2` | 模拟稀疏声呐补观测。 |

判据：如果 D1-b 明显提升 route，说明早期 lane1 去偏是关键；如果 D1-c 明显优于强制离线，说明需要低频全局锚点，而非持续高质量声呐。

### Phase D2：磁/局部路径记忆增强

目标：让声呐消失后仍能沿最近可信电缆方向向前推进。

优先调：

| 参数 | 调整方向 | 风险 |
| --- | --- | --- |
| `local_path_max_age_s` | 从 `120s` 增到 `180s` 代表点 | 过久会保留错误方向。 |
| `local_path_capacity` | 从 `24` 增到 `36` 代表点 | 会变钝，可能降低转弯响应。 |
| `local_path_min_observation_spacing_m` | 从 `0` 增到 `2m` 代表点 | 减少近距离重复点污染，但可能降低更新频率。 |
| `forgetting_factor` | 从 `0.70` 降到 `0.60` 代表点 | 更偏近期观测，但噪声更敏感。 |

不建议先动：

- 大幅增加 `track_active_zigzag_angle_deg`。
- 直接打开无界 zig-zag 重捕获。
- 把 `reacquire_region_max_duration_s` 简单拉长到很大。此前 maze 已证伪“待久一点就好”。

### Phase D3：无声呐后的推进型 region selector

目标：dropout 后不只是回到局部框，而是选择“沿最近可信切向前进”的候选区域。

需要新增/调整 selector 行为：

| 候选 | 作用 |
| --- | --- |
| `local_tangent_forward_gate` | 用 local path heading 而不是声呐 heading 生成前向区域。 |
| `odometry_projected_gate` | 基于最后可信点 + AUV 航位推算，限制候选区域不能回退。 |
| `turn_memory_gate` | 如果上一段出现 U-turn 迹象，保留转弯侧候选，但降低置信度。 |

进入控制的门控：

- 需要 `local_path_confidence >= 0.25`。
- 需要 `final/当前 route_progress` 相比上一次 region 有正向增长。
- 如果连续多次 region 控制后 route 不增长，应回退到搜索，而不是继续局部循环。

### Phase D4：任务级状态机增加“无声呐推进失败”保护

目标：避免 dropout 卡在局部 TRACK/REACQUIRE 循环。

新增诊断/逻辑：

| 机制 | 判断 |
| --- | --- |
| progress watchdog | `N` 秒内 `route_progress_m` 增长小于阈值。 |
| local loop detector | AUV 位置反复回到同一区域，route 不增长。 |
| dropout mode flag | 声呐强制离线后进入更保守的局部推进模式。 |

动作：

- 首次 watchdog 触发：增大 forward gate。
- 第二次触发：切换到 bounded zig-zag，但必须绑定 last trusted anchor。
- 第三次触发：标记为不可观测失败，避免 route 假阳性。

## 4. 推荐执行顺序

1. 先跑 D0，保存 dropout 的逐帧诊断图和报告。
2. 跑 D1 三个代表点，确认到底需要“延迟断声呐”还是“稀疏声呐补锚点”。
3. 如果 D1 证明需要低频补锚点，优先实现稀疏声呐模式；这比纯磁闭环更现实。
4. 如果 D1 证明延迟断声呐足够，进入 D2 调局部路径记忆。
5. D2 仍无法提升 route 到 `>20%`，再做 D3 selector 改造。
6. D3 后必须加 D4 watchdog，防止局部指标好但全局不前进。

## 5. 当前不做的事

- 不把 `case_maze_no_sonar` 直接设为通过目标。
- 不用大规模网格搜参。
- 不以全程 `mean_heading_error_deg` 作为 dropout 主要目标。
- 不把无界 zig-zag 搜索重新打开作为第一选择。

## 6. D0-D4 执行记录

执行入口：

```bash
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase d1
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase d2
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase d3 --max-steps 12000
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase d4
```

### D0：基线诊断

| variant | health | fused err | TRACK XT | TRACK vehicle err | route | final dist | 结论 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `d0_baseline` | `27.4` | `3.8deg` | `24.3m` | `89.7deg` | `1.5%` | `21.3m` | 局部角度好但闭环失败。 |

补充诊断：

- `deployment_reacquire_required` 占比约 `97%`。
- region reason 主要是 `forward_gate`。
- route progress 最大曾到约 `108.8m`，最终回落到 `30.9m`，说明存在局部循环/回退。

### D1：声呐递减代表点

| variant | health | TRACK XT | TRACK vehicle err | route | final dist | 结论 |
| --- | --- | --- | --- | --- | --- | --- |
| `d1_delay60` | `24.7` | `13.3m` | `86.0deg` | `5.2%` | `1.6m` | 延迟 60s 断声呐不够。 |
| `d1_delay180` | `34.8` | `12.1m` | `84.0deg` | `9.4%` | `1.9m` | 延迟 180s 仍不够。 |
| `d1_sparse_sonar` | `45.3` | `4.3m` | `14.0deg` | `59.4%` | `26.3m` | 稀疏声呐显著改善，但未 endpoint。 |

结论：dropout 需要低频全局锚点或等价可观测补偿；只靠“晚一点断声呐”不能解决。

### D2：局部路径记忆代表点

| variant | health | TRACK XT | TRACK vehicle err | route | 结论 |
| --- | --- | --- | --- | --- | --- |
| `d2_local_age180` | `25.2` | `24.9m` | `89.8deg` | `1.7%` | 无改善。 |
| `d2_local_capacity36` | `31.3` | `24.4m` | `90.0deg` | `1.3%` | 无改善。 |
| `d2_spacing2m` | `27.9` | `24.7m` | `90.1deg` | `2.3%` | 轻微但不够。 |
| `d2_forgetting060` | `27.4` | `24.3m` | `89.7deg` | `1.5%` | 等同基线。 |

结论：纯调 local path 记忆无法替代声呐锚点。

### D3：推进型 region selector

已实现默认关闭的实验开关：

- `reacquire_region_progressive_forward_enabled`
- `reacquire_region_progressive_margin_m`
- selector 新候选：`local_tangent_forward_gate`

代表点 `d3_progressive_gate` 在 `12000` 步短测下：

| variant | health | TRACK XT | TRACK vehicle err | route | final dist | 结论 |
| --- | --- | --- | --- | --- | --- | --- |
| `d3_progressive_gate` | `26.5` | `18.7m` | `89.5deg` | `1.6%` | `28.4m` | 比短测基线更差，默认保持关闭。 |

结论：只让 region 沿旧切向向前滑动会发散，说明缺少真实锚点时“向前”本身不可靠。

### D4：稀疏声呐锚点强度代表点

| variant | health | TRACK XT | TRACK vehicle err | route | final dist | 结论 |
| --- | --- | --- | --- | --- | --- | --- |
| `d4_sparse035` | `44.1` | `4.1m` | `29.9deg` | `55.9%` | `4.7m` | 横偏好，但 route 不如 `0.20`。 |
| `d4_sparse050` | `41.6` | `4.1m` | `26.1deg` | `35.3%` | `5.2m` | 更频繁观测反而退化。 |

结论：不是声呐概率越高越好；当前 `prob=0.20` 是这组代表点里最有价值的方向，但仍不足以 endpoint。下一步应围绕“稀疏锚点触发何时进入/退出 region”做状态机调度，而不是继续调单个概率。

## 7. 下一步建议

1. 保留 `case_maze_sonar_dropout` 作为强制离线失败基准，不把它伪装成通过。
2. 新增一个独立目标场景 `case_maze_sparse_sonar`：低频声呐锚点 + 磁跟踪，用于现实可行方案调优。
3. 对 sparse 场景调状态机，而不是继续调声呐概率：
   - region entry/recovery streak
   - `reacquire_region_max_duration_s`
   - 稀疏声呐命中后的 trusted anchor 更新策略
4. 如果必须继续强制离线，则需要引入外部全局先验或额外传感器；现有磁 + IMU + 局部记忆不足以通过 1x maze。
