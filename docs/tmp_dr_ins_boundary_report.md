# DR/INS 慢漂与稀疏声呐鲁棒边界临时报告

本报告记录一次单独的鲁棒边界测试，目标是把“DR/INS 慢漂强度”和“sparse sonar 锚点稀疏程度”从默认 maze 回归中拆出来，明确当前系统的可用边界。

## 测试对象

基础场景：

- `case_maze_sparse_sonar_dr_ins_prior_distortion`
- `case_maze_sonar_dr_ins_prior_distortion`
- `case_maze_sonar_dropout_dr_ins_prior_distortion`

共同先验误差：

- route prior 平移：`(0.0, 7.5m)`
- route prior 旋转：`3.0°`
- route prior 形变：`nominal_route_prior_scale_xy = (0.99, 1.0)`
- prior correction gain：`0.01`

验收口径：

- `endpoint = yes`
- `maze_geometry_passed = yes`
- `route_progress_large_jump_count = 0`

## DR/INS 漂移档位

| profile | position white noise | position random walk | heading white noise | heading random walk |
| --- | ---: | ---: | ---: | ---: |
| `mild` | `0.15m` | `0.003m/sqrt(s)` | `0.15°` | `0.001°/sqrt(s)` |
| `mid` | `0.20m` | `0.005m/sqrt(s)` | `0.20°` | `0.001°/sqrt(s)` |
| `strong` | `0.35m` | `0.018m/sqrt(s)` | `0.35°` | `0.003°/sqrt(s)` |

## Critical Sweep 结果

命令：

```bash
python tools/dr_ins_boundary_sweep.py --critical --output results/20260628_dr_ins_boundary/critical_sweep.csv
```

CSV：

```text
results/20260628_dr_ins_boundary/critical_sweep.csv
```

| case | drift | sonar prob | health | route | endpoint | geometry | max jump | large jumps | TRACK XT | final XT | pass |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| sparse + DR/INS + distorted prior | mild | `0.15` | `38.0` | `62.7%` | no | no | `74.1m` | `2` | `14.6m` | `104.2m` | no |
| sparse + DR/INS + distorted prior | mild | `0.20` | `80.0` | `99.5%` | yes | yes | `0.5m` | `0` | `4.8m` | `2.0m` | yes |
| sparse + DR/INS + distorted prior | mid | `0.20` | `60.8` | `99.5%` | yes | no | `236.9m` | `3` | `12.5m` | `1.9m` | no |
| sonar + DR/INS + distorted prior | strong | n/a | `74.1` | `99.5%` | yes | yes | `2.9m` | `0` | `6.8m` | `0.5m` | yes |
| dropout-prior + DR/INS + distorted prior | strong | n/a | `78.7` | `99.5%` | yes | yes | `0.2m` | `0` | `2.6m` | `3.0m` | yes |

## 边界结论

1. 连续 sonar 与 dropout-prior 对 DR/INS 慢漂较稳健：在 `strong` 漂移档位下仍能 endpoint 且 no-shortcut。
2. sparse sonar 的鲁棒边界明显更窄：在 `mild` 漂移下，`prob_detection=0.20` 是当前通过点；`prob_detection=0.15` 已失败。
3. sparse sonar 的失败不一定表现为 endpoint 失败。`mid + prob=0.20` 能到 `route 99.5%`，但 `max_jump=236.9m`、`large_jumps=3`，属于几何失败。
4. 当前关键限制不是 route prior 形变本身，而是“稀疏锚点 + DR/INS 漂移 + 迷宫回折投影”的组合风险。

## 后续建议

- 把 sparse sonar 的可接受下限暂定为：`mild` DR/INS 漂移、`prob_detection >= 0.20`。
- 若要支持 `mid` 漂移或 `prob_detection <= 0.15`，需要新增机制，而不是继续调大 correction gain：
  - lane-aware projection guard；
  - route progress continuity gate；
  - sparse anchor confidence memory；
  - 或将 magnetic path 的局部拓扑约束纳入 route projection。

## 估计器调优复测

本轮按上述失败点调整 controller-side prior estimator：

- 对 nominal-route 投影增加进度连续性保护，只在上一帧 progress 附近窗口内匹配候选 route segment。
- 对 prior correction 增加 residual gate、单步平移修正上限和 heading error gate。
- 将 progress guard 只打开在 sparse / dropout DR/INS 场景；连续 sonar 场景保持全局投影，因为连续声呐观测已经足够强，额外进度约束会限制正常纠偏。
- 边界扫描工具新增 `--critical-index`，用于单点复测当前失败锚点。

复测命令：

```bash
python tools/dr_ins_boundary_sweep.py --critical \
  --output results/20260628_dr_ins_boundary/critical_sweep_after_estimator_guard.csv
```

复测 CSV：

```text
results/20260628_dr_ins_boundary/critical_sweep_after_estimator_guard.csv
```

| case | drift | sonar prob | health | route | endpoint | geometry | max jump | large jumps | TRACK XT | final XT | pass |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| sparse + DR/INS + distorted prior | mild | `0.15` | `75.5` | `99.5%` | yes | no | `60.6m` | `2` | `6.6m` | `1.5m` | no |
| sparse + DR/INS + distorted prior | mild | `0.20` | `83.2` | `99.5%` | yes | yes | `0.5m` | `0` | `3.8m` | `3.0m` | yes |
| sparse + DR/INS + distorted prior | mid | `0.20` | `88.5` | `99.5%` | yes | yes | `0.4m` | `0` | `3.1m` | `0.5m` | yes |
| sonar + DR/INS + distorted prior | strong | n/a | `71.6` | `99.5%` | yes | yes | `2.9m` | `0` | `7.0m` | `3.1m` | yes |
| dropout-prior + DR/INS + distorted prior | strong | n/a | `79.3` | `99.5%` | yes | yes | `0.3m` | `0` | `2.5m` | `1.9m` | yes |

更新结论：

1. `sparse + mid DR/INS + prob=0.20` 已从几何失败恢复为通过，`max_jump` 从 `236.9m` 降到 `0.4m`。
2. `sparse + mild DR/INS + prob=0.15` 仍失败，但失败形态从“推进不足”变成“endpoint 到达但仍有 2 次几何跳变”。这说明 `0.15` 已低于当前锚点可观测性边界，不能仅靠调 estimator 解决。
3. 当前可用边界可上调为：sparse sonar 在 `prob_detection >= 0.20` 时可承受 `mid` DR/INS 慢漂；`prob_detection=0.15` 仍应标记为不可用边界。
