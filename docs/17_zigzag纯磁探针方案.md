# zig-zag 纯磁探针方案

## 1. 目标

本阶段把 zig-zag 从“搜索动作”提升为纯磁估计的主动探针：

- 小幅度，不破坏常规易控场景。
- 曲率远小于控制上限，不能因为探针导致车辆难以转弯。
- 足够产生横向激励，让磁场梯度、`B_down/B_perp` 横偏和历史多帧拟合有可观测信息。
- 为电缆位置、航向轴线和埋深估计服务，而不是为了让 AUV 大幅扫线。

当前落地策略是“显式 probe 模式”，不直接替换默认基线：

```bash
/Users/bytedance/miniconda3/bin/python tools/visualize.py --all --zigzag-probe
/Users/bytedance/miniconda3/bin/python tools/visualize.py --variants --zigzag-probe
```

## 2. 参数设计

当前探针参数：

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `track_active_zigzag_angle_deg` | 至少 `3deg` | TRACK 阶段保留小幅跨线激励。 |
| `curve_track_crossing_angle_deg` | 至少 `3deg` | 曲线局部跟踪时仍保留探针激励。 |
| `magnetic_path_observation_enabled` | 仅有名义路线先验时自动开启 | 纯磁隐式路径观测作为诊断通道，不接管无先验 maze。 |
| `magnetic_path_max_cross_track_m` | `25m` | 防止远离电缆时把错误磁矢量投影成电缆点。 |

为什么先用 `3deg`：

- 对 `1m/s` 航速和当前 yaw-rate/转弯半径限制而言，3° 只产生很小的横向激励。
- 对 case1-6 和 case1v-6v，实测未破坏 TRACK 稳定性。
- 它能给纯磁观测制造横向变化，但不会像 SEARCH zig-zag 那样主导控制。

## 3. 纯磁估计的可观测性结论

新增独立测试：

```bash
/Users/bytedance/miniconda3/bin/python -m unittest tests.test_magnetic_turn_observability
```

结论：

- 纯磁水平矢量能提供电缆轴线，但单帧天然有 `180deg` 方向歧义。
- 必须结合历史多帧和运动方向，才能把轴线变成连续航向。
- 必须有横向激励，`B_down/B_perp` 才能稳定反演横偏，并把 AUV 位置投影成隐式电缆点。
- 无横向激励时，横偏接近 0，历史点几何张角不足，不能可靠判断转弯。

因此，纯磁转弯估计必须和 zig-zag/跨线运动绑定。它不能作为静态单帧转弯传感器。

## 4. 当前可视化/报告指标

报告中新增的探针指标：

| 指标 | 解释 |
| --- | --- |
| `Magnetic path observation fraction` | 有多少帧生成了纯磁隐式路径观测。 |
| `Magnetic path axis error` | 纯磁轴线相对真值电缆切向的轴线误差，消除了 180° 符号歧义。 |
| `Magnetic path position error` | 纯磁投影电缆点到真值最近点的距离。 |
| `Magnetic path mean abs offset` | 纯磁反演横偏的平均绝对值。 |
| `Burial inversion coverage` | 埋深反演输出覆盖率。 |
| `Burial inversion MAE` | 有埋深估计帧上的 MAE。 |

CLI 也会输出探针摘要：

```text
mag_probe 90%  axis_err 18.1deg  pos_err 4.3m
```

## 5. case1-6 探针回归

命令：

```bash
/Users/bytedance/miniconda3/bin/python tools/visualize.py --all --zigzag-probe --outdir results/latest_zigzag_probe_all
```

结果：

| case | health | fused err | TRACK XT | TRACK vehicle err | mag probe | axis err | pos err | 结论 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| case1 | 95 | 2.3° | 1.9m | 3.2° | 90% | 18.1° | 4.3m | 稳定。 |
| case2 | 94 | 3.2° | 2.2m | 3.3° | 37% | 22.8° | 4.3m | 稳定。 |
| case3 | 97 | 0.5° | 1.3m | 4.1° | 68% | 45.4° | 7.5m | 控制稳定，纯磁角度受噪声影响。 |
| case4 | 99 | 0.5° | 0.3m | 3.2° | 57% | 54.5° | 1.7m | 控制稳定，姿态扰动下轴线角度不可靠。 |
| case5 | 96 | 10.0° | 0.3m | 3.0° | 66% | 15.5° | 2.0m | 稳定。 |
| case6 | 84 | 27.6° | 2.2m | 5.1° | 37% | 36.0° | 4.4m | 比旧 case6 健康度更贴近实际跟踪。 |

结论：小幅 probe 不破坏 case1-6 控制稳定性；纯磁位置估计多数场景可用，但轴线角度在高噪声/姿态扰动下仍需更强门控。

## 6. case1v-6v 探针回归

命令：

```bash
/Users/bytedance/miniconda3/bin/python tools/visualize.py --variants --zigzag-probe --outdir results/latest_zigzag_probe_variants
```

结果：

| case | health | fused err | TRACK XT | TRACK vehicle err | mag probe | axis err | pos err | 结论 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| case1v | 94 | 10.7° | 1.5m | 3.4° | 93% | 18.5° | 4.1m | 通过。 |
| case2v | 94 | 8.0° | 1.9m | 3.3° | 44% | 17.0° | 3.2m | 通过。 |
| case3v | 98 | 3.1° | 0.7m | 4.0° | 90% | 42.4° | 5.9m | 控制通过，纯磁角度需门控。 |
| case4v | 97 | 6.9° | 0.3m | 3.2° | 74% | 52.1° | 1.7m | 控制通过，姿态扰动影响轴线角。 |
| case5v | 95 | 10.5° | 0.3m | 3.0° | 70% | 14.6° | 2.1m | 通过。 |
| case6v | 86 | 19.9° | 2.8m | 5.8° | 51% | 31.8° | 4.9m | 强弯边界仍可稳定跟踪。 |

结论：对于已通过的 case1v-5v，小幅 zig-zag 探针没有破坏稳定性；case6v 仍是曲率表达边界，但车辆闭环表现可接受。

## 7. maze 负例

额外验证：

```bash
/Users/bytedance/miniconda3/bin/python tools/visualize.py --case case_maze_sonar --zigzag-probe
```

如果在无先验 maze 中直接打开纯磁隐式路径观测，会把 `case_maze_sonar` 从 endpoint 退化到早期失败。修正后，`--zigzag-probe` 在无名义路线先验时只保留小幅 zig-zag，不自动打开纯磁观测：

| 场景 | 结果 |
| --- | --- |
| `case_maze_sonar --zigzag-probe` | `route=99.5%`、`stop=endpoint` |

结论：纯磁观测当前是有参考几何时的探针诊断，不是无先验 maze 的替代定位源。maze 仍需要声呐锚点或更强的 zig-zag 相位绑定。

## 8. 后续路线

1. 保持 `--zigzag-probe` 作为所有常规场景的标准验证入口。
2. 对纯磁轴线角误差增加门控：
   - 姿态扰动门控。
   - `vector_consistency` 门控。
   - 磁横偏幅度门控。
3. 把纯磁隐式位置用于埋深反演的 lateral offset 备选源，但必须先满足 position error 代表点指标。
4. 在无先验/maze 中启用纯磁观测前，必须先显式建模 TRACK zig-zag 的跨线相位；否则会再次污染 local path。
5. 如果探针模式在更多场景稳定，再评估是否把 `3deg` TRACK probe 升级为默认基线。


## 9. case_maze_sonar_dropout 探针调优

本节专门验证 `case_maze_sonar_dropout`：声呐只负责初始捕获，TRACK 后强制离线。目标是确认 zig-zag 是否真正提升纯磁电缆位置估计和预瞄，而不仅是改善局部角度指标。

评估入口：

```bash
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe
```

门控候选因为完整时长较慢，单独短测：

```bash
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p9_probe10_mag_gate20 --max-steps 12000
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p10_probe10_mag_gate10 --max-steps 12000
```

### 9.1 性能列表

| variant | health | TRACK XT | TRACK vehicle err | route | final dist | mag probe | mag axis err | mag pos err | burial MAE | 结论 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `d0_baseline` | 27.4 | 24.3m | 89.7° | 1.5% | 21.3m | 0% | — | — | 0.490m | dropout 基线失败。 |
| `p1_probe3_nomag` | 14.9 | 21.7m | 89.2° | 2.9% | 16.9m | 0% | — | — | 0.311m | 仅 3° zig-zag 无有效推进。 |
| `p2_probe6_nomag` | 31.0 | 23.0m | 89.8° | 1.6% | 25.8m | 0% | — | — | 0.340m | 仅 6° zig-zag 无有效推进。 |
| `p3_probe10_nomag` | 27.4 | 24.3m | 89.7° | 1.5% | 21.3m | 0% | — | — | 0.490m | 仅 10° zig-zag 等同基线。 |
| `p4_probe3_mag` | 33.7 | 9.3m | 14.3° | 17.2% | 9.4m | 35.8% | 45.3° | 79.0m | 228.363m | 有局部收益，但纯磁位置严重污染。 |
| `p5_probe6_mag` | 29.1 | 9.5m | 14.0° | 16.9% | 694.1m | 26.7% | 41.3° | 163.5m | 305.866m | 位置/埋深发散，不可用。 |
| `p6_probe10_mag` | 41.3 | 7.0m | 16.0° | 60.4% | 73.8m | 35.9% | 37.3° | 120.5m | 3.893m | route 表面提升，但预瞄位置不可接受。 |
| `p6d_probe10_mag_diag` | 27.4 | 24.3m | 89.7° | 1.5% | 21.3m | 96.0% | 49.0° | 19.4m | 0.490m | 只诊断不接入：估计可观测，但不产生控制收益。 |
| `p7_probe6_mag_age180` | 29.1 | 9.5m | 14.0° | 16.9% | 694.1m | 26.7% | 41.3° | 163.5m | 305.866m | 延长 local path 记忆无改善。 |
| `p8_probe10_mag_age180` | 41.3 | 7.0m | 16.0° | 60.4% | 73.8m | 35.9% | 37.3° | 120.5m | 3.893m | 与 p6 相同，仍不可靠。 |
| `p9_probe10_mag_gate20` | 29.3 | 14.9m | 21.5° | 15.5% | 11.6m | 83.3% | 33.9° | 18.3m | 0.733m | 门控降低位置误差，但推进不足。 |
| `p10_probe10_mag_gate10` | 26.5 | 7.3m | 21.8° | 0.0% | 33.1m | 80.5% | 28.0° | 8.3m | 0.713m | 位置估计最好，但过度门控后不前进。 |

### 9.2 结论

1. 仅增加 TRACK zig-zag 幅度无法解决 dropout，`3°/6°/10°` 都没有真正推进。
2. `10° + 纯磁接入` 能把 route 推到 `60.4%`，但纯磁位置误差达到 `120.5m`，这不是可靠预瞄，而是错误观测驱动下的投影收益。
3. 只诊断不接入时，纯磁位置误差降到 `19.4m`，说明 zig-zag 产生了可观测信息，但还不足以直接控制。
4. 加创新门控后，位置误差可降到 `8.3-18.3m`，但 route 降到 `0-15.5%`，说明当前门控尚不能同时满足“估计准”和“可推进”。
5. 本轮没有找到可合入默认 dropout 流程的有效参数，因此不应把 `case1v-6v` 的默认探针策略改为更大幅度。

当前判断：zig-zag 对纯磁估计是必要激励，但 `case_maze_sonar_dropout` 还缺一个“跨线相位 -> 可信观测 -> 预瞄目标”的中间层。下一步应先实现显式 zig-zag 相位识别，只在跨线相位完整、磁横偏符号变化成立、创新距离受控时生成 lookahead，而不是把每帧纯磁投影直接喂给 local path。

### 9.3 跨线相位识别验证

已新增 `MagneticZigzagPhaseDetector`：它不直接接受单帧纯磁投影，而是等待 zig-zag 横偏完成一次符号翻转，并要求两侧横偏幅度、相位时长、轴线稳定性满足门控后，再输出相位确认观测。该机制默认关闭，仅作为 dropout 代表点候选。

新增短测入口：

```bash
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p11_probe10_mag_phase --max-steps 12000
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p12_probe10_mag_phase_loose --name p13_probe10_mag_phase_lowoffset --max-steps 12000
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p14_probe10_mag_phase_latch --max-steps 12000
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p15_probe10_mag_lookahead --name p16_probe10_mag_lookahead_local --max-steps 12000
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p17_probe10_mag_lookahead_age180 --max-steps 12000
```

| variant | health | TRACK XT | TRACK vehicle err | route | final dist | raw mag pos err | phase pos err | phase amp | 结论 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `p11_probe10_mag_phase` | 34.7 | 7.7m | 41.2° | 2.1% | 3.6m | 11.4m | 16.1m | 6.8m | 相位事件过稀疏，推进失败。 |
| `p12_probe10_mag_phase_loose` | 26.1 | 18.0m | 88.2° | 1.7% | 23.5m | 11.8m | — | — | 放宽接入门控仍无有效相位输出。 |
| `p13_probe10_mag_phase_lowoffset` | 22.1 | 8.6m | 40.1° | 3.2% | 21.7m | 10.2m | 5.5m | 7.3m | 相位位置较准，但事件太少，不能连续预瞄。 |
| `p14_probe10_mag_phase_latch` | 26.7 | 7.7m | 41.2° | 2.0% | 33.0m | 10.5m | 8.7m | 4.5m | 相位信任窗口仍不能形成持续控制收益。 |

相位识别使纯磁位置误差进入 `5.5-16.1m` 区间，比 `p6_probe10_mag` 的 `120.5m` 明显更可信；但 route 仍停在 `1.7-3.2%`。这说明当前失败点已经从“单帧纯磁观测污染”转移为“相位事件太稀疏，未形成连续 lookahead 目标”。继续推进时应新增独立的 `magnetic lookahead target` 生成器：用相位事件校正局部电缆轴线和横偏符号，再在事件之间用运动学外推保持前视点，而不是继续依赖 `local path` 被动吃稀疏观测。

### 9.4 Magnetic lookahead target 验证

已新增 `MagneticLookaheadTargetBuilder`：相位事件更新局部电缆轴线，事件之间将车辆位置投影到该轴线，持续输出 `cable_point` 与前方 `lookahead` 诊断目标。控制侧在声呐失效后优先用 lookahead 局部线计算 TRACK 横偏，再退回旧的磁比值横偏。该机制默认关闭，仅用于 dropout 候选验证。

| variant | health | TRACK XT | TRACK vehicle err | route | final dist | lookahead coverage | lookahead axis err | lookahead pos err | lookahead age | 结论 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `p15_probe10_mag_lookahead` | 25.5 | 23.5m | 104.0° | 20.0% | 93.7m | 0.0% | — | — | — | 关闭 local path 后未形成有效相位 lookahead，route 增益不可解释为可靠预瞄。 |
| `p16_probe10_mag_lookahead_local` | 24.3 | 19.5m | 45.2° | 5.9% | 35.0m | 35.0% | 14.3° | 11.1m | 32.7s | lookahead 能进入控制链路，但推进仍很弱。 |
| `p17_probe10_mag_lookahead_age180` | 20.9 | 21.0m | 26.7° | 7.7% | 55.7m | 44.0% | 15.9° | 18.1m | 50.1s | 延长保持时间提高覆盖和 route，但位置误差退化，不能合入。 |

lookahead 使相位事件从“离散诊断”变成了可被控制器消费的连续局部线，但当前 `case_maze_sonar_dropout` 仍没有达到有效推进：最佳 route 仅 `7.7%`，且依赖更老的外推目标。下一步不应继续延长保持时间，而应提高相位事件频率或让控制律显式追踪 lookahead 前方目标，例如在 `TRACK_ACTIVE` 中加入 lookahead pure-pursuit 航向项，并用相位事件刷新目标。
