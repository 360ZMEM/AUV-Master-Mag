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

