# 三次浅角横切：原生 PVS 航迹包络诊断

这是图 45 的 90° 折线横切失败后的**隔离方法候选**。它沿局部先验布置三段连续余弦半波，在不要求急转掉头的前提下得到三个空间分离的磁剖面。现有 ROS 巡检入口、任务状态机、FWHM 深度发布门、PC104 和历史结果均未切换到这条航迹。图 46 的 22 m／125 m／1 m 配方保留为调参前证据；其声明的 1 m 导航余量小于零流 PVS 最大约 2.54 m 参考偏差，因此不作为收口配方。

![图 47：原生 PVS 中三次浅角横切的航迹、横偏、磁剖面与几何许可](publication/fwhm_v2_experiment_results/figures/47_shallow_zigzag_pvs_envelope_3m.png)

![图 48：覆盖预算、最小转弯半径、走廊净距的交集以及正确起始侧的 PVS 横偏](publication/fwhm_v2_experiment_results/figures/48_shallow_zigzag_geometry_budget_insight.png)

## 可核查的几何与动力学边界

候选配方为横向振幅 24 m、每半波沿线 135 m、三段合计 405 m、名义速度 0.3 m/s；走廊为 $x\in[-10,415]$ m、$y\in[-42,42]$ m，参考航迹至少留出 5 m 边界净距。余弦曲线的解析最小曲率半径 $L^2/(A\pi^2)=76.94$ m，高于设定的 50 m **候选**转弯半径。振幅覆盖预算为起始横误差 6 m、先验切线误差 $1.15^\circ$ 在 405 m 上引起的 8.13 m 偏差、目标半高交点 5.2 m、尾部 1.5 m、导航参考横偏 3 m，总计 23.83 m；名义余量仅 0.17 m。输入超出任一声明界限就应拒绝这份配方，不能靠固定配置硬跑。这个几何不等式只是必要条件，车辆转弯、停止、障碍、底高和真实定位误差都未由它证明。

图 48 左侧把三个静态条件写成振幅--波长可行域：$A\geq 6+3L\tan1.15^\circ+5.2+1.5+3$、$A\leq L^2/(50\pi^2)$ 和 $A\leq42-5$。旧 22 m／125 m 点若使用新声明的 3 m 导航预算便不满足覆盖条件；这只是合同自检，并非控制效果的单变量消融。右侧为当前配方三个正确侧案例的原生 PVS 最大参考横偏；错侧案例超出横轴范围，说明起始侧选择本身也是准入条件。[图 48 原始汇总及哈希](publication/fwhm_v2_experiment_results/data/48_shallow_zigzag_geometry_budget_insight_manifest.json)可独立核对。

以原生 PVS REMUS 100、PID 航速反馈、72 字节下行报文实际编解码、合成磁源和噪声种子 101 运行九个独立案例。下表中的“到端”只是**搜索航迹走到末端**；“许可”只是根据三次实际导航位置与磁剖面恢复局部测线，不是 Track、目标身份或巡检完成。横偏统计为车辆相对于**参考扫描轨迹**的横向误差，不能当作对真海缆的跟踪误差。

| 案例 | 搜索航迹 | 最大参考横偏 | 测线许可 | 许可中心相对目标线 MAE |
| --- | --- | ---: | --- | ---: |
| 零流单源 | 到端，1454 s | 1.435 m | 是 | 0.009 m |
| $+0.05$ m/s 横流，负侧起步 | 到端，1426 s | 1.497 m | 是 | 0.022 m |
| $-0.05$ m/s 横流，正侧起步 | 到端，1426 s | 1.498 m | 是 | 0.012 m |
| $-0.05$ m/s 横流，错误负侧起步 | 边界余量耗尽，874 s | 30.494 m | 否 | — |
| 中间场形，零流 | 到端 | 1.435 m | 是 | 0.007 m |
| 无源／单站暂态／相距 7 m 邻源 | 各到端 | 各 1.435 m | 均否 | — |
| 相距 1 m 融合邻源 | 到端 | 1.435 m | **是** | **0.505 m** |

前三个正确起始侧的动力学案例中，最大参考横偏小于声明的 3 m；错侧例超过余量并在走廊安全线处拒止。`select_shallow_start_side` 只在外部提供有界横流向量及其不确定度时选择上游侧；本次 PVS 探针用命令行显式指定起始侧和仿真流向，**没有**验证真实流测量、自动选边、实时航迹准入或回收。航迹到端案例也没有在在线循环中把估计测线接入 Track。

几何门能拒绝无源、单站暂态与可分邻源，但 1 m 邻源融合成单峰仍获几何许可，故 **`geometry_valid` 不得解释成目标身份**。无声呐分支只能在作业前独立核实走廊及传感器作用范围内相关磁源唯一时考虑目标身份；若可能有近距并行缆，就需要有时效、经标定的声呐或测绘身份线索。所有九例 `burial_depth_m=null`；场形不同下中心准确不意味着 FWHM 埋深已校准。PVS 与实艇响应、导航协方差、侧滑和制动包络均需独立验收。

## 复现与证据

原始每例的 `trajectory.csv`、`summary.json` 和探针运行日志位于 [`results/v2/shallow_zigzag_pvs_envelope_3m_20260927/`](../../results/v2/shallow_zigzag_pvs_envelope_3m_20260927/)；[图 47 的汇总](publication/fwhm_v2_experiment_results/data/47_shallow_zigzag_pvs_envelope_3m_summary.json)与[哈希清单](publication/fwhm_v2_experiment_results/data/47_shallow_zigzag_pvs_envelope_3m_manifest.json)记录逐例轨迹及源码版本。此处的 9 是**九种案例各一个噪声种子**，不据此估计统计成功率。

```bash
PYTHONPATH=AUV-Master-Mag/src python3 -m pytest -q AUV-Master-Mag/tests/v2/test_corridor_survey.py
python3 tools/probe_v2_shallow_zigzag_pvs.py --case wire --seed 101 --output-dir /tmp/shallow_zigzag_zero
python3 tools/probe_v2_shallow_zigzag_pvs.py --case wire --seed 101 --current-speed-mps 0.05 --current-direction-deg 270 --current-compensation --start-side 1 --output-dir /tmp/shallow_zigzag_reverse
python3 tools/plot_v2_shallow_zigzag_pvs_envelope.py
```

下一步如要升级为运行入口，需让带时间戳的横流估计和先验质量共同决定起始侧与走廊准入；逐站在线记录 `scan_id`、半高左右交点和失败原因；将测线与**外部身份许可**、执行许可分层接到 ROS Track，并在同一 ROS--PID--守卫--协议--PVS 运行中验证越界前停止、无源 Hold、横流两向和身份撤权。未完成这些接口前，该候选只能引用为动力学约束下的主动磁测线方法验证。

进一步发现扫描出口侧影响后续 Track 接入：三段扫描对反向横流虽能取得测线，却以约 37.7 m 横距结束，按 10° 方向修正估算需约 214 m 前进距离。改为正确上游侧的四段扫描后，该横距降至约 9.87 m，几何接入估算约 56 m；对正向横流反而三段更合适。见[图 49 与独立奇偶段数对照](INSPECTION_SHALLOW_ZIGZAG_EXIT_PARITY_20260927.md)。这些数字仍不是已经执行 Track 的证明，且四段案例若再巡检 80 m，局部测线约需向前外推 176 m，必须约束再观测时效与目标曲率。
