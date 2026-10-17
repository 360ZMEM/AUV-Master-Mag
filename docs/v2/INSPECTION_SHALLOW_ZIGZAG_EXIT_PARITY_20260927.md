# 三次还是四次横切：出口侧与后续接入几何

[图 47 的原生 PVS 诊断](INSPECTION_SHALLOW_ZIGZAG_PVS_ENVELOPE_20260927.md)已证明，在有限合成走廊中三次连续浅角横切可以采到有效局部测线；但扫描航迹到端与沿缆巡检之间还隔着**从最后一侧接入目标线**。在反向横流、正确上游侧起步的三段配方中，艇到端时位于目标线另一侧约 37.7 m。若后续沿测得目标线方向最多修正 $10^\circ$，仅按直线几何便约需 $37.7/\tan10^\circ=214$ m 沿线距离；这还没有计入转向响应、边界与停止预算。因此直接把三段扫描输出接到 80 m Track 不能当作已解决任务。

![图 49：相同横流下改变半波段数，扫描出口侧与名义接入距离随之改变](publication/fwhm_v2_experiment_results/figures/49_shallow_zigzag_exit_parity.png)

## 隔离 PVS 对照

对每个流向先选上游起始侧，再使扫描终点落在预期目标侧附近：正向 $0.05$ m/s 横流从负侧起步，用**奇数三段**到正侧；反向 $0.05$ m/s 横流从正侧起步，用**偶数四段**回正侧。四段配方改为振幅 27 m、半波长仍 135 m、扫描沿程 540 m；起始误差 6 m、航向误差 $1.15^\circ$、半高交点 5.2 m、尾部 1.5 m、导航横偏预算 3 m 合计需约 26.54 m，名义覆盖余量约 0.46 m；解析最小曲率半径约 68.39 m，仍高于设定的 50 m 候选值。走廊横向半宽增至 45 m，保留 5 m 参考边界净距。

以下四例均使用原生 PVS、PID、72 字节协议编解码和同一个合成噪声种子 101；海流 $+y/-y$ 对应脚本方向 90°/270°。所有案例都走完**搜索航迹**、三站或四站测线几何许可有效、最大参考扫描横偏低于声明的 3 m，埋深保持空值；这不是 Track 完成率。

| 横流与段数 | 起始侧 | 末端到目标线真值的横距 | 按 $10^\circ$ 相对线向修正估算的接入距离 | 测线中心 MAE |
| --- | --- | ---: | ---: | ---: |
| $+0.05$ m/s，三段 | 负侧 | 9.499 m | 53.9 m | 0.022 m |
| $-0.05$ m/s，三段 | 正侧 | 37.700 m | 213.8 m | 0.012 m |
| $-0.05$ m/s，四段 | 正侧 | 9.872 m | 56.0 m | 0.016 m |
| $+0.05$ m/s，四段 | 负侧 | 43.483 m | 246.6 m | 0.012 m |

这说明“多扫一次”本身并不改善后续任务：只有把**上游起始侧、横切段数的奇偶性、目标侧向区间和末端接入走廊**联合准入，才会缩短进入 Track 的距离。表中接入距离只是采用目标真值末端横距与名义 $10^\circ$ 修正角计算的几何估算，不是控制器已经走过的距离、实艇最短距离或现场可用性证明。实际系统只能从独立先验区间和已许可测线估计该距离，不能读取目标真值。反向横流四段案例的最后一个实际磁中心约在沿线 500 m；若从扫描终点 540 m 再接入约 56 m 并巡检 80 m，测得局部直线将被外推约 176 m；真实目标曲率、再观测时效与走廊纵向长度均需要另外约束。本试验仍只到扫描末端，**尚未执行接入或 Track**。

若先验不能给出目标位于哪侧的有界区间，即使流向已知，也不能仅凭“上游”决定扫描段数。可以继续完成磁测线调查并报告未具名异常；目标身份仍由经审查的单源走廊、实测声呐或窄测绘线索决定。近距融合邻源的反例未随增加站数自动消失。四段配方的 0.46 m 解析余量仍很小，当前 PVS 单种子结果不能冻结现场阈值。PC104、ROS 任务接入、真实流估计、磁传感器与实艇机动／停止包络均未覆盖。

原始五例（另含零流、正侧四段）位于 [`results/v2/shallow_zigzag_exit_parity_20260927/`](../../results/v2/shallow_zigzag_exit_parity_20260927/)；[图 49 汇总](publication/fwhm_v2_experiment_results/data/49_shallow_zigzag_exit_parity_summary.json)和[图像与源码哈希](publication/fwhm_v2_experiment_results/data/49_shallow_zigzag_exit_parity_manifest.json)可核对同版配方。复现命令示例：

```bash
python3 tools/probe_v2_shallow_zigzag_exit_pvs.py --seed 101 --case wire --station-count 4 --amplitude-m 27 --current-speed-mps 0.05 --current-direction-deg 270 --current-compensation --start-side 1 --output-dir /tmp/shallow_exit_reverse_four
python3 tools/probe_v2_shallow_zigzag_exit_pvs.py --seed 101 --case wire --station-count 3 --amplitude-m 24 --current-speed-mps 0.05 --current-direction-deg 270 --current-compensation --start-side 1 --output-dir /tmp/shallow_exit_reverse_three
python3 tools/plot_v2_shallow_zigzag_exit_parity.py
```
