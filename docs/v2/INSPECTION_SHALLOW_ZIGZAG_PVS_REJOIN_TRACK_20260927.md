# 连续浅角横切后的前向接入与 80 m 跟踪：隔离 PVS 验证

图 47–49 先证实三／四段连续横切能形成局部磁测线，并发现**扫描出口侧**会决定后续接入距离。本补项在同一原生 PVS REMUS 100、PID 航速反馈和 72 字节下行协议编解码中，真正执行以下隔离工作流：

`三或四段连续浅角扫描 → 各站 FWHM 横向中心与跨站局部线位 → 合成目标身份区间核对 → 按测得线向前向接入 → 80 m Track 或明确拒绝`。

![图 50：正反横流的实际扫描、前向接入和沿缆跟踪，以及九类终态](publication/fwhm_v2_experiment_results/figures/50_shallow_zigzag_pvs_rejoin_track.png)

这次确实有**实际接入和跟踪航迹**，但仍是隔离 PVS 脚本，不是 ROS 调度、自治守卫、PC104、实艇或真实声呐链路。身份输入是脚本外显的**合成目标真值线** $y=6+0.02x$ 及 $\pm0.25$ m 区间，与磁剖面估计分开计算，并在接入和 Track 期间每周期作为理想持续线索核对；它只用于仿真验证“身份证据足够窄时允许／冲突时拒绝”的软件门，不能称作物理独立声呐证据，也未验证采集时间或失鲜。目标真值另用于实验指标，算法航向使用从实际磁测中心拟合的线向。没有此区间时不会接入，埋深在所有案例中均为空。

## 运行约束与结果

示例约束为沿局部先验直线、目标线初始偏移约 6 m／航向偏差约 1.15°、零或 $\pm0.05$ m/s 横流、名义 0.3 m/s、上游侧起步。正向流从负侧以 24 m 振幅扫描三段，反向流从正侧以 27 m 振幅扫描四段，每半波沿线 135 m；相应扫描参考走廊横向半宽为 42／45 m。接入引导以测得线向为基准，修正角限幅 $\pm10^\circ$，并在扫描末端之后 120 m 内要求横距降至 1 m；成功后按实际 Track 起点继续 80 m。纵向仿真边界另留接入与跟踪空间；真实障碍、导航误差、转弯与停止包络尚未量测。

| 同源种子 101 案例 | 扫描后决策 | 实际接入沿线距离 | Track 结果／真值横偏 |
| --- | --- | ---: | --- |
| 正向横流、三段单源 | 测线与合成身份许可 | 72.12 m | 完成 80 m；MAE 0.351 m，最大 1.012 m |
| 反向横流、四段单源 | 测线与合成身份许可 | 77.64 m | 完成 80 m；MAE 0.348 m，最大 1.044 m |
| 反向横流、四段中间场形 | 横向线位许可，深度不发布 | 77.79 m | 完成 80 m；MAE 0.333 m，最大 1.028 m |
| 反向横流、错误三段出口 | 测线有效，但 120 m 接入门耗尽 | — | 拒绝，0 个 Track 样本 |
| 无磁源、单站暂态、可分邻源 | 各自的测线几何门拒绝 | — | 均未进入 Track |
| 相距 1 m 的融合邻源 | 磁测线几何有效，但相对合成身份线偏差 0.513 m $>0.25$ m | — | 身份门拒绝，0 个 Track 样本 |
| 不提供身份区间 | 磁测线几何有效，但身份未知 | — | 未进入 Track |

图 49 的约 54–56 m 是\emph{忽略载体转向的几何估算}；图 50 实际接入用去约 72–78 m，差距表明不能把解析下界当作执行时长。Track 横偏统计使用整个正向 80 m 阶段的全部约 1052／1053 个 0.25 s 样本，不能把扫描和接入阶段混入。沿缆误差曲线初始约 1 m，随后逐步收敛；本次没有声称全程每一点都低于 1 m。错误出口虽然观测测线正确，仍未通过接入门，这正是防止“磁中心准确但艇接不上线”功能错配的必要处置。

另以种子 102、103 对正向三段单源、反向四段单源和反向四段融合邻源各重跑两次；加上图 50 的种子 101，单源两组各 **3/3 完成 80 m**，融合邻源 **3/3 被身份区间拒绝**。两组完成案例的 Track MAE 分别为 0.351–0.359 m 与 0.305–0.348 m。这是同一解析场形、同一流速和 PVS 控制配方下的噪声种子复验，不是海况分布下的成功率估计。场形指数 0.7 的单次案例仅验证横向巡检可继续、埋深置空，不能替代现场多芯铠装缆场形标定。

## 可引用结论与仍需解决的接口

可引用的是：在**声明的直线先验、先验误差和横流包络内，且有可信目标身份线索时**，多次浅角主动横切可用实测磁中心构造局部线位；把起始侧与横切奇偶数联合设计，可以在原生 PVS 中完成前向接入和有限 80 m 横向巡检；测线、身份、接入三个门可以各自拒绝而不误报任务完成。这是对单次浅角模式之外的探索，不改变已发布 FWHM 深度门。

当前尚未证明真实系统可按这份曲线起航。缺口包括：带时效和不确定度的横流来源、能说明**目标位于哪侧**的独立先验、外部线索真正对应哪条物理海缆、ROS 巡检状态机与执行守卫的同次接入、PC104 收帧、实艇转弯与零推力滑行、局部线向沿 Track 的曲率和再观测时效。在这些证据缺失时，现场发布只保留匿名磁异常测线，不将 `track_complete` 的仿真标签外推为实艇目标巡检完成。

[隔离实验参数合同](../../config/v2/shallow_zigzag_rejoin_pvs_candidate.yaml)、[九例与六次追加种子的原始轨迹](../../results/v2/shallow_zigzag_rejoin_pvs_20260927/)、[图 50 汇总及每例哈希](publication/fwhm_v2_experiment_results/data/50_shallow_zigzag_pvs_rejoin_track_summary.json)、[图像与源码哈希](publication/fwhm_v2_experiment_results/data/50_shallow_zigzag_pvs_rejoin_track_manifest.json)支持复核。复现示例：

```bash
python3 tools/probe_v2_shallow_zigzag_rejoin_pvs.py --seed 101 --case wire --station-count 3 --amplitude-m 24 --current-speed-mps 0.05 --current-direction-deg 90 --current-compensation --start-side -1 --identity-fixture --output-dir /tmp/shallow_rejoin_positive
python3 tools/probe_v2_shallow_zigzag_rejoin_pvs.py --seed 101 --case wire --station-count 4 --amplitude-m 27 --current-speed-mps 0.05 --current-direction-deg 270 --current-compensation --start-side 1 --identity-fixture --output-dir /tmp/shallow_rejoin_negative
python3 tools/probe_v2_shallow_zigzag_rejoin_pvs.py --seed 101 --case fused_neighbor_1m --station-count 4 --amplitude-m 27 --current-speed-mps 0.05 --current-direction-deg 270 --current-compensation --start-side 1 --identity-fixture --output-dir /tmp/shallow_rejoin_fused
python3 tools/plot_v2_shallow_zigzag_rejoin_pvs.py
```
