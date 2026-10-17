# 浅角度巡检：侧偏与横流组合复核

本补项沿用已发布的合成单导线、预对准、直线先验、390×60 m 走廊、ROS2＋PID＋自治守卫＋本地 UDP＋原生 PVS 配方，种子固定为 101。它检验侧向区间 `±(2–8) m` 与 `0.05 m/s` 南北横流**组合**时的任务终态。所有场形与磁噪声仍由同一仿真脚本生成；没有真实地图、海缆、PC104 或实艇制动证据。历史 5600 次 FWHM 方法稳定性数据和此前图 24 的零流幅值扫掠不改写。**本补项修正横向确认与执行边界，不证明未知物理场形下的宽度—埋深系数；未标定深度继续不发布。**

## 诊断与路径选择

原配置在 `|δ|=2,8 m` 的两向横流与双向穿越共八个角点中，到达终点 4/8。`|δ|=2 m` 的困难流向因磁尾较早触发 Confirm，450 s 扫描时限在终点前耗尽；把扫描上限有界提高到 600 s，并将 80 m 路由终点容差从 1.5 m 调为 2.0 m 后，镜像两个角点到达终点。容差调整不改变沿缆偏差数据；必须将“到达终点”和“达到横偏验收指标”分开。

`|δ|=8 m` 的向外横流角点并非缺少半高交点：旧终点剖面有完整双侧交点、中心约 8.0004 m、宽约 8.822 m，却以 `ambiguous_anomaly_peaks` 拒止。重建确定性注入磁剖面发现，5 点中值滤波把单导线峰顶分成两个相距约 0.046 m 的等高平台，鞍点仅低约 0.0028 nT，远低于噪声尺度。旧 `find_peaks` 将两者各算为完整主峰。现在只有相邻峰**同时**满足距离不超过 `0.05W` 且鞍点低于 `max(3σ_n,0.01P)` 才合并，其中 `W` 是实测半高宽、`P` 是主峰突出度、`σ_n=P/10^(SNR/20)`；其余多峰继续拒绝。独立单测覆盖浅鞍点通过、深鞍点和分离双峰拒绝。新判别使正负 8 m 的困难流向均到达终点，且未发布埋深。

无条件把穿越终点从 ±15 m 扩到 ±18 m 的探索虽然修复了一个向外流角点，却令另一流向出现多峰拒止或边界停机；按失败原因再延伸 3 m 仍未解除 +8 m 多峰拒止。因此这些扩大幅值策略只保留在本地诊断记录，**没有进入正式 v2 逻辑**。本次选择“有限时间／终点容差＋峰顶噪声判别”，同时保留保守拒止。

有界扫描时限还引出了另一风险：错误侧向符号可能不再靠 450 s 超时拒止。现在完成扫描后还要求磁测横向中心落在声明侧向区间的 ±0.5 m 容差内；不一致时返回 `side_hint_observation_inconsistent`，不得进入 Track。它验证的是“观测与任务输入是否相容”，不能证明任务地图是真值，也不能证明已观测异常必是目标电缆。

![峰顶假双峰与侧偏／横流组合终态](publication/fwhm_v2_experiment_results/figures/25_current_offset_peak_gate.png)

图 25(a) 的磁曲线由原始 CSV 中的 PVS `y` 轨迹和脚本固定种子重建；图 25(b) 的红 × 表示原配置拒止，蓝格表示修正后到达终点，**不是实艇验收通过**。点位矩阵只采样 `δ=±2,±8 m` 与南北 `0.05 m/s`，不证明连续区间内所有组合。图和逐例来源见[独立摘要](publication/fwhm_v2_experiment_results/data/25_current_offset_peak_gate_summary.json)。

| 真实偏差 | 横流方向 | 原配置 | 修正后终态 / 时间 | 完整沿缆真实横偏 MAE / 最大值 |
|---:|---|---|---|---:|
| −8 m | 北 | 到终点 | 到终点 / 1107.00 s | 0.868 / 1.490 m |
| −8 m | 南 | 多峰拒止 | 到终点 / 1105.75 s | 0.836 / 1.447 m |
| −2 m | 北 | 到终点 | 到终点 / 1113.00 s | 1.051 / 1.936 m |
| −2 m | 南 | 扫描超时 | 到终点 / 1114.75 s | 1.296 / 2.286 m |
| +2 m | 北 | 扫描超时 | 到终点 / 1114.75 s | 1.296 / 2.280 m |
| +2 m | 南 | 到终点 | 到终点 / 1113.00 s | 1.056 / 1.971 m |
| +8 m | 北 | 多峰拒止 | 到终点 / 1105.75 s | 0.837 / 1.450 m |
| +8 m | 南 | 到终点 | 到终点 / 1107.00 s | 0.866 / 1.498 m |

表中误差逐样本取进入正向 80 m Track 后的 `|y−δ|`，不包含穿越与返回起点；任务真值不输入估计器。八组均未发布埋深。虽然八组都触发终点停止，四个 `|δ|=2 m` 案例的真值 MAE 超过示例配置的 `mean_route_offset_m: 1.0`，其中两个逆向流角点的最大值还超过 `max_route_offset_m: 2.0`。因此它们**只证明任务流程到达终点，不证明横偏验收合格**。不得把 8/8 写成“巡检精度通过率”；近侧偏差需另行改进跟踪或收紧作业条件，并在新的独立多种子集上复核。固定磁噪声种子仍可能受 ROS 异步调度影响；重复运行中两个顺流近侧案例的横偏指标发生数厘米级变化，故上表仅是所存逐例值，不是统计置信区间。

后续已按此结论新增[有界上游接入段与三种子复核](INSPECTION_REJOIN_POLICY_20260926.md)。本页仍保留图 25 和原始 CSV 作为修改前冻结基线；新增图 26 与 24 次运行单独报告，不能追溯改写本页的四个近侧超门限反例。

最终错误符号反例使用南侧配置、真实 `δ=−6 m`、零流和 600 s 时限；横向中心与声明正侧区间冲突，于 584.25 s 保持，当周期 PID 推力为 0、UDP 解码 RPM 为 0，未发布埋深或误报完成。正确 +8 m 向外流重跑仍到终点。对应逐周期记录位于忽略的 `results/v2/inspection_side_hint_corner_current_20260926/`；绘图脚本为主仓 `tools/plot_v2_current_offset_peak_gate.py`。本机复核通过 Mag v2 单测 55 项、ROS 巡检单测 17 项及独立 colcon 构建；现场可用性仍按[受限运行契约](INSPECTION_RELEASE_ENVELOPE_20260926.md)验收。

在总仓根目录、ROS Humble 和既有 `auv_interfaces`／控制包已构建的环境中，可复现两个关键案例；完整八角点的参数与原始目录逐例列在图 25 JSON 中。每个并行案例必须使用独立 DDS 域和输出目录。

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
source /tmp/auv_inspection_colcon/install/setup.bash
source /tmp/auv_side_hint_envelope_final/install/setup.bash
export PYTHONPATH="$PWD/brain_linux/src/auv_cable_tracking_v2:$PWD/brain_linux/src/auv_controller:$PWD/AUV-Master-Mag/src:$PYTHONPATH"
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=161 python3 tools/smoke_v2_flythrough_ros_pvs.py \
  --case wire --seed 101 --current-speed-mps 0.05 --current-direction-deg 90 \
  --cable-lateral-m 8 --with-bridge \
  --config-file brain_linux/src/auv_cable_tracking_v2/config/cable_tracking_v2_flythrough.yaml \
  --output-dir AUV-Master-Mag/results/v2/recheck_plus8_north
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=162 python3 tools/smoke_v2_flythrough_ros_pvs.py \
  --case wire --seed 101 --cable-lateral-m -6 --with-bridge \
  --config-file brain_linux/src/auv_cable_tracking_v2/config/cable_tracking_v2_flythrough.yaml \
  --output-dir AUV-Master-Mag/results/v2/recheck_wrong_side
python3 tools/plot_v2_current_offset_peak_gate.py
```

最后一条绘图命令使用本报告对应的归档八角点目录，不能只跑上述两个案例后直接生成完整矩阵。`/tmp` 安装空间是本机隔离构建产物，新主机需先构建相应 ROS 包。
