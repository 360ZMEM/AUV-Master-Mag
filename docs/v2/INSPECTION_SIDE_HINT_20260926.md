# 侧向先验决定浅角度穿越方向：可达包络与反例

独立浅角度巡检在 PVS 中不宜把“电缆可能位于先验任意一侧”交给同一条固定方向航迹兜底。欠驱动载体在确认后需要转向有限路由起点：穿越结束位置若落在电缆远侧，即使磁剖面已足以定位，回归段仍可能因转向迟缓而无法开始 80 m 巡检。这里将**先验只给出侧向符号，允许有米级位置误差**作为任务约束：若预任务地图、上一段跟踪或声学线索可信地表明电缆位于先验北侧，从南侧预对准并向北穿越；位于南侧则反向。侧向线索的真实性不是本合成试验已验证的传感器能力；没有该线索时，本模式不声称覆盖两侧，须使用其他搜索或等待重新布置。

![侧向提示匹配与错误时的原生 PVS 航迹](publication/fwhm_v2_experiment_results/figures/22_side_hint_flythrough_paths.png)

两套配置保持同一条 80 m 直线先验、390×60 m 空旷走廊、预对准起点、0.3 m/s 目标航速、零流、种子 101 和相同控制/守卫/协议/PVS 链，只交换穿越端点在先验两侧的顺序。模拟电缆与先验中心线的真实横向偏差为 ±6 m，深度因无现场标定始终为空。

| 穿越方向 / 真实偏差 | 横向确认 | 80 m 终点 | 全体沿缆 TRACK 横偏 MAE | 主要结果 |
|---|---:|---:|---:|---|
| 南→北 / +6 m | 是 | 是，1121.75 s | 1.026 m | 轨迹最大绝对横坐标 16.893 m |
| 北→南 / −6 m | 是 | 是，1121.75 s | 1.026 m | 轨迹最大绝对横坐标 16.892 m |
| 南→北 / −6 m（方向错误） | 否 | 否 | — | 469.75 s 扫描超时并零推进，未误报完成 |

方向错误时，磁异常虽然在航线上被检测到，确认阶段从过早的侧翼触发，450 s 扫描时限先于右侧覆盖完成。单纯把半幅从 ±15 m 加到 ±20 m 并延长扫描时限，可以让真值 +10 m 的一个试验取得横向中心，却在另一侧 −6 m 工况经历较大返回横移并触发边界/越过起点保护；“无条件增大之字形幅度”并非稳定修复。因此最终建议是**按有置信度的侧向先验选择单向穿越方向**，仍以双侧半高与横向几何门决定是否开放跟踪，证据不足时保持，而非把 FWHM 深度强行发布。

为把“可信侧向线索”落实到可检查输入，ROS 浅角度模式现在强制读取 `side_hint_min_m`、`side_hint_max_m` 和 `side_hint_source`。区间必须完全处在先验同一侧，方向要与扫描终点一致，最外侧还必须落在扫描终点内。缺失、非数值、跨线、方向不符和越界五种配置在实际 ROS／PID／桥守卫／本地 UDP／PVS 回路中均于首周期保持，PID 推力与 UDP 解码 RPM 均为零，且返回各自原因码；节点不崩溃。提供的 YAML 将来源明确写成 `synthetic_test_fixture`，**它只是仿真输入，不是受过验证的现场地图或声呐线索**。ROS 节点在 `use_sim_time=false` 时还会拒绝此标签，避免误把示例配置直接用作非仿真任务；这一点已由节点单测验证，不能代替现场任务身份和线索真实性认证。启动门仅检查声明区间的自洽性，不能判断声明是否真实：把正侧区间交给真值 −6 m 时仍会运动，最终于 469.75 s 扫描超时保持。这一负例界定了任务约束的责任分界。

![侧向启动门与镜像横偏复核](publication/fwhm_v2_experiment_results/figures/23_side_hint_gate_and_envelope.png)

| 条件 | 运行次数 | 完成 / 拒止 | 沿缆真值横偏 MAE | 最大逐样本横偏 | 深度发布 |
|---|---:|---|---:|---:|---:|
| 南侧起步，电缆 +6 m，零流，种子 101–103 | 3 | 3/3 完成 | 1.0258–1.0263 m | 1.8190 m | 0 |
| 北侧起步，电缆 −6 m，零流，种子 101–103 | 3 | 3/3 完成 | 1.0258–1.0260 m | 1.8195 m | 0 |
| 北侧起步，电缆 −6 m，北向 0.05 m/s，种子 101 | 1 | 完成 | 0.9830 m | 1.7780 m | 0 |
| 北侧起步，电缆 −6 m，南向 0.05 m/s，种子 101 | 1 | 完成 | 1.0057 m | 1.7593 m | 0 |
| 五种无效声明区间，各一次 | 5 | 首周期保持，双层零推进 | — | — | 0 |
| 声明正侧但真值 −6 m，种子 101–103 | 3 | 扫描超时保持，不误报完成 | — | — | 0 |

上表的 3/3 是固定实现与指定噪声种子的逐例检查，不是海况成功率或置信区间；横偏仅统计全部处于 Track 且沿缆坐标 $x\geq0$ 的样本。北侧双向 0.05 m/s 横流各只有一个种子，不能据此宣称一般横流鲁棒性。实际侧向先验须提供可审计来源、位置区间及可靠性论证，载体需要按已测转弯和停止包络预对准；真实海缆场形、导航偏差、PC104 和海床高度仍是现场验证条件。此结果可引用为**条件化的航迹几何选择与安全拒绝机制**，不能表述为未知侧向先验的全面搜索成功率。

[启动门与多种子独立摘要](publication/fwhm_v2_experiment_results/data/23_side_hint_gate_and_envelope_summary.json)逐例记录原始 CSV 路径、完成/拒止、真值横偏及推力口径。图由 `tools/plot_v2_side_hint_gate.py` 从本地 `results/v2/inspection_side_hint_gate_20260926/` 和 `inspection_side_hint_robustness_20260926/` 再生，原始逐周期 CSV 不纳入发布包。

## 侧向幅值扫掠：扫描端必须覆盖外侧半高交点

上述最初示例声明的正侧区间 `2–10 m`（负侧镜像 `−10–−2 m`）只按“峰中心在扫描端内”筛选，过宽。合成单直导线使用
$B(y)=1+20/\sqrt{1+((y-\delta)/3)^2}$ nT，背景修正半高交点距中心 $3\sqrt{3}=5.196$ m；扫描端在 $\pm15$ m。因此 $|\delta|=10$ m 时外侧交点约在 $15.196$ m，理论上已越过扫描端。在真实 ROS／PID／桥守卫／UDP／PVS 扫掠中，正负 `10 m` 都于 584.25 s 返回 `flythrough_lateral_rejected`，没有误报完成或发布深度；正负 `9 m` 虽完成，但解析余量仅约 0.804 m。图 24 把场形几何与实际终态对齐。

![侧向幅值、半高交点与原生 PVS 终态](publication/fwhm_v2_experiment_results/figures/24_side_hint_offset_envelope.png)

| 电缆偏差幅值，正负两侧各一例 | 正/负侧终态 | 沿缆真实横偏 MAE（两侧约值） | 备注 |
|---|---|---:|---|
| 2 m | 均完成 | 1.274 m | 最大逐样本横偏约 2.313 m |
| 4 m | 均完成 | 1.158 m | — |
| 6 m | 均完成 | 1.026 m | 另有 101–103 三种子复核 |
| 8 m | 均完成 | 0.896 m | 收紧后声明区间外缘，外侧解析余量约 1.804 m |
| 9 m | 均完成 | 0.834 m | 余量约 0.804 m，故不纳入声明区间 |
| 10 m | 均拒绝 | — | 外侧半高交点超出终点约 0.196 m |

为使任务接入条件与观测几何一致，两套正式演示 YAML 已收紧为 `+2…+8 m` 与 `−8…−2 m`，同时设 `minimum_halfmax_tail_m: 7.0`：侧向区间任一端距扫描端都须至少留出 7 m，包含约 5.196 m 的合成半高距离及约 1.804 m 运动/采样余量。旧 `2–10 m` 区间现于首周期以 `flythrough_side_hint_support_insufficient` 保持，PID 与解码 RPM 均为零；不配置峰外预算也于首周期以 `flythrough_side_hint_support_unconfigured` 双层零推进，避免默认零余量继续任务。最终配置的正负 8 m 完整闭环仍完成。这个 7 m 数值**只针对本合成场形和当前 PVS 配方**，不能直接移植到多芯海缆或真实埋深。图中正负方向标记为便于辨识水平错开 0.1 m，真实测试偏差仍是表中整数值。

[图 24 逐例摘要](publication/fwhm_v2_experiment_results/data/24_side_hint_offset_envelope_summary.json)含解析交点、预检拒止和各原始 CSV 路径。幅值扫掠除 6 m 的额外复核外只使用种子 101 与零流；有限点位不能证明区间内每个连续偏差都保证成功，现场仍须以真实场形宽度和载体包络重定覆盖余量。

后续将侧偏外缘与 0.05 m/s 横流组合，发现原配方八个角点仅四个到达终点。峰顶数值假双峰、有界扫描时间与终点容差经针对性修正后八个角点均到终点；但近侧 `|δ|=2 m` 的四例未达到示例横偏验收门限。放宽扫描时限后另加“磁测中心须与声明区间相容”的在线拒止门，防止符号错误的侧向线索因超时推迟而进入 Track。图 25、逐例误差、负例与限制见[侧偏／横流组合复核](INSPECTION_CURRENT_OFFSET_JOINT_20260926.md)。

[独立数值摘要](publication/fwhm_v2_experiment_results/data/22_side_hint_summary.json)记录逐样本误差口径和任务终态。在仓库根目录、ROS Humble 与 `auv_interfaces`/v2 已构建的环境中，可按下列命令复现两次镜像运行；两次使用不同隔离 `ROS_DOMAIN_ID`：

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
source /tmp/auv_inspection_colcon/install/setup.bash
colcon --log-base /tmp/auv_side_hint_gate_build/log build --base-paths brain_linux/src/auv_cable_tracking_v2 --build-base /tmp/auv_side_hint_gate_build/build --install-base /tmp/auv_side_hint_gate_build/install --packages-select auv_cable_tracking_v2 --symlink-install
source /tmp/auv_side_hint_gate_build/install/setup.bash
export PYTHONPATH="$PWD/brain_linux/src/auv_cable_tracking_v2:$PWD/brain_linux/src/auv_controller:$PWD/AUV-Master-Mag/src:$PYTHONPATH"
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=151 python3 tools/smoke_v2_flythrough_ros_pvs.py --case wire --with-bridge --cable-lateral-m=6 --output-dir AUV-Master-Mag/results/v2/inspection_flythrough_ros_pvs_bridge_side_final_plus6_20260926
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=152 python3 tools/smoke_v2_flythrough_ros_pvs.py --case wire --with-bridge --cable-lateral-m=-6 --config-file brain_linux/src/auv_cable_tracking_v2/config/cable_tracking_v2_flythrough_north_start.yaml --output-dir AUV-Master-Mag/results/v2/inspection_flythrough_ros_pvs_bridge_side_final_minus6_20260926
```

上述 `/tmp/auv_inspection_colcon` 是原有接口依赖安装空间；在全新主机应先构建 `auv_interfaces` 等主仓依赖。错误方向保留南侧配置但设 `--cable-lateral-m=-6`。图由 `tools/plot_v2_flythrough_side_hint.py` 生成。原始 CSV 在本地忽略的 `results/v2/inspection_flythrough_ros_pvs_bridge_*_20260926/`；历史 v1 和 5600 次方法稳定性结果均未改动。
