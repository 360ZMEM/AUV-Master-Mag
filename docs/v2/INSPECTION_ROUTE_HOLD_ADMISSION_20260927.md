# 浅角度巡检的走廊准入与 Hold 空间余量

这项增量把“故障时才发现剩余空间不够”前移到任务准入。原合成任务边界为 `x∈[-270,120] m, y∈[-30,30] m`，已知穿越段从 `(-250,-15) m` 到 `(-80,15) m`。候选 Hold 空间预算为停止边界余量 `10 m`、零推力滑行界 `5 m`、导航边界余量 `0.6 m`、流速上界 `0.05 m/s × (30+10) s`，合计 `17.6 m`；再预留声明的 `2 m` 路线偏差，起航前需要严格大于 `19.6 m` 的净距。旧走廊已知线段最小只有 `15.0 m`，因此不应起航。

新增独立合成配置把矩形边界扩为 `x∈[-280,120] m, y∈[-40,40] m`，已知线段最小净距为 `25.0 m`，准入余量 `5.4 m`。这不是通过改写旧实验让其“通过”；旧 YAML、图 31–34 和结果保持原口径。图 35 直接显示走廊、任务线和逐点净距。

![图 35：新旧走廊的任务准入](publication/fwhm_v2_experiment_results/figures/35_route_hold_admission.png)

软件采用显式 opt-in `v2.route_hold_admission`：只有声明了凸边界、Hold 配置和最大路线偏差的 flythrough 任务才运行这项预检。凸多边形内的边界净距沿直线路段为凹函数，因此最小值在已知航点；非凸边界拒绝，不能靠稀疏采样猜测安全。任务准入失败时锁存 `route_hold_budget_insufficient`、发布 `RECOVERY_REQUIRED` 并拒绝任务。起航后在 Search、Confirm、Track 与 Reacquire 周期用当前位置和新鲜导航协方差重新检查**完整 Hold 反应时间**所需余量；不够时同周期将 setpoint 速度置零并上报 `motion_hold_budget_insufficient`。任务重发不能解除锁存。

这项预检只覆盖已知的初始穿越段。磁观测决定的回归和跟踪航迹不能事前由两个端点证明；运动期余量门负责限制它们当前所在位置。它也不能计算真实海流下的未来轨迹，只使用声明的流速与滑行上界。超出这两个实测上界时必须重新标定配置和走廊。

原生 PVS 正常闭环使用新增宽走廊和合成声呐身份线索，于 `1105.25 s` 到达路线终点；完整沿缆横偏 MAE 为 `0.754 m`、最大 `1.332 m`，通过在线质量门，未发布埋深，也未误触发处置告警。故障组进入 Track 后将目标线索改指邻线，在 `585.0 s` 进入 Hold；原任务及改速任务各重发一次，均被拒绝。接下来 `10 s` 内本地 UDP 解码主推转速无非零周期，但车辆仍漂移 `2.182 m`，末速度约 `0.151 m/s`。在这段已声明空间预算内，状态保持 `HOLD`，没有错误地把“处于预算内”报成 `RECOVERY_REQUIRED`。这也再次说明零推进不是定点控制或上级处置。两组逐周期文件分别见 `results/v2/route_hold_admission/normal/` 和 `fault/`。

本地桥闭环最初在正常任务约 `1085.5 s` 遇到 `AMD_UPLINK_STALE`：测试夹具没有物理上行流，只在仿真步刷新合成遥测，而 DDS 等待跨过守卫超时后，桥按设计退回遥控并清空命令。最终夹具在健康场景的 DDS 等待期间持续刷新合成遥测，故障注入档仍停止刷新；守卫阈值和生产桥代码未放宽。上述正常结果以这一明确的合成上行条件为前提，不代表真实上行链路已经连续通过。

对最终夹具另做一次刻意上行失鲜回归：故障后的 `241/241` 个周期 PID 和本地 UDP 主推均为零，未误报任务完成，说明连续刷新没有屏蔽守卫的真实失鲜分支。巡检的 `inspection_status` 在该桥层故障下仍显示横向定位状态 `LOCALIZED_DEPTH_PENDING`；操作界面必须同时显示仲裁/桥停止原因，不能仅凭定位状态判断推进许可。随后新增的[桥运行状态接口](INSPECTION_OPERATIONAL_STATUS_20260927.md)已将两者并列发布，并用命令序号核对了本地闭环；本页冻结的三组结果未回写。

复现入口：

```bash
PYTHONPATH=brain_linux/src/auv_cable_tracking_v2 python3 -m pytest -q brain_linux/src/auv_cable_tracking_v2/test/test_hold_budget.py
python3 tools/plot_v2_route_hold_admission.py
```

ROS 包测试还覆盖旧走廊起航拒绝、新走廊接收、运动期不足撤权、无关任务不会锁存，以及非仿真模式下误开启未标定 FWHM 埋深的拒绝。原生 PVS 需先 source 仓库 ROS 安装空间并使用隔离的 `ROS_DOMAIN_ID`；命令参数见 `tools/smoke_v2_flythrough_ros_pvs.py --help`，[三组逐例哈希](publication/fwhm_v2_experiment_results/data/35_route_hold_admission_runs.json)由 `python3 tools/audit_v2_route_hold_admission.py` 生成，[候选快照](publication/fwhm_v2_experiment_results/data/inspection_v2_candidate_snapshot_20260927_route_admission.json)固定对应实现。本图和三次 PVS 运行均使用合成停止、横流、导航与声呐条件；**真实走廊、零推力滑行、横流上界和 PC104 处置时延尚无物理验收**。因此本页证明的是候选准入逻辑和合成闭环，不是实艇启航许可。

本次收口检查：v2 ROS 包 `44/44` 测试通过，磁反演/标定/巡检相关 `47/47` 测试通过，`auv_cable_tracking_v2` 隔离 colcon 安装构建成功，`make auv-thesis` 编译为 208 页且新增图 5.32 排版正常。测试计数是软件回归覆盖，不转换为实海况成功概率。
