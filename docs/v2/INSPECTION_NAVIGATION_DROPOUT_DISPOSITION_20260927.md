# 短时导航无效后的双通道处置：当前巡检配置复核

前期不带路线 Hold 预算的安全链路试验显示，15 s 导航无效结束后可以继续任务。当前候选配置已加入任务开始时的走廊准入和运行时 Hold 预算，因此需要重新核对“感知恢复”是否等于“执行恢复”。本次使用现有 ROS2 节点、PID、专用桥、本地 72 字节 UDP 报文和原生 PVS，使用合成单直导线、脚本目标声呐和健康 AMD 上行夹具；没有连接物理 PC104 或实艇。

独立进程、隔离 ROS 域 181、种子 108 的运行在首次 Track 后 8 s，即仿真 $t=585.0$ s，将导航位置的 $x$ 输入改为 NaN，持续 15 s。当前配置的保持预算在第一个无效位置上锁存 `hold_geometry_invalid`，对外状态为 `RECOVERY_REQUIRED`。$t=600.0$ s 后有效导航恢复，感知状态重新显示 `track`，但**桥自治许可不自动恢复**，PID 与 UDP 仍维持零主推。外部复核和重新起航是独立步骤，不能由重新出现的 Track 标签或任务重发代替。

| 观察窗口 | 周期数 | 感知输出 | 执行与任务结果 |
|---|---:|---|---|
| 故障前 | 2336 | 搜索、确认并进入 Track | 2336 周期桥许可且 UDP 正主推 |
| 导航无效 $[585,600)$ s | 60 | `hold`，原因 `navigation_invalid` | 60/60 周期目标速度、PID 推力、UDP 主推均为零；预算锁存 |
| 导航恢复后至 1200.75 s | 2404 | 2404 周期重新显示 `track` | 0 周期获自治许可；2404/2404 周期 UDP 零主推；未宣布任务完成，未发布埋深 |

![图 38：导航短暂失效后，感知状态与执行许可分离](publication/fwhm_v2_experiment_results/figures/38_inspection_nav_dropout_disposition.png)

图 (a) 是本次最关键的跨层诊断：`track` 是线索/状态机标签，绿色虚线的桥许可才表示可执行自主推进。图 (b) 给出实际本地协议输出；图 (c) 给出原生 PVS 自首次 Hold 起仍累计移动 4.842 m，说明零主推命令不能解释成定点保持。仿真在 1200.75 s 按设定时限结束，脚本退出码为 0 只表示正常写出记录，**不表示任务完成**。

从仓库根目录复现：

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=181 python3 tools/smoke_v2_flythrough_ros_pvs.py \
  --case wire_navigation_dropout --seed 108 --max-time-s 1200 \
  --with-bridge --sonar-identity-mode target \
  --config-file brain_linux/src/auv_cable_tracking_v2/config/cable_tracking_v2_flythrough_sonar_identity_corridor_budget_sim.yaml \
  --output-dir AUV-Master-Mag/results/v2/inspection_recovery_20260927/navigation_dropout
python3 tools/audit_v2_inspection_nav_recovery.py
python3 tools/plot_v2_inspection_nav_disposition.py
```

[逐周期审计](publication/fwhm_v2_experiment_results/data/inspection_v2_transient_nav_recovery_audit_20260927.json)记录 CSV、摘要及源码/安装空间哈希；`checks_passed=true` 仅指上述**安全处置不变量**满足，而 `completed=false`、`autonomous_resume_observed=false`、`field_release_authorized=false`。[绘图清单](publication/fwhm_v2_experiment_results/data/38_inspection_nav_dropout_disposition_manifest.json)固定图片和脚本哈希。旧 15 s 导航恢复结论适用于此前未开启运行时 Hold 预算的配方，不能挪用于当前候选；当前候选更保守，代价是一次短故障也需要外部复核。若以后要允许自动恢复，必须另证明导航重新有效、故障期间位移上界与剩余边界净距仍满足同一预算，并以新配置重新跑完整负例；不能简单清除锁存。
