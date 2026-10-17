# 巡检故障 Hold 的限时空间预算与上级处置告警

图 31 证明：身份线索失效后，PID 和本地 UDP 可以持续输出零主推，但原生 PVS 车辆仍会因惯性及横流离开 Hold 点。为使这一限制成为运行时可读条件，独立 v2 ROS 适配器增加 `HoldBudgetMonitor`。它在 Hold 和传感器降级时发布 `/auv/cable/recovery_required` 布尔值、`/auv/cable/hold_budget` JSON，以及 `/auv/cable/tracking` 的 `diagnostics.hold_budget`。一旦触发，告警及零速命令在当前进程内锁存，任务验收增加 `hold_budget_recovery_required`；状态机与旧实验仍保持原语义。任务话题不提供重置入口，只有外部安全评估后重启巡检实例才可重新起航。

![图 32：原生 PVS 横流下的 Hold 空间预算及首周期告警](publication/fwhm_v2_experiment_results/figures/32_bounded_hold_disposition.png)

预算使用可解释的保守不等式：

```text
可用边界净距 ≥ 内侧停止余量 + 零推力滑行上界 + 导航/边界余量
               + 横流上界 × (剩余最长 Hold 时间 + 上级处置时延)
```

监视器还独立检查 Hold 墙钟时长、导航及 ES-EKF 协方差新鲜度，并要求水平位置的两倍标准差不超过声明的导航余量。缺少显式配置、搜索边界、可信位置，或者预算非正时直接报告 `RECOVERY_REQUIRED`；越过边界的点按零净距处理。阈值不是场形分类概率，也不会替代任务目标身份核验。它只说明当前配置是否留出了所声明的处置空间，**只锁住零推进，不调用 PC104，也不执行自主上浮或回收**。

合成测试配置取停止余量 10 m、零流 PVS 观测后的候选滑行上界 5 m、导航余量 0.6 m、横流上界 0.05 m/s、最长 Hold 30 s 和上级响应 10 s。因此需有 `10+5+0.6+0.05×(30+10)=17.6 m` 净距。同一身份改指邻线、0.05 m/s 横流的原生 PVS 运行在 `t=583.5 s` 首次 Hold 时只有 `14.481 m` 净距，缺 `3.119 m`；同周期 ROS `tracking` 诊断报告 `hold_boundary_budget_exhausted`，主推 RPM 为零。继续记录 10 s 的 41 个 Hold 周期，RPM 全零，预算始终为负。这个结果表明**原示例走廊不满足本次显式处置预算**；不能据图 31 的“60 s 仍未越界”倒推出部署许可。

所用 5 m 为图 31 零流 PVS 约 4.65 m 滑行的向上取整候选值，0.05 m/s 也是本合成输入；二者并非真实车辆、真实海况的验收上界。报警首周期同样不是完成了上级动作。现场必须独立测零推力运动包络、导航和边界误差、可执行处置时延，并给 `recovery_required` 接入已认证的上级安全决策与 PC104 动作。若不能提供这些实测量，运行时“未配置预算”即请求处置，不应解释为安全待机。

任务重发还暴露了一个独立撤权漏洞：实验任务注入器在启动窗口以 2 Hz 同时向两条任务话题发送同一 JSON，旧 v2 适配器把每次接受消息都当作新任务来清除预算告警。现已删除任务消息的重置副作用；若预算已报警或穿越状态机已锁存失败，后续任务消息直接拒绝。原生 PVS 的身份改指横流案例在首次 Hold 后经 DDS 重发一次原任务、一次改速任务，两条均被拒绝，后续 10 s 共 41 个 Hold 周期的主推 RPM 均为零、无完成误报。该负例证明了**任务话题不能旁路已触发的撤权**，但仍不等于已具备认证的上级回收通道。[逐例摘要及原始 CSV 哈希](publication/fwhm_v2_experiment_results/data/33_hold_rearm_dedup_summary.json)保留复核证据。

原始 [CSV 与运行摘要](../../results/v2/inspection_hold_budget_20260927/switch_cross/wire_seed_101_summary.json)、[图 32 摘要与哈希](publication/fwhm_v2_experiment_results/data/32_bounded_hold_disposition_summary.json)、[图像哈希](publication/fwhm_v2_experiment_results/data/32_bounded_hold_disposition_manifest.json)及[当前软件候选快照](publication/fwhm_v2_experiment_results/data/inspection_v2_candidate_snapshot_20260927_hold_budget.json)可复核。复现需 ROS Humble、已构建的 v2 包及原生 PVS，在隔离 ROS 域执行：

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=228
python3 tools/smoke_v2_flythrough_ros_pvs.py --case wire --seed 101 \
  --config-file brain_linux/src/auv_cable_tracking_v2/config/cable_tracking_v2_flythrough_sonar_identity_sim.yaml \
  --sonar-identity-mode target --sonar-track-fault switch --with-bridge \
  --current-speed-mps 0.05 --current-direction-deg 90 --post-hold-coast-s 10 \
  --output-dir AUV-Master-Mag/results/v2/inspection_hold_budget_20260927/switch_cross
python3 tools/plot_v2_hold_budget.py
```

任务重发负例另以同一命令增加 `--retransmit-mission-on-hold`，并把 `--output-dir` 改为 `AUV-Master-Mag/results/v2/inspection_hold_rearm_dedup_20260927/switch_cross`。脚本经 DDS 等待两条拒绝回调，再继续原生 PVS 10 s；不能用直接修改内部告警位替代该验证。

整栈重启会重建上述进程内锁存，故不能把“重启成功”当成批准重新起航。后续[启动入口复核](INSPECTION_LAUNCH_RESTART_BOUNDARY_20260927.md)已将通用 `stack-v2` 的自动任务默认关闭，并阻止飞越巡检配置误接通用控制器/桥；现场仍需外部复核与重新授权。
