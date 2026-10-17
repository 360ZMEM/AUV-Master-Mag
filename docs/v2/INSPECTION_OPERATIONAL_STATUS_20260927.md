# 巡检定位状态与自治执行许可的对外分离

`/auv/cable/inspection_status` 描述巡检定位与任务状态，不是推进许可。桥守卫因上行失鲜撤权时，横向中心仍可被磁/声证据定位为 `LOCALIZED_DEPTH_PENDING`；若界面只展示这一标签，会把仍有位置估计误读成车辆可以继续自治。v2 专用桥现另发 `/auv/cable/operational_status`（`std_msgs/String`，JSON），由**有效仲裁结果**、守卫、预算门和被动模式共同给出 `autonomous_execution_authorized` 与 `authority_reason`，并并列附上新鲜的 `localization_status`。定位消息超过候选 1.5 s 未更新时，合并字段改为 `UNKNOWN`；这不改写定位原话题或桥的执行门。

主字段是 `autonomous_execution_authorized`、`authority_reason`、`arbiter_mode`、`arbiter_source`、`guard_state`、`guard_reason`、`localization_status`、`localization_status_fresh`、`timestamp_s`、`command_sequence`。命令序号使日志分析可把 ROS 状态与 UDP 指令配成同一决策周期。`true` 的含义仅是桥当期允许来自 Jetson MPC 的自治命令进入发送路径；它**不保证车辆正在移动、保持定点，或物理 PC104 已接受指令**。界面应将这一许可和定位状态同时显示，并在任一数据失鲜时将其标记为未知；不能缓存最后一次 `true` 当作持续许可。

![图 36：横向定位状态与自治许可分离](publication/fwhm_v2_experiment_results/figures/36_localization_vs_autonomy_authority.png)

随后把这条状态并入 v2 专用桥已有的 `cable_monitor` Zenoh 遥测，PySide6 上位机诊断页将“感知验收”和“自治许可”分成两行。界面使用消息序号与时间戳去重，重复缓存包不会刷新许可有效期；超过 1.5 s 无新状态时显示“自治许可未证实（状态失鲜）”。本地隔离回环已从 v2 桥经 Zenoh `rt/auv/telemetry` 到 PySide6 标签收到“已定位（埋深未许可）/自治许可禁止（上行失鲜）”，并验证重复包后的失鲜显示。下图是该回环的操作员界面，不是实艇遥测截图。

![PySide6 上位机隔离回环的巡检许可标签](publication/fwhm_v2_experiment_results/figures/operator_console_status_loopback.png)

合成 ROS2--PID--自治守卫--本地 UDP--原生 PVS 中，刻意停止刷新上行遥测前，`2336` 个配对周期许可为真；失鲜发生于仿真 `585.0 s`，后续 `241/241` 个周期许可为假，原因均为 `AMD_UPLINK_STALE`，而定位状态均仍为 `LOCALIZED_DEPTH_PENDING`，本地 UDP 主推均为零。图 36 只放大转换前后 36 s。另一个独立目标声呐身份改指邻线案例于 `585.0 s` 进入 Hold，后续 `41/41` 个周期桥状态均为 `SAFETY_FALLBACK`/`mpc_command_unavailable`、自治许可为假、UDP 主推为零；此时定位/任务状态为 `HOLD`，空间预算告警并未锁存。两个场景说明撤权可分别来自桥守卫和任务状态机，操作员不应把它们合成同一个原因。

复现使用隔离 ROS domain、本地 UDP 端口及合成声呐/上行夹具，需先 source ROS Humble 与仓库 ROS 安装空间，再为每次运行设置 `ROS_LOCALHOST_ONLY=1` 和空闲的 `ROS_DOMAIN_ID`。两组执行参数分别为：

```bash
python3 tools/smoke_v2_flythrough_ros_pvs.py --case wire_guard_telemetry_loss --seed 101 --max-time-s 750 --with-bridge --sonar-identity-mode target --config-file brain_linux/src/auv_cable_tracking_v2/config/cable_tracking_v2_flythrough_sonar_identity_corridor_budget_sim.yaml --output-dir AUV-Master-Mag/results/v2/operational_status/guard_loss_verified
python3 tools/smoke_v2_flythrough_ros_pvs.py --case wire --seed 101 --max-time-s 700 --with-bridge --sonar-identity-mode target --sonar-track-fault switch --post-hold-coast-s 10 --retransmit-mission-on-hold --config-file brain_linux/src/auv_cable_tracking_v2/config/cable_tracking_v2_flythrough_sonar_identity_corridor_budget_sim.yaml --output-dir AUV-Master-Mag/results/v2/operational_status/identity_hold
python3 tools/plot_v2_operational_status.py
```

上位机回环另用 `ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=<空闲隔离域> QT_QPA_PLATFORM=offscreen python3 tools/smoke_v2_operator_side_channel.py`；脚本为桥和上位机分配本机 UDP/Zenoh 端口，禁用 PC 命令发布，使用合成状态而不连接 PC104。输出见 [Zenoh/PySide6 回环](../../results/v2/operator_chain/zenoh_ui/)，两组更新后的 PVS 数据见 [操作员链实验目录](../../results/v2/operator_chain/)。

逐周期原始数据在 [失鲜组](../../results/v2/operational_status/guard_loss_verified/)与[身份改指组](../../results/v2/operational_status/identity_hold/)；作图脚本从失鲜组 CSV 生成图 36、统计和 SHA-256 清单。初版[桥状态候选快照](publication/fwhm_v2_experiment_results/data/inspection_v2_candidate_snapshot_20260927_operational_status.json)保持冻结；加入上位机显示后的[前一候选快照](publication/fwhm_v2_experiment_results/data/inspection_v2_candidate_snapshot_20260927_operator_chain.json)由 `python3 tools/audit_v2_operator_chain.py` 在当时源码上固定两组 PVS 和独立 Zenoh/PySide6 回环，明确 `field_release_authorized=false`。当前桥和界面源码已继续加入下述独立状态发送，旧审计脚本若按当前源码重算会拒绝改写历史快照；新审计入口见下段。最初两种回环是分开的测试，不宣称实艇 PVS 与跨设备上位机在同一次任务中贯通。旧图 1–36 和其统计均未回写。v2 ROS 包测试 `46/46`、上位机状态测试 `4/4` 通过，v2 ROS 包隔离构建成功，thuthesis 编译为 209 页；这里没有物理声呐、实艇、PC104 或真实上行连续性验收，零主推也不能证明艇体静止。

后续[同次 PVS—上位机状态闭环](INSPECTION_OPERATOR_SAME_RUN_20260927.md)补上了上述两段软件链之间的空缺：桥不再等 AMD 上行才转发撤权原因，而由决策路径独立发布轻量状态；一个合成失鲜任务中，ROS/PID/桥/UDP/PVS 与 Zenoh/PySide6 同时运行，上位机按序号与时间戳收到 7,732 条桥状态，241 个故障后 PVS 周期均零 RPM。旧候选快照仍是历史版本，新证据有单独的哈希审计；仍未连接 PC104 或跨设备硬件。
