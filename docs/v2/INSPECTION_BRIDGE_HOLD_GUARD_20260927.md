# 巡检 Hold 告警到专用协议桥的独立自治出口门

巡检算法已在身份故障时下发零速，并在 Hold 预算不足时锁存 `recovery_required`。这仍留下一个交叉发布风险：ROS 图中若另有自治控制消息，单靠算法节点和 PID 的零命令不能保证专用桥输出零执行器字段。现于 **v2 专用** `InspectionBridgeNode` 再加一道出口门；原通用桥与 v1 不改。

专用桥订阅 `/auv/cable/recovery_required`。在首次收到新鲜的 `false` 前、状态超过候选 1.5 s 未更新时，自治仲裁结果转为 `SAFETY_FALLBACK`；收到一次 `true` 后在桥进程内锁存，不受后续 `false` 清除。对自治结果，桥在协议发送前将主推、四舵、侧推及启动/开启类工作指令置零，保留上游明确的取消任务或关闭执行器指令，并发布 `/auv/cable/bridge_stop_reason`；手动遥控由原仲裁规则处理。算法节点的巡检状态同样优先显示 `RECOVERY_REQUIRED`、`HOLD` 或 `SENSOR_DEGRADED`，避免只显示定位许可。

| 原生 PVS、合成单缆/声呐、0.05 m/s 横流、种子 101 | 正常持续线索 | Track 后改指邻线、Hold 后重发任务 |
| --- | ---: | ---: |
| 到达路由终点 | 是，`1105.75 s` | 否；`583.5 s` Hold |
| 桥告警锁存 | 否 | 是 |
| Hold 后原任务/改速任务 | 不适用 | 2 条均被拒绝 |
| Hold 后 10 s 协议主推 | 不适用 | 41/41 周期零 RPM |
| Hold 期桥仲裁来源 | 不适用 | 仅 `SAFETY_FALLBACK` |
| 最终巡检状态 | `LOCALIZED_DEPTH_PENDING` | `RECOVERY_REQUIRED` |
| 埋深发布 | 否 | 否 |

正常案例在首次预算状态到达前观察到短暂 `inspection_budget_status_stale`，桥先保持自治零输出；状态新鲜后正常推进并完成，说明“缺状态即零”不妨碍此受控任务。故障案例桥收到预算告警并锁存，任务重发没有解锁。两组均走 ROS DDS、PID、自治守卫、本地 UDP 编解码及原生 PVS。它们验证的是**候选软件出口门**，不含物理 PC104、真实声呐或真实海流；ROS 发布源也未做认证，故现场须控制 DDS 发布权限。零协议字段不能直接推断实艇已经停住或上浮。

[逐例摘要及原始 CSV SHA-256](publication/fwhm_v2_experiment_results/data/34_bridge_hold_guard_summary.json)钉住两组结果和实现哈希，[最终软件候选快照](publication/fwhm_v2_experiment_results/data/inspection_v2_candidate_snapshot_20260927_bridge_guard.json)继续标记不批准实艇发布。[限时 Hold 预算报告](INSPECTION_BOUNDED_HOLD_20260927.md)解释所需 17.6 m 与实际 14.481 m 的差额；本报告关注告警是否从巡检节点传至协议出口。现场仍须量测实艇惯性与水流包络，并验收具处置权限的上级决策、PC104 实际响应及人工接管时延，才能给出真正的持续安全等待结论。

复现两组时先构建 `auv_cable_tracking_v2`，使用隔离 ROS 域。故障组在下列共同命令中增加 `--sonar-track-fault switch --post-hold-coast-s 10 --retransmit-mission-on-hold`，输出目录改为 `switch_cross`；正常组输出目录为 `normal_cross`。

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=232
python3 tools/smoke_v2_flythrough_ros_pvs.py --case wire --seed 101 \
  --config-file brain_linux/src/auv_cable_tracking_v2/config/cable_tracking_v2_flythrough_sonar_identity_sim.yaml \
  --sonar-identity-mode target --with-bridge \
  --current-speed-mps 0.05 --current-direction-deg 90 \
  --output-dir AUV-Master-Mag/results/v2/inspection_bridge_hold_gate_20260927/normal_cross
```
