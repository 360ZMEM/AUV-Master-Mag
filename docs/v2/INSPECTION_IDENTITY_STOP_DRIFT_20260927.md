# 身份线索故障后的零推力运动与走廊余量

运行时身份门在 Track 中发现线索丢失或改指邻缆后，会进入 Hold，使 PID 推力和本地 UDP 主推转速归零。[前一补项](INSPECTION_IDENTITY_CONTINUITY_20260927.md)只在首次 Hold 时结束运行，因而不能说明载体是否已经停住。本次保持 ROS DDS、PID、自治守卫、UDP 编解码和原生 PVS 内环不变，在首次 Hold 后继续约 60 s，以同一控制链逐周期验证零推进，并读取 PVS 真值运动。

![图 31：零推力后的位移、地速与走廊余量](publication/fwhm_v2_experiment_results/figures/31_identity_hold_drift.png)

| 种子 101 配方 | 首次 Hold / 原因 | 60 s 内零 RPM 周期 | 距 Hold 点最终净位移 | 横向位移 | 60 s 地速 | 最小边界净距 |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 线索丢失，零流 | 587.0 s / `flythrough_runtime_identity_lost` | 241/241 | 4.641 m | 0.865 m | 0.0103 m/s | 13.721 m |
| 改指邻线，零流 | 585.0 s / `flythrough_runtime_identity_diverged` | 241/241 | 4.646 m | 0.857 m | 0.0104 m/s | 13.835 m |
| 改指邻线，横流 0.05 m/s | 583.5 s / `flythrough_runtime_identity_diverged` | 241/241 | 5.477 m | 2.969 m | 0.0511 m/s | 11.525 m |

三例均未报完成、未发布埋深。边界为本合成配置的矩形 `x∈[-270,120] m, y∈[-30,30] m`；“最小边界净距”逐周期取到四条边的最短距离。图中 10 m 虚线是配置使用的**候选停止余量参考值**，现行实现只在回归接入点做相应几何预检，并未在 Hold 中执行持续边界保持。横流组 60 s 后仍比该参考线多约 `1.525 m`，且地速已接近流速；若无回收、复位或上级安全动作，持续横流会继续消耗有限走廊。不能把本组的“60 s 尚在边界内”写成“可以无限期安全保持”。

采样行覆盖 Hold 后 `t=0…60 s`，共 241 次零命令；表中的净位移来自最后一次 `0.25 s` PVS 更新后的状态，即 `t=60.25 s`。末地速与最小边界净距由记录的 `t≤60 s` 位姿计算。这一时间口径已在[逐例摘要](publication/fwhm_v2_experiment_results/data/31_identity_hold_drift_summary.json)分别列明。

本实验给出的工程推论是把 Hold 作为**限时撤权状态**：入口需要预留足以容纳零推力惯性位移与有流漂移的空间，同时指定现场实测的最大等待时间、通信/人工接管时延以及超时后的认证安全动作。一个可审计的保守预算形式是

```text
可用边界净距 > 实测零推力惯性位移 + 已验收流速上界 × 最大撤权处置时长
              + 导航/边界误差余量 + 命令与处置时延余量。
```

这只是设计约束，不可用图 31 的 PVS 数值直接冻结实艇的走廊宽度或等待时间。当前 v2 能保持零命令，却尚未将身份故障 Hold 的**超时升级/自主回收**接入并通过真实 PC104 验收；实际车辆的低速水动力、横流、海底障碍与停机距离必须测量。答辩宜说“故障撤权在完整合成执行链成立；停住及持续保持依赖实艇运动包络和上级处置”，而不是把零 RPM 当作零位移。

后续已将上述预算做成[独立 ROS 告警监视器](INSPECTION_BOUNDED_HOLD_20260927.md)：可在预算不足或超时时请求上级处置，但仍未接入经认证的实际回收动作。图 31 原始运动结果保持冻结。

[图 31 逐例摘要和原始 CSV/JSON SHA-256](publication/fwhm_v2_experiment_results/data/31_identity_hold_drift_summary.json)、[出图哈希清单](publication/fwhm_v2_experiment_results/data/31_identity_hold_drift_manifest.json)保留配方与版本。复现时使用隔离 ROS 域，三组分别运行：

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=227
cfg=brain_linux/src/auv_cable_tracking_v2/config/cable_tracking_v2_flythrough_sonar_identity_sim.yaml
base=AUV-Master-Mag/results/v2/inspection_identity_stop_drift_20260927
for spec in dropout_zero:dropout:0:0 switch_zero:switch:0:0 switch_cross:switch:0.05:90; do
  IFS=: read -r name fault current direction <<< "$spec"
  python3 tools/smoke_v2_flythrough_ros_pvs.py --case wire --seed 101 \
    --config-file "$cfg" --sonar-identity-mode target --sonar-track-fault "$fault" \
    --with-bridge --current-speed-mps "$current" --current-direction-deg "$direction" \
    --post-hold-coast-s 60 --output-dir "$base/$name"
done
python3 tools/plot_v2_identity_stop_drift.py
```
