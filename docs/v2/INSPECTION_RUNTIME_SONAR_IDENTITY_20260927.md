# v2 目标缆身份：运行时声呐线索确认门控

此前的双源压力例把目标缆放在 `y=6 m`，在 `y=9 m` 加入半幅合成磁异常。宽地图区间 `[2,8] m` 下磁包络仍是单峰，磁中心约 `7.026 m`，任务错误宣布完成。此前的静态窄区间只能证明**独立线索存在时可以拒绝**，没有把线索接入 ROS 运行时。本增量将已有 `SonarCableObservation` 和 `PerceptionQuality` 两个接口接入独立的 v2 浅角度巡检节点，完成路径“独立识别 → 证据不足或冲突则拒绝”的可执行合同。旧 v1、默认 v2 配置和既有实验结果不变。

![图 29：运行时声呐身份门与零推进](publication/fwhm_v2_experiment_results/figures/29_runtime_sonar_identity_gate.png)

## 方法与许可边界

`v2.flythrough.require_runtime_identity_cue` 默认关闭。显式启用后，节点只接收同一声呐坐标系、与导航里程计时间戳相符且不超过 `0.5 s` 的原始声呐线与质量结果。质量须为 `VALID`、无原因码、已标定的 `p_track ≥ 0.9`、标定 ID 匹配且至少 8 点；原始线点必须与质量线的横偏和方向一致，并在声呐视域中实际跨过用于投影的最近位置，不能用远端短线段无界外推。配置提供声呐到艇体系杆臂和航向外参，结合导航位置、航向、声呐线拟合协方差和外参不确定度，得到世界 NED 中的目标横向中心与两倍标准差区间；半宽超过 `0.5 m`、航向偏离先验超过 `15°` 或横距超过 `30 m` 均拒绝。

Confirm 到达横切终点后，磁中心必须落入这条**新鲜独立线索**的误差区间才能进入 Track。声呐缺失、过期、不合格或磁声中心冲突时，`lateral_valid=false`，停在 Hold，目标任务不得报完成；埋深许可依然由原有场形/空间支持门独立决定，本实验全部不发布深度。非仿真入口没有启用运行时线索时，继续限制静态侧向区间宽度为候选 `1.0 m`；启用运行时线索后可用宽地图指引 Search，但必须在 Confirm 取得声呐许可。样例 `synthetic*` 标定 ID 不能在非仿真入口启用。

| 固定配方 | 磁中心 / m | Confirm 后 | 末周期 PID / UDP | 埋深 |
| --- | ---: | --- | --- | --- |
| 单源 + 合格合成声呐线 | 6.014 | 进入 Track，完成 80 m | 0 / 0（终点） | 不发布 |
| 双源 + 指向目标的合格合成声呐线 | 7.026 | `runtime_identity_cue_inconsistent`，未进 Track | 0 / 0 | 不发布 |
| 单源 + 声呐缺失 | 6.014 | `runtime_identity_cue_unavailable`，未进 Track | 0 / 0 | 不发布 |
| 单源 + 未标定概率 | 6.014 | `sonar_identity_quality_unqualified` → 无许可，未进 Track | 0 / 0 | 不发布 |

以上均使用种子 101、零流、预对准直线先验和原生 PVS，经 ROS DDS、PID、自治守卫、本地 UDP 桥闭环；双源和两个证据缺失/无效例均在约 `577 s` 的确认终点拒绝。单源对照在 `1105.25 s` 完成。双源拒绝结果另在种子 102、103 的同配方直接编码链复现，但那两次是在最后补充声呐线角度协方差校验前运行，**只作为辅助一致性观察，不与最终版闭环结果混合统计**。图 29 仅用最终版种子 101 与冻结的原始宽地图错误完成例。

## 证据及复现

- [图 29 逐例摘要](publication/fwhm_v2_experiment_results/data/29_runtime_sonar_identity_gate_summary.json)记录原始 CSV、JSON 和各自 SHA-256；[图像/脚本清单](publication/fwhm_v2_experiment_results/data/29_runtime_sonar_identity_gate_manifest.json)记录出图文件哈希。
- 另有隔离 DDS 单测使**真实 `PerceptionQualityNode` 软件**读取一份声明为 `synthetic`、不获物理部署批准的测试概率曲线，从合成声呐原始点计算质量，再由 v2 ROS 节点订阅并转换为约 `y=6 m` 的运行时线索；这一检查覆盖软件消息链，仍不涉及物理声呐。可运行 `brain_linux/src/auv_cable_tracking_v2/test/test_identity_cue_ros.py`。
- 仿真配置：[合成声呐身份门 YAML](../../../brain_linux/src/auv_cable_tracking_v2/config/cable_tracking_v2_flythrough_sonar_identity_sim.yaml)。算法转换：[identity_cue.py](../../../brain_linux/src/auv_cable_tracking_v2/auv_cable_tracking_v2/identity_cue.py)；ROS 订阅：[cable_tracking_v2_node.py](../../../brain_linux/src/auv_cable_tracking_v2/auv_cable_tracking_v2/cable_tracking_v2_node.py)；确认门：[flythrough.py](../../src/auv_mag_tracking/v2/flythrough.py)。
- 运行示例：

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=224
python3 tools/smoke_v2_flythrough_ros_pvs.py --case wire --seed 101 \
  --config-file brain_linux/src/auv_cable_tracking_v2/config/cable_tracking_v2_flythrough_sonar_identity_sim.yaml \
  --sonar-identity-mode target --secondary-cable-lateral-m 9 \
  --secondary-cable-gain-nt 10 --with-bridge \
  --output-dir AUV-Master-Mag/results/v2/inspection_sonar_identity_gate_20260927/recheck
python3 tools/plot_v2_runtime_sonar_identity.py
```

这里的“合格声呐”是脚本**直接注入的合成线点和合成已标定概率**，未经过物理声呐探头，也没有验证上游探测器及概率标定。`p_track` 衡量线拟合/跟踪质量，**不是“这条线属于目标缆”的身份概率**；若声呐本身锁到邻缆，声磁几何一致也可能共同认错。因此上游仍须由可信地图、已确认上一段航迹或人工复核把声呐线关联到目标。双磁源是标量包络叠加，不是多芯铠装海缆实测电磁场。当前声呐桥还可能把**消息到达时间**当作采集时间；现场必须取得可靠物理采集时间戳，验证声呐外参与误差上界，完成真实数据独立留出校准、并行缆/误检测试和 PC104 停机距离验收。图 29 证明运行时门控逻辑及所列合成场景的安全拒绝，不是实物目标身份验收。若现场不能满足这些证据条件，应保留拒绝策略，不发出目标缆巡检完成结论。

后续[Track 身份连续性补项](INSPECTION_IDENTITY_CONTINUITY_20260927.md)在进入跟踪后持续检查线索，并用三个固定噪声种子复核两类正常和两类故障；图 29 仍是 Confirm-only 冻结结果。非仿真入口后来增加[物理标定文件及哈希预检](INSPECTION_FIELD_INPUT_READINESS_20260927.md)，该防误用改动不追溯改变本图的代码哈希或合成证据等级。
