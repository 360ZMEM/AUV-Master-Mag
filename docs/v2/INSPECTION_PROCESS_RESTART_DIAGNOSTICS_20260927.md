# 巡检 v2 独立进程任务与故障输出复核

为核对当前源码在独立启动下的输出合同，使用原生 PVS、合成单直导线、脚本声呐线索与 AMD 健康上行夹具，分别启动 ROS2 巡检节点、PID、守卫、本地 UDP 桥四次。每趟使用隔离的 ROS 域与独立结果目录；保存的 CSV 和摘要由[独立审计](publication/fwhm_v2_experiment_results/data/inspection_v2_process_restart_audit_20260927.json)按源码、安装空间、文件哈希和逐周期结果核查。审计本身不能认证进程隔离，复现命令中的不同域号才是运行安排。**四趟不是单进程连续耐久试验，也没有连接物理 PC104 或实艇。**

| 独立运行 | 合成时间/周期 | 可复核结果 |
|---|---:|---|
| 正常，种子 104 | 1105.25 s / 4418 | 完成 80 m 正向沿缆段；路由观测横偏 MAE 0.758 m，最大 1.338 m；深度为空 |
| 正常复跑，种子 107 | 1105.25 s / 4418 | 同样完成；MAE 0.757 m，最大 1.337 m；深度为空 |
| 上行失鲜，种子 105 | 645.00 s / 2577 | 桥在故障后撤权，241/241 周期 PID 与 UDP 主推零；不报完成或深度 |
| 跟踪线索改指，种子 106 | 645.00 s / 2577 | 585.00 s 进入 Hold，此后 60 s 主推零；PVS 仍位移 4.646 m；不报完成或深度 |

![图 37：独立进程正常任务、沿缆横偏、上行撤权及零命令后的运动](publication/fwhm_v2_experiment_results/figures/37_inspection_process_restart_diagnostics.png)

图 (a) 展示两次任务起航与接入几何；图 (b) **只取** CSV `stage=inspect_along_cable` 且 `route_observed_lateral_samples>0` 的正向沿缆样本，与报告中的 1 m 均值、2 m 最大横偏示例门同口径。若把此前返回巡检起点的入线段混入，会画出明显超过 2 m 的横移，却不能据此反驳这里的正向路由指标。图 (c) 的零值是本地协议命令，图 (d) 同时给出零命令和非零滑行；它们刻意不使用“已停车”的表述。

运行时先加载 `/opt/ros/humble/setup.bash` 与 `brain_linux/install/setup.bash`，设 `ROS_LOCALHOST_ONLY=1` 和互不相同的 `ROS_DOMAIN_ID`（四趟分别使用 177、180、178、179），然后以[同一仿真巡检配置](../../../brain_linux/src/auv_cable_tracking_v2/config/cable_tracking_v2_flythrough_sonar_identity_corridor_budget_sim.yaml)执行[现有闭环脚本](../../../tools/smoke_v2_flythrough_ros_pvs.py)：两次正常用 `--case wire --seed 104/107 --with-bridge --sonar-identity-mode target --max-time-s 1200`；失鲜用 `--case wire_guard_telemetry_loss --seed 105 --max-time-s 800`；改指用 `--case wire --seed 106 --sonar-track-fault switch --post-hold-coast-s 60 --max-time-s 1000`，后三趟同样开启桥和目标声呐夹具。各自传入独立 `--output-dir`，不要覆盖既有历史结果。

复核命令：

```bash
python3 tools/audit_v2_inspection_process_restarts.py
python3 tools/plot_v2_inspection_process_restarts.py
```

[图像与数据清单](publication/fwhm_v2_experiment_results/data/37_inspection_process_restart_diagnostics_manifest.json)固定绘图脚本及 PNG/PDF 哈希。[审计结果](publication/fwhm_v2_experiment_results/data/inspection_v2_process_restart_audit_20260927.json)的 `checks_passed=true` 仅表示所列合成运行达到预设的任务与故障输出断言，`field_release_authorized=false`。它不能估计现场成功率，不能证明超过观察窗口仍稳定，也不能替代真实目标身份、PC104 收帧、推进器响应、边界净距或实艇滑行包络。答辩可据此说“在明确合成包络中，独立启动后正常任务可重复完成，两个故障各自撤销推进且持续报告状态”；实艇版仍按[现场冻结矩阵](INSPECTION_V2_FREEZE_AND_FIELD_GATE_20260927.md)验收。
