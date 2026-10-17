# 加速飞越夹具的同次 DDS 接收与回调等待边界

为把[长任务轨迹审计](INSPECTION_TASK_LIVENESS_AUDIT_20260928.md)与[独立 ROS 接收采证](INSPECTION_ROS_RECEIPT_RECORDER_20260928.md)放入**同一次**运行，本次给原生 PVS 飞越夹具增加默认关闭的仿真时间戳—本机单调钟发布时间旁路，并另起只订阅的 DDS 记录进程。结果是一个有价值的**负向时序诊断**：数据可联表，短运行却无法稳定满足健康输出节拍；完整任务时序与现场许可均未通过。历史图 61 的八份轨迹和产品节点没有被重写。

![图 67：独立 DDS 接收与同进程飞越夹具回调等待](publication/fwhm_v2_experiment_results/figures/67_same_session_flythrough_receipt_gap.png)

| 独立运行 | 记录范围与结果 | 独立接收关键量 | 解释 |
| --- | --- | --- | --- |
| 01 | 首周期桥拒止；任务中断 | Setpoint 和 PID 均收到；原始仿真戳不可按墙钟解释 | 桥原因码 `inspection_setpoint_stale`，UDP 零主推，不能写成正向完成 |
| 02 | 4 周期，4 次本地 UDP 正主推；尚在 Search | Setpoint/PID 最大接收空档 1.803/1.806 s；发布时间到接收最大 0.323/0.444 s | 独立订阅无丢条，但不满足 0.30 s 的短探针健康参考门 |
| 03 | 首周期桥拒止；任务中断 | 独立 PID 在磁测发布后 0.298 s 收到；夹具内观察 PID 晚于独立接收 1.048 s | PID 计算段约 0.028 s，主要长尾在同进程观察回调等待；桥因失鲜零主推 |
| 04 | 4 周期，4 次本地 UDP 正主推；尚在 Search | Setpoint/PID 最大接收空档 1.656/1.689 s | 五次原生 PVS 积分最长约 0.099 s；状态/算法回调等待段出现约 0.68–1.15 s 长尾 |
| 05 | 试验性四线程夹具在第 2 周期桥拒止 | 第 2 周期 PID 独立接收延迟达 1.401 s | 多线程反事实未恢复连续性；开关已撤回，不进入产品或图 61 配方 |
| 06 | 不启独立记录器，4 周期均本地 UDP 正主推；尚在 Search | 无独立 DDS 接收表 | 单周期墙钟耗时仍最高 1.478 s，观测进程不是长空档的必要条件 |

`02`、`03`、`04`、`05` 的[逐趟原始目录](../../results/v2/inspection_same_session_full_20260928/)均有单次会话清单、运行日志、夹具发布时间表、阶段时间表、独立 Setpoint/PID/导航/磁测接收表及逐文件 SHA-256；[联表审计器](../../../tools/audit_v2_flythrough_receipt_capture.py)复核仿真戳与夹具发布时间的唯一对应。`02`、`03`、`04` 的联表 `recording_reviewable=true` 只说明这些**局部短数据**可核对，不代表任务完成或时序通过。原接收器要求 Unix 历元来源戳，因此对于 `/clock` 的 1.0、1.25 s 直接标 `data_reviewable=false` 是正确行为；本文只用夹具发布时采下的同机单调时刻计算“夹具发布到独立接收”的时间，不把它冒充真实传感器采集年龄或跨设备对时。

这些运行在逐步增加阶段仪表的过程中保存，`02` 与 `03/04` 的夹具源码并非逐字相同；`05` 的试验性多线程开关已撤回，也没有冻结独立源码快照。因此表中各值仅作**逐例故障定位**，不作仪表开销、多线程收益或算法性能的配对统计。`06` 的无记录器对照同样只证明长尾不以记录器为必要条件，不给出记录器的因果效应大小。

图 67(a) 直接显示在 `03` 的首周期中，独立订阅进程已经收到 PID，而共享单线程执行器中的观察端仍等待约 1 秒；图 67(b) 显示另外两趟四周期短试的独立接收间隔约 1.7–1.8 s。结合 `04` 的 PVS 积分上界及 `06` 无记录器对照，可以排除“单纯把 Python PID 计算优化到更快”或“只移除观测器”就会稳定满足健康门这两种解释。更符合数据的是加速、同进程夹具中 `/clock`、状态发布和多个 ROS 节点共用执行器导致的阶段性排队；这仍是**根据这些短探针作出的推断**，不能量化目标 Jetson 的最坏调度时延。

因此，图 61 可继续支持“受限合成场景下能完成有限路线、异常时能撤权”的**方法与状态机证据**，不能补充为“同版目标平台完整任务持续 10 Hz 输入输出”。这次试验也没有物理声呐、磁测、导航、PC104 或实艇运动。现场冻结仍须在目标计算平台上用**分进程产品拓扑**完成一次完整 F 任务，按[同次时序合同](INSPECTION_SAME_SESSION_TIMING_ENVELOPE_20260928.md)记录健康输出最大空档、消息来源时效、桥/UDP/PC104 及故障后持续零命令，同时独立核验真实目标和艇体滑行。若该包失败，不能用本机图 61 或回顾性调门替代。现场许可 `false`。

从仓库根目录复核数据联表与出图：

```bash
python3 tools/audit_v2_flythrough_receipt_capture.py \
  --run-dir AUV-Master-Mag/results/v2/inspection_same_session_full_20260928/smoke_pvs_step_03
python3 tools/plot_v2_flythrough_receipt_gap.py \
  --run AUV-Master-Mag/results/v2/inspection_same_session_full_20260928/smoke_stage_timing_02 \
  --run AUV-Master-Mag/results/v2/inspection_same_session_full_20260928/smoke_pvs_step_03 \
  --run AUV-Master-Mag/results/v2/inspection_same_session_full_20260928/smoke_pvs_step_04 \
  --output-base AUV-Master-Mag/docs/v2/publication/fwhm_v2_experiment_results/figures/67_same_session_flythrough_receipt_gap
```

新增的[联采包装器](../../../tools/run_v2_flythrough_receipt_capture.py)要求先 `source brain_linux/install/setup.bash` 并设置独立 `ROS_DOMAIN_ID` 和 `ROS_LOCALHOST_ONLY=1`；它强制新结果目录，不覆盖历史运行。其默认飞越时长是试验上限而非已通过项。不要将本报告四周期的 `runner_returncode=0` 解释为 80 m 巡检成功：该值只表示设定的短循环正常结束。
