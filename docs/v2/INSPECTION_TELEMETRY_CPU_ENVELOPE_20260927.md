# 巡检决策回调的 CPU 包络与监控节流反例

本补项回答“独立 ROS 进程的 10 Hz 健康输出为何不稳定”。它延续[原墙钟短探针](../../results/v2/inspection_process_wallclock_20260927/README.md)，不改变磁场形、目标身份或既有完整任务结果。所有新试验只在本机以静止合成导航、磁测和协方差运行算法/PID 两个独立进程；没有桥、UDP、PVS、PC104 或实艇。

![CPU 占用与仍存在的算法输出空档](../../results/v2/inspection_process_cpu_wall_20260927/figures/inspection_callback_cpu_and_output_gap.png)

图 (a) 为同样五秒供数预热、CPU 3–5 亲和性下的**两次独立运行**，横轴是回调墙钟耗时，纵轴是同线程实际 CPU 耗时。新探针用 `time.thread_time()` 同时记录整个回调、算法步进、完整状态发布及基础监控发布，不把调度等待误算为计算。原频率中最长回调 0.465 s、CPU 0.458 s，基础发布最长 0.311 s；试验性 0.5 s 监控最短间隔中最长回调仍为 0.723 s、CPU 0.681 s。该反事实的回调中位耗时较低（0.043→0.019 s），但两次宿主机负载不同，而且首次节流试验没有逐周期更新全部 Hold 预算，**不能把差值当成安全优化收益**。带 cProfile 的另一次独立对照中，55 次基础发布累计 1.827 s，其中递归 JSON 安全转换 0.699 s、基础派生监控话题 0.281 s、诊断 JSON 发布 0.396 s；这些是该次运行的累积开销，不能逐项直接外推到最长回调。

图 (b) 为**第四次运行**：修正试验性节流，使 Hold 预算严格逐周期评估；允许 CPU 2–9 且关闭追踪开销。健康五秒内夹具输入最大发送缺口仅 0.233 s，32 条算法 Setpoint 却只有 2 条为正，最大输出空档 1.143 s，严格健康门失败。图只画夹具收到的消息点，阴影表示无新 Setpoint 的间隔，不把点间空档插值成连续命令。该次故障守卫窗 12/12 条 Setpoint 和 8/8 条 PID 努力为零；它说明故障后软件撤权，**不证明健康段持续运行**。

| 隔离运行 | 候选条件 | 健康正 Setpoint / 总数 | 夹具最大发送缺口 | 算法最大输出缺口 | 严格门 |
|---|---|---:|---:|---:|---|
| [原频率＋线程 CPU 记录](../../results/v2/inspection_process_cpu_wall_20260927/prewarm5_affinity_5_3_4/summary.json) | CPU 3–5；原逐周期监控 | 4/29 | 0.452 s | 1.116 s | 失败 |
| [早期节流＋线程 CPU 记录](../../results/v2/inspection_process_cpu_wall_20260927/telemetry05_5_3_4/summary.json) | CPU 3–5；监控 0.5 s，预算被连带降频 | 9/32 | 0.743 s | 1.053 s | 失败 |
| [早期节流＋八核允许](../../results/v2/inspection_process_cpu_wall_20260927/telemetry05_8cpu_5_3_4/summary.json) | CPU 2–9；预算被连带降频 | 29/32 | 0.262 s | 1.206 s | 失败 |
| [预算逐周期修正＋八核允许](../../results/v2/inspection_process_cpu_wall_20260927/telemetry05_budget_each_tick_8cpu_5_3_4/summary.json) | CPU 2–9；监控 0.5 s，预算逐周期 | 2/32 | 0.233 s | 1.143 s | 失败 |

试验性 `inspection_telemetry_period_s` 曾只在独立 v2 ROS 节点添加，未进入默认 launch。首次试验发现节流入口放在 Hold 预算评估之前，导致安全评估也降频；修正顺序后仍未通过连续性门。**该产品参数与节流逻辑现已撤回**，现行 v2 节点继续维持原监控、Setpoint 和安全预算语义。桥的独立 Setpoint 新鲜度门不受这些试验影响。早期 29/32 正 Setpoint 不能作为可用包络或性能改善引用。

诊断脚本、各次原始回调/消息、状态摘要和 [PNG/PDF 的 SHA-256 清单](../../results/v2/inspection_process_cpu_wall_20260927/inspection_callback_cpu_and_output_gap.json)可复核。早期节流源码 SHA-256 为 `429856402a319193b9ffad12732989fe4f62106d53ab714dfa4d537df9a35a59`。安全顺序修正候选撤回前未另存独立源码快照，因此该趟原始 CSV/JSON 只用于负结果，不作为可独立重建的发布版本。最终撤回版 v2 节点源码与安装文件 SHA-256 同为 `bccd3166e7d268e909ea6a80466a2cb7c774579540a79cac12531ee1e1c8d5e7`；候选期间 86/86 包测不跨版本引用。

下一步的工程决策不应是继续放宽导航时效阈值。先在目标 Jetson、实际导航频率与预计峰值并发负载下，分别记录采样发送、DDS 接收、算法开始/结束、Setpoint、PID、桥和板端时间；若同线程 CPU 长尾仍越过 0.5 s 门，则把监控序列化从控制回调中隔离，或压缩其逐周期内容，并对状态突变的即时发布与线程安全做同次回归。只有完整任务的健康输出时序、故障撤权时差和艇体滑行均通过，才能冻结实际运行包络。当前 `field_release_authorized=false`。

最终撤回版已普通复制式 `colcon build --packages-select auv_cable_tracking_v2` 成功。首次包级复跑为 83/84：既有 IMU sidecar 测试在 1 s 宽限后多收到一条排队导航；该项空闲单测 1/1 通过，随后独立整包复跑 **84/84 通过**。这项偶发性测试时序不证明导航产品故障，也不能被最后一轮绿灯抹去。新的 CPU 探针只增补诊断字段，不修改算法、PID、桥或原有 5600 次磁稳定性基线。

撤回版的隔离复核入口：

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  python3 -m pytest -q brain_linux/src/auv_cable_tracking_v2/test
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=219 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  taskset -c 3-5 python3 tools/smoke_v2_ros_process_wallclock.py \
  --healthy-s 5 --dropout-s 3 --recovery-s 4 --prewarm-s 5 \
  --trace-algorithm --output-dir AUV-Master-Mag/results/v2/inspection_process_cpu_wall_20260927/new_run
```

这条命令复核当前原频率诊断，不会重现已撤回的试验性节流版本；图所列节流数据按冻结原始记录和哈希审查。
