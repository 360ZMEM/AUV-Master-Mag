# 分进程产品节点与实时 PVS 供数的短时活性边界

图 61 的完整合成路线把多个 ROS 节点放在同一加速夹具中，图 67 已发现观察回调排队。本补项改用**四个独立进程**：已安装的巡检算法、已安装的 PID 控制器、按墙钟节拍推进的原生 PVS 声磁供数夹具，以及只订阅的独立 DDS 记录器。算法和控制器保留 ROS 接口，供数夹具只用已收到的 Setpoint/PID 驱动本地协议编解码和 PVS。它没有运行实际巡检桥、PC104 或真实传感器，不能替代现场验收。

![图 68：分进程短试的接收空档与 PVS 夹具实际推进](publication/fwhm_v2_experiment_results/figures/68_paced_pvs_process_liveness.png)

| 独立短试 | 名义供数周期 | 磁测发布最大空档 | 独立磁测接收最大空档 | 独立 Setpoint/PID 接收最大空档 | 正向 Setpoint / PID / PVS 夹具推进 | 完整路线 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| `smoke_03_four_hz` | 0.25 s | 1.147 s | 1.280 s | 1.239 / 1.181 s | 11/37、20/80、6/40 | 未完成；未到 Confirm/Track |
| `smoke_04_ten_hz_final` | 0.10 s | 1.112 s | 1.334 s | 1.237 / 1.126 s | 5/50、9/156、1/80 | 未完成；未到 Confirm/Track |

两趟均为同一合成单导线和理想化声呐目标线索；除名义供数周期外，夹具源码和配置的记录哈希相同，但运行时调度并非受控配对，因此**不能将正向次数差异解释为“降到 4 Hz 改善可靠性”**。4 Hz 一趟的最长预定节拍滑移为 0.924 s，10 Hz 一趟为 5.979 s。两趟原始文件均通过[独立联表审计器](../../../tools/audit_v2_pvs_process_capture.py)的 `recording_reviewable=true`：导航、磁测、Setpoint、PID 的来源戳可映射到本次夹具时间表，无记录器丢条，最终 Setpoint 只见一个发布者；这个标志只证明短数据可审阅。它不表示健康节拍、任务完成或物理执行通过。

联表采用同一主机的单调钟。导航和磁测的发布调用前后时刻都保留；DDS 订阅可以在 `publish()` 返回前收到消息，因此只以**调用前**时刻作为延迟下界。Setpoint/PID 只是沿用了某个输入来源戳，不能仅凭同戳推断它们由这一条磁测产生，审计器只核对戳可映射，不计算虚假的逐消息因果时延。原记录器对 `/clock` 的 1–11 s 来源戳按 Unix 历元规则判为不可计算墙钟来源年龄，这一判定是正确的；本报告不能用夹具映射冒充跨设备采集时间。

图 68(a) 显示 4 Hz 短试中磁测与最终 Setpoint 的独立接收空档都超过既有短探针参考线 0.25/0.30 s；图 68(b) 显示 PVS 夹具只在 6/40 周期收到可用正向组合，其他周期的上游速度非正或输出接收超时都会使本地推进归零。图中转速是本地协议编解码后的**数字孪生参考**，没有经过实物桥或 PC104。图只作工程诊断与答问备份，不写成现场成功率或巡检精度图。

![图 69：同次 PVS 供数器与算法回调的双端节拍预算](publication/fwhm_v2_experiment_results/figures/69_pvs_process_timing_budget.png)

为分辨供数器本身与产品算法的长尾，另作一趟 `profile_07_ten_hz`：保持四进程和 0.10 s 名义供数，但算法改用只包裹已安装节点的计时入口，并在完整状态发布内启用 `cProfile`。本趟 60 个夹具周期只覆盖搜索开头，独立联表可审阅，磁测/最终 Setpoint 最大接收空档 1.075/1.098 s，本地 PVS 正向 0/60；**诊断开销已进入耗时，不能把它当生产版 WCET 或与上表配对求性能差**。包装器在启动前冻结并在结束后复核 9 个关键输入文件哈希，均未变化。

图 69(a) 联结每个夹具周期的工作耗时与**下一个**磁测发布时间间隔。59 个相邻间隔中，8 个超过 0.25 s；其中 6 个的前一周期工作已超过 0.25 s。供数器单周期工作耗时中位 0.089 s、最长 1.045 s；其内部磁测等发布阶段最长 0.640 s，原生 PVS 两步积分最长 0.496 s（该次线程 CPU 为 0.298 s），故至少部分空档由仿真供数进程自身解释。余下两次长空档不能仅凭这个关联归因，且本趟 PVS 与算法都运行在本机；目标实物平台无需同机运行 PVS。

图 69(b) 记录任务开始后的 16 次算法定时回调：中位 0.089 s、最长 0.558 s，回调起点最大间隔 1.092 s；核心步进最长 0.063 s，而完整状态发布最长 0.302 s。`cProfile` 对 15 次完整状态发布累计约 1.006 s，其中递归 JSON 安全转换累计约 0.453 s，多主题派生发布累计约 0.370 s；这些是嵌套累计耗时，**不能相加**。一趟未启用内部 `cProfile` 的计时包装短试 `trace_06_ten_hz` 也见回调最长 0.561 s、状态发布最长 0.284 s，但两趟运行时环境不受控，不能据此量化剖析器开销。此前[监测分频反事实](INSPECTION_MONITOR_PARTITION_LIMIT_20260928.md)在一趟健康供数达标的运行中仍未让最终 Setpoint 全部正向，故本报告不把“减少状态发布”提升为已经验证的产品修复。

首次开发短试 `smoke_01` 的供数夹具在节拍超时时不能持续处理输出订阅，因而**作废，不用于节点性能判断**。`smoke_02` 已改为专用接收线程，但接收年龄记录在单周期内仍采用较早的 tick 起点；其原始数据保留、可联表，不用于上表对照。`smoke_05_provenance` 是最后的 3 s 包装器烟测：运行前后算法/PID 入口脚本和已安装模块、供数器、记录器、配置等 8 个文件哈希一致；12 条源映射可复核，仍非任务完成。`03/04` 运行清单记录了夹具/记录器/配置与结果文件哈希，已安装产品模块哈希未在**运行前**冻结；它们的本机文件修改时间早于两趟运行，但本报告不把这一点夸大为完整软件物料清单。[原始包及各趟清单](../../results/v2/inspection_process_pvs_20260928/)保留此差异。

现在最有价值的下一步不再是本机反复微调阈值。既有加速全路线能证明有限状态机与合成任务推进，而本机分进程短试表明“持续健康输入输出”仍没有成立；继续放大为千秒全任务只会把这个已观察到的时序失败重复更久。需要在目标 Jetson 上、以与实际部署一致的独立进程和资源配置，完成一次完整 F 任务，并按[同次时序合同](INSPECTION_SAME_SESSION_TIMING_ENVELOPE_20260928.md)采集来源时效、最终 ROS/PID、桥 UDP、PC104 真正接收与艇体运动。目标身份和物理埋深也须由真实声呐/磁测/导航和标定独立证明。当前现场许可 `false`，没有据此冻结为可部署算法。

本机复核命令（需先 `source brain_linux/install/setup.bash`，选用隔离的 `ROS_DOMAIN_ID`，并设 `ROS_LOCALHOST_ONLY=1`）：

```bash
python3 tools/audit_v2_pvs_process_capture.py \
  --run-dir AUV-Master-Mag/results/v2/inspection_process_pvs_20260928/smoke_03_four_hz
python3 tools/audit_v2_pvs_process_capture.py \
  --run-dir AUV-Master-Mag/results/v2/inspection_process_pvs_20260928/smoke_04_ten_hz_final
python3 tools/plot_v2_pvs_process_liveness.py \
  --run-dir AUV-Master-Mag/results/v2/inspection_process_pvs_20260928/smoke_03_four_hz \
  --output-base AUV-Master-Mag/docs/v2/publication/fwhm_v2_experiment_results/figures/68_paced_pvs_process_liveness
python3 tools/audit_v2_pvs_process_capture.py \
  --run-dir AUV-Master-Mag/results/v2/inspection_process_pvs_20260928/profile_07_ten_hz
python3 tools/plot_v2_pvs_process_budget.py \
  --run-dir AUV-Master-Mag/results/v2/inspection_process_pvs_20260928/profile_07_ten_hz \
  --output-base AUV-Master-Mag/docs/v2/publication/fwhm_v2_experiment_results/figures/69_pvs_process_timing_budget
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tests/test_v2_pvs_process_capture.py
```

新实验可通过 `tools/run_v2_pvs_process_capture.py --output-dir <新目录> --max-time-s <秒> --period-s 0.1` 创建，包装器拒绝覆盖既有目录，结果中始终固定 `field_release_authorized=false`。退出码 0 只代表设定的短循环和记录器正常结束；应分别查 `software_route_complete`、输出空档和实际桥/PC104 证据。
