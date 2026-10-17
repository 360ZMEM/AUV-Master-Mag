# 巡检 v2 同次监督门—DDS 接收—PID 来源时效诊断

**结论。** 在现有固定供数墙钟短探针上，同时接入监督门逐周期追踪和[独立 ROS 接收端记录器](INSPECTION_ROS_RECEIPT_RECORDER_20260928.md)。本机合成健康段的三趟低速运行均未通过原严格总门；即使监督门当时决定正向，Setpoint 到达独立订阅者时仍可超过声明的 1.3 s 候选来源年龄。诊断确认了“门内新鲜、接收端已旧”的风险，并提供以后目标平台同次采证路径；它不构成真实巡检连续性或实艇安全验收。

![同次供数、候选、门决定、DDS 接收和 PID，以及正向 Setpoint 来源年龄](../../results/v2/inspection_same_session_ros_timing_20260928/figures/same_session_receipt_timeline.png)

图对应第二趟，采用论文 Fandol 宋体样式。横轴均为固定供数任务健康窗起算的本机单调秒；橙色背景是主动断流。上半图是各层真实观察事件，蓝线表示正向、橙线表示零输出；下半图只画**正向最终 Setpoint**在独立 DDS 接收端的原始候选来源年龄。四个橙色菱形超过 1.3 s 限值。图中没有路线运动、目标身份或板端执行；[PDF](../../results/v2/inspection_same_session_ros_timing_20260928/figures/same_session_receipt_timeline.pdf)供论文/PPT 放大查看。

图已进入 thuthesis 第 5 章图 5.66，并作为 PPT 可靠性追问备份。最终 `latexmk -xelatex` 构建成功，235 页；图题与图体在印刷页 178（PDF 物理页 200）同页，交叉引用已解析。

| 同配置隔离运行 | 健康供数最大缺口 | 正向/全部最终 Setpoint | 最大最终输出空档 | 接收时已旧的正向消息 | 故障守卫窗零输出 | 严格总门 |
|---|---:|---:|---:|---:|---:|---|
| [第二趟](../../results/v2/inspection_same_session_ros_timing_20260928/fixed_feeder_trial2/summary.json)，PID 来源门关闭 | 0.263 s | 23/35 | 0.745 s | 4/23，最高 1.849 s | Setpoint 13/13；PID 5/5 | 失败 |
| [第三趟](../../results/v2/inspection_same_session_ros_timing_20260928/fixed_feeder_trial3/summary.json)，PID 来源门关闭 | 0.291 s | 13/34 | 0.532 s | 2/13，最高 2.020 s | Setpoint 22/23；PID 12/12 | 失败 |
| [第四趟](../../results/v2/inspection_same_session_ros_timing_20260928/fixed_feeder_pid_source_trial4/summary.json)，PID 来源门 1.3 s | 0.395 s | 29/34 | 0.528 s | 4/29，最高 1.604 s | Setpoint 15/15；PID 7/7 | 失败 |

三趟均固定算法 CPU 0–3、监督门 4、PID 5、供数 6、观察者与记录器共享 7；10 Hz 静止合成导航/磁测、0.2 m/s 任务、0.6 s 传感器门、1.3 s 候选门及相同[低速配置](../../results/v2/inspection_low_speed_envelope_20260928/low_speed_sim_config.yaml)。算法开启决策快照/可恢复输入，监督门开启独立输出线程。记录器和监督门追踪增加了诊断负载；三趟健康供数缺口均超原 0.25 s 门，因而**不能**将健康正向比例差异归因为 PID 来源门，也不能以本机运行证明目标 Jetson 的最坏时延。三次 `capture_complete=true` 仅证明记录器内部写入完成；数据仍是合成场形与合成供数。

第三趟还提供一条值得保留的故障反例：断流后 0.313 s 的最后正向监督决定在独立记录器于 0.668 s 收到，来源年龄为 1.769 s；另一观察者直到 0.848 s 才收到同为正向的 Setpoint，越过既定 0.75 s 守卫窗。0.853 s 的监督门决定已经是零速。这是**两个接收端看到的事件顺序**，不能凭近邻时间强行认定它们是同一个 DDS 包；原始 header、note 和时序使“晚到正向消息”成为可检查的解释。该趟 PID 守卫窗 12/12 为零，仍不能从 PID 零推力推断板端收零或载体停住。

第四趟显式开启候选 PID 的正向 Setpoint 来源门。接收端 PID 表中有 9 条 `inspection_setpoint_source_stale`，推进命令均为零；同次健康段依然有 4 条正向最终 Setpoint 在独立接收端超过 1.3 s。本探针为通过仿真身份门而使用 `use_sim_time=true` 与系统历元 `/clock`，PID 计算来源年龄时读取的是其 ROS 时钟，可能滞后于独立墙钟观察。因此这 9 条只能证明来源门**实际被触发**，不能说明每条晚到消息都被该门一一阻断。真实候选运行必须 `use_sim_time=false` 并在 PID/桥接收端分别审计来源时效，且需要消息因果序号或原始包逐条回看。

整包 `data_reviewable=false` 的具体原因也已分离：合成启动预热时，监督门的零速 Hold 在 `/clock` 尚未就绪的周期使用零 header；第二、三、四趟分别有 119、106、131 条此类零速记录。**正向** Setpoint 的 header 均为系统历元：第三、四趟新 `positive_setpoint_source_reviewable=true`，第二趟旧记录器版本的 55 条全程正向原始行中零条历元异常。因此可以分析正向来源年龄，但不得把整包改写为通过；现场非仿真时钟与零速 header 仍须另验。

结果目录的[证据索引](../../results/v2/inspection_same_session_ros_timing_20260928/evidence_index.json)列出每趟固定供数事件、监督门周期、独立接收 CSV、PID、摘要和图的 SHA-256。先前[第一趟](../../results/v2/inspection_same_session_ros_timing_20260928/trial1/summary.json)用旧不可恢复配置，任务刚接入时导航年龄约 0.84 s 即锁存 `hold_navigation_unavailable`，363 条最终 Setpoint 和 248 条 PID 均为零；它没有正向健康段，**不纳入上表方法比较**。历史记录不覆盖。

本次最有价值的工程结论是把来源时效与输出连续性分别裁决。监督门判断候选何时可发送；PID 和桥应基于各自**接收时**的原始候选时间再检查；接收端记录器则独立给出外部观察时间。可恢复 Hold 能维持有界失鲜处理，但不能替代持续任务进展或零推力后的运动包络。下一阶段应在目标平台同一完整任务同时采集真实导航/磁测、门周期、ROS/PID、UDP `$CKTH`、PC104 板端接收和艇体运动，以[同次时序合同](INSPECTION_SAME_SESSION_TIMING_ENVELOPE_20260928.md)及目标身份/边界清单裁决。当前 F 剖面仍为候选，现场许可 `false`。

复现第二/三趟的配方（使用新的输出目录和未占用的有效 ROS domain）：

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=139 OPENBLAS_NUM_THREADS=1 \
OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
python3 -B tools/smoke_v2_fixed_feeder_process.py \
  --output-dir /path/to/new_run \
  --config-file AUV-Master-Mag/results/v2/inspection_low_speed_envelope_20260928/low_speed_sim_config.yaml \
  --wall-clock-fixture --snapshot-freshness --recoverable-input-stale \
  --output-thread --mission-speed-mps 0.2 --candidate-timeout-s 1.3 \
  --sensor-timeout-s 0.6 --displacement-bound-m 0.36 \
  --pin-processes --trace-supervisor --record-ros-receipts --startup-timeout-s 70
```

第四趟在此命令上额外加 `--controller-source-timeout-s 1.3`。图由 `tools/plot_inspection_same_session_receipts.py --run-dir /path/to/new_run --out-base /path/to/new_figure` 生成。此命令仅启动合成固定供数、算法、监督门、PID 和记录器；不启用桥、PC104 或车辆。
