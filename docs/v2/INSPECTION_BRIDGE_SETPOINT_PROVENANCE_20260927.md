# 巡检桥的上游 Setpoint 新鲜度门（候选）

本补项处理**算法决策间歇卡住、PID 却继续发布新鲜正命令**的层间错配。旧桥只核对 PID/MPC 命令本身的 0.5 s 时效与巡检预算状态；专用 PID 对 Setpoint 和导航的时效为 1.5 s，因此旧 Setpoint 可在算法暂停后继续驱动新 PID 命令。它不改变磁搜索、场形判别、FWHM、目标关联或历史 5600 次稳定性实验。

![旧短探针的可观测传播尾时差与新桥的独立 UDP 对照](../../results/v2/inspection_bridge_setpoint_gate_20260927/figures/bridge_setpoint_provenance_gate.png)

图 (a) 来自**未接桥**的[独立进程短探针与逐事件审计](../../results/v2/inspection_process_wallclock_20260927/observable_command_tail_audit.json)：最后正向 Setpoint 到夹具收到首条零 Setpoint 为 1.013 s，到首条零 PID 为 1.188 s；中间还收到三条正 PID，其中两条晚于夹具收到零 Setpoint。该接收顺序包含 DDS 投递与回调排队，不能推断控制器收到零指令后仍计算正推力，更不能推断 PC104 或实艇推力。只把 PID 输入超时改为 0.5 s 的另一次本机短探针仍未通过连续性门，也没有建立硬实时输出上界。

图 (b) 是**另一条独立回环**：新安装版巡检桥在每次自主决策及命令保活时，除原有预算门，还核对最新 Setpoint 的单调墙钟接收年龄不超过 0.5 s、模式为 `CABLE_TRACKING`、`track_cable=true`、目标速度正且关键数值有限。只要 Setpoint 失鲜或显式 Hold，即使 PID/MPC 消息仍新鲜且命令为正，也把自治下行执行量置零，并在状态中给出 `inspection_setpoint_stale` 或 `inspection_setpoint_hold`。预算锁存优先于此门；遥控决策不经过这项自治拒止。该改动只在独立 `auv_cable_tracking_v2` 巡检桥生效，不修改原上位机/主桥。

| 安装版本地 UDP 阶段 | 输入 PID 推力 | 下行主推 | 桥原因 |
|---|---:|---:|---|
| 新鲜 Setpoint、新鲜 PID | 25% | 25% | 无 |
| Setpoint 超过 0.5 s、仍送新鲜正 PID | 25% | 0% | `inspection_setpoint_stale` |
| 上述失鲜条件下命令保活重发 | 缓存正 PID 仍在有效期 | 0% | `inspection_setpoint_stale` |
| 新鲜显式 Hold、仍送正 PID | 25% | 0% | `inspection_setpoint_hold` |

[五帧原始协议包](../../results/v2/inspection_bridge_setpoint_gate_20260927/loopback_b/packets.json)、[摘要与源码哈希](../../results/v2/inspection_bridge_setpoint_gate_20260927/loopback_b/summary.json)、[绘图哈希清单](../../results/v2/inspection_bridge_setpoint_gate_20260927/bridge_setpoint_provenance_gate.json)可复核。安装版桥文件与源码 SHA-256 同为 `097a4babd9eea5203aa8571d0803e4cd94406858dabecc19bc167f0ce4ec8008`；ROS 包级回归 `84/84` 通过。首次尝试 `--symlink-install` 因共享文件系统不允许建立符号链接而失败，随后普通复制式 colcon 构建成功；该构建错误不作为产品运行崩溃。

另尝试将安装版节点接入 ROS/PID/桥/本地 UDP/PVS 的 5 s 搜索起始段。首次未加载 `brain_linux/install/setup.bash`，缺少 `auv_interfaces`，属启动环境错误；加载后算法、PID 与桥均启动，但在等待算法一次 Setpoint、横向与深度话题齐备时触发 10 s 的 `DDS delivery timeout`，未产生完整结果文件，**不能计为同次链路通过**。一条相同配置但由夹具加载源码节点的独立短运行留下了[原始摘要](../../results/v2/inspection_bridge_setpoint_gate_20260927/integrated_short_5s_configured/wire_seed_101_summary.json)：20 周期内因导航不可用进入 `RECOVERY_REQUIRED`，随后 UDP 无非零主推；未进入 Confirm/Track。它只能说明当前主机负载下的保守失败，不能替代安装版持续时序验收，也不能归因为新 Setpoint 门单独引起。

复核入口：

```bash
source brain_linux/install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  python3 -m pytest -q brain_linux/src/auv_cable_tracking_v2/test
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=213 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  python3 tools/smoke_v2_bridge_setpoint_gate.py \
  --output-dir AUV-Master-Mag/results/v2/inspection_bridge_setpoint_gate_20260927/new_run
python3 tools/plot_v2_bridge_setpoint_gate.py
```

**适用边界：** 新门证明的是本地桥在收到相应输入后不会仅凭“新鲜 PID”继续授权旧决策。它不使算法在本机健康段持续通过时序门，也不证明 0.5 s 是实际下行或板端零执行量的最坏时延；桥自身可能受调度影响。新桥尚未在同一次**算法→PID→守卫→UDP→PC104→实艇**失鲜试验中验收，现场许可仍为 `false`。此门与路径 1→3 的场形／埋深拒绝是两个独立层级，不能用执行桥证据证明磁反演正确。答辩可将其表述为“控制输出现在核对上游决策来源的时效，避免只因 PID 持续发包就误认算法仍在工作”；不可表述为“任意负载下即时停车”。
