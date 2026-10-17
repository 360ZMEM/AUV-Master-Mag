# 巡检决策与 Setpoint 输出分进程候选：安全撤权通过，连续运行未通过

2026-09-28。此报告接续 [CPU 包络负证据](INSPECTION_TELEMETRY_CPU_ENVELOPE_20260927.md)。旧算法、旧结果和默认启动文件均未改；新增的双进程启动文件是**默认不启用的诊断候选**，不是实艇许可。

## 改动与时效契约

安装版 v2 算法通过 ROS remap 将原 `/auv/control/setpoint` 发布到 `/auv/control/setpoint_candidate`。独立的轻量进程以 10 Hz 发布原有接口 `/auv/control/setpoint`；正向输出要求候选接收年龄 ≤0.35 s，导航、磁场和 P15 协方差接收年龄均 ≤0.5 s，预算状态年龄 ≤1.5 s 且未曾报 `recovery_required=true`。异常、失鲜或显式 Hold 时发布零速 `CABLE_TRACKING_DEGRADED_HOLD`。正向重发保留算法候选的原始 header stamp，不能靠更新输出 stamp 掩盖来源年龄。候选数值范围复用巡检控制器的速度上限 0.8 m/s、深度 0–50 m。现有桥的 0.5 s Setpoint 看门狗仍独立生效；输出门进程停止时须由桥撤权。

入口：[新节点](../../../brain_linux/src/auv_cable_tracking_v2/auv_cable_tracking_v2/inspection_setpoint_supervisor_node.py)、[独立启动文件](../../../brain_linux/src/auv_cable_tracking_v2/launch/inspection_two_rate_nodes.launch.py)、[安装版多进程探针](../../../tools/smoke_v2_ros_process_wallclock.py)、[输出门单独 ROS smoke](../../../tools/smoke_v2_setpoint_supervisor.py)。启动命令示例：

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
ros2 launch auv_cable_tracking_v2 inspection_two_rate_nodes.launch.py config_file:=/absolute/reviewed/config.yaml
```

此启动文件只有算法、输出门、PID 控制器；`enable_passive_bridge:=true` 仅加入本地被动桥。它不加载定位、传感器、任务源或 PC104，不是完整任务启动入口。默认既有 launch 不使用此候选。

## 独立证据与结果

三项纯门槛测试通过；ROS 包回归 **87/87** 通过。最终安装版输出门的[严格独立 ROS smoke](../../results/v2/inspection_two_rate_process_20260928/supervisor_final_strict_domain218/)中，健康 3 s 夹具发送 24 组输入，收到 20 条正 Setpoint；最大输出空档 **0.472 s**。候选断流、传感器断流、预算锁存后各自 0.75 s 以后的稳定段均为零，恢复段 9/10 条为正。严格门同时要求健康段至少 24 条 Setpoint、全正且最大空档 ≤0.3 s；结果 `passed=false`。这项测试没有控制器、桥、植物或硬件。较早一次 smoke 以仅检查最大空档的过松标准得到 `passed=true`，但健康 3 s 仅 11 条输出；该布尔值已作废，不能作为 10 Hz 证据。

同版三进程算法／输出门／PID、10 Hz 静止合成供数的两次运行也未通过严格健康门：

| 运行 | 夹具健康段最大供数空档 | 最终 Setpoint 健康段 | 正向 Setpoint | 故障守卫窗 | 结论 |
|---|---:|---:|---:|---|---|
| 早期门，CPU 3–5，预热 5 s | 0.694 s | 26 | 13 | Setpoint 18/18、PID 7/7 为零 | 算法导航失鲜锁存；输出最大空档 1.177 s |
| 早期门，CPU 0–9，预热 5 s | 未进入健康段 | 0 | 0 | 无可用健康/故障阶段 | 首个正向 Setpoint／PID 前已导航失鲜；启动超时 |
| 协方差检查版，CPU 0–9，预热 5 s | 0.330 s | 36 | 6 | Setpoint 16/16、PID 13/13 为零 | 导航失鲜锁存；输出最大空档 0.806 s |

三次运行都没有足够证据归因到单一 CPU、DDS 或夹具问题；扩大 CPU 范围也没有恢复。第一趟夹具自己出现 0.694 s 供数空档，不能拿它计算算法单独的可靠性；最终一趟夹具最大空档 0.330 s 仍发生预算锁存和正向输出不足，已经足以否定这套参数下的连续运行主张。第三趟退出清理时监督进程遇到 ROS context 关闭竞态并返回 1，发生在记录完成后；随后源码仅增加关闭竞态处理，算法／门槛不变。全部 [原始事件与摘要](../../results/v2/inspection_two_rate_process_20260928/)均保留，不覆盖历史 5600 次磁方法实验或图 63 的既有结果。一次 `ROS_DOMAIN_ID=233` 启动被 Fast DDS 端口范围拒绝，仅留基础设施失败日志，不纳入算法统计。

原始 CSV／JSON 的 [SHA-256 清单](../../results/v2/inspection_two_rate_process_20260928/sha256_manifest.txt)用于核对。最终输出门源码与安装版 SHA-256 一致，为 `8ceebb75eddf1671630bbaa8e3059f7b7541c2d3fdbc7534ab4aa8b3a16217de`；独立 launch 源码与安装版 SHA-256 一致，为 `424c8467c0b109ad98ff442a309f03cfca29b9d83db0d72d2544c45bb05b6c0a`。最后一趟三进程使用关闭竞态处理前、协方差检查后的输出门源码；它的独立 SHA-256 为 `7a7c624eb2ad7ff930ffdef0acbb588e996089af753e32950f3b8470dfbf2fa5`，差异只在关停异常处理。

## 可声明边界与下一步

此候选证明了来源时效门与独立零速输出的逻辑；没有证明健康 10 Hz 指令连续、任务完成、PVS/UDP 执行、PC104 零推力或实艇停车。它也没有改变路径 1→3 中磁场形失配时拒绝埋深的独立许可。**默认产品启动不切换到双进程候选，现场许可保持 false。**

下一步只针对现存瓶颈：在隔离负载／目标 Jetson 上记录输入发布、DDS 到达、算法回调 CPU、最终输出及桥命令的同一墙钟时间轴；优先消除算法导航队列被长回调阻塞的来源，再复跑严格健康与故障门。若仍不能在声明的资源包络内满足 0.5 s 桥时效，缩小磁计算频率或路线速度须作为新的操作包络显式验收，不能靠重发旧候选提速。
