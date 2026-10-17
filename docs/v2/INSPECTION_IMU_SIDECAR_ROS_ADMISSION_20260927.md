# IMV1 侧帧进入 ROS 的隔离验证（2026-09-27）

本次只把[IMV1 候选协议](INSPECTION_IMU_SIDECAR_V1_CONTRACT_20260927.md)接到**默认关闭、回环网卡限定、被动**的 v2 ROS 影子链；现行 VxWorks 构建、旧 `$AUV` 协议及已发布的 FWHM/5600 次统计结果均未改变。入口为 `inspection_shadow_stack.launch.py` 的 `enable_imu_sidecar=false` 默认参数，候选桥配置为 `inspection_bridge_sidecar_candidate.yaml`。未选侧帧时沿用原桥行为。

选择 `inspection_sensor_source: imu_sidecar_candidate` 后，巡检桥继续发布 `$AUV` 深度等只读遥测，但停止向 `/auv/sensors/imu` 发布旧姿态，也停止发布**已用旧姿态旋转到世界坐标**的 DVL 速度。后者不能与侧帧姿态混合后假装同源。候选节点只监听 `127.0.0.1` 专用 UDP 端口，严格解码 48 字节 IMV1，检查 CRC、版本、有限数值、递增样本序号、原生 Timer 与 PC104 完整收帧时间及主机接收间隔。首帧仅建立时钟锚；第二条一致的新样本才可发布到 `/auv/sensors/imu_provenance_candidate`。断流后重新准入。ROS 消息标出未知加速度与姿态协方差；未指定姿态安装对齐时使用 `imu/gx3_unaligned` 帧名。

切断旧旋转 DVL 后，桥保留 `$AUV` 报文中**机体系**速度与 PC104 DVL BI 收帧时刻，在独立 `/auv/sensors/dvl_body_pc104_candidate` 上供侧帧节点配对。侧帧姿态与 DVL 必须共享板端时间基准：候选门要求上行内 DVL 样本龄不超过 0.50 s、DVL 时刻不晚于侧帧 IMU 且间隔不超过 0.15 s、该间隔内角速积分不超过 2°、速度模长不超过 2 m/s、映射到 ROS 后不超过 0.50 s，并禁止同一 DVL 收帧时间重复发布。通过后才用**侧帧四元数**旋转机体系速度，发布到 `/auv/sensors/dvl_provenance_candidate`。来源侧帧一旦因 Timer/序号等规则拒绝，缓存姿态立即失去配对资格；对应 ROS 负例先复现了“旧有效姿态仍授权新 DVL”，修复后新 DVL 不再发布。这些阈值是待实测整定的软件候选，不是实艇运动学包络。缺对齐角时不做世界系旋转；板端时间字段在旧 `$AUV` 中为有符号 31 位，回绕和设备重启须重新准入。代码见 `dvl_sidecar_join.py`，具体几何与拒绝情形见图 44。

![图44：旧包航向与侧帧航向相差 90° 时的 DVL 投影错配，及同一板端时间基准的受界配对](publication/fwhm_v2_experiment_results/figures/44_sidecar_dvl_source_match.png)

[图 44 输入与源码哈希](publication/fwhm_v2_experiment_results/data/44_sidecar_dvl_source_match_manifest.json)明确标记为构造的软件夹具。[独立保存的同次 ROS 原始 JSON](../../results/v2/inspection_imu_sidecar_dvl_join_20260927/fixture.json)记录旧包独发时 `IMU=0,DVL=0,depth=1`，匹配后的 7 帧侧帧 IMU、7 帧配对 DVL 和 8 帧深度，以及本次滤波末速度 `(0,0.300044,约0) m/s`；停 IMU 后观察窗无新旧定位。旧 `$AUV` 航向故意为 0°，侧帧夹具对齐后为 90°、机体系前向 DVL 为 0.3 m/s；进入 ROS 的世界系速度沿 Y 而非旧航向的 X，重复 DVL 不重发。这说明隔离后可恢复**软件意义上的同姿态源速度输入**，不能证明实物同步或航向标定。原始包可按 `source brain_linux/install/setup.bash && ROS_DOMAIN_ID=176 python3 tools/probe_v2_sidecar_dvl_join.py` 重新生成；它是一次合成运行，非统计置信区间。[五次顺序复跑汇总](../../results/v2/inspection_imu_sidecar_dvl_join_20260927/repeat_5.json)均得到 7 帧 IMU、7 帧配对 DVL 和停 IMU 后零条旧定位；这检验本地调度重复性，不是五个独立现场工况。

`/auv/sensors/imu` 的侧帧发布只允许显式开启**合成夹具**参数，并要求配置测试用 yaw 对齐角和方差。它不是实物开通选项。真实 GX3 的安装、yaw 偏置与物理协方差均未标定；侧帧亦不含加速度，不能直接宣称满足现行 ES-EKF 的实物输入条件。即使合成夹具打开，ROS 过程无法阻止仓库外另一个发布者向同名导航话题发消息，因此实物集成仍需独立系统级独占检查。

验证命令（使用已构建的 ROS Humble workspace，并将 `ROS_DOMAIN_ID` 设为未占用且小于 232 的值）：

```bash
source brain_linux/install/setup.bash
export PYTHONPATH="brain_linux/src/auv_cable_tracking_v2:brain_linux/src/auv_bridge:.:$PYTHONPATH"
ROS_DOMAIN_ID=175 python3 -m pytest -q \
  brain_linux/src/auv_cable_tracking_v2/test/test_imu_sidecar_gate.py \
  brain_linux/src/auv_cable_tracking_v2/test/test_dvl_sidecar_join.py \
  brain_linux/src/auv_cable_tracking_v2/test/test_imu_sidecar_ros.py \
  brain_linux/src/auv_cable_tracking_v2/test/test_imu_sidecar_bridge_exclusion.py \
  brain_linux/src/auv_bridge/test/test_protocol_imu_packet_time_gate.py \
  tests/test_imu_provenance_sidecar.py
```

结果：24 项通过。其中默认隔离模式的本地 ROS 回环用合成 `$AUV` 与 IMV1 同次注入，记录 `depth=1, legacy imu=0, legacy rotated dvl=0, sidecar candidate imu=1, navigation imu=0`；CRC 破坏、重放及冻结 Timer 不增加候选消息。另一个显式合成夹具把有效侧帧 IMU 与受界配对 DVL 送进现有 ES-EKF，并验证停止 IMU 后不再重发旧定位。门限测试覆盖 Timer/PC104 回绕、源序号复位、断流重新准入、DVL 过期、跨源错时、转动过快和越速。候选桥在打开 UDP 之前拒绝非被动、非 protocol UDP 或非回环配置。`auv_common` 与 v2 包在 `/tmp` 隔离目录完成 `colcon build`，`ros2 launch ... --show-args` 显示新增参数；使用默认桥配置开启侧帧的错误启动也被明确拒绝。仓库所在共享挂载不允许原位 `--symlink-install` 创建链接，因此构建使用 `/tmp/auv_v2_imu_sidecar_{build,install,log}`，没有据此修改已有构建目录。

边界仍是关键结论：接收端无法仅凭 `flags=1`、CRC 或回环合成数据证明 PC104 固件真的从 GX3 验过同一帧；首帧建立的跨机时间锚也不能识别两机同时延迟的历史流。实物侧帧未分配端口、未接入 VxWorks 构建、无真实串口重放、安装姿态和 IMU 物理协方差。旧旋转 DVL 始终暂停；只有受界的新配对支路可恢复速度，失配时巡检应保持 Hold。侧帧不含实测线加速度，现有 ES-EKF 在合成匀速夹具中的表现不能外推到实艇机动或长时间航位推算。现场许可仍关闭。
