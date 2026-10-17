# IMU 来源侧帧 v1：隔离候选协议合同

旧 `$AUV` 145 字节上行继续承载控制相关遥测、DVL、PC104 组包时间和历史扩展；直接复用其参数字段传递 IMU Timer 会与既有语义冲突。因此新增**独立 48 字节候选侧帧**，尚未分配实物端口或接入现行固件。其目的不是增加新的姿态估计算法，而是让同一条经校验的 `0xCF` 传感器应答在 PC104 与 Jetson 之间保留可追踪的时间身份。

| 字节偏移 | 长度 | 字段 | 语义 |
|---:|---:|---|---|
| 0 | 4 | `IMV1` | 类型与版本魔数 |
| 4 | 1 | `version=1` | 拒绝未知版本 |
| 5 | 1 | `flags=1` | PC104 声明已验完整 `0xCF` 帧；接收端仍须审查固件来源 |
| 6 | 2 | `frame_checksum_u16` | 原始 IMU 应答的 16 位校验和，便于与原始串口日志配对 |
| 8 | 4 | `sample_sequence` | PC104 只在新、完整、校验正确的响应上递增 |
| 12 | 4 | `sensor_timer_ticks` | 响应内传感器原生 Timer，单位为 1/62500 s |
| 16 | 4 | `pc104_receive_uptime_ms` | PC104 完整收帧时刻，不是 `$AUV` 组包时刻 |
| 20 | 24 | 6 个 `float32` | 同帧 roll/pitch/yaw（rad）及三轴角速度（rad/s） |
| 44 | 4 | CRC32 | 覆盖偏移 0--43，网络字节序 |

所有多字节字段采用大端。C99 [固定帧校验/Timer 候选](../../../csd_vx6.8_lastest/inspection_v2_candidate/gx3_cf_guard.c)从任意分段串口数据中只输出完整有效的新样本；[C 编码器](../../../csd_vx6.8_lastest/inspection_v2_candidate/imu_provenance_sidecar_v1.c)与[Python 解码器](../../../common/imu_provenance_sidecar.py)已用同一条黄金向量按**逐字节完全相等**验证，长度、CRC、版本、有效标志、有限数值和 Timer 回绕/重放另有负例。源码和主机测试不会自动进入 VxWorks 构建。

主机复核可运行 `python3 -m pytest -q tests/test_gx3_cf_guard_c.py tests/test_imu_provenance_sidecar.py tests/test_gx3_cf_time_provenance.py`；这些测试会调用本机 C 编译器，不需要或假装使用 VxWorks 工具链。项目 `Makefile.mk` 的 `PRJ_OBJS` 和 `vx6.8_lastest.wpj` 源码列表均无本候选目录，因此旧固件镜像不会因新增文件而改变。实际接入时必须先明确 PC104 固件版本、侧帧专用端口/网络路径和 ROS 互斥输入配置，再针对这些配置做本版同次链验收。

Jetson 已加入[隔离的 ROS 候选入口与回环验证](INSPECTION_IMU_SIDECAR_ROS_ADMISSION_20260927.md)：只在显式选择 v1 来源时接收侧帧；拒绝未知版本、损坏 CRC、重复/倒退的 `sample_sequence` 或 Timer、过期的 PC104 完整收帧时间，以及与同一任务时间基准不一致的记录。选择侧帧后，旧 `$AUV` 姿态与依赖其姿态旋转的 DVL 均不进入定位话题；侧帧默认也**不**进入定位话题，只发布独立候选话题。Timer 回绕须与连续序号和受界时差共同判断，设备重启必须重新准入。只拿到组包时间或旧版 `$AUV` 时，巡检保持 Hold。

**边界：**CRC32 仅检出意外损坏，不认证发送者；`flags=1` 是固件声明，接收端无法凭它证明真实传感器。未分配实物端口、未接入 PC104 串口任务、未获得经标定的物理姿态协方差，也未做真实 3DM-GX3-25 回放。接收端已有相对时间映射，但首帧锚不能证明绝对新鲜；侧帧不含加速度，旧姿态旋转的 DVL 被切断，只有[受界配对支路](INSPECTION_IMU_SIDECAR_ROS_ADMISSION_20260927.md)可在合成夹具中恢复速度。侧帧可作为短路径候选，不能据主机合成测试宣称真实 IMU 采样已到 ROS；现场仍需[采样来源预检](INSPECTION_IMU_ACQUISITION_PREFLIGHT_20260927.md)及独立目标/执行包。
