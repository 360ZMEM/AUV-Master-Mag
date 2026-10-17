# IMU 原生采样时间与姿态协方差现场证据预检

[图 42 的协议桥试验](INSPECTION_PROTOCOL_IMU_TIME_PROVENANCE_20260927.md)已阻止板端组包时间重复、倒退、缺失及过期时刷新 ROS IMU，但它不能识别 PC104 不断把同一个旧 IMU 样本组入新包。现有 `$AUV` 解码字段没有 IMU 原生采样时间或样本计数，因此仅凭当前上行协议无法完成这一现场准入门。

为把缺口变为下一次实测的明确交付物，新增独立的[导航来源清单模板](inspection_imu_acquisition_template.json)和[可执行预检脚本](../../../tools/preflight_v2_imu_acquisition_provenance.py)。它与既有目标包、执行包并列，不覆盖历史报告；三份证据须分别成立。模板要求同一任务的驱动源码、IMU 原始日志、PC104 原始上行、ROS IMU bag、跨设备对时轨迹、独立姿态校准报告与逐样本导出 CSV，全部给出路径和 SHA-256。字段 `acquisition_time_basis` 只接受 `sensor_native_timestamp`，不接受 `pc104_packet_time` 或 ROS 接收时间替身。

CSV 每行至少给出传感器样本 ID、原生采集时间、PC104 组包时间、ROS 接收时间、四元数及 3×3 姿态协方差。预检检查文件完整性、五个以上样本、样本 ID 不重复、三个时间序列严格递增、采样→组包→接收顺序、按实测对时误差修正后的候选年龄门，以及四元数和协方差的基本数值性质。它会列出观测到的最大采样→组包/ROS 延时和最大姿态标准差，供后续冻结阈值；冻结样本时间而组包时间递增、零协方差与过期样本的合成负例均已被拒绝。

在仓库根目录运行：

```bash
python3 tools/preflight_v2_imu_acquisition_provenance.py \
  --manifest /path/to/field/inspection_imu_acquisition.json \
  --output-json /path/to/field/imu_provenance_preflight.json
```

当前[空白模板预检结果](publication/fwhm_v2_experiment_results/data/inspection_v2_imu_acquisition_preflight_pending_20260927.json)为 `navigation_evidence_reviewable=false`、`field_release_authorized=false`，列出 16 项缺口。脚本的合成结构测试只证明核验逻辑，并不等于真实传感器数据。即使未来结构预检通过，仍须人工核对驱动与原始日志确实记录**同一 IMU 样本**、时钟映射未在中途重置、协方差来自独立标定且覆盖实际航向误差；还须用 ROS/定位/巡检同次链验证姿态和时间戳匹配。没有原生采集时钟的硬件，可先修改驱动/固件并经协议评审，再用上述包验收；不能用递增的 PC104 组包时间填进 `imu_acquisition_time_s` 来换取“通过”。

进一步源码/协议核对表明，现用 3DM-GX3-25 `0xCF` 原始应答已有传感器 Timer，但 PC104 旧解析丢弃了它，且未检查完整帧长与校验和；[具体恢复路径与离线解码负例](INSPECTION_GX3_TIMER_RECOVERY_PLAN_20260927.md)可用于固件改动评审。该 Timer 是传感器原生**响应时间锚**，还须验证它对应本次姿态样本的时刻并完成对时，方可映射为本模板的 `imu_acquisition_time_s`。预检只解决导航输入证据的**可审查性**，不计算整艇停车距离、目标身份可靠性或埋深精度，也永不自动将 `field_release_authorized` 设为真。[现场冻结矩阵](INSPECTION_V2_FREEZE_AND_FIELD_GATE_20260927.md)仍要求目标包与执行包分别通过物理和定量审查。

独立 C99 串口验帧候选和[版本化 48 字节来源侧帧](INSPECTION_IMU_SIDECAR_V1_CONTRACT_20260927.md)可作为后续固件与桥接评审起点。它们当前只经主机合成故障注入和 C/Python 字节兼容检查，不改变空白现场证据包的拒绝结果。
