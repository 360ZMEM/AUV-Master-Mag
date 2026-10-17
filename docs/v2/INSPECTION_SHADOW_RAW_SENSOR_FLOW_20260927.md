# 巡检影子入口的原始传感器链与 IMU 失鲜处置

这是一组**合成静止输入、被动协议桥**的 ROS 运行对照，补足先前只有空闲接线和显示来源审计的证据。原始 IMU、DVL、深度与磁消息进入六节点影子栈；探针确认 `/auv/state/filtered` 只有定位节点一个发布者、协方差/监督状态/巡检搜索/目标速度/界面状态均有输出。第 12 秒只停止 IMU，继续发送 DVL、深度、磁测和 `/clock`，用以检查旧导航时间锚是否会被反复发布而伪装成新状态。

| 同一版探针、同一合成输入 | IMU 停止后 1 秒宽限期的定位消息 | 巡检能力撤销 | 跟踪降级 | 目标速度归零 |
|---|---:|---|---|---|
| 时效门关闭，对照域 222 | 58 条 | 否 | 否 | 否 |
| 时效门启用，域 221 | 0 条 | 是 | 是 | 是 |

门关闭时，定位节点仍以旧 IMU 采集时间戳连续发出导航消息；监督节点按**收到导航消息的时间**判断健康，使巡检维持 `search` 与 0.3 m/s 的目标速度。门启用后，定位节点同时核对最新 IMU 的本机接收时间与采集时间；超过 0.75 s 便停止发布滤波导航，监督撤销巡检能力，巡检进入 `sensor_degraded` 并将目标速度置零。门启用组在停止后的首个宽限秒内还观测到 14 条定位消息，随后为零；图 40 使用同一探针的逐时刻记录展示因果顺序。0.75 s 是本次仿真候选值，现场需按真实传感器频率、链路抖动和时间同步重新核定。

![图40：只中断 IMU 后，导航时效门对持续发布旧状态及巡检速度的影响](publication/fwhm_v2_experiment_results/figures/40_shadow_raw_sensor_imu_gate.png)

[可独立发布的逐时数据、摘要及哈希清单](publication/fwhm_v2_experiment_results/data/40_shadow_raw_sensor_imu_gate_manifest.json)与[探针源码](../../../tools/probe_v2_shadow_raw_sensor_flow.py)、[绘图源码](../../../tools/plot_v2_shadow_raw_sensor_gate.py)保留对照。复现时先 `source /opt/ros/humble/setup.bash && source brain_linux/install/setup.bash`，在两个隔离的 `ROS_DOMAIN_ID` 中分别启动：

```bash
mkdir -p /tmp/inspection_shadow_data
export AUV_DATA_ROOT=/tmp/inspection_shadow_data ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=221
ros2 launch auv_cable_tracking_v2 inspection_shadow_stack.launch.py \
  config_file:=$PWD/brain_linux/src/auv_cable_tracking_v2/config/cable_tracking_v2_flythrough.yaml \
  params_file:=$PWD/brain_linux/src/auv_cable_tracking_v2/config/inspection_shadow_raw_sensor_sim_params.yaml \
  use_sim_time:=true
```

另一个终端加载相同环境，运行 `python3 tools/probe_v2_shadow_raw_sensor_flow.py --require-safe-dropout --output-dir /tmp/inspection_probe_gated`。对照组换一个隔离域并向 launch 增加 `inspection_navigation_imu_max_age_s:=0.0`，探针不加 `--require-safe-dropout`。绘图脚本针对本次归档的两个固定输出目录作图，不修改原运行数据。

这组结果只证明**原始合成消息到被动任务输出**的短时软件路径，不证明车辆运动、磁场定位精度、目标缆身份、实际 PC104 零推力收帧或实艇停车距离。0.3 m/s 是巡检节点的目标速度；本次被动桥始终未取得自治执行许可，埋深有效值也未发布。现场任务仍需按[冻结门](INSPECTION_V2_FREEZE_AND_FIELD_GATE_20260927.md)逐项取得物理证据。
