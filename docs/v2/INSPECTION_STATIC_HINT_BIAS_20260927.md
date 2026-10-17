# 窄静态地图偏移时的目标身份反例

这项补充检验“先判别场形，失败则保留横向巡检、深度置空”的一个不同前提：横向磁中心究竟属于哪条海缆。现有静态清单和 1.0 m 最大侧向区间宽度能阻止任务配置漂移，却不能证明地图线位无系统偏差。图 11–12 的 5600 次稳定性基线、图 13–15 的场形拒绝及旧多源结果均保持原样。

## 受控反例

保持合成目标缆 $y=6$ m、邻近异常 $y=9$ m（目标幅值的一半）、30 m 浅角横切、0.03 nT 高斯噪声、0.5 m 观测容差和不超过 1.0 m 的区间宽度。仅把独立线索的区间从准确的 `[5.5,6.4]` m 整体偏移 1.1 m 至 `[6.6,7.5]` m。两个区间均宽 0.9 m，也都通过现行配置几何预检。`survey_map` 只是受控软件输入标签，不代表真实测绘。

![图 54：窄静态侧向区间偏移使单源拒绝、融合峰准入](publication/fwhm_v2_experiment_results/figures/54_static_hint_bias_admission.png)

| 地图区间 | 磁场 | 三种子确认中心范围 (m) | 确认终态 |
|---|---|---:|---|
| 准确 `[5.5,6.4]` | 单源 | 5.993–6.004 | Track 准入 3/3 |
| 准确 `[5.5,6.4]` | 目标＋邻源 | 7.007–7.024 | Hold 拒绝 3/3 |
| 偏移 `[6.6,7.5]` | 单源 | 5.993–6.004 | Hold 拒绝 3/3 |
| 偏移 `[6.6,7.5]` | 目标＋邻源 | 7.007–7.024 | **Track 准入 3/3** |

算法级 12 次对照只使用种子 101–103，并在横切确认终点停止；拒绝原因均为 `side_hint_observation_inconsistent`、引导速度为零，所有组均无埋深。三种子只改变同一标量异常解析式上的磁噪声，不能推断现场概率。[逐例数据与脚本哈希](publication/fwhm_v2_experiment_results/data/54_static_hint_bias_admission.json)由 `python3 AUV-Master-Mag/tools/audit_v2_static_hint_bias.py` 再生。图像采用论文 Fandol Song 字体。

另将偏移区间送入 ROS2 巡检节点、PID、自治守卫、本地 UDP 桥与原生 PVS 的**同一合成任务**，只切换邻源包络。单源在 577 s 的 Confirm 阶段拒绝，末周期 PID 推力与 UDP 主推 RPM 均为零；加入邻源后峰中心 7.026 m，任务于 1105 s 被标记完成，在线横偏 MAE 为 0.721 m，相对指定目标缆真值的完整正向 Track MAE／最大横偏却为 **1.090／2.309 m**。两例均不发布埋深。闭环只有种子 101，不能把算法级 3/3 扩写成闭环 3/3。[闭环逐例哈希和真值重算](publication/fwhm_v2_experiment_results/data/54_static_hint_bias_closed_loop.json)由 `python3 tools/audit_v2_static_hint_bias_closed_loop.py` 检验；逐时 CSV 保存在 `results/v2/inspection_static_hint_bias_20260927/`。

这个反例的结论有明确边界：**宽度受限和配置哈希一致仍不足以认证目标身份**。当地图整体偏移且邻源叠加产生单峰时，磁场质量分数、FWHM 和在线贴峰误差都无法从同一条曲线推翻错误地图。不能靠进一步收窄区间自动修复；偏移地图甚至会把真实单源拒绝，却放行融合峰。首验前必须由独立测绘／声学／可信历史航迹给出目标线位误差上界与邻源排除证据，并核对其坐标、时间和目标关联。若这些证据不存在，系统只能报告“未认证磁异常中心”，不得宣称指定目标完成；埋深继续为空。当前物理许可仍为 `false`。

本反例的双源只是标量包络相加，不是多芯铠装缆矢量场、真实海缆测绘误差或 PC104 收帧。路径 1→3 处理的是**埋深不可辨识**；本页指出它不能同时替代**目标身份许可**。

## 复现

先运行算法级审计。闭环需 ROS Humble、已构建的 `brain_linux/install`，并把窄线索 YAML 的两个边界改成 6.6 和 7.5，保存为隔离的合成配置；本地原始配置已保存在上述忽略的结果目录，SHA-256 见闭环 JSON。随后在独立 `ROS_DOMAIN_ID` 和 `ROS_LOCALHOST_ONLY=1` 下分别运行：

```bash
python3 AUV-Master-Mag/tools/audit_v2_static_hint_bias.py
python3 - <<'PY'
from pathlib import Path
import yaml
source = Path('brain_linux/src/auv_cable_tracking_v2/config/cable_tracking_v2_flythrough_narrow_cue.yaml')
target = Path('AUV-Master-Mag/results/v2/inspection_static_hint_bias_20260927/biased_hint_config.yaml')
target.parent.mkdir(parents=True, exist_ok=True)
config = yaml.safe_load(source.read_text())
config['cable_tracking']['v2']['flythrough'].update(side_hint_min_m=6.6, side_hint_max_m=7.5)
target.write_text('# Synthetic displaced-map counterexample; never deploy as a field map.\n'
                  + yaml.safe_dump(config, sort_keys=False))
PY
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=226
python3 tools/smoke_v2_flythrough_ros_pvs.py --with-bridge --case wire --seed 101 \
  --cable-lateral-m 6 --secondary-cable-lateral-m 9 --secondary-cable-gain-nt 10 \
  --config-file AUV-Master-Mag/results/v2/inspection_static_hint_bias_20260927/biased_hint_config.yaml \
  --output-dir AUV-Master-Mag/results/v2/inspection_static_hint_bias_20260927/biased_double
python3 tools/smoke_v2_flythrough_ros_pvs.py --with-bridge --case wire --seed 101 \
  --cable-lateral-m 6 \
  --config-file AUV-Master-Mag/results/v2/inspection_static_hint_bias_20260927/biased_hint_config.yaml \
  --output-dir AUV-Master-Mag/results/v2/inspection_static_hint_bias_20260927/biased_clean
python3 tools/audit_v2_static_hint_bias_closed_loop.py
```

算法级 12/12 断言、路径 1→3 的四个测试文件 47/47 均通过；两条新闭环均按预期到终态。以上是受控合成反例和软件链核查，不替代真实目标包、导航包或执行包。
