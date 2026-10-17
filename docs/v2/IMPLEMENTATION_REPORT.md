# v2 实施与验证状态

日期：2026-09-19。

## 需求闭环

| 原始要求 | 实现 | 证据 |
|---|---|---|
| 不改 v1 与旧结果 | 全部算法位于 `src/auv_mag_tracking/v2/`，ROS 位于独立包 `auv_cable_tracking_v2` | v1 默认入口未替换；新结果仅写 `results/v2/` |
| Prior-guided Search | 先验走廊、初始不确定度、边界、宽度、间距、速度 | Search 单测与 Case A/D |
| Magnetic Detection | 滑窗、差分 MAD 噪声、双阈值迟滞、连续帧、峰值证据 | detector 单测与无目标虚警实验 |
| Active Cross Scan | 4/8/14/20 m 分档扩幅、首侧航向对准、空间 B(x)、沿缆漂移硬门 | pipeline 单测、ROS/PVS smoke |
| Explicit FWHM | 背景修正半峰、左右插值交点、宽度、中心、模型残差、有效性和置信度 | FWHM 单测与模型一致性实验 |
| 更可靠深度 | 模型化 shape factor；交点、模型、系数与高度不确定度分离 | `MODEL_CONSISTENCY.md` 与论文广义公式 |
| Cable Track | 独立横向观测修正有限先验走廊、预瞄、限幅、重捕、终点保持 | endpoint 回归测试与 PVS endpoint hold |
| Cases A–D | 大初误差、埋深、噪声、间距；另含 no-target 与简化跟踪 | `EXPERIMENT_BASELINE.md` |
| 关键设计消融 | 背景/绝对半峰、空间/时间坐标、横切方式与幅度、迟滞、反演来源、先验/观测、单次/重复 | `ABLATION_BASELINE.md`；固定 50 种子与产物清单 |
| 实测信号回放 | 只读 TMR--ArUco 同步数据，按拟合导线坐标分解并逐腿执行 FWHM/轨迹门 | `TMR_JOINT_REPLAY.md`；9 条候选腿全部拒绝伪深度 |
| ROS 全链路 | 保持既有 topic/message，替换算法 facade | 独立 ROS 包、4 个包测试、PVS protocol_udp smoke |
| README/参数/v1 对照/运行说明 | 已建立 | 本目录文档与两份 YAML |

## 关键修正

1. FWHM 使用导航位置投影后的空间坐标，不把非匀速航行简化成固定速度乘时间。
2. `z=W/2` 只属于偶极包络。解析单直导线为 `z=W/(2sqrt(3))`；PVS midpoint 电流元必须单独标定。
3. 左右半高交点不完整、峰值不被包围、profile mismatch、垂直距离小于高度或沿缆漂移过大时，结果硬判无效，不能把负埋深截成零。
4. 检测中心偏离峰值时保持原始中心逐档扩幅，检测在远端释放不会中断重试。
5. Track 不再无限延长单次局部观测线；观测横向修正随有限先验走廊传播，到终点零速保持。
6. profile confidence 描述“该剖面能否稳定测宽”，不等同于埋深准确度；埋深 sigma 另含 shape factor 和高度误差。
7. ROS v2 适配器消费 `/auv/sensors/altitude` 的新鲜值；高度缺失或过期时显式回退静态配置，避免把任务全程固定高度误当作实测高度。

## 已知边界

- PVS 当前磁源是线段中点电流元近似，系数随局部几何与动态航迹改变。单次几何受控运行完整闭环通过，但静态系数造成约 1.0 m 深度偏差；该负结果已保留，不作为精度通过声明。
- 用上一轮动态剖面反算系数后进行的 24 m 单档复测被轨迹质量门拒绝：343 个样本的沿缆范围为 14.189 m，且缺少左半高交点。算法返回重捕且未发布深度，证明校准参数不会越过几何有效性门。
- HoloOcean Python 客户端已可导入，但约 5.1 GiB 的 Ocean 世界资产按用户指示终止下载；本轮证据链以已完成的 PVS 全链路为仿真后端，不声明 HoloOcean 复跑。
- 现有 777 点 TMR--ArUco 联合数据已完成 v2 离线适用性审计：9 条候选腿中 4 条通过轨迹几何，但 0 条具有完整双侧 FWHM，因此没有发布距离。它验证了真实信号拒绝路径，仍不能替代具有同步高度真值的受控双侧横切或水下闭环。
- 论文已把 FWHM 改写为幅值尺度无关、场形模型有条件的广义关系；PVS 单次 smoke 只能作为实现证据，不能升级为海试精度结论。

## 验证命令

```bash
cd AUV-Master-Mag
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_*.py'
PYTHONPATH=src python3 tools/v2_fwhm_model_consistency.py
PYTHONPATH=src python3 tools/run_v2_experiments.py --seeds 20
PYTHONPATH=src python3 tools/run_v2_ablations.py --seeds 50
PYTHONPATH=src python3 tools/v2_replay_tmr_joint.py

cd ../brain_linux
source /opt/ros/humble/setup.bash
colcon build --packages-select auv_cable_tracking_v2 --symlink-install
colcon test --packages-select auv_cable_tracking_v2 --pytest-args -q
```
