# v1 与 v2 差异

| 项目 | v1 | v2 |
|---|---|---|
| 部署入口 | `api.AuvMagTrackingPipeline` | `v2.AuvMagTrackingPipelineV2` |
| Search | 内部仿真存在 SEARCH 状态；部署 facade 以先验投影为主 | 显式输入先验、不确定度、宽度、间距、速度和边界，输出覆盖航点 |
| Detection | 强度/SNR 质量和内部峰值检测分散存在 | 独立 sliding median/MAD score、迟滞、持续帧和原因码 |
| 主动探查 | 角度式/正弦 zig-zag，主要服务跟踪 | Confirm 采用局部法向分段直线横切，4/8/14/20 m 分档扩幅 |
| 埋深 | 标定幅值法 `K I/B`，依赖电流和部署常数 | 显式背景修正 FWHM，深度系数按场模型声明 |
| 电缆位置 | 主要是导航位置对先验图投影，可选声呐/磁比值修正 | 半高交点中点形成独立观测点；横向修正投到有限先验走廊，避免把局部直线无限外推 |
| 有效性 | 幅值/SNR/横向门控 | 左右交点、覆盖、对称性、SNR、prominence、模型残差、沿缆漂移和原因码 |
| Track | 路线投影横偏修正及控制器内部策略 | 观测修正后的有限走廊预瞄、低置信减速、REACQUIRE 和终点零速保持 |
| ROS | `auv_decision_ros` 原节点 | 独立 `auv_cable_tracking_v2` ROS 包；复用消息和 topic |
| 默认行为 | 保持原配置和结果 | 必须显式启动 v2 包或调用 v2 API |

v2 不删除或替换 v1。幅值反演可作为后续一致性检查或退化备份，但不会伪装成 FWHM 结果。
