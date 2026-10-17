# 静态目标清单与实际巡检任务配置的绑定

已知目标、无运行时声呐的浅角单次穿越是[现场首验候选](INSPECTION_V2_FIELD_FIRST_PROFILE_20260927.md)。这一剖面要先证明批准的目标和路线与 ROS 节点实际加载的任务一致。原静态清单只核对源文件、先验路线点、侧向区间和任务／目标 ID；在清单与原测绘文件都不变的情况下，改 YAML 的搜索边界、飞越终点或最大控制速度，原清单仍会通过。这种配置漂移会让“已审查的短直走廊”变成另一项任务。

非仿真静态目标入口现要求 [schema v2 清单](static_target_prior_manifest_template.json)中的 `mission_config_sha256`。该摘要对节点加载的整份 `cable_tracking` 配置做规范 JSON 哈希，只排除 `v2.static_target_prior` 的清单路径和摘要字段以避免循环引用；原始来源文件、路线点和侧向区间仍分别独立核对。旧 schema v1 清单会明确拒绝启动。审查后改任何执行参数，都必须重新审查配置并签发新清单，不能仅保留旧源文件哈希。接令时还要求 `target_depth`、`target_speed_mps` **显式给出**并分别等于已审查配置的 `control.default_depth_m`、`control.default_speed_mps`；任务 ID 一致但深度或速度变了，同样不能出航。

| 同一测试清单之后改动的 YAML | 当前启动结果 |
| --- | --- |
| 保持审查时配置 | 可建立软件节点并接收匹配任务 ID |
| `v2.search_boundary_xy_m` | `mission config hash mismatch`，拒绝启动 |
| `v2.flythrough.end_along_m` | `mission config hash mismatch`，拒绝启动 |
| `control.max_speed_mps` | `mission config hash mismatch`，拒绝启动 |
| 旧 schema v1，即使清单文件哈希已重新填写 | `schema_version 2 required`，拒绝启动 |
| 任务消息改变／省略目标深度或速度 | `STATIC_TARGET_PRIOR_COMMAND_MISMATCH`，拒绝接令 |

这些是合成清单的**配置一致性**负例，不证明走廊真实可通行、地图确属目标缆、评审人身份可信，亦不代表实艇已接入。当前物理地图、导航来源和 PC104 同次执行包仍未通过[现场冻结矩阵](INSPECTION_V2_FREEZE_AND_FIELD_GATE_20260927.md)，`field_release_authorized=false`。旧软件试验和图像保持各自版本快照；此门仅收紧后续非仿真静态目标启动，不追溯改变它们。

审查时先冻结完整任务 YAML，再按[静态先验填写步骤](STATIC_TARGET_PRIOR.md)计算 `mission_config_sha256`、路线与来源文件摘要，填写清单并计算最终清单哈希。可复核当前源码与安装空间：

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
PYTHONPATH=AUV-Master-Mag/src:brain_linux/src/auv_cable_tracking_v2:$PYTHONPATH \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  brain_linux/src/auv_cable_tracking_v2/test/test_v2_package.py -q -k static_survey
sha256sum \
  brain_linux/src/auv_cable_tracking_v2/auv_cable_tracking_v2/static_target_prior.py \
  brain_linux/install/auv_cable_tracking_v2/lib/python3.10/site-packages/auv_cable_tracking_v2/static_target_prior.py
```

本次定向测试通过，完整 ROS v2 包 `66/66` 通过；[源码、安装文件及模板摘要清单](publication/fwhm_v2_experiment_results/data/static_target_prior_mission_envelope_20260927.json)固定复核口径。若将来用物理清单启动，仍需把配置 SHA、源文件 SHA、任务与目标 ID、审查有效期、任务深度／速度及实际加载的 ROS 参数写入同次日志，才能把此软件门接入真实验收包。

后续又用安装空间节点在非仿真墙钟下经 DDS 投递任务：[同次准入复核](INSPECTION_STATIC_PRIOR_ROS_DDS_20260927.md)证实错速消息返回拒绝状态，匹配消息可接收，改边界配置启动失败。该探针无导航、磁测和执行器，只验证 ROS 准入接线；它不扩大上述现场许可。
