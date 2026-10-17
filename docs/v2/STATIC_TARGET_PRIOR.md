# 无运行时声呐的静态目标先验入口

已知短直孤立目标线可在经任务审查的窄测绘线或已确认航迹下，作为声呐缺席时的目标身份线索。此前非仿真 ROS 入口只限制 `side_hint_max_m - side_hint_min_m ≤ 1.0 m`，将配置的 `side_hint_source` 改成 `survey_map` 就可能越过该宽度门。本次增加了启动时的**清单—路线—侧向区间绑定**：非仿真、`require_runtime_identity_cue=false` 的静态目标模式必须提供审查清单及原始来源文件，并以 SHA-256 固定两者；仿真夹具仍按原配置运行或保持拒绝。随后复核发现旧清单只锁了路线点和侧向区间，却没锁飞越起终点、搜索边界和速度；**当前 schema v2 又锁定整份已加载任务配置**，原 schema v1 不再能启动非仿真静态目标任务。

清单[模板](static_target_prior_manifest_template.json)默认不获批准。现场准备时，先人工确认目标身份、测绘坐标系与艇载局部 NED 变换、时间有效期、邻缆及障碍、误差区间和横切走廊；`source_file` 指向保留原始测绘/已确认航迹的文件。复制模板后填写非空 `mission_id`、`target_id`、`reviewer`、UTC 审查和失效时间，并在完成审查后将 `reviewed_for_mission` 写为 `true`。`source_kind` 只能为 `survey_map` 或 `previous_track`，且应和任务 YAML 的 `side_hint_source` 一致。`frame` 必须为 `local_ned`；`side_hint_min_m`、`side_hint_max_m` 必须与任务 YAML 精确一致，且间隔不得超过候选 1 m。路线须在 `prior.format: yaml` 的 `yaml_points_ned` 中逐点写出；清单固定其坐标和坐标系的哈希。`mission_config_sha256` 另固定整份 `cable_tracking` 配置，计算时只排除 `v2.static_target_prior` 的清单路径与哈希以避免自引用；审查后改搜索边界、飞越起终点、速度、控制或质量门均须重审并重算清单。

生成哈希的一个本机命令为：

```bash
PYTHONPATH=brain_linux/src/auv_cable_tracking_v2 python3 - <<'PY'
from pathlib import Path
import hashlib, yaml
from auv_cable_tracking_v2.static_target_prior import mission_config_sha256, prior_points_sha256
mission = yaml.safe_load(Path('/path/to/mission.yaml').read_text())['cable_tracking']
print('prior_points_sha256:', prior_points_sha256(mission['prior']))
print('mission_config_sha256:', mission_config_sha256(mission))
print('source_sha256:', hashlib.sha256(Path('/path/to/source_file').read_bytes()).hexdigest())
print('evidence_sha256:', hashlib.sha256(Path('/path/to/filled_manifest.json').read_bytes()).hexdigest())
PY
```

先固定和审查完整任务 YAML，计算路线、整份任务配置及原始来源哈希，填写 schema v2 清单，再计算**最终清单文件**哈希。任务 YAML 示例字段如下；文件路径可以是绝对路径，或相对项目根目录的路径：

```yaml
cable_tracking:
  prior:
    format: yaml
    frame: local_ned
    yaml_points_ned: [[0.0, 0.0, -1.5], [80.0, 0.0, -1.5]]
  v2:
    mission_geometry: flythrough
    flythrough:
      side_hint_source: survey_map
      side_hint_min_m: 5.5
      side_hint_max_m: 6.4
      require_runtime_identity_cue: false
    static_target_prior:
      evidence_file: /path/to/filled_manifest.json
      evidence_sha256: <最终清单的 SHA-256>
```

启动门检查清单与来源哈希、完整任务配置、路线点、坐标系、侧向区间、任务审查标记和有效期；任何不一致即拒绝启动。任务接令时还必须要求 `/auv/cable/mission_command` JSON 中的 `mission_id`、`target_id` 与已锁定清单**逐字一致**，缺失或不匹配即拒绝接令并报告 `STATIC_TARGET_PRIOR_ID_MISMATCH`，不进入巡检。使用自动任务注入器时，可通过 `cable_mission_id:=...`、`cable_mission_target_id:=...` 两个可选 launch 参数传入；旧任务不配置这两个参数时仍发送原格式。静态现场任务的操作命令示例：

```json
{"mission_type":"CABLE_INSPECTION","mission_id":"survey_mission_01","target_id":"cable_A","target_depth":10.0,"target_speed_mps":0.3}
```

同一非仿真静态任务还要求消息显式携带 `target_depth`、`target_speed_mps`，分别匹配已纳入清单哈希的 `control.default_depth_m` 和 `control.default_speed_mps`；省略或改动时报告 `STATIC_TARGET_PRIOR_COMMAND_MISMATCH` 且不接令。真实任务若需改变深度或速度，应先更新整份任务配置并重新审查清单，而不能只改上位机消息。此检查不作用于合成入口或运行时声呐入口。

哈希与 ID 门只能阻止**配置误改、证据文件掉包或任务与清单错配**，不能证明测绘确属目标缆、审查者有权限或现场线索误差真的小于 1 m。ROS 当前仍未得到物理批准的静态地图包；测试中构造的“审查清单”只是软件夹具。实际部署还须通过[现场冻结矩阵](INSPECTION_V2_FREEZE_AND_FIELD_GATE_20260927.md)中的目标包、走廊包和 PC104/真实滑行执行包，`field_release_authorized` 继续为 `false`。没有这些证据时只报告磁异常或 Hold，不宣布目标缆任务完成。

有效期现分别在启动、首次接令和运行期检查：接令前重新核对清单及来源文件哈希；运行中若审查时间窗失效，v2 锁存 `static_target_prior_expired`，下发零速且 `track_cable=false`，并经恢复/预算状态让桥撤权，重复任务消息不能自行解锁。该软件机制不等于可靠系统时钟或现场测绘持续准确，现场仍需让审查窗口覆盖任务、故障 Hold 与处置，并验证时钟同步与真实目标关联。

[独立增量快照](publication/fwhm_v2_experiment_results/data/inspection_v2_static_target_prior_gate_20260927.json)固定本次源码与测试文件哈希。关闭 pytest 自动插件加载后，ROS 包 47 个测试通过；逐测试文件独立进程运行也为 47/47。默认插件集合下单进程全跑曾在第 47 个附近出现 Python 原生扩展段错误，不能把那次运行计为通过。论文 `make auv-thesis` 重新构建通过，209 页。旧 2,577 周期合成同次链保持原始快照；后续已在当前源码下[独立复跑并审计](INSPECTION_OPERATOR_SAME_RUN_STATIC_PRIOR_20260927.md)，故障后仍为 241/241 周期双零推进，桥/界面 7,730/7,730 状态配对。两次数据分别引用，均不是现场许可。

上述 47 项与同次链复跑属于**清单启动门初版**。接令 ID 绑定加入后，另有[任务目标绑定快照](publication/fwhm_v2_experiment_results/data/inspection_v2_static_target_mission_binding_20260927.json)：重建后的 ROS 安装文件与源码哈希一致，v2 包加自动任务注入器共 49 项测试通过，缺失/错任务/错目标均拒绝，匹配的软件夹具才接令。当前源码又在独立目录完成[同次合成链审计](publication/fwhm_v2_experiment_results/data/inspection_v2_same_run_mission_binding_20260927.json)，故障后 241/241 周期 PID/UDP 双零、桥/界面 7,730/7,730 条逐条配对。此后版本的测试与旧数值不能混写为同一次实验；三者都不构成现场地图或实艇验收。

再加入接令前复核及运行中过期锁存后，[最终静态路径软件快照](publication/fwhm_v2_experiment_results/data/inspection_v2_static_prior_runtime_20260927.json)固定源码与安装文件哈希；49 项测试通过，含启动后来源被改和运行中过期零速拒绝。当前源码另有[合成同次链审计](publication/fwhm_v2_experiment_results/data/inspection_v2_same_run_static_prior_runtime_20260927.json)：失鲜后 241/241 周期 PID/UDP 双零，桥/界面 7,732/7,732 状态配对。论文最新 `make auv-thesis` 构建通过，210 页。上述各快照均留原始口径，不合并计数；最新软件验证仍不授权物理任务。
