# v2 FWHM 场形标定与发布边界

本流程针对“场形看似可信，但 FWHM 宽度到距离的系数错误”这一风险。运行时双场形判别门能拒绝已测试的偶极/单直导线互换，却不能从单条归一化曲线推断真实距离比例。因此系数必须来自独立几何真值。

## 受控扫描记录

在至少两个不同的已知传感器—导线垂距下进行完整双侧横切。每条扫描需记录：导线几何位置、同步位姿、磁包络、背景与噪声、传感器高度及时间同步误差。`true_separation_m` 必须由这些独立量计算，**不得由磁剖面、FWHM 或同源仿真模型反推**。训练与留出扫描按完整 `scan_id` 分割；留出集应覆盖不同日期、距离、速度和安装条件，不能把同一条横切的采样点拆到两组。

CSV 每一行是一个磁包络采样点，列为：

| 列 | 含义 |
|---|---|
| `scan_id`, `split` | 完整横切编号，`train` 或 `holdout` |
| `x_m`, `envelope_nt` | 沿电缆法向的空间坐标、锁相/背景处理后的磁包络 |
| `true_separation_m` | 独立测量的传感器—导线垂距 |
| `baseline_nt`, `noise_nt` | 同条扫描的背景与噪声估计 |

完整扫描的 `true_separation_m`、`baseline_nt`、`noise_nt` 必须一致。埋深还需独立的传感器离底高度；本工具只校准垂距系数。

```bash
cd AUV-Master-Mag
python3 tools/calibrate_v2_fwhm_shape.py \
  --input-csv /path/to/controlled_cross_scans.csv \
  --shape-model physical_single_wire \
  --source controlled_measurement \
  --output-json results/v2/controlled_shape_calibration.json
```

工具对训练组的有效扫描计算 `κ = median(true_separation_m / FWHM_m)`；用完全独立的留出组复核。默认至少 5 条有效训练扫描、5 条留出扫描，训练距离跨度至少 0.5 m，留出有效率至少 90%，有效留出 MAE 不超过 0.2 m、P95 不超过 0.5 m。任一条件失败，JSON 的 `accepted=false`，进程退出码为 2。阈值是当前工程审计门槛，不能代替实物验收标准。输出记录输入 SHA256、系数、离散度、留出误差及拒绝原因；即使 `accepted=true`，也应复核真值链和传感器安装一致性。

标定输出现在将 `statistical_holdout_accepted` 与 `deployment_qualified` 分开。`--source controlled_measurement` 只是调用者标签，不能证明 CSV 来自实艇或已知几何，因此脚本**始终写出 `deployment_qualified=false`**，并要求实物来源复核。旧版曾仅凭该标签及留出通过就写 `deployment_qualified=true`；该字段不得用于现场解锁。

实测数据到位后，先复制 [物理垂距证据清单模板](field_depth_manifest_template.json)，填写标定 JSON 及每条完整 `scan_id` 的原始磁测、独立导线几何真值、同步离底/导航文件路径与 SHA-256。每条还须写 `split`、`session_id`、`geometry_truth_method`。预检会重算哈希，要求清单与标定 CSV 的扫描编号和训练/留出划分完全一致、各至少 5 条，拒绝同一会话或同一原始文件跨划分，也拒绝旧版自称已获现场批准的标定输出：

```bash
python3 tools/audit_v2_depth_field_provenance.py \
  --manifest docs/v2/field_depth_manifest.json \
  --output-json results/v2/field_depth_preflight.json
```

预检通过仅表示文件和分组可追溯，**`field_release_authorized` 始终为 `false`**。人工仍须核实真实采集、几何真值独立性、外参与时间同步、离底高度和适用范围，并在单独的整艇执行包中完成审核。仓库当前仅有空模板，不存在能使该预检通过的实物垂距数据。

当受控训练数据表明 `κ` 随距离系统变化时，可事先选择 `--mapping affine`，拟合 `z=a·FWHM+b`。该模式至少需要三个不同真值距离，只在训练宽度范围内发布，留出组仍完全不参与拟合；运行时可经 `affine_depth_calibration` 接入 v2 pipeline。它多一个自由参数，不能凭训练误差下降就宣称优于常系数，必须以独立留出和适用范围验证。不同模型方案若要择优，还需另外保留最终测试集，避免用同一留出集既选模型又报告精度。

```bash
python3 tools/calibrate_v2_fwhm_shape.py \
  --input-csv /path/to/controlled_cross_scans.csv \
  --shape-model thesis_dipole --mapping affine \
  --source controlled_measurement \
  --output-json results/v2/controlled_affine_width.json
```

## 双模型保守发布

`estimate_with_calibrated_models` 可接收多个已通过留出验证的场形系数。只有一个模型有效时，其他候选必须因“备选场形明显更优”而拒绝；多个模型都有效时，垂距估计差不得超过 0.5 m。否则只保留 FWHM/峰值诊断，垂距与埋深置空并输出 `shape_model_not_distinguishable` 或 `model_depth_disagreement`。无合格标定时输出 `no_accepted_shape_calibration`。默认拒绝仅来自仿真的标定结果；仿真测试必须显式传 `allow_simulation_calibration=True`。

v2 pipeline 提供可选的 `depth_calibrations` 与 `affine_depth_calibration` 构造参数，两者只能择一；均未传时保持旧实验接口与结果语义。ROS 的现行论文仿真配置尚未切换这些参数。实物部署接入前必须完成真实受控标定并显式接线；现有未标定 ROS 配置不得宣称为实测埋深发布就绪。若多个场形具有相同归一化形状却共享错误的宽度—距离系数，多模型一致性也无法发现，须依赖独立几何留出或其他传感器交叉核验。

## 已完成的流程审计（仅仿真）

执行 `python3 tools/run_v2_shape_calibration_audit.py`，复现训练种子 0–49、留出种子 200–249 的三组各 50 条扫描。原始 CSV、摘要和 manifest 位于 `results/v2/shape_calibration_audit/`。数据由解析式生成，不能作为海缆实测精度证据。

| 场景 | 标定系数 | 留出有效率 | 留出 MAE / P95 | 接受 |
|---|---:|---:|---:|---|
| 匹配单直导线 | 0.288236 | 100% | 0.011 / 0.028 m | 是，仅流程 |
| 匹配偶极包络 | 0.498729 | 100% | 0.009 / 0.023 m | 是，仅流程 |
| 相同形状、留出距离比例漂移 50% | 0.288236 | 100% | 1.809 / 2.440 m | **否** |

最后一组是关键负例：形状门仍可认为扫描有效，但独立距离真值揭露了系数漂移。当前仓库缺少满足上述真值契约的受控真实横切数据；`real_experiments/cable_scan/` 的原始数据文件在本工作区也未找到，现有报告不能替代这次标定。
