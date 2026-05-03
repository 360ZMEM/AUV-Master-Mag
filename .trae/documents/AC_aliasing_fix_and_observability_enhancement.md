# AUV 磁电缆跟踪仿真 — AC 振荡混姿修复与可观测性提升计划

## 背景诊断

**病因 1 — AC 瞬时采样混叠**: `MagneticVectorAnalyzer.update()` 直接使用 `anomaly_ned_nt[-1]` 单点瞬时矢量求 `arctan2`，对于 50Hz 交流磁场，采样点落在正弦波不同相位会导致角度完全随机。

**病因 2 — 地磁漏磁**: 地磁场 ~50,000 nT 远大于电缆异常场 ~100 nT。AUV 微小 Pitch/Roll 变化导致坐标转换误差，巨大地磁分量泄漏到异常场中。

---

## Task 1: 引入 PCA/SVD 主成分矢量提取

### 修改文件
- `src/auv_mag_tracking/perception.py` — `MagneticVectorAnalyzer` 类

### 具体步骤

1. **新增 `StreamingVectorPCAFitter` 类** (位于 `MagneticVectorAnalyzer` 之前):
   - 维护一个固定容量的 AC 3D 矢量块 buffer（NED 坐标系下的 XY 分量）
   - 方法 `add_sample(vector_xy: np.ndarray)` 将 `[bx, by]` 加入 buffer
   - 方法 `compute_principal_vector() -> Tuple[np.ndarray, float]`:
     - 当 buffer 样本数 >= 3 时，构建 2×2 协方差矩阵 `C = cov([bx, by])`
     - 计算特征值和特征向量 `eig(C)`
     - 取最大特征值对应的特征向量作为主成分方向
     - 返回主成分矢量 `[vx, vy]` 和一致性分数 `resultant_length`
   - 方法 `clear()` 清空 buffer

2. **修改 `MagneticVectorAnalyzer.__init__`**:
   - 新增 `pca_fitter: StreamingVectorPCAFitter` 实例
   - 新增 `_previous_vector_xy: Optional[np.ndarray]` 用于 180° 符号对齐
   - 新增 `vector_consistency_score: float = 0.0` 状态变量
   - 新增 `attitude_leakage_risk: bool = False` 状态变量

3. **修改 `MagneticVectorAnalyzer.update()` 方法**:
   - 新增签名参数: `pose_measurement: Optional[PoseMeasurement] = None`, `snr_db: float = -120.0`, `signal_mode: str = "dc"`
   - **AC 模式 PCA 路径** (`signal_mode != "dc"`):
     - 将 `anomaly_ned_nt[:2]` 添加到 `pca_fitter`
     - 调用 `pca_fitter.compute_principal_vector()` 获取主成分矢量和一致性分数
     - 进行符号对齐：若 `dot(principal_vector, previous_vector) < 0`，则翻转 principal_vector
     - 用对齐后的主成分矢量计算 `magnetic_vector_heading_deg`
   - **DC 模式路径** (保持现有逻辑不变):
     - 直接使用 `anomaly_ned_nt[:2]` 计算方向
   - **动态门控**:
     - 若 `snr_db < 10.0`，直接 return，不更新
     - 若 `pose_measurement` 有效且 `abs(roll_deg) > 3.0` 或 `abs(pitch_deg) > 3.0`，设置 `attitude_leakage_risk = True` 并 return
     - 否则设置 `attitude_leakage_risk = False`

4. **符号对齐逻辑**:
   ```
   if _previous_vector_xy is not None:
       if dot(principal_vector, _previous_vector_xy) < 0:
           principal_vector = -principal_vector
   _previous_vector_xy = principal_vector.copy()
   ```

---

## Task 2: 姿态/信噪比动态门控

### 修改文件
- `src/auv_mag_tracking/perception.py` — `MagneticVectorAnalyzer.update()` 中的门控逻辑

### 具体步骤

已在 Task 1 中一并设计，核心逻辑:
1. SNR 硬门槛: `snr_db < 10.0` → return
2. 姿态硬门槛: `abs(roll) > 3°` 或 `abs(pitch) > 3°` → return, set `attitude_leakage_risk = True`
3. 现有圆周均值算法保持不变，仅对通过门控的样本进行处理

---

## Task 3: 强化诊断日志与可追溯性

### 修改文件
- `src/auv_mag_tracking/perception.py` — `PerceptionState` dataclass
- `src/auv_mag_tracking/main_viz.py` — `SimulationReport`, `SimulationVisualizer.update()`, 终端报告

### 具体步骤

#### 3a. PerceptionState 新增字段
```python
vector_consistency_score: float = 0.0      # 0.0~1.0
attitude_leakage_risk: bool = False
```

#### 3b. MagneticCablePerception.update() 中传递新参数
- 调用 `self.vector_analyzer.update()` 时传入:
  - `pose_measurement=pose_measurement`
  - `snr_db=snr_db`
  - `signal_mode=self.scenario.signal.mode`
- 从 `self.vector_analyzer` 读取:
  - `vector_consistency_score`
  - `attitude_leakage_risk`
- 在 `PerceptionState` 构造中填充这些字段

#### 3c. main_viz.py — SimulationReport 新增字段
```python
avg_vector_consistency: Optional[float] = None
```

#### 3d. main_viz.py — SimulationVisualizer.update() 状态面板
在现有 `status_text` 中新增一行:
```python
f"VecConsist: {perception.vector_consistency_score:.2f} | LeakRisk: {perception.attitude_leakage_risk} | VecHdg: {fmt_optional(perception.magnetic_vector_heading_deg, '.1f', '°')}",
```

#### 3e. main_viz.py — AuvCableTrackingSimulation.run() 终端报告
- 新增 `vector_consistency_history: List[float]` 收集器
- 在仿真循环中收集 `perception_state.vector_consistency_score`
- 在报告打印时，若 `deployment_mode` 开启（即 `not scenario.tracking.use_nominal_route_prior`），输出:
  ```
  Average Vector Consistency: X.XX
  ```

---

## Task 4: 更新测试用例

### 修改文件
- `tests/test_fusion_features.py`

### 具体步骤

1. 新增测试 `test_magnetic_vector_analyzer_pca_extraction`:
   - 构造一组具有明显方向一致性的 AC 矢量样本
   - 验证 PCA 提取的方向与预期方向一致（误差 < 10°）
   - 验证一致性分数 > 0.8

2. 新增测试 `test_magnetic_vector_analyzer_snr_gating`:
   - 在低 SNR 下调用 `update()`，验证方向不更新
   - 在高 SNR 下调用，验证方向更新

3. 新增测试 `test_magnetic_vector_analyzer_attitude_gating`:
   - 在大 roll/pitch 下调用 `update()`，验证 `attitude_leakage_risk` 为 True 且方向不更新
   - 在小 roll/pitch 下调用，验证正常更新

4. 新增测试 `test_magnetic_vector_analyzer_sign_alignment`:
   - 连续输入方向相近的样本，验证不会发生 180° 翻转

5. 修改现有 `PerceptionState` 构造的测试（如有）以包含新增字段

---

## 验收标准

1. `python -m unittest discover -s tests` 全数通过
2. `python main_demo.py --case case1 --deployment-mode` 运行时蓝/粉箭头明显平滑，不再乱摆
3. 终端报告和状态面板显示 `VecConsist`, `LeakRisk`, `VecHdg` 指标
