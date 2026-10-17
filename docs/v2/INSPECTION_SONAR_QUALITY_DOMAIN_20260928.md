# 声呐身份线索的数据域拒绝与恢复

本次修复只收紧独立 v2 ROS 身份入口，不改变 v1、历史 5600 次方法运行或已保存的完整合成航迹。声呐概率仍只是经标定的线质量，目标身份还须独立锚点/专属走廊；本修复不替代真实目标关联。

## 复现与改动

修改前，在其余字段满足正常夹具条件时，`p_track=1.01` 和 `age_s=-0.1` 都得到 `accepted`。原入口只检查概率下界和年龄上界，遗漏了概率上界与年龄下界。新入口同时要求：

- `minimum_track_probability ≤ p_track ≤ 1`，且概率有限；
- `0 ≤ age_s ≤ maximum_age_s`，且年龄有限。

不满足时输出空身份线索及 `sonar_identity_quality_unqualified`，无需重启即可处理下一条正常消息。与既有合同保持一致：未观测埋深及其协方差允许 NaN，不因埋深未知取消合格横向线索。此拒绝发生在质量入口，未改变算法、PID、桥的时效门限或任务几何。

源码：[identity_cue.py](../../../brain_linux/src/auv_cable_tracking_v2/auv_cable_tracking_v2/identity_cue.py)；定向测试：[test_identity_cue.py](../../../brain_linux/src/auv_cable_tracking_v2/test/test_identity_cue.py)。安装版与源码 SHA-256 均为 `935b84106b8db02432089a8295d991a73ab28750305fcd188d28aefc8fbc7dad`。

## 验证及成立范围

源目录定向测试 6 项通过；重建安装包后，数据域测试和既有声呐质量适配器→DDS→v2 入口测试合计 **7 项通过**。其中数据域测试逐项注入越界、NaN 和 Infinity，检查明确拒绝及随后正常线索恢复；另检查合法边界与未知埋深兼容。DDS 正例验证合成声呐经过实际质量适配器仍能得到合格横向线索。坏值拒绝/恢复属于函数级合同，未据此宣称整条运动任务在输入故障后恢复。

```bash
source brain_linux/install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=178 PYTHONDONTWRITEBYTECODE=1 \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
python3 -m pytest -q \
  brain_linux/src/auv_cable_tracking_v2/test/test_identity_cue.py \
  brain_linux/src/auv_cable_tracking_v2/test/test_identity_cue_ros.py
```

首次 `colcon build` 在 VirtualBox 共享目录替换生成的 `identity_cue.cpython-310.pyc` 时因 `Text file busy` 失败。确认没有残留巡检进程后，仅清理该模块生成缓存，再以 `PYTHONDONTWRITEBYTECODE=1 colcon build --packages-select auv_cable_tracking_v2 --executor sequential` 重建成功；未删除源码或数据。共享目录的构建失败与算法输入拒绝分别记录。

## 与任务冻结的关系

此改动关闭了无效概率/年龄误授权的两项具体输入漏洞，不作为方法创新或现场成功率。它不解决已有连续性负例；现场仍需目标机完整同次任务、物理输入及执行包验收。[统一冻结矩阵](INSPECTION_V2_FREEZE_AND_FIELD_GATE_20260927.md)继续保留 `field_release_authorized=false`。
