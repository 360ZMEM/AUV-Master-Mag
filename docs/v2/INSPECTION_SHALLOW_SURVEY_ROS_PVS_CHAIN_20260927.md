# 多横切巡检 v2：同次 ROS／PID／守卫／UDP／原生 PVS 证据

图 50 已证明独立 PVS 脚本能完成三／四段连续横切、磁测线、前向接入和 80 m Track；本补项把该方法接成**新的 opt-in ROS `mission_geometry: shallow_survey`**，保留旧 `cross_scan` 和单次 `flythrough`。图 51 展示最终同版种子 101 的正反横流航迹、完整 Track 横偏和故障后原生 PVS 滑行。

![图 51：多横切 ROS 链、目标身份拒绝与 Hold 后滑行](publication/fwhm_v2_experiment_results/figures/51_shallow_survey_ros_pvs_chain.png)

## 可引用的方法合同

先验是一段近直线、长度足以容纳扫描／接入／80 m Track 的路线，工作边界是凸走廊。任务启动前需要两类**独立于磁峰拟合**的证据：有时间戳、世界坐标速度和不确定度的横流输入，以及有时间戳、标定 ID 和半宽的目标线侧向区间。横流方向决定从上游哪侧起步，目标相对于先验的侧向符号决定取三段或四段，使扫描出口落在目标侧。没有任一证据、方向含糊、越过流速包络或未在起点完成预对准，任务输出锁存 Hold。

扫描参考线为 $y(s)=\sigma A\cos(\pi s/L)$，候选 $L=135\,\mathrm m$、$v=0.3\,\mathrm{m/s}$：目标在正侧时，正向 $0.04\,\mathrm{m/s}$ 横流从负侧以 $A=24\,\mathrm m$ 扫三段；反向横流从正侧以 $A=27\,\mathrm m$ 扫四段。解析最小曲率半径分别为 $76.94/68.39\,\mathrm m$，高于示例 $50\,\mathrm m$ 下界。取起点侧向不确定度 $6\,\mathrm m$、航向误差 $1.16^\circ$、半高与尾部预算 $6.7\,\mathrm m$、导航横向余量 $3\,\mathrm m$ 后，振幅覆盖余量只有 $0.099/0.366\,\mathrm m$；这是**很紧的合成研究配方**，不是可直接移植的实艇路线。

每段扫描仅按实际导航点与磁场幅值组成空间剖面。三／四个剖面各自通过单峰、双侧半高、空间采样和横向中心质量门后，再检验跨站线残差和线向。FWHM 在这里用于**横向中心几何**，不发布埋深。拟合磁测线与独立目标区间冲突则拒绝；许可后以测得线向、最多 $10^\circ$ 修正，在扫描结束后 $120\,\mathrm m$ 内接入到 $1\,\mathrm m$ 横距，再从实际接入点前进 $80\,\mathrm m$。接入与 Track 持续复核身份线索，失鲜超过 $2\,\mathrm s$ 或线位相背则 Hold；局部直线外推另受 $220\,\mathrm m$ 上界。ROS 命令经专用 PID、自治守卫和协议桥时，Hold 产生零目标速度、零 PID 推力和零 UDP 主推 RPM。

## 最终同版复跑

以下逐例在隔离 ROS 域中运行，使用**脚本依据真值每周期生成的合成声呐线索与合成横流证据**；声呐线索由 ROS 消息和现有声呐质量转换门进入算法，横流由独立 ROS 消息进入。磁场由原生 PVS 航迹上的解析单源／邻源模型生成。桥组的 UDP 包通过本地回环接收，未连接 PC104。所有案例种子均为 101，不能解释为海况成功率。

| 案例 | 同次软件链 | 终态 | 关键观测 |
| --- | --- | --- | --- |
| 正向横流、单源、三段 | ROS→PID→守卫→UDP→PVS | 完成 80 m Track | 全 Track 横偏 MAE 约 0.334 m；埋深为空 |
| 反向横流、单源、四段 | ROS→PID→协议编解码→PVS | 完成 80 m Track | 全 Track 横偏 MAE 约 0.291 m；埋深为空 |
| 1 m 融合邻源 | ROS→PID→协议编解码→PVS | `survey_identity_rejected` | 磁测线几何有效，0 个 Track 样本 |
| 无磁源 | ROS→PID→协议编解码→PVS | `survey_geometry_rejected` | 各站剖面不满足显著性／单峰门，0 个 Track 样本 |
| Track 中撤去目标线索 | ROS→PID→守卫→UDP→PVS | `survey_identity_lost` | Hold 后 61／61 周期三层零命令；15 s PVS 滑行约 2.81 m |

图 51 的正向成功与失鲜组**确实经过守卫与 UDP 回环**；反向成功、融合邻源和无源组使用同一 ROS 节点与 PID，但只作协议编解码回环，图例和清单分别标注。失鲜组守卫输出来源含 `JETSON_MPC` 与 `SAFETY_FALLBACK`。滑行说明“零推力”并非“原地停车”，尤其不能以合成 $2.81\,\mathrm m$ 替代实艇停止包络。首次并行五域复跑中的守卫失鲜组曾遇到测试脚本 1 s UDP 接收超时；其余案例完成后进行了单域复跑，[失败日志](../../results/v2/shallow_survey_ros_pvs_final_20260927/parallel_udp_timeout.log)仍保留，不据该一次超时推断算法失效或长期运行可靠性。

### 同版噪声种子复验

为检验上述正例是否只依赖种子 101，使用相同源码、配置和解析场形串行补跑种子 102、103。种子 101 正向组经守卫／UDP，其余五个正例是 ROS／PID／协议／PVS 回环；因此下表只比较共有的**感知—轨迹—Track**口径，不把六例都称作守卫链验证。

| 横流方向 | 种子 101 Track 横偏 MAE | 种子 102 | 种子 103 | 完成情况 |
| --- | ---: | ---: | ---: | --- |
| $+0.04~\mathrm{m/s}$，三段 | 0.3342 m | 0.3336 m | 0.3392 m | 3／3 完成 80 m Track |
| $-0.04~\mathrm{m/s}$，四段 | 0.2914 m | 0.2928 m | 0.2921 m | 3／3 完成 80 m Track |

六例均不发布埋深；新增四例各自的 CSV SHA-256、摘要 SHA-256、运行时源码哈希与完整统计见[三种子审计](publication/fwhm_v2_experiment_results/data/51_shallow_survey_ros_pvs_three_seed_audit.json)和[原始目录](../../results/v2/shallow_survey_ros_pvs_seed_extension_20260927/)。共有源码文件的运行时哈希全部一致，轨迹文件哈希也逐例重新计算匹配。这是同一合成场、相同参数下的三个固定噪声种子复验，**不是**横流测量误差、未知电缆曲率、非同源场形或海况成功率试验；图 51 仍只绘制种子 101 的过程。

图 51 与上面六例是**Hold 仿真时基修正前**的冻结源码快照，原始结果未覆盖。随后 [图 52 的 300 s 同场景复验](INSPECTION_SHALLOW_SURVEY_HOLD_CLOCK_20260927.md)发现旧墙钟预算在加速 PVS 的约 233 s 仿真时间才请求恢复；双时基修正后为声明的 30 s，两版持续零命令和约 12 m 合成流漂均保留。新旧计时结果不得当作同一源码版本合并。图 51 仍用于解释磁测线、身份门和轨迹，图 52 用于解释持续处置与时间基准。

## 证据边界与部署门

独立目标线索虽然通过 ROS 声呐接口，但其点列、概率和校准 ID 由脚本按 PVS 真值制造；横流消息也是按仿真设定每周期提供，尚无物理来源。新模式在非仿真启动时直接拒绝，通用 `auv_stack_v2` 同样拒绝将它误接到通用执行链。当前结论是：**在此合成直线走廊、有界横流且持续提供窄目标线索的条件下，新方法的软件任务链可完成有限巡检，或分层拒绝并维持有状态输出。** 这不证明目标物理身份、真实声呐概率校准、实艇转弯和停止距离、PC104 收帧、海况成功率或现场许可。

将来若要解除仿真门，至少需要可审查的目标身份或经批准的单源窄测绘走廊、带时效和不确定度的实测横流／导航、扫描与接入后的目标线曲率再观测、PC104 原始收帧与实艇滑行包络。若身份线索不能持续提供，则该多横切模式只能输出**未具名磁异常测线**并 Hold，不能声称已巡检指定海缆。现有单次飞越模式仍可作为较窄工程条件下的另一路候选；两种路线的现场许可均保持 `false`。

## 复现与数据

参数在 [`cable_tracking_v2_shallow_survey_sim.yaml`](../../../brain_linux/src/auv_cable_tracking_v2/config/cable_tracking_v2_shallow_survey_sim.yaml)；核心算法在 [`shallow_survey.py`](../../src/auv_mag_tracking/v2/shallow_survey.py)，ROS 入口在仓库 `brain_linux/src/auv_cable_tracking_v2/`。运行时哈希与轨迹 SHA-256 写入各例 `*_summary.json`；[图 51 清单](publication/fwhm_v2_experiment_results/data/51_shallow_survey_ros_pvs_chain_manifest.json)逐例核对图像与原始轨迹，[最终原始数据](../../results/v2/shallow_survey_ros_pvs_final_20260927/)保留五例。早期航向单位错误、过紧航向门以及五域并行 UDP 超时的试跑保留在前一实验目录或日志中，没有覆盖旧图 47–50。

```bash
source /opt/ros/humble/setup.bash
source brain_linux/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=221
python3 tools/smoke_v2_shallow_survey_ros_pvs.py --case wire --current-y-mps 0.04 --with-bridge --output-dir /tmp/shallow_survey_example
python3 tools/plot_v2_shallow_survey_ros_pvs.py --allow-historical-source
```

现有代码已不是图 51 的运行时版本；以上第二条只按冻结 CSV 重绘旧图，若只核对原始哈希而不写图，可再加 `--verify-only`（已验证通过）。第一条运行的是**当前**源码，输出 `/tmp`，不应与图 51 的旧哈希混合。该入口是合成实验脚本；`shallow_survey_nodes.launch.py` 可装配专用 ROS 节点，默认不自动注入巡检任务，不能把直接启动 launch 当成同次闭环证据。

图 51 版本的收尾复核：重新构建 `auv_cable_tracking_v2` 成功，安装版 launch 的 `use_sim_time` 默认值为 `true`，且安装版 launch 与配置分别和源码 SHA-256 一致。当时 ROS 包测试 64 项、磁算法 v2 测试 83 项均通过；图 51 生成器重新核对五例运行时源码哈希和 CSV 哈希后生成清单。thuthesis 第 5 章引用图 5.23，整篇 221 页 PDF 编译通过，并检查了该图所在页面的版式。后续双时基修正的 ROS 包 66 项结果见图 52 报告，不能把计数回填为图 51 的运行时版本。
