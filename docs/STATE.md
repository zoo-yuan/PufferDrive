# STATE

As-of: 5cfe970a
（2026-11-04 会话：车辆/观测实测 + ACTION_TYPE_CONTROL + 数值对拍；本次提交仅补 As-of，不改代码）
约定：本仓库自有 `AGENTS.md` 仍是放置规则权威；strata 分层（STATE / adr / lessons）自 2026-10-09 启用。

## Where（现状，均已对照代码核实）

- 分支 `dev/zoo_v3.0`（3.0 族）。仿真核心 `pufferlib/ocean/drive/`：`drive.h` + 拆分头（`constants.h`/`datatypes.h`/`idm.h`/`map_data.h`/`rng.h`）。
- 行为编排：per-agent `CONTROLLER_{STATIC,POLICY,REPLAY,IDM}`（`sdc/non_sdc/non_vehicle_controller`），`simulation_mode: gigaflow|replay`；车流混合靠 `control_mode`（`control_vehicles/control_agents/control_tracks_to_predict/control_sdc_only`）+ per-agent controller + `init_mode`/`goal_source`/`resample_frequency`。**本仓无 fraction 旋钮**——`mix_traffic/ppo_fraction/idm_fraction/expert_fraction` 属 behavior-bench（2026-10-31 订正）。
- 地图二进制（`map_data.h`）：agents(log 轨迹+route+goals) + roads + traffic + 静态 objects + `scenario_id[128]`；map/neighbor cache。
- 评估：`config/evaluation/benchmark.yaml`（carla/womd/nuplan），种子化失败回放（#533，已取代旧 `mining_viz.py`），18 项指标。
- 训练：PPO + V-trace + 优先采样 + advantage filtering（`adv_filter_*`）。
- **本分支没有**：pose/brownian 噪声（仅在 `origin/8_bernhard_dev`、`origin/9_bernhard_dev`）；comfort 仅有软奖励+指标，无动作空间硬约束。
- **本仓库不实现 Nova 契约**：契约真值在 nova 仓（`~/labs-x/agent_group/nova`）——数据规范 `docs/dataset/数据规范V1.1.3_0911修正版.md`（0911 数据）+ nova `docs/adr/0003`（参考线槽位）/ `0004`（19 输入/4 输出、static_polygon_*）。本仓只负责 bridge（PufferDrive 世界 → 19 输入/4 输出）。

### Docs 台账（冻结检查依据）
- `docs/2026-09-03-self-play-in-autonomous-driving-survey.md` — owner 自有 survey，**现行**（owner 修订至 2026-09-28）。
- `docs/2026-09-04-*-self-play架构分析.md`（+.html 于 docs/html/）— 研究叙事；PufferDrive 能力描述基于 2.0，已加状态注记。
- `docs/2026-09-09-GigaFlow-vs-GigaPixel-*.md` — 文献对比仍有效；PufferDrive 章节 2.0 时代，已加注记。
- `docs/2026-09-11-PufferDrive加速Nova-model-planner迭代方案.md` — 基于 2.0 的旧方案，**部分被取代**，已加注记指向现行 gap 分析。
- `docs/integration/` — 09-11 文献综述与调研计划、**10-09 三份现行文档**（分数悖论与自有数据策略 / gap 分析 / 感知抖动=第③轴）；`CONTEXT.md` 为集成术语表（PufferDrive Core / 语义外挂 / Scene / M1-M2 / E1a-E1b / Nova 部署契约 / 参考线槽位 / ego-centric RFU）。
- **外部锚点（非本仓）**：Nova 现行部署契约 = 19 输入 / 4 输出（nova `docs/dataset/数据规范V1.1.3_0911修正版.md` + nova ADR-0003/0004，2026-10-30 生效）；本仓对应裁决见 `docs/adr/0006-nova-contract-19-in-4-out-static-polygon.md`。
- `docs/adr/0001-0007` — 架构裁决（见 Open rulings）。
- `docs/2026-10-31-PufferDrive-vs-TerraZero-仿真器对比.md` — 外部参照（TerraZero 闭源技术报告口径）：数据哲学、吞吐量级、场景多样性来源；**不做选型裁决**，三个开放问题见 Open rulings。
- `docs/2026-11-03-Terra系列全景与PufferDrive落地路线.md` — Terra 系列**全四件工作**（TerraZero/SPACeR/TerraTransfer/TerraTransfer++）一手页面核对 + 五成分因果链（含 SPACeR 纯 PPO 基线反证）+ 逐工作落地映射；扩展 10-31 文档（仿真器单轴 → 全系列）；核实 3D 碰撞 z 轴拒绝已达标；**不做选型裁决**，对三个开放问题给建议。
- `docs/2026-11-04-badcase-as-scenario-seed-constraint-loop.md` — **现行**（bad case 闭环轴）：bad case = 场景种子；A-only 约束反馈下 1→100 的回路、20 条约束模板库、两个 KPI（约束覆盖率 / 回归保留率）；含三项本轮修正（A/B 执行层等价、PPO 似然来源=90 模态 categorical、Nova 降级为基线）与两项待实测（积分器差异、plan→pose 误差标定）。**不重复 Terra 对比**。
- `docs/vehicle_profile.md` — **车辆参数单一真值源**（非叙事台账，可复用）：3799 / P04-161X 的原始字段与语义解析（方向盘 vs 前轮、后轴 vs 包围盒中心、`min_turn_radius` 是外前角）、派生量（δ_max 0.60564 rad / δ̇_max 0.42336 rad/s / κ_max 0.3147 / 后轴→中心偏移 1.085 m / 满舵 1.43 s）、PufferDrive 限幅映射与三个已知不一致、`a_lat` 实测（暂定 8.0 m/s²，596 s 样本未冻结）与数据质量发现（湖里有 38 m/s 的定位跳变，入口必须做物理可行性门禁）、新车型接入模板、工具（`data_utils/apollo_record.py` + `data_utils/measure_lateral_accel.py`）。新增车型按 §6 复制，不另起文件。
- `docs/adr/0007-badcase-as-scenario-seed-and-constraint-only-feedback.md` — accepted 2026-11-04：反馈契约 A-only、RL 为核心手段、canonical spec 模型无关、occ 为物理实体、两个合法性域、舒适硬约束、KPI 换口径。
- `docs/adr/0008-map-topology-encoding-adjacency-core-turn-sidecar.md` — accepted 2026-11-05：地图拓扑编码——邻接（predecessor/successor）用 Core 已有的 `entry_lanes/exit_lanes` 不改结构；`turn_type` 走 map 侧表；`overlap` 只在语义外挂消费。不触发 40 个 map bin 重建。
- `docs/2026-11-04-session-handoff.md` — **会话交接**：30 秒恢复上下文、八条裁决 + 证据、已完成产物表、关键实测数字、四个阻塞项、下一步依赖顺序、复现命令。**新会话从这里开始。**
- `docs/2026-11-05-canonical-spec-v0.md` — **现行**（M1/M2）：`spec/canonical_v0.yaml` 73 字段注册表 + 两个可复跑闸门（`scripts/spec_report.py` 覆盖率、`scripts/spec_audit.py` 三源审计）。含两处被真数据推翻的假设（occ 不是占据栅格、Core 已有 entry/exit）与 7 个「仅数据源」字段清单。验收物 `docs/html/spec_v0_coverage.html` / `spec_v0_data_audit.html`。
- `docs/2026-11-05-canonical-spec-v0.md` §7–§8 — **M3/M4**：观测包络（70 m / 0.78 s）与约束模板库 v0（20 条，20/20 可执行谓词，3 控制器可行性裁判）。验收物 `docs/html/spec_v0_obs_envelope.html` / `constraints_v0_library.html` / `constraints_v0_feasibility.html`。
- `docs/html/milestone_review.html`（`scripts/milestone_review.py`）— **milestone 总览评审看板**：流程条 + 每站 KPI + 图表 + 被推翻的假设台账 + gate 台账。**每个 milestone 收口必须重新生成**（仪式见 `docs/2026-11-05-canonical-spec-v0.md` §10）。数字一律从原始产物重算，不手抄。
- `docs/2026-11-04-session-causal-chain.md`（HTML 于 `docs/html/`）— 本次会话的因果链复盘：五个转折（事实层 → 双视图 → 关键变量 → RL/种子 → 执行链）、总流程图、三条因果链（执行/世界/迭代）、数字表、现在能/不能、未决依赖图。**给人看的那份。**

## Next（下一步）

- 【effort】TerraZero 复刻路线 wayfinder 图：`../Slipstream/.wayfinder/map.md`。基底已裁决（2026-11-03）：**Slipstream 为主仓**，基线 = 上游 `3.0`（**2c1ca3e4**，拉最新后比 80a681b4 新 6 提交：Longest6v1/Alpasim eval、self-play eval 改进、overspeed reward 调整）保留历史导入；**已执行**：Slipstream master = 3.0 全历史 + tag `baseline/pufferdrive-3.0-2c1ca3e4` + 资产/strata 提交（a920284、0ebd01d）。本仓定位 = 只读跟随上游 + Nova bridge 线（`ACTION_TYPE_CONTROL` WIP 归此线收尾）；`9_bernhard_dev` 为挖矿源（噪声/权重按需 cherry-pick，P0 不变）。
- P0：从 `origin/9_bernhard_dev` 移植 pose/brownian 噪声 → 扩展为 track 级时间相关、CIPV 加权、进 16 帧历史的注入（ADR-0004；`docs/integration/2026-10-09-perception-jitter-as-the-real-axis3.md` §3）。
- P0：**积分器一致性**（ADR-0007 §6.1）——`ACTION_TYPE_CONTROL` + `INTEGRATION_SCHEME_MODEL_EULER` + `skip_actuator_limits` **已落地**，数值对拍 **通过**（`tests/unit_tests/test_control_mode_parity.py`，60 步 / 6 s，max `|Δxy|` 1.7e-5–3.4e-5 m）。**剩余**：桥接层实现 **acc wash-in**（Nova inference 对前4步加速度做 0.2/0.4/0.6/0.8 插值；不洗则 6 s 内偏离 0.39–0.92 m，`test_raw_control_diverges_from_model_intent` 已钉住）。限幅值取 `docs/vehicle_profile.md` §3。
- P0 环境（2026-11-04）：`.venv` 已用 uv + 清华镜像建好（torch 2.15.0+cu132 / numpy / gymnasium / hydra / omegaconf / matplotlib / pytest 均可用）。**`pufferlib._C` 编译不了**：本机 g++ 9.4（Ubuntu 20.04）不支持 `-std=c++20`（`torch/include/c10/util/intrusive_ptr.h` 需 `<compare>`）——与本次代码改动无关，阻塞的是 `puffer train` 本身。修法：装 g++-11/13 或换新工具链。`pufferlib/ocean/drive/binding*.so` 已用 gcc 直编就位（C 部分无此问题）。
- P0：**执行限幅标定落地**（`docs/vehicle_profile.md` §4）——`STEERING_LIMIT` 0.667→0.60564（classic+jerk 两处同源）、转角速率 0.6→0.42336（`drive.h:4241/4384`）、`ACCEL_LONG_LIMIT`→{-6.0,3.0}、ego `wheelbase`→2.20；`base_max_speed_mps` → **底盘设计上限 16.7**（12.5 是 AD 系统能力，另立为约束/指标，见 §4.1）；control 模式撤掉 `ACCEL_LAT_LIMIT` 硬夹，暂用实测代理上限 8.0 m/s²（`data_utils/measure_lateral_accel.py`，596 s 样本，待在数据湖上重跑）。
- P0：**观测包络对齐**（`docs/vehicle_profile.md` §4.2）——**2026-11-05 已落地**：6 条录包实测冻结 `agent_range_m=70`（>100 m 障碍物 0 个）与 `observation_delay_s=0.78`（control 侧中位；原候选 0.53 仅来自 rx-2714 单条）。仿真侧：`obs_range_partner_m` 200→70，新增 `Drive._apply_observation_delay` ring buffer（量化到 dt，实际上报 `effective_observation_delay_s`），6 例对拍测试钉住语义。验收物 `docs/html/spec_v0_obs_envelope.html`。
- P0：**约束模板库 v0（20 条）+ 可行性裁判**（ADR-0007 §3–4）——**2026-11-05 已落地库层**：`spec/constraints_v0.yaml`（violation 16 = 80% / comfort 3 / progress 1），20/20 谓词可执行（合成触发/不触发两行验证），85% 只用引擎现有 18 项指标；3 条需桥接新指标（crosswalk/pedestrian 间距、junction 占用）。可行性裁判已能真跑（3 控制器 × 40 步 ≈ 2 s，18 可解 / 2 IDM 不足 / 0 不可解）。**遗留**：`replay` 在 gigaflow 下不是真见证人，真正裁判必须跑在录制场景上；IDM 自身触发 comfort 阈值待查。
- P0：**bad case 自动挖掘 + 场景切片**（ADR-0007 §3 步骤 1–2）——用 `metrics_array` 18 项当谓词批量扫数据湖，切片窗口 16+80，`scenario_id` 编码 `bag_id#frame`，附 provenance（地图/感知版本）。违规型零标注，可夜间批处理。入口必须做**物理可行性门禁**（实测湖里有 38 m/s 的定位跳变；`docs/vehicle_profile.md` §5.1）。
- P0：最小 Nova 契约 bridge（**19 输入 / 4 输出**，ADR-0006），先在公开 WOMD 场景跑通：产出 `static_polygon_*` 四路（≤32 polygon，cap 截断须上报丢弃数）+ 3 槽几何序参考线（slot0 必须真实 ego 几何，全零帧跳过推理）+ ego-centric RFU 重抽（M1 复用录制帧 14ch；生产侧在 `wt-apollo/record_exporter/frame_builder.h`）。**定位为基线实例，不是训练接口的唯一定义（ADR-0007）。**
- P1：canonical scene/feature spec 冻结（模型无关；含 provenance/感知版本/地图版本 + `map_range_m` / `agent_range_m` / `observation_delay_s` 三个观测包络字段）——L0 是唯一不可替换资产。**2026-11-05 进展**：`spec/canonical_v0.yaml` 已落地（73 字段 / 8 组 / frozen 56 = 76.7% / blocked 1），gate 与三源审计均可复跑；6 条真 bag + 3 张真地图实证：40 三源齐备、26 待桥接、7 仅数据源、0 凭空。剩 16 draft 等 M3 观测包络落地后一次冻。见 `docs/2026-11-05-canonical-spec-v0.md`。
- P1：对手混合 Σ（复用 behavior-bench traffic registry / `conditioned_mix`）；Apollo HD map → PufferDrive bin 最小转换（仅几何+交通控制+route）。
- P2：程序化长尾 hazard 生成层（TerraZero 路线，见 11-03 文档 §4A）；Comfort-by-Construction 硬约束动作空间（ADR-0007 已升为必需项）。

## Open rulings（裁决状态）

- ADR-0001 Core+语义外挂：**accepted** 2026-09-11（owner）。
- ADR-0005 M1 先行 + Scene 契约：**accepted** 2026-09-11（owner）。
- ADR-0002 部署契约内训练 / ADR-0003 混合对手必需 / ADR-0004 track 级抖动：**proposed** 2026-10-09，待 owner 确认。
- ADR-0006 Nova 契约代际切换（19 输入 / 4 输出）：**accepted** 2026-10-31（依据 nova 一手文档与 nova ADR-0003/0004）；同时修正 ADR-0002 / ADR-0005 的代际指针。
- ADR-0007 bad case=场景种子 / A-only 约束反馈 / canonical spec 模型无关 / occ 为物理实体 / 两个合法性域 / 舒适硬约束 / KPI=约束覆盖率+回归保留率：**accepted** 2026-11-04（owner 本会话裁决）。ADR-0002「仅在 Nova 契约内训练」由此**降级**为「基线模型在契约内」。
- ADR-0008 地图拓扑编码（邻接留 Core / turn 走 map 侧表 / overlap 只在语义外挂）：**accepted** 2026-11-05（owner 裁决，依据 3 张真地图 100% 拓扑覆盖 + Core `entry_lanes/exit_lanes` 已存在）。
- 新增未决（2026-11-04）：canonical spec 字段清单与版本策略；走廊域 vs 漫游域的合法性判据归属；约束模板库 owner 与入库流程（体感型 20% 的转写人）；plan→pose 误差标定口径；**AD 系统能力上限 12.5 m/s 的观测层成因**（感知最远距离 / perception→planning 延迟 / planning→control 延迟）待从 bag 实测，它决定仿真的 obs 范围与延迟参数。
- 未决（阻塞项，见 `.scratch/handoff/`）：抖动过滤归属层（感知平滑 / planner 自滤波 / 独立估计层）；观测契约全量 vs 子集；Σ 用固定 fraction 还是 policy pool/league；基线是否钉死 `dev/zoo_v3.0`。
- 新增未决（2026-10-31）：PufferDrive world 系（heading 0 = +x，`Agent.cos_heading/sin_heading`）→ Nova ego-centric RFU（直行 = π/2）的偏置换算未实测；`static_polygon` cap 32 的 PufferDrive 侧丢弃数上报口径未定。
- 开放问题（2026-10-31，见 `docs/2026-10-31-PufferDrive-vs-TerraZero-仿真器对比.md` §4；2026-11-03 补充分析与建议见 `docs/2026-11-03-Terra系列全景与PufferDrive落地路线.md` §6）：是否引入**程序化场景生成层**（TerraZero 路线，影响 ADR-0003 的 Σ 与长尾轴）；19 输入契约下是否改 **C 引擎 + GPU 零拷贝推理** 形态；背景车多样性用 PufferDrive per-agent controller 还是移植 behavior-bench 的 fraction 混合。

## Long-term（长期不变量）

- 集成目标：rino 数据湖（5000+ 车、自有 Apollo HD map、真实感知质量）+ 混合智能体闭环世界（replay/IDM/特殊行为/self-play）。**训练接口 = 模型无关 canonical spec（ADR-0007）；上线验收门 = Nova 19 输入 / 4 输出（ADR-0006）。**
- 泛化模型：三条独立轴（①地图几何 / ②交通行为 / ③观测可观测性）；self-play 的定位是**对手分布生成器 + 全场安全测试台**，不是闭环分数引擎。
- 表示一致性（训练契约 = 部署感知契约）与长尾覆盖度是两个命门；WOMD/nuPlan 仅作外部可比基准与冷启动。
