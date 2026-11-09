# STATE

As-of: d9efa3c9（代码）/ 2026-11-06 会话③（scripts+docs 同日更新，见 ADR-0011）
（2026-11-05 会话：回归上游 3.0 → M1 spec 冻结 → M2 三源审计 → M3 观测包络 → M4 约束库+可行性 → M5 挖掘 v0 → M6 桥接 wash-in + 地图匹配。全部 gate PASS。
2026-11-06 会话①：撤销冻结基线（harness bug）→ 修复 harness（ADR-0010：mean 动作选择 + 训练 dt 契约）→ roundC 诚实数字 150/154，重训成为必经路。
2026-11-06 会话②：事件级越道奖励落地（`reward_lane_departure`，每集一次性、谓词与 judge 同源）→ 重训 event_reward_v1（900k/20min）→ **负结果**：154/154 违规。
2026-11-06 会话③：奖励尺度审计推翻会话②的归因 → 诊断出两个结构性阻塞：**①训练快照 `reward_randomization: true` 让配置里的奖励系数全部失效**（旧计划"减 reward_velocity"本是空操作）；**②dt=0.3 下横向动作退化成 bang-bang**（IDM 对照：max lane_dist 4.0 vs 0.1 s 的 1.2）。落地 `scripts/reward_scale_report.py` 四道尺度门 + 三轮实验（v2a/v3/v4），见 ADR-0011。）
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
**现行（作为事实来源）**
- `docs/2026-11-04-session-handoff.md` — 会话交接：30 秒恢复上下文、裁决+证据、产物表、阻塞项、复现命令。**新会话从这里开始。**
- `docs/2026-11-05-canonical-spec-v0.md` — 主线：M1/M2（73 字段注册表 + 覆盖率/三源审计两个闸门）、M3/M4（§7–§8 观测包络 + 约束库）、M5/M6/M7（§9–§11 挖掘契约 + 桥接 & 地图匹配 + 场景化计划；§10.5 是"分块地图≠缺地图"的撤销记录）。验收物 `docs/html/` 下同名 HTML。
- `docs/2026-11-04-badcase-as-scenario-seed-constraint-loop.md` — bad case 闭环轴：种子化、A-only 约束反馈、两个 KPI（约束覆盖率 / 回归保留率）。
- `docs/baselines.md` — **冻结基线清单**（checkpoint / sha256 / 数字 / 复现命令 / 失效条件）；`docs/partner-agents-explained.md` — 补对手机制科普；`docs/vtrace-explained.md` 与 `docs/ppo-training-walkthrough.md` — 训练实现参考。
- `docs/vehicle_profile.md` — **车辆参数单一真值源**（3799 / P04-161X 原始字段 + 派生量 + 限幅映射 + 新车型模板），新增车型按 §6 复制，不另起文件。
- `docs/integration/CONTEXT.md` — 集成术语表（PufferDrive Core / 语义外挂 / Scene / M1-M2 / E1a-E1b / Nova 部署契约 / ego-centric RFU）。
- `docs/adr/0001-0011` — 架构裁决（状态见 Open rulings）。**外部锚点**：Nova 部署契约 19 输入/4 输出以 nova 仓 `数据规范V1.1.3_0911修正版.md` + nova ADR-0003/0004 为准（2026-10-30 生效）。

**参考（有代际注记，不当现行事实）**
- `docs/2026-09-03-self-play-*-survey.md`（owner 现行至 2026-09-28）/ `2026-09-04-*架构分析` / `2026-09-09-GigaFlow-vs-GigaPixel-*` / `2026-09-11-*加速Nova*方案`：PufferDrive 能力描述基于 **2.0**，均已加状态注记；09-11 方案**部分被取代**。
- `docs/2026-10-31-PufferDrive-vs-TerraZero-*.md` / `docs/2026-11-03-Terra系列全景与PufferDrive落地路线.md`：外部参照（不做选型裁决），开放问题见 Open rulings。
- `docs/2026-11-04-session-causal-chain.md`（+HTML）：上一轮因果链复盘，**给人看**，不作事实源。
- `docs/html/*.html`：全部由 `scripts/*.py` 生成的验收物，数字一律从 `docs/measured/*.json` 重算 —— 不手抄。

## 冻结基线（2026-11-06：**当前无冻结基线**）

- **roundC 已撤销且经修复后 harness 实测否定**：`checkpoints/lane_departure_roundC.pt` 在能动的评测里违规 **150/154**、中位车速 1.37 m/s——旧 97.4% 修复率全部是趴窝假象（argmax 自锁，ADR-0010）。重训前不得引用任何 roundC 数字。
- **2026-11-06 会话③同样没有达标 checkpoint**，但阈值口径已定（ADR-0011），三臂诚实数字（154 场景 `crossbag_scenes`，各在自己快照下评测）：

| 臂 | 配置 | 违规 | 中位车速 | 中位 lane_dist 峰值 | clamp(≥3.99) |
|---|---|---|---|---|---|
| roundC | dt 0.3（自己的快照） | 150/154 | 1.37 | 1.007 | 19 |
| event_reward_v1 | dt 0.3 + 一次性越道罚 5.0 | 154/154 | 1.85 | 4.0 | 154 |
| v2a | dt 0.3 · 关 randomization · 删一次性罚 · lane_center 0.04 | 154/154 | 4.54 | 4.0 | 154 |
| v3 | dt 0.1 · lane_center 0.08 · **只跑 300k** | 141/154 | 1.15（爬行） | 1.953 | 70 |
| v4 | dt 0.1 · lane_center 0.08 · velocity 0.05 · 900k | 153/154 | 2.34 | **1.445** | 70 |

  记录件：`docs/measured/regression_retention_v4.json`（v4）+ `.scratch/experiments_v{2a,3,4}/regression_retention.json`；负结果与推理链在 `.scratch/experiments_v4/RATIONALE.md`。
- 判据侧仍有效：录制司机 154/154 违规（**但峰值中位只有 1.13 m，离 0.9 m 门限仅 ~0.3 m 余量，可行性口径待重标**）、IDM 0/168（口径未动）。
- 教训：`num_reward_coefs` 是观测向量的一部分（`context_dim = num_reward_coefs + goal_dim`），
  往系数表里加项等于改观测契约、让所有旧权重失效 —— 新 shaping 系数必须绕开这个数组。
  **同一条也适用于改系数的"值"**：同一份权重换一份快照评测，中位车速 1.85 → 4.56 m/s、
  clamp 场景 19 → 131（会话③实测）。多轮实验必须 `--run-config <data_dir>/config.yaml`。

## Next（下一步）

- **P0（当前焦点）：把策略从「median peak 1.445 m」推进到「全程 < 0.9 m」**。会话③把奖励与 dt 两个结构性阻塞清掉了（ADR-0011），剩下的问题很具体：v4（dt 0.1、lane_center 0.08、velocity 0.05、900k）中位 `lane_dist` 峰值 **1.445 m**、中位车速 2.34 m/s，但 153/154 仍越道；v3 只跑 300k 时违规 141/154 却在爬行（1.15 m/s）。`episode_length` 从 16/60 升到 43–56/60 但**物理时长没变**（≈4.4–4.8 s）——说明瓶颈已不是"活多久"而是**横向精度**。下一步候选（按性价比）：
  ① 训练量：v4 只 900k 步（dt 0.1 下等于 1/3 的仿真秒），先跑到 2.7M 步看 `num_goals_reached` 能否跳出 0（它一直是 0，意味着正向通道只有 velocity 一项在起作用）；
  ② 横向动作粒度：离散 3 档 jerk {-4,0,+4} 在 dt 0.1 下仍是 ±0.4 m/s²/步，做 <0.9 m 的保持未必够——候选是加细横向档（如 {-4,-1.5,0,1.5,4}）或换连续动作头；改档位等于改动作表，旧权重全废，须新开一轮；
  ③ 判据可行性重标：`lane_departure` 的 0.9 m 门限 vs 录制司机峰值中位 1.13 m，余量只有 ~0.3 m；"avoidable"口径要按实测余量重新定义，否则"全修好"可能本来不可达；
  ④ 课程学习（先直道后弯道）仅在 ①②③ 都不动时再上。
  开训前必须先过四道尺度门：`python scripts/reward_scale_report.py --config <data_dir>/config.yaml --check` + `pytest tests/unit_tests/test_reward_scale_contract.py`（`live_coefs` / `lane_signal` / `event_dominance` / `positive_channel`）。验收门不变（ADR-0010 + 中位车速 gate + 参考 154/154），达标才重冻基线。
- **训练/评测的三个坑（会话③新踩）**：① `reward_randomization: true` 时配置里的奖励系数**全部不生效**（被 `REWARD_BOUNDS` 抽样覆盖），调参轮必须关；② 奖励系数的**值**也是观测上下文，换快照评测会造出 2.5 倍的行为差异（用 `--run-config`）；③ `num_reward_coefs` 数组不许加项（旧权重全废），新 shaping 走独立 env 字段。
- 训练配置注意：base `puffer_drive.yaml` 默认 `simulation_mode: gigaflow`，场景 bin 训练必须 override `=replay`（gigaflow 模式在场景 bin 上 spawn 全败）；override 从 `.scratch/config_roundC_reconstructed.yaml` diff 生成，`train.data_dir` 每轮独立防覆写；迭代成本 13 min/900k 步（dt 0.3）或 ~17 min/900k（dt 0.1）。
- 【effort】TerraZero 复刻路线 wayfinder 图：`../Slipstream/.wayfinder/map.md`。基底已裁决（2026-11-03）：**Slipstream 为主仓**，基线 = 上游 `3.0`（**2c1ca3e4**，拉最新后比 80a681b4 新 6 提交：Longest6v1/Alpasim eval、self-play eval 改进、overspeed reward 调整）保留历史导入；**已执行**：Slipstream master = 3.0 全历史 + tag `baseline/pufferdrive-3.0-2c1ca3e4` + 资产/strata 提交（a920284、0ebd01d）。本仓定位 = 只读跟随上游 + Nova bridge 线（`ACTION_TYPE_CONTROL` WIP 归此线收尾）；`9_bernhard_dev` 为挖矿源（噪声/权重按需 cherry-pick，P0 不变）。- P0：从 `origin/9_bernhard_dev` 移植 pose/brownian 噪声 → 扩展为 track 级时间相关、CIPV 加权、进 16 帧历史的注入（ADR-0004；`docs/integration/2026-10-09-perception-jitter-as-the-real-axis3.md` §3）。
- P0：**桥接层 acc wash-in**（ADR-0007 §6.1）——**2026-11-05 已落地**：`pufferlib/ocean/drive/control_bridge.py`（复刻 nova 前 4 样本插值）；洗过 6 s 内 max |Δxy| 3.0e-5 m，不洗 0.39–0.92 m（`docs/html/bridge_washin_report.html`，5 例单测钉住）。
- P1：**地图匹配层**——**2026-11-05 已落地**：`data_utils/apollo_lane_index.py`（车道点列 + 网格索引 → 有符号横向偏移/航向差/限速/turn）。**两种地图形态都要支持**：单 `base_map.bin`（wuhan/chongqing/8997）与**分块地图**（6501/7235：`base_map.catelog` + 6,249/10,874 个 500 m tile，按 bag 轨迹 bbox 选片）。6/6 条 bag 匹配率 100%，置信匹配率 5 条 100% / 2292 72%（场站挪车区无车道线）。匹配器硬门限：航向差 ≤60°、横向 ≤3 m，否则不出判定。解锁 4 条车道类约束（挖掘契约 8→12，切片 487→779）。
- P0：**M7 场景化**（§11）——① route/goal 推导**已落地**：`scripts/route_preview.py`（单切片几何+车道链）+ `scripts/route_coverage.py`（批量）；**779/779 切片能串出连续 route**，其中走满 1000 m 的 497 条，短 route 282 条（3139 受引擎 64 段上限限制 425 m，2688 是小区环道 307 m）。② 切片转 PufferDrive 场景 bin **已落地**：779/779 写成 `load_map_binary` 可直接吃的 bin（结构问题 0，场景车道中位 68，route 段中位 56），replay 模式逐步对拍 log 偏差 **0.000 m**（每 bag 抽 2 条 × 40 步）；③ 场景级可行性裁判**已落地**：3183 的 153 个场景 × 3 controller 真跑，**可测切片复现率 71/76 = 93%**，IDM 0 条违规（无常"逼人违规"场景）；抓出 `lane_departure` 缺运动 guard（已补 0.5 m/s）与 replay 模式指标恒 0 的口径问题；④d 交互类奖励项：**已接进引擎但方向错**。`reward_safety_gap`（用 `distance_to_collision`，因为 `ttc` 实测只取 0/5.0 二分）可调可跑；但逐帧几何惩罚把自车类违规从 70 推到 143、对手类仍 39→39，7 条变坏 → checkpoint 被拒。正确形态应是**事件级**约束奖励（场景结束按谓词给一次 ±奖励），见 §14.6。
④c 扩到 328 场景（154 自车 + 174 补对手）**失败并被拒**：自车类 70→84 违规（27 变坏），对手类 39→39 无变化，回归保留率 88%。原因：`total_timesteps` 数所有 agent 步、25 车场景里只有自车被控 → 2M agent-steps 仅 ≈80k 自车步（round C 是 900k）；且对手类模板缺对应奖励项。**生产口径仍用 round C**。
④b 补对手（`scripts/partner_scenes.py`）：257 个 `needs_partners` 切片里 **174 个（68%）** 补上对手后能复现；3123 从 0/39 → 39/39 是因为 `cut_in_conflict` 的 guard 阈值与引擎 `lane_dist` 有系统偏差（0.97 vs 1.0），已改 0.9。
④ 回归保留率**跨 3 条 bag 闭环跑通**：154 个可避场景（3183:71 / 3123:50 / 3139:33），round A（300k）141 违规 → B（600k）20 → C（900k）**4**，相对录制司机修复率 **97.4%**，回归保留率 **100%**、被改坏 0；剩 4 条集中在 3139 同一路口碎段簇。原先单 bag 版本：71 个可避场景，replay 参考 71/71 违规，round A（300k 步）降到 1/71，round B（续训 300k 步）降到 **0/71 → 修复率 100%**，回归保留率 100%、被改坏 0；同 seed 可复现。沿途抓出三个静默问题：`scenario_id` 撞车（779 切片实际 82 场景，已修）、场景 bin 缺限速区段（引擎拒载，已修）、replay 下 `sim_speed` 恒 0 导致速度门限谓词全失效（已改为相邻位姿反推）。**场景包络 20→40 m 会改变结论**（多数早期"越道"是包络不足的假阳性），已记为口径变更。
  **⚠ 2026-11-06 作废注记**：本段 round A/B/C 全部违规数与修复率系趴窝 harness（argmax 自锁）产物，已被 ADR-0010 修复后实测否定（roundC 真实违规 150/154）；判据侧（154/154、静默问题修复）仍有效。详见 `docs/2026-11-05-canonical-spec-v0.md` §15。
- P0：**积分器一致性**（ADR-0007 §6.1）——`ACTION_TYPE_CONTROL` + `INTEGRATION_SCHEME_MODEL_EULER` + `skip_actuator_limits` **已落地**，数值对拍 **通过**（`tests/unit_tests/test_control_mode_parity.py`，60 步 / 6 s，max `|Δxy|` 1.7e-5–3.4e-5 m）。**剩余**：桥接层实现 **acc wash-in**（Nova inference 对前4步加速度做 0.2/0.4/0.6/0.8 插值；不洗则 6 s 内偏离 0.39–0.92 m，`test_raw_control_diverges_from_model_intent` 已钉住）。限幅值取 `docs/vehicle_profile.md` §3。
- P0 环境（2026-11-06 更新）：**`puffer train` 已可用**。系统 g++ 9.4 编不了 `pufferlib._C`（torch 强制 `-std=c++20`，且 9.4 缺 `<compare>`），但**不需要升级系统编译器**：gcc 13 装在 conda 的 build-only 环境 `pufdrive-gcc13` 里，只有 `scripts/build_cpu_ext.sh`（编 `_C`）与 `source scripts/puffer_env.sh`（训练时 inductor 现编 C++）会把工具链挂到当前 shell。`setup.py` 新增 `PUFFER_BUILD_CPU_ONLY=1` 开关绕开 CUDA 12.6/13.2 不匹配。实测：C 测试全 PASS、Python 187 passed / 21 skipped、`python -m pufferlib.pufferl train puffer_drive ...` 跑完并写出 checkpoint。**GPU 已可用**（2026-11-06）：`.venv` 的 torch 从 `2.15.0+cu132` 换成 **`2.14.1+cu126`**（driver 535 / CUDA 12.2 通过 minor version compatibility 跑 cu126），与本机 toolkit 12.6 一致 → `_C` 的 CUDA 内核可编，`torch.cuda.is_available()` 为 True（RTX 3060 / sm_86）；CPU 与 CUDA 两路 V-trace 均已对拍通过，`train.device=cuda` 在 `compile=false` 与 `compile=true`（inductor）下都能跑完并写出 checkpoint。回滚清单在 `.scratch/venv-freeze-before-torch.txt`。
- 旧记录（2026-11-04）：`.venv` 已用 uv + 清华镜像建好（torch 2.15.0+cu132 / numpy / gymnasium / hydra / omegaconf / matplotlib / pytest 均可用）。**`pufferlib._C` 编译不了**：本机 g++ 9.4（Ubuntu 20.04）不支持 `-std=c++20`（`torch/include/c10/util/intrusive_ptr.h` 需 `<compare>`）——与本次代码改动无关，阻塞的是 `puffer train` 本身。修法：装 g++-11/13 或换新工具链。`pufferlib/ocean/drive/binding*.so` 已用 gcc 直编就位（C 部分无此问题）。
- P0：**执行限幅标定落地**（`docs/vehicle_profile.md` §4）——`STEERING_LIMIT` 0.667→0.60564（classic+jerk 两处同源）、转角速率 0.6→0.42336（`drive.h:4241/4384`）、`ACCEL_LONG_LIMIT`→{-6.0,3.0}、ego `wheelbase`→2.20；`base_max_speed_mps` → **底盘设计上限 16.7**（12.5 是 AD 系统能力，另立为约束/指标，见 §4.1）；control 模式撤掉 `ACCEL_LAT_LIMIT` 硬夹，暂用实测代理上限 8.0 m/s²（`data_utils/measure_lateral_accel.py`，596 s 样本，待在数据湖上重跑）。
- P0：**观测包络对齐**（`docs/vehicle_profile.md` §4.2）——**2026-11-05 已落地**：6 条录包实测冻结 `agent_range_m=70`（>100 m 障碍物 0 个）与 `observation_delay_s=0.78`（control 侧中位；原候选 0.53 仅来自 rx-2714 单条）。仿真侧：`obs_range_partner_m` 200→70，新增 `Drive._apply_observation_delay` ring buffer（量化到 dt，实际上报 `effective_observation_delay_s`），6 例对拍测试钉住语义。验收物 `docs/html/spec_v0_obs_envelope.html`。
- P0：**约束模板库 v0（20 条）+ 可行性裁判**（ADR-0007 §3–4）——**2026-11-05 已落地库层**：`spec/constraints_v0.yaml`（violation 16 = 80% / comfort 3 / progress 1），20/20 谓词可执行（合成触发/不触发两行验证），85% 只用引擎现有 18 项指标；3 条需桥接新指标（crosswalk/pedestrian 间距、junction 占用）。可行性裁判已能真跑（3 控制器 × 40 步 ≈ 2 s，18 可解 / 2 IDM 不足 / 0 不可解）。**遗留**：`replay` 在 gigaflow 下不是真见证人，真正裁判必须跑在录制场景上；IDM 自身触发 comfort 阈值待查。
- P0：**bad case 自动挖掘 + 场景切片**（ADR-0007 §3 步骤 1–2）——**2026-11-05 已落地 v0**：契约 `spec/bag_mining_v0.yaml` 显式写死 8 条可挖（ttc/braking/collision/cut_in/lat_accel/long_accel/jerk/stall）与 12 条不可挖（需地图匹配/责任判定）；`scripts/badcase_mine.py` 扫 6 条录包 → 8992 可信帧 → 切片；M6 加地图匹配后契约 **12/20 可挖**，切片 **779**（jerk 231 / cut_in 156 / lane_departure 156 / long_accel 104 / ttc 60 / wander 27 / braking 27 / collision 14 / stall 4；overspeed 与 heading_mismatch 为 0）。入口物理可行性门禁拒绝 22 帧（−16.8 m/s²）。验收物 `docs/html/badcase_mining_v0.html`。**遗留**：余 8 条不可挖需要责任判定 / 跨车道时间真值 / 斑马线与行人指标，不是地图匹配能解决的。
- P0：最小 Nova 契约 bridge（**19 输入 / 4 输出**，ADR-0006），先在公开 WOMD 场景跑通：产出 `static_polygon_*` 四路（≤32 polygon，cap 截断须上报丢弃数）+ 3 槽几何序参考线（slot0 必须真实 ego 几何，全零帧跳过推理）+ ego-centric RFU 重抽（M1 复用录制帧 14ch；生产侧在 `wt-apollo/record_exporter/frame_builder.h`）。**定位为基线实例，不是训练接口的唯一定义（ADR-0007）。**
- P1：canonical scene/feature spec 冻结（模型无关；含 provenance/感知版本/地图版本 + `map_range_m` / `agent_range_m` / `observation_delay_s` 三个观测包络字段）——L0 是唯一不可替换资产。**2026-11-05 现状**：`spec/canonical_v0.yaml` 73 字段 / 8 组 / **frozen 59 = 80.8% / draft 14 / blocked 0**；三源审计 **41 齐备 / 26 待桥接 / 6 仅数据源 / 0 凭空**。剩 14 draft 待桥接层补齐后一次冻。见 `docs/2026-11-05-canonical-spec-v0.md`。
- P1：对手混合 Σ（复用 behavior-bench traffic registry / `conditioned_mix`）；Apollo HD map → PufferDrive bin 最小转换（仅几何+交通控制+route）。
- P2：程序化长尾 hazard 生成层（TerraZero 路线，见 11-03 文档 §4A）；Comfort-by-Construction 硬约束动作空间（ADR-0007 已升为必需项）。

## Open rulings（裁决状态）

- ADR-0001 Core+语义外挂：**accepted** 2026-09-11（owner）。
- ADR-0005 M1 先行 + Scene 契约：**accepted** 2026-09-11（owner）。
- ADR-0002 部署契约内训练 / ADR-0003 混合对手必需 / ADR-0004 track 级抖动：**proposed** 2026-10-09，待 owner 确认。
- ADR-0006 Nova 契约代际切换（19 输入 / 4 输出）：**accepted** 2026-10-31（依据 nova 一手文档与 nova ADR-0003/0004）；同时修正 ADR-0002 / ADR-0005 的代际指针。
- ADR-0007 bad case=场景种子 / A-only 约束反馈 / canonical spec 模型无关 / occ 为物理实体 / 两个合法性域 / 舒适硬约束 / KPI=约束覆盖率+回归保留率：**accepted** 2026-11-04（owner 本会话裁决）。ADR-0002「仅在 Nova 契约内训练」由此**降级**为「基线模型在契约内」。
- ADR-0008 地图拓扑编码（邻接留 Core / turn 走 map 侧表 / overlap 只在语义外挂）：**accepted** 2026-11-05（owner 裁决，依据 3 张真地图 100% 拓扑覆盖 + Core `entry_lanes/exit_lanes` 已存在）。
- ADR-0009 地图匹配接受判据（航向 ≤60° / 横向 ≤3 m 否则无判定 / 置信率是数据质量 / 分块地图按 bbox 选片）：**accepted** 2026-11-05（owner 指出误判后实测修正）。
- ADR-0010 评测 harness = 训练契约镜像（mean 动作选择 + 训练 dt）：**accepted** 2026-11-06（会话①）。
- ADR-0011 训练/评测的 dt 与奖励尺度是契约（场景 bin 线 dt=0.1；调参轮关 `reward_randomization`；训练与评测同一快照；四道尺度门是开训前置）：**accepted** 2026-11-06（会话③，owner 授权奖励搜索）。
- 新增未决（2026-11-04）：canonical spec 字段清单与版本策略；走廊域 vs 漫游域的合法性判据归属；约束模板库 owner 与入库流程（体感型 20% 的转写人）；plan→pose 误差标定口径；**AD 系统能力上限 12.5 m/s 的观测层成因**（感知最远距离 / perception→planning 延迟 / planning→control 延迟）待从 bag 实测，它决定仿真的 obs 范围与延迟参数。
- 未决（阻塞项，见 `.scratch/handoff/`）：抖动过滤归属层（感知平滑 / planner 自滤波 / 独立估计层）；观测契约全量 vs 子集；Σ 用固定 fraction 还是 policy pool/league；基线是否钉死 `dev/zoo_v3.0`。
- 新增未决（2026-10-31）：PufferDrive world 系（heading 0 = +x，`Agent.cos_heading/sin_heading`）→ Nova ego-centric RFU（直行 = π/2）的偏置换算未实测；`static_polygon` cap 32 的 PufferDrive 侧丢弃数上报口径未定。
- 开放问题（2026-10-31，见 `docs/2026-10-31-PufferDrive-vs-TerraZero-仿真器对比.md` §4；2026-11-03 补充分析与建议见 `docs/2026-11-03-Terra系列全景与PufferDrive落地路线.md` §6）：是否引入**程序化场景生成层**（TerraZero 路线，影响 ADR-0003 的 Σ 与长尾轴）；19 输入契约下是否改 **C 引擎 + GPU 零拷贝推理** 形态；背景车多样性用 PufferDrive per-agent controller 还是移植 behavior-bench 的 fraction 混合。

## Long-term（长期不变量）

- 集成目标：rino 数据湖（5000+ 车、自有 Apollo HD map、真实感知质量）+ 混合智能体闭环世界（replay/IDM/特殊行为/self-play）。**训练接口 = 模型无关 canonical spec（ADR-0007）；上线验收门 = Nova 19 输入 / 4 输出（ADR-0006）。**
- 泛化模型：三条独立轴（①地图几何 / ②交通行为 / ③观测可观测性）；self-play 的定位是**对手分布生成器 + 全场安全测试台**，不是闭环分数引擎。
- 表示一致性（训练契约 = 部署感知契约）与长尾覆盖度是两个命门；WOMD/nuPlan 仅作外部可比基准与冷启动。
