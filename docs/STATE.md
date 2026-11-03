# STATE

As-of: 7a36d34a
（2026-10-31 纯文档会话；docs HEAD = 7a36d34a，代码 HEAD = 80a681b4）
约定：本仓库自有 `AGENTS.md` 仍是放置规则权威；strata 分层（STATE / adr / lessons）自 2026-10-09 启用。

## Where（现状，均已对照代码核实）

- 分支 `dev/zoo_v3.0`（3.0 族）。仿真核心 `pufferlib/ocean/drive/`：`drive.h` + 拆分头（`constants.h`/`datatypes.h`/`idm.h`/`map_data.h`/`rng.h`）。
- 行为编排：per-agent `CONTROLLER_{STATIC,POLICY,REPLAY,IDM}`（`sdc/non_sdc/non_vehicle_controller`），`simulation_mode: gigaflow|replay`，混合旋钮 `mix_traffic / ppo_fraction / idm_fraction / expert_fraction`。
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
- `docs/adr/0001-0006` — 架构裁决（见 Open rulings）。

## Next（下一步）

- P0：从 `origin/9_bernhard_dev` 移植 pose/brownian 噪声 → 扩展为 track 级时间相关、CIPV 加权、进 16 帧历史的注入（ADR-0004；`docs/integration/2026-10-09-perception-jitter-as-the-real-axis3.md` §3）。
- P0：最小 Nova 契约 bridge（**19 输入 / 4 输出**，ADR-0006），先在公开 WOMD 场景跑通：产出 `static_polygon_*` 四路（≤32 polygon，cap 截断须上报丢弃数）+ 3 槽几何序参考线（slot0 必须真实 ego 几何，全零帧跳过推理）+ ego-centric RFU 重抽（M1 复用录制帧 14ch；生产侧在 `wt-apollo/record_exporter/frame_builder.h`）。
- P1：对手混合 Σ（复用 behavior-bench traffic registry / `conditioned_mix`）；Apollo HD map → PufferDrive bin 最小转换（仅几何+交通控制+route）。
- P2：数据湖长尾挖掘（以 `scenario_id` 为键）；Comfort-by-Construction 硬约束动作空间。

## Open rulings（裁决状态）

- ADR-0001 Core+语义外挂：**accepted** 2026-09-11（owner）。
- ADR-0005 M1 先行 + Scene 契约：**accepted** 2026-09-11（owner）。
- ADR-0002 部署契约内训练 / ADR-0003 混合对手必需 / ADR-0004 track 级抖动：**proposed** 2026-10-09，待 owner 确认。
- ADR-0006 Nova 契约代际切换（19 输入 / 4 输出）：**accepted** 2026-10-31（依据 nova 一手文档与 nova ADR-0003/0004）；同时修正 ADR-0002 / ADR-0005 的代际指针。
- 未决（阻塞项，见 `.scratch/handoff/`）：抖动过滤归属层（感知平滑 / planner 自滤波 / 独立估计层）；观测契约全量 vs 子集；Σ 用固定 fraction 还是 policy pool/league；基线是否钉死 `dev/zoo_v3.0`。
- 新增未决（2026-10-31）：PufferDrive world 系（heading 0 = +x，`Agent.cos_heading/sin_heading`）→ Nova ego-centric RFU（直行 = π/2）的偏置换算未实测；`static_polygon` cap 32 的 PufferDrive 侧丢弃数上报口径未定。

## Long-term（长期不变量）

- 集成目标：rino 数据湖（5000+ 车、自有 Apollo HD map、真实感知质量）+ 混合智能体闭环世界（replay/IDM/特殊行为/self-play），在 Nova 19 输入 / 4 输出部署契约内训练 planner（ADR-0006）。
- 泛化模型：三条独立轴（①地图几何 / ②交通行为 / ③观测可观测性）；self-play 的定位是**对手分布生成器 + 全场安全测试台**，不是闭环分数引擎。
- 表示一致性（训练契约 = 部署感知契约）与长尾覆盖度是两个命门；WOMD/nuPlan 仅作外部可比基准与冷启动。
