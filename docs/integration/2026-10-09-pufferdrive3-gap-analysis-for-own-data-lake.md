# PufferDrive 3.0 对"自有数据湖 → 混合智能体闭环世界"方案的 Gap 分析

> 日期：2026-10-09
> 基线：本仓库当前分支 `dev/zoo_v3.0`，HEAD `80a681b4`（观测布局=goal 归一化+road 输入，已移除外挂渲染，加入 stop 逻辑/IDM/交通灯/route-goal/per-agent controller）
> 目标方案（用户）：把 rino 自有实车数据（5000+ 无人车、自有 HD map、真实感知质量）+ PufferDrive 高性能仿真 + 混合智能体编排（log-replay / IDM / 特殊策略 / self-play）打通成一个闭环世界，用于训练/迭代 Nova 契约下的 planner。
> 本文件只做 **capability gap 盘点**，不含实现方案。

---

## 0. 结论速览

- **PufferDrive 3.0 已经提供了这个方案里最难、最贵的两块**：高吞吐多智能体仿真内核 + per-agent 行为编排（static/policy/replay/idm）+ route/goal + 交通灯 + 奖励条件化 + 失败回放。
- **真正缺的只有三块**，且都在"边界"而非"内核"：
  1. **Apollo HD map / 感知 → PufferDrive scene 的转换**（几何 + 交通控制 + 40 静态 objects + route/goal）；
  2. **Nova 19 输入 / 4 输出契约的 sidecar/bridge**（PufferDrive 自己的观测不覆盖 Nova 富语义；代际切换见 `docs/adr/0006-nova-contract-19-in-4-out-static-polygon.md`）；
  3. **感知噪声注入**（当前分支只有行为异常 `blindness/phantom_braking`，没有 pose/detection/latency 噪声；噪声实现在 `8/9_bernhard_dev` 分支未合并）。
- **另有两个"设计空白"**（不是 bug，是领域空白）：**对手采样矩阵 Σ（PSRO/league）** 与 **动作空间舒适硬约束**（Comfort by Construction）。这两者正是 survey ④.8 / ④.12 指出的、决定"分数真实性与泛化"的关键。

---

## 1. PufferDrive 3.0 现有能力（已具备，无需从零造）

### 1.1 行为编排（轴②的核心）
| 能力 | 出处 | 说明 |
|---|---|---|
| per-agent controller | `constants.h`: `CONTROLLER_{STATIC,POLICY,REPLAY,IDM}`；`drive.py`: `sdc_controller/non_sdc_controller/non_vehicle_controller` | 每个 agent 独立行为源，**这正是混合智能体世界的原语** |
| 日志回放 | `SIMULATION_MODE_REPLAY` | 背景车沿 GT |
| IDM 反应式 | `idm.h`（866 行）；`move_idm` | 跟驰 + route 投影 + 交通灯/停止线；含 lateral/heading 限制 |
| 自博弈 | `SIMULATION_MODE_GIGAFLOW` + `CONTROLLER_POLICY` | 共享策略控制全部车辆 |
| 车流混合 | `puffer_drive.yaml`: `control_mode` / `sdc_controller` / `non_sdc_controller` / `non_vehicle_controller` / `init_mode`；`constants.h`: `CONTROLLER_{STATIC,POLICY,REPLAY,IDM}` | **本仓无 fraction 旋钮**——`mix_traffic/ppo_fraction/idm_fraction/expert_fraction/idm_target_velocity/idm_random_velocity` 属 behavior-bench（`behavior-bench/pufferlib/ocean/drive/drive.py:84-89`）；比例混合需从 behavior-bench 移植（见 §3 可复用资产） |
| 行为异常注入 | `partner_blindness_*` / `phantom_braking_*` | 对手"看不见/幽灵刹车"，服务于对手鲁棒性 |
| 奖励条件化 + 随机化 | `reward_conditioning`、18 个 `reward_coefs`、`reward_randomization` | 单策略表现多种驾驶风格（GigaFlow 式） |

### 1.2 地图 / 场景 / 导航
| 能力 | 出处 | 说明 |
|---|---|---|
| scene 二进制 | `map_data.h`: `num_total_agents/num_roads/num_traffic/num_objects` + `scenario_id[128]` + `objects_of_interest` | 含 **40 静态 objects** 与 **128B scenario_id**（sidecar 对齐可用） |
| 多目标 / route / goal | `MAX_GOALS=20`、`goal_source: route|map|gt`、`goal_regen_mode`、`goals` 观测 | 支持长时程、多 waypoint、GT 终点播种 |
| 交通控制 | `TRAFFIC_CONTROL_TYPE_{LIGHT,STOP_SIGN,YIELD}`、`traffic_lights_enabled` | 从 scene 的 traffic elements 加载（非纯程序化） |
| 2.5D | `Z_BUFFER`、`obs_norm_z_m` | 支持立交近似 |
| 地图/邻居缓存 | `use_map_cache`、`use_neighbor_cache`、`preload_map_cache` | 服务"上千张自有地图"的加载 |

### 1.3 评估 / 挖掘 / 训练
| 能力 | 出处 | 说明 |
|---|---|---|
| 配置化 benchmark | `config/evaluation/benchmark.yaml`（carla / womd_multi / womd_single / nuplan_multi / nuplan_single） | 可直接加"自有数据 split" |
| 失败回放 | `b5765053`（#533）：确定性多 worker + seeded failure replay + `failure_replay_csv` / `render_filter` | **旧的 `mine_failures`/`mining_viz.py` 已被它取代** |
| 18 项指标 | `NUM_METRICS=18`（at-fault collision / TTC / distance-to-collision / progress / multi-lane / comfort…） | 比单一 score 丰富 |
| 训练算法 | PPO + V-trace + 优先采样 + **advantage filtering**（`adv_filter_*`，GigaFlow 式） | 500B timesteps 配置 |
| 数据获取 | `data_utils/datasets.yaml`（WOMD / nuPlan S3 bins）、`fetch_data.py`、`mirror_map_bin.py` | 公开数据可直接用 |

---

## 2. Gap 清单（需要新建或合并）

### 轴① 地图/几何
| ID | Gap | 现状 | 备注 |
|---|---|---|---|
| **G1** | **Apollo HD map → PufferDrive scene 二进制** | ❌ 仓库内无转换器（`data_utils` 只有下载/镜像）；WOMD/nuplan 走外部 ScenarioMax / py123d+123Drive | 需实现：Lane/Boundary/Road/Junction → road elements；Signal/StopSign/Crosswalk → traffic elements；geometry 投影到 PufferDrive 局部系 |
| **G2** | **Apollo 红绿灯语义 → traffic elements** | ⚠️ PufferDrive 支持 traffic elements，但 gigaflow 用程序化灯；导入真实灯（color/关联/停止线）需映射 | `map_signal.proto` 有 `stop_line` + `overlap_id`；`map_overlap.proto` 提供灯-车道关联 |
| **G3** | **Apollo route → PufferDrive route/goal** | ⚠️ 有 `goal_source: route/gt`，但 Apollo routing_reference_line → lane 序列的映射需实现 | `record_exporter` 的 `refline_generator.h` 可复用；另须满足模型侧 3 槽几何序（slot0 = 真实 ego 几何）与"全零参考线跳过推理"（nova ADR-0003） |

### 轴② 行为多样性
| ID | Gap | 现状 | 备注 |
|---|---|---|---|
| **G4** | **自有数据湖的场景挖掘/策展** | ⚠️ 有评估期失败回放，但**训练期场景选择**（长尾/交互密集挖掘）无 | 5000 车数据量巨大，价值在策展不在堆量（见 2026-10-09 分析） |
| **G5** | **对手采样矩阵 Σ（PSRO / league / policy pool）** | ❌ 只有 naive self-play + 固定 fraction 混合 | survey ④.8 明确指出驾驶领域此格空白；BehaviorBench 的"过拟合自身对手"正是其后果 |
| **G6** | **IDM 之外的异构行为模型（如 SMART）** | ⚠️ 核心无；`behavior-bench` 有 SMART/prediction 模块 | 可在 behavior-bench 里复用 |

### 轴③ 观测/感知
| ID | Gap | 现状 | 备注 |
|---|---|---|---|
| **G7** | **track 级感知抖动模型（不是 pose noise）** | ❌ 当前分支只有行为异常；`pose_noise_xy_m`、`pose_noise_yaw_deg`、brownian noise 仅存在于 `origin/8_bernhard_dev`、`origin/9_bernhard_dev` | **已锐化，详见 `2026-10-09-perception-jitter-as-the-real-axis3.md`**：需时间相关噪声（自相关匹配）、按 CIPV 加权、进入 16 帧历史、多感知版本域随机化、抖动专用评测轴；简单 pose noise 不足 |
| **G8** | **Nova 19 输入 / 4 输出观测契约（sidecar/bridge）** | ❌ PufferDrive 用自有 obs；Nova 富语义（agent 类型/16 帧历史/地图 15 维/3 槽参考线/红绿灯关联/静态障碍四路）不覆盖 | 现行代际见 `docs/adr/0006-nova-contract-19-in-4-out-static-polygon.md`；语义外挂决策见 `docs/integration/CONTEXT.md`；14ch 生产侧在 `wt-apollo/.../frame_builder.h` |
| **G9** | **静态障碍（`static_polygon_*`）的可观测暴露** | ⚠️ scene 有 `num_objects`（40 静态 objects），但 Python 侧是否完整暴露给 bridge 需确认 | 部署契约已改四路定长（points/types/heights/mask，≤32 polygon × 32 点）；cap 32 为**有损截断**（取最近 32）且丢弃数须上报（ADR-0006）；`occ_polygons` 只留训练侧建 ESDF，不进 ONNX |

### 动作空间 / 评测协议
| ID | Gap | 现状 | 备注 |
|---|---|---|---|
| **G10** | **舒适有界动作空间（硬约束）** | ❌ `comfort` 只是软奖励 + 指标；`COMFORT_ACCEL_THRESHOLD` 用于打分 | Comfort by Construction 证明无约束网格会让策略用"急刹+急打方向"刷安全分（jerk 越限 4513%） |
| **G11** | **交互自然性评测协议** | ⚠️ 有 18 指标 + WOSAC 类 realism，但缺"跨对手"的标准协议 | BehaviorBench 的 Interactive1k / conditioned_mix 可直接借鉴 |
| **G12** | **场景/OOD 分桶与 lineage** | ⚠️ 有 `scenario_id`，但感知迭代 → 训练数据版本的一致性管理无 | 表示一致性是命门 |

---

## 3. 可复用资产（不必重造）

| 资产 | 位置 | 能补哪个 Gap |
|---|---|---|
| **behavior-bench**（完整） | `Self-Play/behavior-bench` | G5/G6/G11：planner/traffic registry、`conditioned_mix`（异质车流）、SMART、PDM、nuplan_integration、Interactive1k/Random1k、预训练权重 |
| **Bernhard 分支噪声** | `origin/8_bernhard_dev`、`origin/9_bernhard_dev` | G7：`pose_noise_xy_m/yaw_deg`、brownian noise、nuPlan 10Hz cosim、strongest self-play model（1M km/infraction） |
| **Apollo 生产侧** | `wt-apollo/modules/tools/record_exporter` | G1/G2/G3/G8：`frame_builder.h`（14ch map / agents / TL / refline）、`refline_generator.h` |
| **Comfort by Construction** | 论文 arXiv:2609.13011（代码待确认位置） | G10 |
| **外部转换器** | ScenarioMax（WOMD）、py123d+123Drive（nuPlan） | G1 的范式参考 |
| **GPUDrive-self-play / nocturne_lab** | `Self-Play/` | 历史基线、消融对照 |

---

## 4. 补齐优先级建议（按"杠杆/成本"排序）

1. **P0 — G7 感知噪声合并**：从 `9_bernhard_dev` 摘出 pose/brownian 噪声，扩展为 Apollo 统计对齐的检测级噪声。**成本低、直接决定轴③是否成立**。
2. **P0 — G8 Nova 契约 bridge（最小版）**：复用 `wt-apollo` 的 14ch + agent 编码，先在**公开 WOMD 场景**上跑通"PufferDrive 世界 → 19 输入 / 4 输出"的闭环，暂不碰 Apollo 地图。这是把"训练契约 = 部署契约"落地的第一步。
3. **P1 — G5 对手采样 Σ**：把 behavior-bench 的 traffic registry 接进来，做 fraction 混合 vs policy-pool/league 的对照。这是 survey 里最大的设计空白，且不需要新仿真。
4. **P1 — G1/G2/G3 Apollo 地图转换**：几何 + 交通控制 + route 的最小转换（不追 14ch 语义，语义走 sidecar）。
5. **P2 — G4 数据湖挖掘**：基于 `scenario_id` + 交互度评分，从自有数据选长尾场景入库。
6. **P2 — G10 舒适硬约束**：集成 Comfort by Construction 的自适应动作参数化。
7. **P3 — G11/G12 评测协议与 lineage**。

---

## 5. 附：你的方案 vs DriveZero（差异澄清）

你的直觉没错——你**确实在向 DriveZero 的"世界构建"思想靠拢**，但有三处**本质差异**，而且其中一处让**你的问题比 DriveZero 更简单**：

| 维度 | DriveZero | 你的方案 | 差异性质 |
|---|---|---|---|
| **部署观测模态** | 相机-only → 必须有"特权结构化教师 DriveRL + DriveVFM 感知预训练 + winner-takes-all 蒸馏" | Nova 吃的是 **Apollo 感知向量**（agents/occ/online-map）→ **部署输入本身就是向量化特权** | ★ **你不需要 teacher-student + VFM 那条链路**，可以直接在部署契约里训练。少了整整一层 |
| **数据来源 / ODD** | nuPlan 公开日志（有限城市/ODD） | 5000+ 自有无人车、自有城市、自有 HD map、自有感知质量 | 你的 ODD 与部署完全一致；数据规模与相关性都更高 |
| **self-play 的角色** | 教师**不是** self-play（log/IDM mixed50 单自车）；self-play 仅真车部署时启用，且附录消融无收益 | 你明确要在**训练期**混合 self-play | 你比 DriveZero 更激进地用 self-play |
| **目标** | 论文/SOTA benchmark | 量产 planner + 自有数据闭环 | 目标函数不同 |

**一句话**：你复用 DriveZero 的"日志 → 混合智能体反应式世界"洞察，但因为部署模态是向量而非像素，你**跳过了它的特权教师与感知蒸馏**，只需在**部署契约内**直接做闭环 RL。因此在**观测可观测性**上，你比 DriveZero 更接近 Pictura 的哲学（约束到部署模态），只是模态是"向量"而非"像素"；而在**对手多样性**上，你比两者都更需要 self-play + Σ。

**未被消灭的风险**（与 DriveZero 共有）：① 自有日志仍是**开环**，反应性来自 sim（IDM/self-play），其保真度决定上限；② 表示一致性（训练契约 = 部署感知契约）是命门；③ 数据覆盖度（长尾）决定最终能力。
