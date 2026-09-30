# 用 PufferDrive Self-Play 快速迭代 Nova 两段式端到端 Model Planner：落地方案

> 日期：2026-09-11
> 涉及系统：
> - **PufferDrive**（本仓库 `Self-Play/PufferDrive`）：320K SPS 多智能体自博弈仿真器（C 内核 + PufferLib）
> - **Nova**（`~/labs-x/agent_group/nova`）：两段式端到端规划模型（场景编码 → Token 解码 → 多任务 → 模仿学习）
> - **deploy**（`~/labs-x/agent_group/nova/deploy`）：Nova 的 C++/TensorRT 车端部署链路
>
> 目标读者：Nova 模型训练 / 评测 / 部署工程团队。本文给出一套**工程可落地**的桥接架构与迭代闭环，回答两个核心问题：
> 1. 如何用 PufferDrive 的反应式 self-play 快速定位、归因、修复 Nova planner 的问题？
> 2. 如何在修复的同时**不引发旧问题复发**？

---

## 0. 结论摘要（TL;DR）

1. **不要试图把 PufferDrive 的 1848 维 BEV 观测直接喂给 Nova**。两者观测契约不同：PufferDrive 是向量化 ego-centric BEV，Nova 吃的是 17 张量场景图。正确做法是**以 PufferDrive 为世界模拟器 + 反应式对手 + 真值 oracle**，在桥接层用世界全局状态**重建 Nova 的 OnboardFrame / 17 张量**。
2. PufferDrive 对 Nova 的**最大价值**不是"又一个数据集"，而是补齐 Nova 闭环评估的最大短板：**当前 nuplan 闭环是 non-reactive（背景车沿 GT 轨迹，不会对 Nova 反应）**。PufferDrive 提供**反应式多智能体对手**，能把 Nova 逼进真实交互失败区。
3. 方案 = **三桥一闭环**：
   - **状态桥**：PufferDrive world state → Nova OnboardFrame/17 张量
   - **动作桥**：Nova 规划轨迹 → PufferDrive (accel, steer)
   - **数据桥**：PufferDrive 自博弈 rollout → Nova tfrecord（可选，训练侧）
   - **迭代闭环**：反应式测评 → 失败挖掘 → 分层归因 → 定向修复 → 回归门禁
4. **归因核心**：两段式 Nova 有天然的分段审计接口。用 PufferDrive 的**真值 oracle 注入**做分层消融——`agent future` 喂真值可判断"是交互预测错，还是规划错"；反事实场景编辑可定位"哪个 agent 是罪魁"。
5. **防复发核心**：把每个修过的 bug 沉淀成**最小场景 → 场景库 → 自动回归门禁**；训练期用**混合重放 + policy pool** 防遗忘。

---

## 1. 两边系统的接口事实（先对齐契约）

### 1.1 Nova：两段式端到端 planner 的输入/输出契约

**架构（对照 `docs/model_arch_NOVA.jpg`）**：

```
17 张量输入
   │
   ▼
┌─────────────┐     场景 Token       ┌──────────────────┐
│  场景编码器   │ ─────────────────▶ │  Token 解码器      │
│ (ego/agent/  │   encoder_output     │ (prediction + ego) │
│  map/routing │                      │                    │
│  /occ/esdf)  │                      │  多任务输出         │
└─────────────┘                      └──────────────────┘
                                              │
                              ┌───────────────┴────────────────┐
                              ▼                                ▼
                    agent prediction                    ego planning
                    (障碍物未来轨迹)                    (自车未来轨迹)
                              └───────────────┬────────────────┘
                                              ▼
                                      模仿学习损失
```

**输入契约（17 张量，见 `deploy/ads_models/nova-input-format.md`）**：

| 张量 | 维度 | 语义要点 |
|---|---|---|
| `ego_vector_features` | [1,16,13] | ego 16 帧历史（坐标 global/ego-t0 RFU 表达） |
| `ego_vector_features_ori` | [1,16,13] | 未增广版本，用于算 t0 与前一帧 heading 差 |
| `agent_vector_features` | [1,16,79,11] | 最多 79 个 agent × 16 帧历史；padding=0 |
| `agent_semantic_labels` | [1,79] | 0=车 1=行人 2=骑行 |
| `agent_vector_mask` | [1,16,79] | 每帧每 agent 有效性 |
| `agent_focal_mask` | [1,79] | focal agent 标记 |
| `map_vector_features` | [1,800,11,15] | 800 段 × 11 点 × 15 维地图语义 |
| `local_routing_line` | [1,3,28,10,14] | 3 条参考线 × 28 元素 × 10 点 × 14 维 |
| `local_routing_mask` | [1,3,28,10] | 参考线元素有效性 |
| `traffic_light_color/blink/*` | [1,3,5]/[1,3,2]/... | 红绿灯 one-hot + 停止线关联 |
| `stop_line_info` | [1,3,4] | 停止线端点（局部系） |
| `occ_polygons` / `occ_polygon_len` | [1,1024] / [1] | 占据栅格多边形流 |
| `ego_planning_status` | [1,14] | t0 自车状态（含 steer/kappa/wheel_base） |

**输出契约（5 输出，见 `deploy/include/nova_deploy/io_contract.h`）**：

| 输出 | 维度 | 语义 |
|---|---|---|
| `planning_nms_trajectory_xy_yaw_global` | [1,3,6,60,6] | **自车规划轨迹**：3 参考线 × 6 模态 × 60 帧(6s@10Hz) × [x,y,yaw,speed,acc,kappa] |
| `planning_nms_trajectory_prob_global` | [1,3,6,1] | 规划模态概率 |
| `nms_trajectory_xy_yaw_global` | [1,80,6,60,3] | 障碍物预测：80 模态 × 6 类 × 60 帧 × [x,y,yaw] |
| `nms_trajectory_prob_global` | [1,80,6,1] | 障碍物模态概率 |
| `pred_traffic_light_decision` | [1,1] | 红绿灯决策 |

**坐标系契约（重要）**：
- 模型内部原生 **RFU**（x=右, y=前, 直行 heading=π/2）；
- 仿真 / nuplan / cache 全程 **FLU**（x=前, y=左, 直行 heading=0）；
- 转换集中在 `adfm/simulation/coordinate_frames.py::CoordinateFrameConverter`，进出模型各转一次；
- 转换公式：`RFU_x=-FLU_y, RFU_y=FLU_x, RFU_heading=FLU_heading+π/2`。

**当前闭环评估（`adfm/simulation/runner.py` + `planner_adapter.py`）**：
- 25 个 nuplan metric，`closed_loop_nonreactive_agents` 加权聚合；
- planner 三种：`ModelPlanner`（每步真闭环重推理）/ `ModelOpenPlanner`（t0 开环回放）/ `GTPlanner`（GT 上限）；
- controller：`perfect` / `two_stage`（LQRTracker + KinematicBicycle）；
- **关键短板：agents are non-reactive**（`runner.py` 文件头原文），背景车沿 GT 轨迹行进，不响应 Nova 的行为。

### 1.2 PufferDrive：可复用的 self-play 能力清单

**它是什么**：C 实现的多智能体驾驶仿真器，`drive.h`（3200 行）为内核，Python 经 `binding.c` / `env_binding.h` 绑定；单 GPU 端到端训练 ~320K steps/s。

**可直接复用的能力**：

| 能力 | 接口 / 说明 |
|---|---|
| **反应式多智能体自博弈** | `control_mode=control_agents`：场景内全部 agent 由共享策略控制、互为对手 |
| **混合博弈（self-play + 回放）** | `control_mode=control_mixed_play` + `max_controlled_agents`：策略只控 N 个，其余回放日志（可调 self-play 比例） |
| **纯 GT 回放** | `control_mode=control_sdc_only`：只控自车，其余沿 GT（≈ nuplan non-reactive） |
| **全局真值状态** | `get_global_agent_state()`：x,y,z,heading,id,length,width（当前帧全部 active agent） |
| **全局 GT 轨迹** | `get_ground_truth_trajectories()`：x,y,z,heading,valid,id,is_vehicle,is_track_to_predict,scenario_id（整条 91 帧） |
| **道路几何** | `vec_get_road_edge_counts/polylines()`：x,y,lengths,scenario_id |
| **场景 ID** | `vec_get_scenario_ids()` |
| **逐环境日志** | `env_log` / `get_env_logs()`：collision_rate / offroad_rate / score / completion_rate 等 |
| **动作空间** | discrete 91（7 accel × 13 steer）或 continuous 2D（accel, steer） |
| **动力学** | classic（运动学自行车，dt=0.1s）或 jerk；10Hz，与 Nova 同频 |
| **WOSAC 分布真实性评估** | `puffer eval puffer_drive --eval.wosac-realism-eval True` |
| **场景编辑器** | 浏览器端 WOMD JSON / `.bin` 编辑（轨迹、道路、expert 标记、tracks_to_predict） |
| **地图重采样** | 每 `resample_frequency` 步重新随机分配场景 |

**关键限制（必须知道，决定方案的 gap 处理）**：

| 限制 | 影响 | 应对 |
|---|---|---|
| **无红绿灯**（roadmap 3.0 才支持） | Nova 的 `traffic_light_*` / `stop_line_info` 无法从 PufferDrive 直接得到 | 初期只跑无红绿灯场景；红绿灯张量置零 + 在场景库中标记该 gap |
| **地图语义较薄** | PufferDrive road type 仅 LANE/LINE/EDGE/CROSSWALK 等，缺 lane_turn_type / speed_limit / boundary 类型 | 扩展 map binary 转换（源 JSON 里有更多语义），或在桥接层做语义映射 + 默认值 |
| **无独立 occ 流** | Nova 的 `occ_polygons` 需要占据栅格 | 从 agent box + road edge 合成 occ 多边形 |
| **无参考线/routing line** | Nova 需要 3 条参考线 | 用 PufferDrive lane graph + goal 合成参考线 |
| **agent 上限 32/64**（`MAX_AGENTS`） | Nova 是 79 槽位 | 桥接层 padding 到 79，mask 置位 |
| **观测是 1848 维 BEV，非 17 张量** | 不能直接当作 Nova 输入 | **必须经桥接层用全局状态重建 OnboardFrame** |
| **不直接输出轨迹数据集** | 训练侧需要 tfrecord | 用数据桥转换（见 §3.1.4） |

### 1.3 差距矩阵（Gap Matrix）

| Nova 需要 | PufferDrive 能否提供 | 方案 |
|---|---|---|
| 反应式对手（关键！） | ✅ 原生 self-play / mixed-play | 直接用，桥接层组合 Nova + RL policy 动作 |
| ego / agent 全局真值状态 | ✅ `get_global_agent_state` | 状态桥 |
| agent 历史 16 帧 | ⚠️ 只给当前帧 + GT 轨迹 | 桥接层维护 rollout 历史 RingBuffer |
| 地图 15 维语义 | ⚠️ 部分 | 扩展 map 转换 or 桥接层映射 |
| 参考线 | ❌ | 从 lane graph + goal 合成 |
| 红绿灯 | ❌ | 初期跳过；置零 + gap 标记 |
| occ | ❌ | 从 box/road 合成 |
| 快速批量失败挖掘 | ✅ 320K SPS + per-env log | 直接用 |
| 可编辑反事实场景 | ✅ 场景编辑器 | 直接用 |
| 分布真实性指标 | ✅ WOSAC | 与 nuplan metric 并列 |
| 训练数据（tfrecord） | ⚠️ 需转换 | 数据桥 |

---

## 2. Nova 当前迭代瓶颈的诊断

结合 Nova 代码现状，planner 迭代的四个瓶颈：

1. **闭环评估非反应式**（最大瓶颈）：`runner.py` 明确 "Agents are non-reactive (they follow their GT trajectories)"。Nova 面对的是"僵尸车流"，学到的交互策略在真实反应式车流下会失效——**评估乐观偏差**。
2. **离线模仿的分布上限**：训练数据是离线 tfrecord，状态覆盖有限，长尾交互缺失；闭环下 compounding error 无法被评估暴露。
3. **黑盒归因难**：两段式虽有中间 token，但"agent 预测错 → ego 规划错"的因果链缺少可控干预手段；没有真值 oracle 做对照。
4. **修复易复发**：模型是全局参数，修一个场景的感知/预测偏差会扰动其他场景；缺少场景级回归门禁。

PufferDrive 恰好对四个瓶颈逐一补位：反应式对手（1）、自博弈数据生成（2）、真值 oracle + 反事实（3）、高速批量 + 场景库（4）。

---

## 3. 总体方案：三桥一闭环

```
┌────────────────────────────────────────────────────────────────────────────┐
│                             迭代闭环 Iteration Loop                          │
│                                                                            │
│   ┌───────────────────┐   world state    ┌────────────────────────────┐    │
│   │   PufferDrive     │ ───────────────▶ │  状态桥 State Bridge        │    │
│   │   C 仿真内核       │                  │  world → OnboardFrame      │    │
│   │   320K SPS        │ ◀─────────────── │  → 17 张量                 │    │
│   │   self-play       │   (accel,steer)  └────────────┬───────────────┘    │
│   └─────────┬─────────┘  动作桥 Action Bridge          │ 17 张量           │
│             │                          ◀───────────────┘                    │
│             │ 反应式对手 / GT回放                        ▼                    │
│             │                              ┌────────────────────────────┐  │
│             │                              │      Nova Planner          │  │
│             │                              │  encoder → decoder         │  │
│             │                              │  agent pred + ego plan     │  │
│             │                              └────────────┬───────────────┘  │
│             │                                           │ 5 输出             │
│             │                                           ▼                    │
│             │                              ┌────────────────────────────┐  │
│             │                              │  动作桥: plan(60帧)        │  │
│             │                              │  → tracker → (accel,steer) │  │
│             │                              └────────────────────────────┘  │
│             │                                                                │
│   ┌─────────▼──────────────────────────────────────────────────────────┐    │
│   │  失败挖掘 → 场景库 → 分层归因 → 定向修复 → 回归门禁                  │    │
│   └────────────────────────────────────────────────────────────────────┘    │
└────────────────────────────────────────────────────────────────────────────┘
```

### 3.1 桥接层 Bridge（核心工程）

建议放在 Nova 侧或独立包（如 `nova/integrations/pufferdrive/`），避免污染 PufferDrive 与 Nova 主仓。模块划分：

```
pufferdrive_bridge/
├── state_bridge.py      # world → OnboardFrame → 17 张量
├── action_bridge.py     # Nova plan → PufferDrive action
├── coordinate.py        # PufferDrive global ↔ FLU ↔ RFU（复用 Nova 的转换器）
├── history_buffer.py    # agent/ego 16 帧历史维护
├── map_builder.py       # PufferDrive road → Nova map_vector_features
├── routing_builder.py   # lane graph + goal → local_routing_line
├── occ_builder.py       # agent box + road edge → occ_polygons
├── nova_runner.py       # Nova 模型加载/前向封装（含 batch）
├── reactive_eval.py     # 反应式闭环评测主循环
└── failure_miner.py     # 失败场景挖掘 + 导出
```

#### 3.1.1 状态桥：PufferDrive world → Nova OnboardFrame / 17 张量

**核心原则**：PufferDrive 的 1848 维 obs **不参与** Nova 输入；桥接只读其**全局真值状态**，重建 OnboardFrame，再走 Nova 自己的特征组装逻辑（Python 侧 `adfm/datasets` 或 C++ `libnova_feature` 的等价封装）。

**数据来源映射**：

| Nova 张量 | PufferDrive 来源 | 构造方式 |
|---|---|---|
| `ego_vector_features` [16,13] | `get_global_agent_state` 的 SDC + `history_buffer` | 16 帧 ego 状态，转 ego-t0 RFU |
| `agent_vector_features` [16,79,11] | `get_global_agent_state` 全体 agent + `history_buffer` | 按 id 稳定跟踪，padding 到 79 |
| `agent_semantic_labels` [79] | PufferDrive entity `type`（VEHICLE/PED/CYCLIST） | 映射 0/1/2 |
| `agent_vector_mask` [16,79] | 是否有历史 / 是否 valid | bool |
| `map_vector_features` [800,11,15] | `vec_get_road_edge_polylines` + 原始 map JSON | 3m 重采样、11 点、15 维语义（见 §3.1.5） |
| `local_routing_line` [3,28,10,14] | lane graph + goal | 从 ego 到 goal 的路径采样 |
| `traffic_light_*` | ❌ 无 | 置零（gap），场景库标记 |
| `stop_line_info` | ❌ 无 | 置零（gap） |
| `occ_polygons` | agent box + road edge | 合成多边形流 |
| `ego_planning_status` [14] | ego 状态 | 直接构造 |

**接口草案**：

```python
class PufferToNovaStateBridge:
    """PufferDrive 全局状态 → Nova OnboardFrame/17 张量。无副作用，可批量。"""

    def __init__(self, converter, map_builder, routing_builder, occ_builder,
                 history_len: int = 16, max_agents: int = 79):
        ...

    def update(self, env, env_idx: int) -> None:
        """每步调用：从 PufferDrive 拉全局状态，写入历史缓冲。"""
        states = env.get_global_agent_state()
        self.history.push(states)   # RingBuffer by stable agent id

    def build_onboard_frame(self, env_idx: int, t0: int) -> OnboardFrame:
        """组装车端单帧原始语义契约（未选择/未截断，与 nova-input-format 对齐）。"""
        ...

    def build_tensors(self, frame: OnboardFrame) -> dict[str, np.ndarray]:
        """复用 Nova 训练侧特征组装逻辑，产出 17 张量。"""
        return self.feature_builder.build(frame)
```

**关键实现点**：
- **历史缓冲**：PufferDrive 只暴露当前帧全局状态（`get_global_agent_state`）。策略控制 agent 的未来不由 GT 轨迹决定，所以**不能**用 `get_ground_truth_trajectories` 当历史；必须每步 `push` 实际 rollout 状态。GT 轨迹仅作 oracle/对照。
- **ID 稳定**：PufferDrive entity id 来自 map 实体索引，跨帧稳定，可直接作 tracking key。
- **t0 对齐**：Nova 的 t0 = 当前仿真步；16 帧历史需覆盖 PufferDrive 的 `init_steps` 之前。若从 episode 中途起评，前若干帧用 GT 补齐。
- **坐标系**：PufferDrive 全局系 →（`to_ego_frame`）FLU →（Nova `CoordinateFrameConverter`）RFU。**禁止在桥接层手写转换公式**。

#### 3.1.2 动作桥：Nova plan → PufferDrive action

Nova 输出的是 **6s 轨迹**（60 帧 × [x,y,yaw,speed,acc,kappa]），PufferDrive 需要**逐步 (accel, steer)**。两种方案：

**方案 A：直接微分（简单，先跑通）**
```python
# Nova plan[0], plan[1] 已含 acc/kappa
accel = plan[1].acc                       # 或 (speed[1]-speed[0])/dt
steer = atan(plan[1].kappa * wheel_base)  # 自行车模型反解
```

**方案 B：轻量 tracker（推荐，等价 nuplan two_stage）**
- 复用/移植 Nova nuplan runner 的 `LQRTracker + KinematicBicycleModel`；
- 输入 Nova 的 60 帧轨迹（FLU），输出当前步 (accel, steer)；
- 与 Nova 现有闭环评估使用**同一套控制语义**，保证可对拍。

**接口草案**：

```python
class NovaPlanToPufferAction:
    def __init__(self, controller: str = "lqr", time_step: float = 0.1): ...

    def __call__(self, plan_flu: np.ndarray, ego_live: EgoState) -> np.ndarray:
        """plan_flu: [60,6] FLU; 返回 PufferDrive continuous action [accel, steer]。"""
        ...
```

**与 PufferDrive 动力学的匹配**：
- PufferDrive classic 用运动学自行车 + dt=0.1，与 Nova 10Hz 一致；
- `steer` 与 `kappa` 关系：`kappa = tan(steer)/L`，反解 `steer=atan(kappa*L)`；注意 PufferDrive `wheelbase = 0.6 * length`，Nova 用 `vehicle_param.wheel_base`——需在桥接层统一（否则产生系统性跟踪偏差）。
- 建议用 PufferDrive 的 continuous 动作空间（`action_type=continuous`），避免离散 91 action 的量化误差污染归因。

#### 3.1.3 坐标系桥

| 层 | 坐标系 |
|---|---|
| PufferDrive 全局 / 仿真 | 全局系（heading=atan2，FLU 约定） |
| nuplan / Nova cache | FLU（x=前, y=左, 直行 heading=0） |
| Nova 模型内部 | RFU（x=右, y=前, 直行 heading=π/2） |

桥接层**必须复用** Nova 的 `CoordinateFrameConverter`（`adfm/simulation/coordinate_frames.py`），只允许在"进模型前 / 出模型后"两个点转换。任何手写旋转都会成为最隐蔽的 bug 来源（Nova 的 `docs/coord_FLU_to_RFU.md` 已记录过这类事故）。

#### 3.1.4 数据桥（可选，训练侧）：PufferDrive rollout → Nova tfrecord

用途：把自博弈/反应式 rollout 转成 Nova 训练数据。路径：

```
PufferDrive rollout（实际轨迹 + 场景）
   → 导出 WOMD JSON（场景编辑器已支持；或直接序列化）
   → 转 Nova tfrecord（data_process/data_feature.proto）
   → 混入 Nova 训练集
```

**Gap**：需要补齐地图 15 维语义与红绿灯（PufferDrive 无）。**建议分两步**：
- **Step 1（立即）**：只用于 **agent prediction 分支**的数据增强——用 PufferDrive 反应式轨迹生成更丰富的障碍物交互未来，转成预测监督。
- **Step 2（中期）**：扩展 PufferDrive 的 map converter，从源 WOMD JSON 保留 Nova 需要的道路语义，再生成完整 planner 训练数据。

#### 3.1.5 地图语义 gap 的两种解法

- **解法 1（低成本，先跑通）**：在桥接层做语义映射 + 默认值。
  - PufferDrive `ROAD_LANE/ROAD_LINE/ROAD_EDGE/CROSSWALK` → Nova `polyline_type/semantic_label`；
  - `lane_type/lane_turn_type/speed_limit/boundary_*` 缺省填充并计数上报（对齐 Nova 的"两级溢出契约"思想：软缺省 + 计数）。
- **解法 2（高保真，中期）**：扩展 PufferDrive 的 map binary 格式与 `save_map_binary()`，让地图转换从源 JSON 保留 Nova 的 15 维语义；这样同一份地图可同时服务仿真与模型输入。

---

## 4. 迭代闭环：五阶段落地

### Phase 0 — 桥接层 MVP + 对拍（约 1–2 周）

**目标**：证明"PufferDrive 世界 → Nova 输入 → Nova 输出 → PufferDrive 动作"能闭环跑通，且与现有 nuplan 评估**可对拍**。

**做法**：
1. 用 `control_sdc_only`（只控 SDC，其余 GT 回放）复现 nuplan non-reactive 场景；
2. 桥接层重建 17 张量，Nova 前向，动作桥驱动 SDC；
3. 与 Nova 现有 nuplan 闭环对拍同一场景的 ego 轨迹 / metric——**两者应接近**。若差异大，先修桥接 bug，再谈后续。

**验收（DoD）**：同一批场景上，PufferDrive 闭环的 Nova ego 轨迹与 nuplan 闭环的 **FLU 轨迹 RMSE < 门限**，metric 排序一致。

### Phase 1 — 反应式闭环评测（约 1–2 周）

**目标**：用反应式对手暴露 Nova 的交互失败。

**做法**（PufferDrive 侧无需改 C，Python 组合动作）：
```python
# 伪代码：Nova 控 SDC，PufferDrive 自博弈 RL policy 控其余 agent
obs = env.observations                      # PufferDrive BEV obs（给 RL policy）
rl_actions = puffer_policy(obs)             # 全体 agent 动作
nova_action = action_bridge(nova(state_bridge(...)))   # SDC 动作
actions = rl_actions.copy()
actions[sdc_index] = nova_action            # 覆盖 SDC
env.step(actions)
```
**三种对手配置对比**，分离"反应性"的影响：
| 配置 | 背景车行为 | 对应现有评估 |
|---|---|---|
| `control_sdc_only` | GT 回放 | nuplan non-reactive |
| `control_mixed_play` | N 个 self-play + 其余回放 | 中间态 |
| `control_agents` + RL policy | 全部反应式 | **新能力：反应式 stress test** |

**指标**：`score / collision_rate / offroad_rate / completion_rate`（PufferDrive）+ 可选 WOSAC realism + nuplan 25 metric 子集。
**产出**：Nova 在"非反应式 vs 反应式"下的**性能差**——这个差值本身就是最重要的迭代信号。

**验收**：能稳定产出"反应式对手下 Nova 的失败率"，且比 nuplan non-reactive 显著更高（证明压力测试有效）。

### Phase 2 — 失败挖掘 + 场景库（约 1 周）

**目标**：自动把失败变成可复现、可回归的资产。

**做法**：
1. 大批量 rollout（PufferDrive 高吞吐），用 `get_env_logs()` + 自定义事件检测挖掘：
   - 碰撞 / 出路 / 近失（min distance < δ）/ 被逼停 / 死锁 / 急刹；
2. 记录 `scenario_id` + t0 + 涉事 agent + 事件签名；
3. 用**场景编辑器**做最小化（删掉无关 agent / 截短时域），导出 `.bin`；
4. 场景库目录化 + 版本化：
```
scenario_bank/
├── bank.jsonl              # {id, tag, source, seed, t0, event, severity, owner}
├── scenes/*.bin            # 最小可复现场景
└── README.md               # 场景语义与期望行为
```

**验收**：每个已知失败都有对应最小场景，可一键回放复现。

### Phase 3 — 分层归因 + 定向修复（持续）

**目标**：找到根因，而不是"猜着改"。

**归因矩阵**（详见 §5）：用真值 oracle 注入 + 反事实编辑定位到"编码 / 预测 / 规划 / 仿真"哪一层。

**定向修复手段**（按根因选）：
| 根因 | 修复手段 |
|---|---|
| 交互预测不足 | 用 PufferDrive 反应式数据增强 agent prediction 分支；DAgger 式闭环数据 |
| planner 短视/保守 | 调整规划目标/损失权重；用 RL/DAgger 微调 planner 头 |
| 参考线/地图特征错 | 修 map/routing builder；扩展地图语义 |
| 数据覆盖不足 | 场景库样本回灌训练集（混合重放） |
| 决策后处理问题 | 修输出消费库（NMS/选择） |

**高级选项（可选）**：把 PufferDrive 自博弈 RL 策略当作**特权 teacher**，对 Nova 的 planner 做 on-policy 蒸馏 / DAgger（GigaPixel 的 self-play DAgger 思路）——teacher 看真值世界、跑得快；student（Nova）在访问到的状态上接受纠正。

### Phase 4 — 回归门禁 + 防复发（持续）

把场景库 + 多指标门禁做成 CI（详见 §6）。

---

## 5. 黑盒归因手册（专治"端到端不知道错哪"）

### 5.1 核心思想：真值 oracle 注入 + 反事实

PufferDrive 提供两样别人给不了的东西：**完整全局真值**（oracle）与**可编辑场景**（counterfactual）。二者组合就是归因的显微镜。

### 5.2 两段式 Nova 的分层归因实验矩阵

| 实验 | 干预 | 观察 | 结论 |
|---|---|---|---|
| E0 Baseline | 无 | ego plan / collision | 基线 |
| E1 Oracle-encode | 用真值场景状态替换 encoder 输入 | plan 是否改善 | 感知/编码层问题 |
| E2 Oracle-pred | 用 PufferDrive **真值 agent future** 替换 Nova 预测的 agent future，再跑 decoder | ego plan 是否改善 | **交互预测 → 规划** 的因果强度 |
| E3 Oracle-plan | 注入 GT/规则 planner 轨迹 | 是否还失败 | planner 能力 vs 环境/仿真问题 |
| E4 Counterfactual | 删除/移动关键 agent | 失败是否消失 | 罪魁 agent 因果定位 |
| E5 Sensitivity | 逐个 agent/map 特征做 mask | plan 变化幅度 | 输入敏感度 / 特征重要性 |

**E2 是两段式的黄金实验**：它直接回答"Nova 的 ego 规划错，是因为它把别人预测错了，还是因为它规划本身不行"。这一刀切开后，修复方向立刻明确。

### 5.3 一段式端到端的归因（迁移）

一段式没有中间语义接口，归因只能靠外部干预：
1. **输入遮挡/扰动**：单相机遮挡、agent 局部抹除、sensor dropout → 找失败依赖的输入；
2. **privileged vs sensor 双通道对照**：同一 planner 分别吃真值世界（特权）与感知输入跑闭环，两者差距 = 感知引起的失败比例（GigaPixel 的 teacher/student 架构天然提供这个接口）；
3. **行为签名分类**：急刹 / 轨迹曲率异常 / 反应时延超常 → 映射到根因族；
4. **反事实场景编辑**：同两段式。

### 5.4 归因产出物

每个失败场景在场景库中记录：
```json
{
  "id": "womd_xxxx_t0_23",
  "event": "rear_end",
  "severity": "collision",
  "attribution": {
    "layer": "interaction_prediction",
    "evidence": "E2 oracle-pred 下 collision 消失，ego plan minTTC 从 0.8s 升至 4.1s",
    "culprit_agent": 17
  },
  "fix_pr": "#1234",
  "regression": "pass"
}
```

---

## 6. 防复发与稳妥修复（不引发旧问题）

### 6.1 三层防线

```
第一层：场景库回归测试（bug → 最小场景 → CI 必过）
第二层：训练期防遗忘（混合重放 + policy pool + KL 约束）
第三层：多级评估门禁（任务指标 + 分布真实性 + 人类兼容性 + 舒适性）
```

### 6.2 第一层：场景库回归门禁

- 每个修复的 bug → 一个最小场景入库；
- 每次改动（数据/损失/结构/后处理）→ **全量回放场景库**：
  - 旧 bug 场景必须过（**不复发门**）
  - 新修复场景必须过（**修复生效门**）
  - 随机对照场景（防过拟合场景库）
- 建议做成 CLI + CI：
```
puffer-regress --bank scenario_bank/ --ckpt <new.ckpt> --baseline <old.ckpt>
  → per-scene diff table + gate pass/fail
```

### 6.3 第二层：训练期防遗忘

- **混合重放**：新数据/新场景 batch 里持续掺入旧分布 + 场景库样本；
- **Policy Pool / League**（借鉴 PufferDrive mixed-play 与 policy pool）：保留历史 checkpoint 作为对手/数据源，避免策略漂移到只擅长新场景；
- **KL 锚定**：微调 planner 时对旧策略加 KL 约束，限制分布漂移。

### 6.4 第三层：多指标门禁

单一 `score` 会漏掉分布退化。门禁需覆盖：
| 轴 | 指标 | 工具 |
|---|---|---|
| 任务能力 | collision / offroad / completion / score | PufferDrive log |
| 交互鲁棒性 | 反应式对手下失败率 | PufferDrive self-play |
| 分布真实性 | WOSAC realism | PufferDrive WOSAC |
| 人类兼容性 | human-replay 评估 | PufferDrive / nuplan |
| 舒适性/动力学 | jerk / lat-acc / lon-acc | nuplan 25 metric |
| 规划偏差 | PDMS/L2 vs GT | nuplan / NAVSIM 类 |

**门禁规则**：任何一项掉点超过阈值 → 阻断，输出"新旧对拍报告"，而不是静默通过。

### 6.5 变更影响分析

每次改动输出三张表：
1. **场景库逐场景对拍表**（旧 vs 新）；
2. **指标分桶表**（按场景类型：直行/路口/加塞/行人…）；
3. **失败迁移表**（新增失败 / 修复失败 / 复发失败）。

---

## 7. 工程落地

### 7.1 目录与依赖

```
nova/integrations/pufferdrive/       # 新增（或独立 repo）
├── bridge/                          # 三桥
├── eval/                            # reactive_eval / failure_miner
├── scenario_bank/                   # 场景库
├── gates/                           # 回归门禁
└── tools/                           # CLI
```

依赖：PufferDrive（`pip install -e .` + 编译扩展）、Nova 的模型加载与特征组装、nuplan metric（复用 Nova 的 vendored nuplan）。

### 7.2 关键接口清单（汇总）

| 接口 | 方向 | 说明 |
|---|---|---|
| `PufferToNovaStateBridge.build_tensors(env, t0)` | PufferDrive → Nova | 世界状态 → 17 张量 |
| `NovaPlanToPufferAction(plan_flu, ego)` | Nova → PufferDrive | 轨迹 → (accel, steer) |
| `ReactiveEvaluator.run(nova_ckpt, opponent_policy, scenes)` | 闭环 | 反应式评测 |
| `FailureMiner.mine(rollouts, rules)` | 日志 → 场景库 | 失败挖掘 |
| `OracleInjector.inject(level)` | oracle | 分层归因（E1–E3） |
| `RegressionGate.run(bank, ckpt)` | CI | 防复发门禁 |

### 7.3 里程碑与工作量

| 阶段 | 交付物 | 估算 | 风险 |
|---|---|---|---|
| P0 | 桥接 MVP + 对拍通过 | 1–2 周 | 地图语义/坐标系 |
| P1 | 反应式评测 CLI | 1–2 周 | RL policy 质量 |
| P2 | 场景库 + 挖掘器 | 1 周 | 事件规则设计 |
| P3 | 归因矩阵 + 定向修复 | 持续 | 根因多样 |
| P4 | 回归门禁 CI | 1 周 | 指标阈值标定 |
| 中期 | 数据桥 / DAgger / 地图语义扩展 | 3–6 周 | 红绿灯/地图 gap |

---

## 8. 风险与边界

| 风险 | 说明 | 缓解 |
|---|---|---|
| **地图语义 gap** | PufferDrive 地图比 Nova 薄，可能引入 domain shift | 先做语义映射 + gap 计数；中期扩展 map converter；可用"同源 WOMD 地图"降低差异 |
| **红绿灯缺失** | Nova 红绿灯分支无法在 PufferDrive 闭环验证 | 初期只跑无红绿灯场景并显式标记；红绿灯专项用 nuplan/实车数据 |
| **动力学/控制语义不一致** | PufferDrive wheelbase 与 Nova vehicle_param 不同，动作桥可能引入偏差 | 统一参数；用 nuplan same-tracker 对拍 |
| **RL 对手过强/不合理** | self-play 对手可能学出不拟人的对抗行为，导致虚假失败 | 用 `control_mixed_play` 混合人类回放；WOSAC 校验对手拟真度 |
| **仿真≠真实** | 所有结论都在仿真内，不能替代实车安全验证 | 作为"归因实验室 + 回归筛子"，不当作安全证明 |
| **场景库过拟合** | 只修场景库 → 模型记住场景 | 保留随机对照集 + 分桶指标 + 混合重放 |

---

## 9. 一句话总结

> **把 PufferDrive 当作 Nova 的"反应式压力测试台 + 失败矿机 + 真值 oracle + 回归裁判"**：用 self-play 反应式对手暴露 nuplan 看不到的交互失败，用全局真值做分层归因切开"感知/预测/规划"的责任，用高吞吐把每个失败沉淀成最小场景与自动门禁——从而在**黑盒端到端**的约束下，实现**快速、可归因、不复发**的 planner 迭代。

---

## 附录 A：PufferDrive 侧建议的小改动（可选，按需）

1. **导出接口**：增加"rollout → WOMD JSON/`.bin`"的一键导出（场景编辑器已有基础），服务数据桥与场景库。
2. **地图语义扩展**：`save_map_binary()` 保留 Nova 需要的 15 维语义（lane_turn_type / speed_limit / boundary 等）。
3. **固定场景 API**：允许按 `scenario_id` + t0 精确加载（当前 `shared()` 是随机采样），服务可复现回归。
4. **红绿灯支持**（roadmap 已列）：长期补齐 Nova 红绿灯分支的闭环验证。

## 附录 B：Nova 侧建议的小改动（可选）

1. **可插拔 planner 接口**：让 `ModelPlanner` 能从"外部世界状态"构造观测（当前依赖 `scene.model` 预存的 per-frame obs），以适配 PufferDrive 在线状态。
2. **批量推理**：桥接评测需要多 env 并行，Nova 前向应支持 batch>1（当前 `_to_batched_features` 是 bs=1 布局）。
3. **归因开关**：暴露 `oracle_agent_future` / `oracle_map` 等调试注入点，服务 §5 的 E1–E3。

---

*参考：*
- *PufferDrive：`docs/src/simulator.md`、`docs/src/evaluation.md`、`docs/src/scene-editor.md`、`pufferlib/ocean/drive/drive.h`、`pufferlib/ocean/drive/drive.py`、`pufferlib/ocean/env_binding.h`*
- *Nova：`CONTEXT.md`、`README.md`、`deploy/ads_models/nova-input-format.md`、`deploy/include/nova_deploy/io_contract.h`、`adfm/simulation/runner.py`、`adfm/simulation/planner_adapter.py`、`adfm/models/nova_model_pluto_light.py`、`docs/coord_FLU_to_RFU.md`*
- *同源工作：GigaFlow（arXiv:2502.03349）、GigaPixel（arXiv:2606.19641）*
