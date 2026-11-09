# canonical spec v0：字段注册表 + 三源数据审计（M1 / M2）

> 日期：2026-11-05
> 基线：`5cfe970a`（代码未改，本次只新增 `spec/` + `scripts/` + 生成物）
> 上位裁决：`docs/adr/0007-badcase-as-scenario-seed-and-constraint-only-feedback.md`（decision 3：canonical spec 模型无关、本仓拥有）
> 验收物：`docs/html/spec_v0_coverage.html`（M1）、`docs/html/spec_v0_data_audit.html`（M2）

---

## 0. 一句话

L0 canonical spec 从"要冻结的一张表"变成**可校验、可审计、数字可复跑**的注册表：73 字段、8 组、59 frozen（80.8%），并对 6 条真 bag + 3 张真地图逐字段给出了"数据源有没有真值 / 仿真有没有实现"的实证判定。

---

## 1. 产物

| 文件 | 作用 | 状态 |
|---|---|---|
| `spec/canonical_v0.yaml` | 73 字段注册表，唯一真值源（散文里不再手写字段表） | 现行 |
| `spec/README.md` | 冻结协议 + 三条硬约束 + 与其他文档边界 | 现行 |
| `scripts/spec_report.py` | 校验 + 覆盖率报告（`--check` 可做门禁） | 可用 |
| `scripts/spec_audit.py` | 三源审计（6 bag + 3 map + 车型档案），带缓存 | 可用 |
| `docs/html/spec_v0_coverage.html` | M1 验收物：组×属性热图、状态分布、全字段表、阻塞清单 | 生成物 |
| `docs/html/spec_v0_data_audit.html` | M2 验收物：bag 通道矩阵、地图拓扑实测、派生证据、字段×三源判定 | 生成物 |
| `scripts/milestone_review.py` | M1–M4 总览评审看板（流程 + KPI + 图 + 假设台账 + gate 台账） | 可用 |
| `docs/html/milestone_review.html` | 总览评审看板（每个 milestone 收口时重新生成） | 生成物 |
| `spec/bag_mining_v0.yaml` | bag 侧挖掘契约：8 条可挖 / 12 条不可挖 + 门禁 + 切片窗口 | 现行 |
| `scripts/badcase_mine.py` | 挖掘器（bag → 特征 → 谓词 → 切片）+ 报告 | 可用 |
| `docs/html/badcase_mining_v0.html` | M5 验收物：命中矩阵、时间轴、特征分布、切片清单 | 生成物 |
| `docs/measured/audit_canonical_v0.json` | 审计摘要（bag 通道计数、地图统计、判定行） | 入库，供看板复算 |
| `docs/measured/obs_envelope_6bags.json` | 观测包络测量结果（6 条 bag） | 入库，供看板复算 |

复现：

```bash
source .venv/bin/activate
python scripts/spec_report.py          # M1：覆盖率 + gate
python scripts/spec_audit.py           # M2：三源审计 + gate（首跑约 20 s，之后走缓存）
```

---

## 2. M1 数字（gate PASS）

| 量 | 值 |
|---|---|
| 字段总数 / 组数 | 73 / 8（ego 17、agents 11、occ 6、map 11、traffic 6、route 5、provenance 11、vehicle 6） |
| status | frozen 56（76.7%）· draft 16（21.9%）· blocked 1（1.4%） |
| sim | native 38 · bridged 26 · absent 9 |

gate 四项：字段数 ≥40 ✓、必填属性缺失 = 0 ✓、frozen ≥60% ✓、每组 ≥1 frozen ✓。

## 3. M2 数字（gate PASS）

| 判定 | 字段数 | 含义 |
|---|---|---|
| 三源齐备 | 40 | 数据源有真值 + 仿真已实现 |
| 待桥接 | 26 | 数据源有真值，仿真需桥接层产出 |
| 仅数据源 | 7 | 仿真侧完全没有（见 §5） |
| 仅仿真（凭空） | 0 | 修正后清零 |
| 两侧皆无 | 0 | — |

### 3.1 源 A：6 条 bag 通道命中率

9 个关键通道里 8 个是 6/6，唯一缺口是 `/apollo/perception/traffic_light`（**5/6**，`white-rhino-3139` 无此通道）。

bag 总量：26–36 通道 / 1.0–3.0 万消息 / 每条 10–20 s。

### 3.2 源 B：3 张真地图的拓扑实测

| 地图 | lane | predecessor_id | successor_id | turn | overlap_id |
|---|---|---|---|---|---|
| wuhan_qianjinjiayuan_shunfeng | 4672 | 4988（×1.07） | 4988（×1.07） | 4672（×1.00） | 5223（×1.12） |
| chongqing_yongnan_shunfeng | 6690 | 7237（×1.08） | 7237（×1.08） | 6690（×1.00） | 6532（×0.98） |
| 8997_shunfengsuyun…-1906 | 31186 | 32770（×1.05） | 32770（×1.05） | 31186（×1.00） | 31057（×1.00） |

结论：**拓扑在 Apollo 侧 100% 存在**，密度略大于 1（存在一 lane 多后继），`turn` 精确 1.00/lane。`base_map.bin` 的解析结果与 `base_map.txt` 的 grep 计数逐项吻合（chongqing：pred 7237 / succ 7237 / turn 6690 / speed_limit 6690），可作为审计器的自校验。

注：`6501_shunfengtongchenghuoyun-1595`、`7235_huolalalinyi-1588` 只有 `base_map.txt`（34 GB / 21 GB），无 bin，本次跳过（3/6 张图进入审计）。

### 3.3 源 C：车型档案

`~/Downloads/data_xijiao-master-3799_vehicle_param.pb.txt` 在位；`release.info` 只有 1 份（`rx-2714`），所以 `map_version` / `perception_version` 的台账来源在所有批次上**不均匀**。

---

## 4. 本次被数据推翻的两个假设（重要）

### 4.1 occ 不是"占据栅格"，是"对象多边形 + 地图静态物"派生物

原 spec 写 `occ_polygons` 的真值来源是"bag perception 占据栅格"。**实证：不存在。** 6 条 bag 的全部通道名里没有任何 occ/polygon/grid/free 通道；`/apollo/perception/safetyobstacles` 只有 ~1.3 KB（不是占据栅格）。

真实的可用来源是 **`/apollo/perception/obstacles` 的对象级 `polygon_point`**（Apollo `PerceptionObstacle` field 8）：

| 量 | 实测（3 条 bag × 20 帧） |
|---|---|
| 障碍物实例 | 6885 |
| 带 `polygon_point` | 6885（**100%**） |
| 多边形点数 中位 / 最大 | 8 / 35 |
| 带 `height` / 带 `type` | 6885 / 6885（100%） |

因此：
- `occ_*` 五个字段的 truth 已改写为派生物；`static_polygon_*` 四路（ADR-0006）的派生源被证实存在。
- 点数量级（中位 8、最大 35）与 `static_polygon_points[1,32,32,2]` 的 32 点等弧长重采样**不同**，重采样是必需步骤，不是可选优化。
- ADR-0007「occ 是物理实体」不变；变的是它的数据来源描述。

### 4.2 地图拓扑：Core 已经有 entry/exit，我原先把整块判成 absent 是错的

`RoadMapElement` 实际含 `num_entries/entry_lanes` 与 `num_exits/exit_lanes`（`datatypes.h:145-148`），`map_data.h:526-544` 从 bin 加载。ADR-0007 的原话（"只有 type/entry/exit/speed_limit，没有 boundary_type/turn_type/overlap"）本来就对——是我把 entry/exit 误当成"无拓扑"。

修正后：

| 拓扑项 | Apollo 源 | Core | 差额 |
|---|---|---|---|
| predecessor / successor | 100%（×1.05–1.08） | **native**（lane 索引） | 只需转换器做 id→索引 |
| turn_type | 100%（×1.00） | **absent** | 真缺 |
| overlap | 100%（×0.98–1.12） | absent | 缺，用于 stop_line / crosswalk 关联 |

结果：`blocked` 从 3 降到 1（只剩 `map_turn_type`），`frozen` 从 74.0% 升到 76.7%。

---

## 5. 7 个"仅数据源"字段（桥接层的确定工作清单）

| 字段 | 证据 | 落地成本 |
|---|---|---|
| `vehicle_param_id` | 车型档案在建，3799 = 车型类 | 场景元数据加字段 |
| `map_turn_type` | 3/3 图 100%，1.00/lane | 31186 lane × i32 ≈ 125 KB/场景（或 sidecar 列） |
| `bag_id` | 6/6 bag 可由文件名/路径得到 | 场景构建时写入 |
| `sensor_age_s` | 6/6 bag 有 obstacles 发布时序 | 桥接层计算 publish_lag |
| `map_version` | `release.info`（version/commit/branch）+ routing_response JSON | 读台账写入 |
| `perception_version` | `release.info`（仅 1/6 批次有） | 需补台账；否则置 unknown |
| `models_version` | 同上 | 同上 |

---

## 6. 未决（需 owner 裁决）

| # | 问题 | 建议 | 依据 |
|---|---|---|---|
| S1 | `map_turn_type` 放 Core 结构还是 map 侧表 | **map 侧表**（随 scenario 下发的 lane→turn 列） | Core 热路径不需要 turn（route 已预计算）；改 `RoadMapElement` 要动 bin 格式版本并重建 40 个 bin |
| S2 | `overlap`（stop_line/crosswalk 关联）是否进 L0 | 进，但仅在语义外挂侧消费 | ADR-0007 双视图；Core 不需要 |
| S3 | occ 多边形 32 点等弧长重采样的误差门限 | 定 `max 点到重采样折线距离 ≤ 0.1 m` 为验收线 | 中位 8 点、最大 35 点，重采样必然引入误差，需可量化 |
| S4 | 16 个 draft 字段的裁决时点 | 等 M3（观测包络 70 m / 0.53 s）落地时一并冻 | owner 2026-11-04：先都不动，等数据 |

---

## 7. M3 —— 观测包络落地（已完成）

验收物：`docs/html/spec_v0_obs_envelope.html`（双面板：距离直方图 + 数据年龄标线）；测量脚本 `data_utils/measure_obs_latency.py`；绘图 `scripts/spec_envelope_report.py`。

### 7.1 六条录包的实测分布（推翻了 handoff 的 0.53 s）

| bag | per-message max p50 | p100 | control 数据年龄 | planning 数据年龄 |
|---|---|---|---|---|
| 2292 | 66.6 m | 76.1 m | 1.06 s | 0.81 s |
| 2688 | 66.8 m | 78.8 m | 1.04 s | 0.93 s |
| 2714 | 66.0 m | 72.3 m | **0.53 s** | 0.46 s |
| 3123 | 66.4 m | 71.7 m | 1.21 s | 1.00 s |
| 3139 | 30.1 m | 65.4 m | 0.41 s | 0.32 s |
| 3183 | 62.8 m | 74.5 m | 0.41 s | 0.32 s |
| **中位** | **66.2 m** | **73.4 m** | **0.78 s** | 0.64 s |

- 距离：89,304 个障碍物观测中，`100–150 m` 与 `≥150 m` 两桶为 **0** → 70 m 是数据上界，不是保守假设。
- 延迟：**0.53 s 恰好等于 `rx-2714` 这一条的 control 值**，是单点低端；6 条中位 0.78 s、最差 1.21 s。
- 结论：冻结 `agent_range_m = 70`、`observation_delay_s = 0.78`，并把后者保留为 **per-scenario provenance**（每条 bag 带自己的实测值，范围 3 倍）。

### 7.2 仿真侧实现

| 参数 | 原值 | 现值 | 位置 |
|---|---|---|---|
| `obs_range_partner_m` | 200.0 | **70.0** | `pufferlib/config/puffer_drive.yaml`（C 侧过滤 `drive.h:3861`） |
| `observation_delay_s` | 不存在 | **0.78** | `Drive._apply_observation_delay`（ring buffer），`pufferlib/config_schema.py` 白名单 |

延迟语义（`tests/unit_tests/test_observation_envelope.py`，6 例全绿）：

- ring 大小 = `round(delay/dt)`；读最旧槽后再覆盖 → K 槽即 K 步延迟，`t ≥ K` 时与「K 步前的原始 obs」**逐位相等**。
- warmup 用 reset obs 填充，不会读到零或未来状态；`reset()` 与 map resample 都重新预热。
- 零延迟不分配 ring（零开销快速路径）。
- 量化误差显式上报：dt=0.3 时 0.78 s → 3 步 = 0.9 s，`effective_observation_delay_s` 记录实际生效值，不静默截断。

### 7.3 回归

`tests/unit_tests` 49 passed / 1 skipped；3 例 `test_action_selection` 与 7 个模块收集失败均因已知 B2（`pufferlib._C` 需 C++20，本机 g++ 9.4），与本次改动无关。

### 7.4 遗留

- `dt = 0.3 s`（`puffer_drive.yaml`）相对 0.78 s 的延迟偏粗（量化 ±0.15 s）；如要更细的延迟分辨率需评估改 dt 的连带影响（动力学/奖励阈值都按 dt 调过）。
- 6 条录包全部来自 `white-rhino` 一类车型；跨车型延迟分布未知。

---

## 8. M4 —— 约束模板库 v0 + 可行性裁判（已完成）

验收物：`docs/html/constraints_v0_library.html`（20×18 指标矩阵）、`docs/html/constraints_v0_feasibility.html`（三控制器判定表）。
库源：`spec/constraints_v0.yaml`；裁判：`scripts/constraint_judge.py`（可执行性）、`scripts/constraint_feasibility.py`（真跑）。

### 8.1 数字

| 量 | 值 |
|---|---|
| 模板总数 | 20（violation 16 = 80% / comfort 3 / progress 1） |
| 谓词可执行 | **20/20**（每条都能同时造出“触发”与“不触发”两行） |
| 引擎直接可评 | **85%**（17/20 只用 `metrics_array` 18 项 + 状态量） |
| 需桥接新指标 | 3 条：`crosswalk_clearance_m` / `pedestrian_clearance_m` / `junction_occupancy` |
| 可行性裁判 | 3 控制器 × 40 步 × 8 张 carla 图，**约 2 s** |
| 判定结果 | 可解 18 / IDM 不足 2 / 不可解 0 |

### 8.2 设计要点

- **谓词是结构化的**（`{metric, op, threshold_param}` + `guard` + any-of 列表），不是字符串；指标词表从 `constants.h` 的 `*_IDX` 定义**读出来**，不手抄，所以引擎改指标顺序时库会自动对不上。
- **可执行性是硬门禁**：裁判为每条模板合成两行（触发行 / 基线行），两边都对才算入库。这直接堵住了“写了个漂亮的约束但跑不起来”的窟固。
- **派生指标的 baseline 是显式声明的**（`baseline_safe`）：间距类指标“无行人”= 1000 m，占用类“不占用”= 0。否则基线会被误判为触发。
- **可行性三分判定**：IDM 未触发 → 可解；IDM 触发但 REPLAY 未触发 → “IDM 不足”（场景仍需更强策略，不能判死）；两者都触发 → 不可解，禁止进 RL。

### 8.3 裁判自己发现的两个问题

1. **`stall_crawl` 缺 warmup**：一开始它在“静止起步”时就触发（静止 = 不前进）。已加 `warmup_s: 2.0`，并修正 gate 语义——进度类谓词对着 STATIC 触发是**预期**（负对照在工作），不算坏谓词。
2. **IDM 自己就违反 comfort 阈值**：`lat_accel_over`（8.0 m/s²）与 `jerk_over`（5.0 m/s³）在 gigaflow 随机场景下被 IDM 触发。两个可能：IDM 参数未按 3799 车型标定，或 8.0 代理值偏紧。待定，记入遗留。

### 8.4 遗留（进 M5 前必须知道）

- **`replay` 控制器在 gigaflow 下不是真见证人**：gigaflow 是随机 spawn，没有录制轨迹可回放。真正的可行性裁判必须跑在**录制场景**上（即 bad case 挖掘产物）。当前实现只验证了“模板会/不会误触发”的机制，不是场景统计。
- 3 条谓词依赖的指标引擎没有 → 桥接层任务（与 M2 的 6 个“仅数据源”字段一起排期）。

---

## 9. M5 —— bad case 自动挖掘（已完成）

验收物：`docs/html/badcase_mining_v0.html`（命中矩阵 + 逐 bag 时间轴 + 特征分布 + 切片清单）；
契约：`spec/bag_mining_v0.yaml`；脚本：`scripts/badcase_mine.py`；结果：`docs/measured/badcase_mining_v0.json`。

### 9.1 先把"能不能挖"写成数据

`constraints_v0` 的谓词读引擎 `metrics_array`（`lane_dist` / `red_light` 都要地图匹配），而数据湖里只有 bag。
契约显式写下差集：**8 条可挖、12 条不可挖并写明缺什么**。这样挖掘脚本不会因为"算不出来"而悄悄少挖。

| | 条数 | 内容 |
|---|---|---|
| 可挖 | 8 | ttc_shortfall、braking_distance_short、collision_any、cut_in_conflict、lat_accel_over、long_accel_over、jerk_over、stall_crawl |
| 不可挖 | 12 | 需要地图匹配（红绿灯/停车标志/越野/超速/车道偏离/朝向/多车道/汇入）、责任判定、或引擎与 bag 都没有的指标（斑马线/行人间距） |

### 9.2 结果数字

| 量 | 值 |
|---|---|
| 录包 | 6 条（8992 个可信帧） |
| 被门禁拒绝 | 22 帧（`implausible_long_accel`：raw 里有 −16.8 m/s²，超出 −6.0 硬限近 3 倍） |
| 切片 | **487**（占可信帧 5.4%），每条带 `bag#frame` + 16 历史 + 80 未来 |

| 模板 | 切片 | 分布 |
|---|---|---|
| jerk_over | 161 | 散在各条包 |
| cut_in_conflict | 117 | 强集中在 2688 / 3139 / 3123（城区） |
| long_accel_over | 104 | 2714 / 3123 为主 |
| ttc_shortfall | 60 | 2688 / 3139 |
| braking_distance_short | 27 | 全在 3139 |
| collision_any | 14 | 全在 3139（gap −0.04…−0.29 m、横向 0.69 m、≈5 m/s：真实轻碰） |
| stall_crawl | 4 | 2292 / 2714 / 3183 |
| lat_accel_over | **0** | 阈值 8.0 m/s²，而这批数据 \|a_lat\| 最大 7.92 → 这批包没有横向极限工况 |

### 9.3 挖掘器自己抓到的三个 bug（都改掉了）

1. **"最近障碍" ≠ "前车"**：in-path 判定用了双方车宽之和，邻道并排车被当成前车，58% 的帧被判成追尾（5207 切片）。
   收紧到「半个车宽 + 0.3 m」后命中率降到 5.4%。**碰撞重叠是双方车宽之和，在不在我这条车道是我的车宽** —— 两个概念。
2. **pose 加速度差分 ≠ jerk**：实测 \|jerk\| 中位 5.5 m/s³、51% 的帧超阈值、p90 到 41 —— 那是 20 Hz 姿态噪声。
   改为**滑窗三次多项式拟合有符号纵向速度**再求导，窗口扫描结果：

   | 窗口 | \|jerk\| p50 | p90 | >5 m/s³ 帧占比 |
   |---|---|---|---|
   | 0.4 s | 4.19 | 9.86 | 39.9% |
   | 0.8 s | 1.29 | 5.09 | 10.1% |
   | **1.0 s** | **1.11** | 4.20 | 7.4% |
   | 1.5 s | 0.79 | 3.08 | 4.5% |

   取 1.0 s：噪声地板 1.11 对阈值 5 → SNR 4.5×，代价是时间分辨率 ±0.5 s（已写进契约）。
3. **cut_in 与 in-path 横向互斥**：in-path 收紧后横向必然 ≤1.0 m，而 cut_in 的 guard 要求 >1.0 m，永远不触发。
   新增**邻道候选**特征（横向 1.0–3.5 m 的前方障碍的 TTC），用 band 代替引擎的 `lane_dist` guard。

另外补了一个门禁缺口：纵向加速度也要做物理可行性判定（`max_plausible_long_accel_mps2 = 10.0`）。

### 9.4 遗留

- 12 条不可挖的约束全部依赖地图匹配（把 ego 贴到 Apollo map 的 lane 上）。这是 M6 之后的第一优先级，
  否则 60% 的违规型约束永远只能靠仿真挖。
- `cut_in_conflict` 是最松的代理（横向 band + TTC），117 个切片里预计有偏多；需要在真场景上标定。
- `replay` 控制器在 gigaflow 下仍不是真见证人；用这 487 个切片跑可行性裁判才能给出场景级判定。

---

## 10. M6 —— 桥接 wash-in + 地图匹配（已完成）

验收物：`docs/html/bridge_washin_report.html`（Δxy 曲线）、`docs/html/map_match_report.html`（匹配质量 + 轨迹上色）。
实现：`pufferlib/ocean/drive/control_bridge.py`、`data_utils/apollo_lane_index.py`。

### 10.1 acc wash-in：把 0.39–0.92 m 的静默偏差关掉

`control_bridge.py` 复刻 nova 的 `target = 2·mean(acc[:5] − a0) + a0`，前 4 个样本按 0.2/0.4/0.6/0.8 插值。
**不改进公式**——它是模型侧既有行为，改了就与模型不一致。

| seed | 不洗 max \|Δxy\| | 洗过 max \|Δxy\| |
|---|---|---|
| 0 | 0.916 m | 1.28e-05 m |
| 1 | 0.527 m | 2.98e-05 m |
| 2 | 0.394 m | 8.53e-06 m |

gate：洗过 < 1e-4 m（float32 地板），不洗 > 0.1 m（否则测试失去分辨力）。
`tests/unit_tests/test_control_mode_parity.py` 12 例全绿，其中 5 例专门钉这个生产模块
（只动前 4 个样本、常量加速度恒等、短序列不崩、长度不匹配报错）。

### 10.2 地图匹配：解锁 4 条车道类约束

`data_utils/apollo_lane_index.py` 把 map 二进制里 `Curve → Segment → LineSegment → Point` 解析成车道点列
+ 网格索引，给出有符号横向偏移、航向差、车道限速、turn。

**两种地图形态都要支持**（此前我判错了，见 §10.5）：

| 形态 | 地图 | 加载方式 |
|---|---|---|
| 单文件 | wuhan / chongqing / 8997（47–406 MB 的 `base_map.bin`） | 整图解析 |
| **分块** | 6501（10,874 个 tile）、7235（6,249 个 tile） | 读 `base_map.catelog`（84–115 MB，tile 列表 + 500 m bbox）→ 按 bag 轨迹 bbox 只加载相关 tile |

| bag | 地图形态 | 匹配率 | lateral 中位 | 行驶中 \|heading\| p90 | 置信匹配率 |
|---|---|---|---|---|---|
| 2292 | 分块（6501） | 100% | +0.42 m | 33.7° | **72%** |
| 2688 | 单 bin | 100% | +0.01 m | 0.6° | 100% |
| 2714 | 单 bin | 100% | −0.31 m | 2.7° | 100% |
| 3123 | 分块（7235） | 100% | +0.12 m | 1.1° | 100% |
| 3139 | 分块（7235） | 100% | −0.59 m | 6.9° | 100% |
| 3183 | 单 bin | 100% | +0.01 m | 0.3° | 100% |

- **匹配必须有航向门限**：双向道路的对向车道中心线只差一个车道宽，只按"最近点"匹配会把车贴到对向车道上
  （实测 p90 航向差 179°）。现在候选点要求 `|heading_diff| ≤ 60°`，否则不匹配。
- **横向超过 3 m 直接拒绝**：宁可"不出判定"，也不伪造车道偏离（否则邻道/无车道区域的帧会变成假违规）。
- **置信匹配率**（`|heading| ≤ 10°` 且 `|lateral| ≤ 3 m` 的帧占比）是数据质量指标：2292 只有 72%，
  因为它大半时间在场站里挪车（速度 1.5–2.1 m/s，那里本来没有车道线）。其余 5 条都是 100%。

### 10.3 挖掘契约扩到 12 条（M5 的 8 条 + 4 条车道类）

新增：`lane_departure`、`heading_mismatch`、`overspeed`、`multi_lane_wander`。重跑结果：

| 量 | M5 | M6 |
|---|---|---|
| 可挖约束 | 8/20 | **12/20** |
| 切片 | 487 | **779** |

| 模板 | 切片 | 主要来源 |
|---|---|---|
| jerk_over | 231 | 全部 6 条 |
| cut_in_conflict | 156 | 2688 / 3139 / 3123 |
| lane_departure | 156 | 3183(73) / 3123(50) / 3139(33) |
| long_accel_over | 104 | 2714 / 3123 |
| ttc_shortfall | 60 | 2688 / 3139 |
| multi_lane_wander | 27 | 3139 |
| braking_distance_short | 27 | 3139 |
| collision_any | 14 | 3139 |
| stall_crawl | 4 | 2292 / 2714 / 3183 |
| heading_mismatch | 0 | — |
| overspeed | 0 | 车队未超速，或匹配车道限速更高 |

### 10.5 撤销一条错误结论（分块地图不是缺地图）

上一版我写的是"3 条 bag 没有 `base_map.bin`，属于数据供给缺口"。**这是错的。**
`6501_shunfengtongchenghuoyun-1595` 与 `7235_huolalalinyi-1588` 是**分块地图**：
`base_map.catelog`（84 MB / 115 MB）+ 6,249 / 10,874 个 500 m tile bin（每个 0.1–9 MB），
另外 `sim_map.bin` 是 0 字节、`routing_map.bin` 是 298–311 MB 的路由图（不含几何）。
正确做法是按 bag 轨迹 bbox 从 catalog 里选 tile，而不是要求一张整图 bin。修正后 6/6 可匹配。

教训：判断"数据缺失"之前先看目录里有没有索引文件（`*.catelog`）—— 大图本来就不会有整图 bin。

### 10.4 又抓到两个 bug（都改掉了）

1. **车道偏离在停车时触发**：3183 的 684 个"偏离"帧里 **523 帧速度只有 0.02 m/s** —— 那是停在装卸点，
   不是偏离。车道类谓词全部加"行驶中" guard（`ego_speed_mps > 0.5`），偏离切片 219 → 73。
2. **`lane_id` 解析带进了 protobuf 头**：解出来是 `'\n\x0815790859'`。某些地图代际把 id 包了一层 message，
   现在先尝试整串 ASCII，失败则再拆一层。

---

## 11. M7-1 —— route/goal 推导（已落地）

切片只说明"车在哪、错在哪"，不说明"车想去哪"。PufferDrive 的场景 bin 里每个 agent 需要
`route`（车道索引序列）与 `goals`，引擎侧已有 `ROUTE_TARGET_DISTANCE = 1000 m`、
`MAX_ROUTE_LENGTH = 64`、`GOAL_SOURCE_GT_MAP`（把 log 轨迹投影到同向车道）。

**做法**：自车轨迹 → 车道匹配 → 沿 `successor_id` 走到 1000 m ✂ 拓扑沉默时按几何连续性接续
（起点间距 ≤20 m 且航向差 ≤60°），每段的走法都记在 `why` 里（`successor` / `branch` /
`geometric(no successor)`），可审计；分块地图按需补 tile（`TileLoader` + catalog 的元素 id 表）。

| 量 | 结果 |
|---|---|
| 切片数 | **779** |
| 能串出连续 route | **779 / 779 = 100%** |
| 走满 1000 m | 497（`ok`） |
| 连续但短于 1000 m | 282（`usable_short`，goal 放 route 末端） |
| 未匹配帧 | **0.0%** |
| route 长度 | 中位 1001 m（p10 425 m / p90 1016 m） |

按 bag 看，失败**不是随机分布**：

| bag | 切片 | route 中位 | 停止原因 | 处置 |
|---|---|---|---|---|
| 2714 / 3183 / 3123 / 2292 | 497 | 1001–1016 m | `target_reached` | 直接可用 |
| 3139 | 211 | **425 m** | `route_length_cap`（64 段上限，车道平均 6.6 m 的路口碎段） | goal 放 425 m 处 |
| 2688 | 71 | **307 m** | `loop`（小区环道，后继为空且能接上的候选都走过） | goal 放 307 m 处 |

验收物：`docs/html/route_preview.html`（单切片几何 + 车道链）与 `docs/html/route_coverage.html`（779 切片覆盖率）。

### 11.1 这一步被真数据推翻的三条假设

1. **turn 枚举从 1 开始**（`NO_TURN=1, LEFT=2, RIGHT=3, U_TURN=4`）。按 0 起读会把 25 段 route 全标成
   `LEFT_TURN`（"转弯 25"）。`LaneType` 同理（`CITY_DRIVING=2`）。
2. **纯拓扑走不通**。2688 的链在 307 m 处后继为空，但空间上明显接着下一条车道；3123/2292 的后继
   落在没加载的 tile 里。两种都要显式处理（几何兜底 + 按需补 tile），否则 779 条里 589 条被误判成"断链"。
3. **1000 m 不是硬指标**。3139 的车道路口碎段平均 6.6 m，引擎 64 段上限只够 425 m。硬要求 1000 m
   会把"数据没问题、只是需要把 goal 拉近"的场景判死。

---

## 12. M7-2 —— 场景构建（已落地）

779 个切片写成 `load_map_binary` 能直接吃的 map bin：agents(log 轨迹) + roads(车道几何/邻接)
+ lane_graph + 元数据。脚本 `scripts/scene_from_slice.py`（单条）/ `scripts/scene_build_report.py`（批量）。

| 量 | 结果 |
|---|---|
| 写成 bin | **779 / 779** |
| 结构校验问题 | **0**（route ≤64、邻接不越界、cum_lengths 与 length 一致、lane_graph 尺寸对） |
| 场景车道（中位） | 68 条（route + 20 m 内邻道，按引擎网格密度裁剪） |
| route 段数（中位） | 56 段（上限 64） |
| 回放偏差 | **0.000 m**（每 bag 抽 2 条 × 40 步，replay 模式逐步对拍 log 位姿） |

验收物：`docs/html/scene_build_v0.html`。场景文件在 `.scratch/scenes/`（不入库）。

### 12.1 写场景时踩到的四个坑（都会静默出错）

1. **坐标必须先平移**。Apollo 的 UTM 量级 ~3.4e6 m，float32 在该量级最小分辨率是 **0.25 m** ——
   不平移等于把车宽磨掉一个数量级，而引擎不会报错。
2. **route / lane_graph 的下标是 road 数组下标**，不是 Apollo 的 `lane_id`。写错不会崩，只会让
   lane-graph 距离观测读到别的车道。
3. **尾部三段可选段顺序固定**（相位 → 车道宽 → 限速区）。要写车道宽就必须带相位标签，否则
   读取器把 `LANEWID1` 当相位标签报格式错。
4. **引擎网格是 5 m 一格、每格最多 64 个实体**。2292 有个路口 5 m 格里挤了 67 条车道，引擎直接
   拒绝建图。所以邻道要按格占用裁剪（软上限 48，留余量），route 车道一条都不能删。

另外两个"看起来是 bug、其实是比错了"：
- 回放偏 0.68 m → 实际是我拿第 k 步对 log 第 k 帧，而 replay 模式第 k 步输出的是 log[k+1]，对齐后 **0.000 m**；
- `agent.get("x")` 静默返回 0（state 里叫 `sim_x`），量出来全是 23.8 m 的假偏差。

### 12.2 已知边界（M7-3 之后补）

- 场景只有自车，**没有背景交通**（partner 车道已写入，但没生成对手）。
- 不含交通控制（红绿灯/停止线）—— 车道类型统一写 `LANE_SURFACE_STREET`。
- 回放对拍只证明"场景编码正确"，不证明"bad case 在仿真里仍会发生"——后者是 M7-3 场景级可行性裁判要做的事。

---

## 13. M7-3 —— 场景级可行性裁判（已落地）

对 3183 的 153 个场景，逐个用 3 个 controller 真跑（replay / idm / static，各 80 步），
用 `spec/constraints_v0.yaml` 的谓词（与 M4 同一套实现）判定它代表的 bad case 是否复现。

| 判定 | 场景 | 含义 |
|---|---|---|
| 复现且可避 | **70** | REPLAY 判得出违规，IDM 不违规 → 违规来自驾驶员选择，场景可解 |
| replay 下指标恒为 0 | 46 | comfort（accel/jerk）需动力学积分，replay 贴位姿不积分 |
| 场景缺背景交通 | 31 | ttc / 碰撞类需要对手，M7-2 场景只有自车 |
| 引擎判不出 | 5 | 探测器与引擎口径不一致，需逐条看 |
| STATIC 也触发 | 1 | `stall_crawl`（进度型，原地不动天然成立，非谓词缺陷） |

**可测切片复现率 71/76 = 93%**，IDM 全部不违规（0 条"场景逼人违规"）。
验收物：`docs/html/scene_feasibility_v0.html`。

### 13.1 这一步抓到的三个真问题

1. **`lane_departure` 缺运动 guard**。STATIC 控制器（原地不动）就能触发车离道 —— M5 挖掘侧早已按
   0.5 m/s 加门限（219→73），约束库里漏了。已补 `guard: sim_speed > 0.5 m/s`，M4 两个 gate 重跑仍 PASS。
2. **replay 模式下 `sim_speed` / `accel_long` / `jerk_long` 全是 0**。replay 直接贴 log 位姿、不积分
   动力学，所以：① 带速度门限的谓词在 replay 下永远不成立（补 guard 后 lane_departure 一度掉到 1/76）；
   ② comfort 违规无法用 replay 验证。处置：replay 下速度由相邻位姿反推（`|Δp|/dt`），comfort 判成
   "replay 不可测"而不是"未复现"。
3. **分母不能是全部切片**。153 条里 46 + 31 条在当前场景形态下根本测不出（没有动力学、没有对手），
   把它们算进失败率会把 93% 说成 46%。KPI 改按"可测切片"计，并把不可测的原因单列。

另外自造了一个静默失败：`metric_index` 用大写建表、按小写查 → KeyError 被 except 吞掉，
整批判成"未复现"。现在 controller 抛异常会计数并让 gate 失败。

---

## 14. M7-4 —— 回归保留率（闭环首次跑通，71/71 违规被消除）

两轮训练 + 三次评测，全部同口径（同批场景 bin、同一条谓词、60 步、确定性动作、固定种子）：

| 评测对象 | 违规场景 | lane_dist 中位峰值 |
|---|---|---|
| **replay 参考**（录制司机按 log 开） | **71 / 71** | 2.00 m |
| round A（随机初始化，300k 步 PPO） | 1 / 71 | 0.53 m |
| round B（从 A 续训 300k 步） | **0 / 71** | 0.53 m |

| KPI | 值 |
|---|---|
| **修复率（对录制司机）** | **71/71 = 100%** |
| 相对上轮 checkpoint 修复 | 1/1（A 剩下的那条也被 B 消掉） |
| **回归保留率** | **100%**（70 个原本正常的场景，B 之后仍正常） |
| 被改坏 | 0 |

验收物：`docs/html/regression_retention_v0.html`。训练产物 `experiments/roundA`、`experiments/roundB`
（不入库）。复跑：

```bash
python scripts/regression_retention.py \
  --baseline-checkpoint experiments/roundA/final_model.pt \
  --checkpoint          experiments/roundB/final_model.pt
```

### 14.1 这条链上抓到的三个数据/度量问题（都会静默出错）

1. **`scenario_id` 撞车**：`bag#int(t_start/0.1)` 在 86 Hz 的 pose 上把约 9 帧压成同一个 id ——
   779 条"切片"其实只有 **82 个不同场景**，场景文件互相覆盖、统计被放大 9.5 倍。改成毫秒 + 模板 + 去重后
   变成 779 个唯一场景。
2. **场景 bin 缺 `SPDZONE1` 段**：`speed_limit_random_prob > 0` 时引擎整份 map 拒载，训练一开就崩。
3. **`sim_speed` 在 replay 下恒为 0**：replay 贴位姿、不积分动力学，于是所有带速度门限的谓词
   （`lane_departure` 的 0.5 m/s guard）在参考跑里全部失效，一度量出"参考零点 0 违规"这种假结论。
   现在 replay 下的速度由相邻位姿反推 `|Δp|/dt`。

### 14.2 场景包络 20 m → 40 m（会改变结论，必须记录）

初版只把 route ±20 m 的车道写进场景。策略一开快就会开出车道集，`lane_dist` 撞上 4.0 的归一化上限，
被判成"越道"。扩到 ±40 m（覆盖 3 条并行车道 + 一个路口）后，同一批场景里 replay 参考仍是 71/71 违规，
而策略侧从"71 条全违规"变成 1 条 —— 说明此前的多数"违规"是**地图包络不足造成的假阳性**。

教训写进 §14.3：**场景包络是谓词的一部分**。改包络等于改判据，必须和数字一起记录，否则同一句
"71 个 bad case"在两次实验里指的不是同一个东西。

### 14.3 现在的判据可信度

- 这一轮的"修复"只在**引擎自己的 `lane_dist` 指标 + 当前包络**下成立；
- 还没验证策略在**真车坐标系/更大包络/其它 bag** 上的泛化；
- `lane_departure` 的语义仍是"离最近车道 > 0.9 m"，而"离 route 车道"才是驾驶意义上的偏离 ——
  这是一条待裁决的口径（见 §6 未决项）。

---

## 15. 评审仪式（每个 milestone 收口必须交的四件东西）

收口不是"写完了"，而是"可被 review 了"。每个 milestone 必须同时产出：

1. **一条可复跑的 gate 命令**：`python scripts/<report>.py --check`，失败时退出码非 0。
2. **一份 HTML 验收物**：数字 + 图，落在 `docs/html/`，由脚本生成而非手写。
3. **总览看板重新生成**：`python scripts/milestone_review.py` → `docs/html/milestone_review.html`
   （流程条 + 每站 KPI + 图表 + 被推翻的假设台账 + gate 台账）。
4. **会话里贴三样**：gate 状态、关键数字、**被真实数据推翻的假设**（这一项最有 review 价值）。

现有六份验收物：

| 文件 | 覆盖 | 生成命令 |
|---|---|---|
| `docs/html/milestone_review.html` | M1–M6 总览看板 | `python scripts/milestone_review.py` |
| `docs/html/spec_v0_coverage.html` | M1 字段覆盖率 | `python scripts/spec_report.py` |
| `docs/html/spec_v0_data_audit.html` | M2 三源审计 | `python scripts/spec_audit.py` |
| `docs/html/spec_v0_obs_envelope.html` | M3 观测包络 | `python scripts/spec_envelope_report.py` |
| `docs/html/route_preview.html` / `route_coverage.html` | M7-1 route 推导 | `python scripts/route_preview.py` / `route_coverage.py` |
| `docs/html/scene_build_v0.html` | M7-2 场景构建 | `python scripts/scene_build_report.py` |
| `docs/html/scene_feasibility_v0.html` | M7-3 场景裁判 | `python scripts/scene_feasibility.py` |
| `docs/html/regression_retention_v0.html` | M7-4 回归保留率 | `python scripts/regression_retention.py` |
| `docs/html/constraints_v0_library.html` | M4 约束库 | `python scripts/constraint_judge.py` |
| `docs/html/constraints_v0_feasibility.html` | M4 可行性裁判 | `python scripts/constraint_feasibility.py` |
| `docs/html/badcase_mining_v0.html` | M5 bad case 挖掘 | `python scripts/badcase_mine.py` |
| `docs/html/bridge_washin_report.html` | M6 桥接 wash-in | `python scripts/bridge_washin_report.py` |
| `docs/html/map_match_report.html` | M6 地图匹配 | `python scripts/map_match_report.py` |

原则：**看板里的数字一律从原始产物重算**（`spec/*.yaml`、`docs/measured/audit_canonical_v0.json`、`docs/measured/obs_envelope_6bags.json`、真跑一遍可行性裁判），
不手抄、不复用历史值。手抄的数字一旦漂移，看板就从"验收工具"变成"幻觉放大器"。
