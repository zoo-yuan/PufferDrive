# canonical spec v0：字段注册表 + 三源数据审计（M1 / M2）

> 日期：2026-11-05
> 基线：`5cfe970a`（代码未改，本次只新增 `spec/` + `scripts/` + 生成物）
> 上位裁决：`docs/adr/0007-badcase-as-scenario-seed-and-constraint-only-feedback.md`（decision 3：canonical spec 模型无关、本仓拥有）
> 验收物：`docs/html/spec_v0_coverage.html`（M1）、`docs/html/spec_v0_data_audit.html`（M2）

---

## 0. 一句话

L0 canonical spec 从"要冻结的一张表"变成**可校验、可审计、数字可复跑**的注册表：73 字段、8 组、56 frozen（76.7%），并对 6 条真 bag + 3 张真地图逐字段给出了"数据源有没有真值 / 仿真有没有实现"的实证判定。

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
| `.scratch/m2/audit.json` | 审计缓存（bag 通道计数、地图统计、判定行） | 中间物 |

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

## 9. 下一步（M5）

bad case 自动挖掘 + 场景切片：用 `metrics_array` 18 项 + 20 条模板谓词批量扫数据湖，切片窗口 16+80，`scenario_id` 编码 `bag_id#frame`，附 provenance（地图/感知版本），入口做物理可行性门禁（实测湖里有 38 m/s 的定位跳变）。验收物：扫描结果直方图（每模板命中数 / 每 bag 命中密度）+ 切片清单。
