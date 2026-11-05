# bad case 即场景种子：A-only 约束反馈下的 1→100 闭环

> 日期：2026-11-04
> 触发：owner 会话裁决 —— 目标不是"给 Nova 定制仿真"，而是"用 PufferDrive 得到一个吃矢量化输入（agents / occ polygons / HD map / traffic lights）的 planner，并能持续吃 bad case 反馈、迭代到上车"。
> 基线：HEAD `9d813755`；PufferDrive 行号按当前分支核实，Nova 行号按 `~/labs-x/agent_group/nova`。
> 与既有文档的关系：`docs/2026-10-31-PufferDrive-vs-TerraZero-仿真器对比.md` 与 `docs/2026-11-03-Terra系列全景与PufferDrive落地路线.md` 已完成 Terra 系列对比与五成分因果链。**本文只做"反馈闭环"这一轴**，不重复 Terra 分析，只在需要处引用其结论。

---

## 0. 一句话

在 **A-only**（反馈只给约束、没有专家修正、没有好样本、没有标签）的契约下，bad case 的价值不是"标签"而是"种子"：一条 `bag + 时间戳` 加一张可下载的 HD map，能长出一个**场景族**。

**1→100 的引擎 = 自动挖违规 + 约束模板化 + 回归保留。** RL 是发动机，约束是方向盘，评测是刹车。三者缺一，车都会撞墙——只是撞的墙不同。

---

## 1. 本次会话确立的六个判断（含对上轮三个错误的修正）

| # | 判断 | 依据 | 修正了什么 |
|---|---|---|---|
| J1 | **RL 是核心手段**，IL 降级为可选 anchor | TerraZero：nuPlan val14 **94.19（#1）**、InterPlan **70.87（#1）**、零人类演示、零日志轨迹；吞吐 560K（消费卡）/1.3M（服务器）/2.8M（8×节点）agent-steps/s（一手页面，2026-11-04 抓取） | 修正本文作者上一轮"RL 只是 0→1 加速器、1→100 靠回归"的排序 |
| J2 | **A/B 在执行层等价**：Nova 的 trajectory 已含控制量 | `diffusion_head.py:_integrate_control` 输出 `(x, y, yaw, v, acc, kappa)`，dt=0.1 逐步积分；PufferDrive 的 jerk 动力学在内部把 `(j_long, j_lat)` 积成 `(a_long, a_lat)`，且 `a_lat = κ·v²` | 修正"轨迹级 RL 需要额外 tracker"的判断：**不需要 tracker 就能执行**；但需新增直连控制的 action 模式（§6.1.1） |
| J3 | **PPO 的可用性取决于似然从哪来**，不取决于 A/B | `motion_prob`（3 refline × 30 long_mode = 90）是 categorical，`modality_sample_policy` / `sample_temperature` 在采它 → 有 `log π`；diffusion 连续精修头无似然 | 修正"PufferDrive 的 PPO 不再使用"：Discrete(90) 分支可承接；**无似然的是 diffusion，不是整个模型** |
| J4 | **Nova 不是契约，是基线 + 参考实现** | owner："不必拘泥 nova 网络结构"；Nova 只是"在我们的采集数据上实现 model planner 的一条路径" | 修正"19 张量即契约"：canonical 规格必须模型无关 |
| J5 | **occ 是物理实体**，进碰撞/offroad 热路径 | owner 裁决 | 修正"occ 只作观测通道" |
| J6 | **走廊锚定与全图漫游都要**，是两个合法性域 | owner 裁决 | 修正"M1 复用录制帧即足够" |

架构后果（L0–L3 分层，**顺序即优先级**）：

| 层 | 内容 | 为什么在这个位置 |
|---|---|---|
| **L0 数据契约** | canonical scene spec + 三源 adapter（WOMD / nuPlan / rino），带 provenance + 感知版本 + 地图版本 | 模型会换三代，这一层不能换 |
| **L1 评测与回归** | bad case → 可复现场景切片 → 回归集（单调增长）→ 指标 | **1→100 的引擎在这里，不在 RL** |
| **L2 世界** | PufferDrive Core + 反应式背景 + occ 物理 + 积分/执行保真 | 让"仿真里变好"这件事有意义 |
| **L3 算法与模型** | IL / RL / 模态选择 / 任意 encoder | 最可替换的一层 |

> 这不是"RL 不重要"的另一种说法。恰恰相反：**RL 没有人类轨迹作天然约束，会主动寻找仿真器的缝**。Comfort by Construction 在 PufferDrive 上实测：无约束动作网格下共享策略学会"最后一刻急刹 + 急打方向"刷安全分，jerk 越限 4513%；BehaviorBench 实测纯自博弈对 IDM 碰撞率高一个数量级。**J1 抬高的是 L1/L2 的门槛，不是降低它。**

---

## 2. 反馈契约：A-only 的定义域

owner 口径：**违规型 80% / 体感型 20%**（2/8 原则，owner 自陈可能与主观有关，后续以数据湖统计替换）。

| 类型 | 例子 | A-only 能否处理 | 处理方式 |
|---|---|---|---|
| **违规型** | 碰撞、压线/越界、闯红灯、越停止线、TTC 过低、jerk 越限、逆行 | ✅ 直接是谓词 | 全自动：指标即谓词，无需标注 |
| **体感型** | "这个路口太犹豫"、"该让没让"、"贴边太近"、"汇入太慢" | ⚠️ 需翻译成量化谓词 | 半自动：进度下限 / 停车时长上限 / 横向偏移上限 / 让行条件 / 加速度包络 |
| **纯偏好** | "开得不像老司机" | ❌ 不可表达 | **明确放弃，并写进文档** |

**这条界限是方案的定义域，不是缺陷。** 翻译不了的 bad case 不要假装能修——否则你会用 RL 去优化一个不存在的信号，得到一个"约束全绿、司机不认"的模型。这是本方案最可能的失败形态。

按 80/20 分配人力：

- **80% 安全线**：零人工。夜间批处理跑数据湖，自动产出场景族与回归条目。
- **20% 体感线**：需要一个人把自由描述转写成 `(模板, 参数)`。这是"约束工程"角色存在的唯一理由。

---

## 3. 1→100 的回路（可直接开工）

```
路测 bag + HD map（可下载，非卡点）
  │
  ├─[1] 自动违规挖掘
  │     PufferDrive `metrics_array[NUM_METRICS]`（18 项，`datatypes.h:78-86`）
  │     已含 collision / offroad / red_light / stop_sign / ttc /
  │         lane_dist / lane_angle / comfort_violation / at_fault_collision
  │     → 对日志轨迹批量扫，捞出违规帧。**不需要任何标注**
  │
  ├─[2] 场景切片
  │     窗口 16 历史 + 80 未来（t0 = 第 16 帧）；goal = GT 轨迹终点（ADR-0005 Scene 契约）
  │     `scenario_id` 编码 `bag_id#frame`（128B 字段现成，`map_data.h:700+`）
  │     provenance：地图版本 + 感知版本 + 车辆 ID
  │
  ├─[3] 约束束生成
  │     模板匹配 + 参数化（§4）。违规型自动；体感型人工转写
  │
  ├─[4] 场景族
  │     同场景扰动 N 份：对手速度/时序偏移、ego 起始偏移、信号相位、occ 增删
  │     —— 单条 case 直接 RL 会过拟合；RL 的工作单元是**族**
  │
  ├─[5] 可行性裁判
  │     同场景用 IDM / REPLAY / STATIC 也跑一遍
  │     全挂 → 场景不可解或约束写错 → 弃用，不进 RL
  │
  ├─[6] RL fine-tune
  │     reward = 进度 + 舒适（平滑）；constraint = 硬（碰撞/越界/红灯/TTC/jerk 上界）
  │     rollout 级可行性 mask + 只在存活 advantage 上更新（kine_critic / adv_filter 同款模式）
  │
  └─[7] 回归集
        老 case 全绿 + 新 case 转绿 → 上线；任一老 case 转红 → 拒绝本次 checkpoint
```

**KPI 换成两个数，不再是分数：**

1. **约束覆盖率** = 数据湖中可自动检出且已转写成约束的违规帧占比
2. **回归保留率** = 每次 fine-tune 后老 case 仍全绿的占比

分数会涨，但分数在 A-only 下不可信；这两个数才说明 1→100 有没有发生。

---

## 4. 约束模板库 v0（20 条，可规模化的唯一形式）

全局"不许碰撞"修不了具体 case——它不告诉你在那个路口错在哪。每个 bad case 必须落到 `(模板, 参数, 场景切片)`。

| # | 模板 | 关键参数 | 谓词形态 |
|---|---|---|---|
| 1 | 人行横道让行 | 行人距离阈值、速度阈值 | 行人进入 ego 路径 τ 秒内不得推进 |
| 2 | 无保护左转 | 对向 TTC、等待窗口 | 对向车 TTC < X 时不得进入路口 |
| 3 | 汇入让行 | 主路车距、加速度上限 | 合流点到达前速度不得高于 Y |
| 4 | 加塞应对 | 最小跟车时距 | 时距不得低于 1.2s（可调） |
| 5 | 红灯/停止线 | 停止线距离、越线判定 | 红灯期间车头不得越过停止线 |
| 6 | 施工区 | occ 多边形、横向余量 | 与 occ 保持 ≥ Z 米且不越锥桶线 |
| 7 | 窄路会车 | 宽度、对向车位置 | 通行时横向余量不得低于 W |
| 8 | 掉头 | 最小半径、对向间隙 | 掉头期间不得越出可行驶域 |
| 9 | 环岛 | 入环让行、出口选择 | 环内车优先 |
| 10 | 公交站/临时停车 | 停靠区、绕行余量 | 绕行时不得压线 |
| 11 | 匝道分流 | 速度匹配、最晚变道点 | 分流点前 D 米内不得仍在错误车道 |
| 12 | 并线/变道 | 目标车道间隙、jerk 上限 | 不满足间隙不得开始变道 |
| 13 | 行人横穿（非横道） | 行人轨迹预测、TTC | 同 #1，阈值不同 |
| 14 | 非机动车混行 | 侧向余量 | 与两轮车横向距离 ≥ M |
| 15 | 故障车/事故车 | occ 位置、绕行空间 | 不得碰撞且必须绕行 |
| 16 | 学校/限速区 | 限速、加速度上限 | 速度 ≤ 限速、jerk 受控 |
| 17 | 铁道口/减速带 | 减速要求 | 通过速度 ≤ V |
| 18 | 隧道/明暗过渡 | 速度、跟车时距 | 时距不得低于基准 |
| 19 | 上下坡/立交 | 纵向加速度包络 | jerk 越限即违规 |
| 20 | 目标不可达 | 进度下限、超时 | T 秒内进度增长 < P 即违规 |

> 体感型（20%）落在这张表里的 #1/#3/#4/#7/#12/#20 居多——它们本来就有可量化的近似谓词，只是以前没人写下来。

---

## 5. RL 的形态与三条防线

```
reward     = 进度 + 舒适（平滑、可塑形）
constraint = 碰撞 / 越界 / 红灯 / TTC / jerk 上界（硬）
policy     = categorical over 90 模态（有 log π）→ PPO / GAE
diffusion  = 给定模态后的确定性精修，**不进梯度**
mask       = rollout 级可行性判定，只在存活 advantage 上更新
```

**舒适必须进动作空间硬约束**（Comfort by Construction），不能只做软惩罚。A-only 下 RL 没有专家轨迹约束它，它一定会用"急刹 + 猛打方向"换安全分；那时你收到的下一批 bad case 会变成"乘客投诉刹车太猛"，而它同时满足所有安全约束。

**Goodhart 三防线（A-only 下没有第四道）：**

1. 每次迭代报**全 18 指标**，不是被优化的那个；
2. 留出一族约束（模板层面）完全不参与训练，只做泛化检验；
3. 人工抽检 rollout —— A-only 没有专家轨迹当参照，这是唯一真实的防线。

---

## 6. 三个硬技术发现（本轮新增，前两项需实测）

### 6.1 积分器不一致：同一组 `(acc, κ)` 会走出两条轨迹

| | Nova `_integrate_control` | PufferDrive jerk 动力学 |
|---|---|---|
| 速度 | `v_k = v_0 + Σ acc·dt`（左 Riemann） | `v_new = v + 0.5(a_new + a_old)·dt`（梯形） |
| 曲率 | 直接用 `κ` | `κ = a_lat / v²` |
| 航向 | `yaw_k = yaw_0 + Σ(κ·v)·dt` | `θ = (v·dt)·κ` |
| 位置 | `x = x_0 + Σ(v·cos yaw)·dt`（折线 Euler） | `d = ½(v_new+v_old)·dt`，`dx = sinθ/κ`, `dy = (1−cosθ)/κ`（**精确圆弧**） |

后果不是"有点不准"，而是：**模型以为自己在 A 点，sim 里它在 B 点，critic 基于 B 打标 → 闭环信号是假的，且不报错。**

处理：给 PufferDrive 加 `integration_scheme = model_euler`（几十行 C），保证"模型以为的位置 == sim 里的位置"。量级待实测（粗估 v=10 m/s、κ=0.05、80 步，横向差可达数十厘米量级）。

### 6.1.1 动作语义：jerk 接口不能直接吃 `(acc, κ)`

`move_dynamics` 的 jerk 分支（`drive.h:4300-4428`）有四处会在"直接对接"时静默改写模型的控制量：

| 机制 | 代码事实 | 后果 |
|---|---|---|
| **动作是 jerk，不是加速度** | `j_long/j_lat` 由 action 反缩放；`a_new = a_old + c_throttle·j·dt` | 要执行 `(acc, κ)` 得先反解 `j = (a_target − a_now)/dt`；越过 `JERK_LONG=-15 / JERK_LAT=±4` 时饱和 |
| **per-agent 条件化系数** | `c_throttle / c_steer` 取自 `reward_coefs`（奖励条件化随机化） | 同一 action 在不同 agent 上产生不同加速度；**训练时必须置 1**，否则执行不可复现 |
| **零穿越吸附 + 限幅** | `if (a_old·a_new < 0) a_new = 0`；`clip(ACCEL_LONG_LIMIT)` | 模型细小的换向指令被抹平；与 Nova 的积分器不一致 |
| **低速曲率口径** | `v_eff = max(|v|, 1.0)`，`κ_eff = a_lat / v_eff²` | v < 1 m/s 时 κ 与模型定义（`κ = a_lat/v²`）不同源 |

**结论：新增 `ACTION_TYPE_CONTROL`（直接给 `a_long, a_lat` 或 `acc, κ`），并在该模式下跳过条件化系数与零穿越吸附。** 这是几十行 C，且比"反解 jerk"稳得多；否则你会花几周追一个"模型明明没让刹车、车却刹了"的幽灵。

**已实现（2026-11-04）**：

- `ACTION_TYPE_CONTROL = 2`（`constants.h`），action = `(a_long [m/s²], kappa [1/m])`，完全跳过 jerk 反解、`c_throttle/c_steer/c_acc` 与零穿越吸附。
- `integration_scheme ∈ {arc, model_euler}`：`model_euler` 复刻 Nova 的半隐式 Euler（`v += a·dt; yaw += κ·v·dt; xy += v·(cos yaw, sin yaw)·dt`）。
- `skip_actuator_limits`（仅 control 模式）：置 1 时不做加速度/转角/转角速率/速度限幅，专供对拍；默认 0（施加限幅）。
- 测试：`tests/drive/test_drive_dynamics.c` 新增 4 例（直控执行、忽略奖励系数、限幅生效、三步 Euler 参考值），`make -C tests/drive unit` 全绿。
- **数值对拍已过**（`tests/unit_tests/test_control_mode_parity.py`，7 passed）：60 步 / 6 s、随机 `(acc, κ)` 三组，max `|Δxy| = 1.7e-5 ~ 3.4e-5 m`，`|Δheading| < 8e-7 rad`，`|Δv| < 6e-7 m/s`（float32 噪声级，远低于 1e-3 门限）。
- **一个必须知道的发现**：Nova 的 inference 积分**不是裸 Euler**。`_integrate_control` 对前 4 个加速度做 wash-in（权重 0.2/0.4/0.6/0.8，目标 = `2·mean(acc[:5] − a0) + a0`）；直接把原始 acc 喂给仿真，6 s 内偏离 **0.39–0.92 m**——比积分器噪声大 4 个数量级，与横向安全裕度同量级。
  ⇒ **acc 的 wash-in 是桥接层的必需预处理**，不属于仿真器。`test_acceleration_wash_in_reproduces_model_intent`（喂洗过的 acc → 精确复现模型意图）与 `test_raw_control_diverges_from_model_intent`（不洗就必然偏离 > 0.1 m）把这个要求钉死。

### 6.2 执行误差模型要标定，不能假设

| 执行方式 | 误差来源 | 与实车的关系 |
|---|---|---|
| 回放控制量 `(acc, κ)` 进 sim 动力学 | 动力学/积分器差异 | 接近"模型以为的世界" |
| tracker 跟踪模型轨迹 | 跟踪器差异 | 接近"实车 Apollo 控制" |
| 强制状态（perfect tracking） | 无（但也丢掉了可行性检验） | 只适合做上限对照 |

正确做法：**用 bag 里真实的 plan→pose 误差分布来标定**，选能复现该分布的模式。一次标定对所有模型生效。

顺带：`constants.h:125-133` 的限幅已按 3799 车型标定完毕（见 `docs/vehicle_profile.md` §3–§4）：`STEERING_LIMIT` 0.60564 rad、转角速率 0.42336 rad/s、`ACCEL_LONG_LIMIT {-6.0, 3.0}`、`wheelbase 2.20`、最高车速 12.5 m/s。**唯一没有车辆参数的是横向加速度**：车辆侧由转角+轮胎决定，而 PufferDrive 用 `ACCEL_LAT_LIMIT = ±4` 硬夹（`drive.h:4345`）——在 12.5 m/s 下把最小可行半径限到 39 m，是一阶保真损失；control 模式应撤掉，改用 bag 实测的轮胎代理上限。

### 6.3 PPO 的似然来源

`diffusion_head.py:~420-462` 的 `motion_prob` 是 `(3 refline × 30 long_mode)` 的 categorical，`topk_index` 在其上取 top-k；`config/develop/dev_v1.1.5.yaml:605-608` 的 `modality_sample_policy` / `sample_temperature: 1.0` 就是在采它。所以：

- **可 PPO 的动作 = 选模态**（Discrete(90)，有 `log π`），与 PufferDrive 的 Discrete 分支天然兼容；
- **不可 PPO 的动作 = diffusion 连续控制**（无似然）。它只能作为给定模态后的确定性精修。

这条必须写进任何方案文档，否则一定有人试图对 diffusion 头做 policy gradient。

---

## 7. 与既有裁决的关系

| 既有裁决 | 本文态度 |
|---|---|
| ADR-0001 Core + 语义外挂 | **沿用**，但把"外挂"的对象从"仅 Nova"扩展为**模型无关 canonical spec** |
| ADR-0002 在 Nova 部署契约内训练 | **降级**：基线模型在契约内；canonical spec 优先（J4） |
| ADR-0005 M1 先行 / Scene 契约 | **沿用** Scene 契约（16+80、goal=GT 终点），M1/M2 的选择交由 J6（两个域都要） |
| ADR-0006 Nova 19 输入 / 4 输出 | 作为**基线实例**沿用，不再是唯一契约（J4） |
| ADR-0003 混合对手必需 | **强化**：A-only 下对手分布是唯一的负样本来源 |

新增：`docs/adr/0007-badcase-as-scenario-seed-and-constraint-only-feedback.md`（J1/J2/J4/J5/J6 + A-only 契约 + 80/20）。

---

## 8. 待裁决 / 待实测

**待实测（有明确验收口径）：**

1. 积分器差异量级 —— 同 `(acc,κ)` 序列双积分逐点比，目标 <1e-3 m（§6.1）；先落 `ACTION_TYPE_CONTROL` 再测，否则测的是 jerk 反解误差不是积分器差异（§6.1.1）
2. 实车 plan→pose 跟踪误差分布、以及 `a_lat = v²·κ` 的 p99.9/max —— 从 bag 统计，前者用于 §6.2 标定，后者用于替换 `ACCEL_LAT_LIMIT`（`docs/vehicle_profile.md` §5）
3. 接 Nova 规模推理后的 decisions/s —— 决定"海量"的量级（PufferDrive 320K SPS 是 tiny MLP 口径，不可直接外推）

**待裁决：**

1. canonical spec 的字段清单与版本策略（L0 的唯一资产，必须先冻结）
2. 走廊域与漫游域各自的"合法性判据"由谁裁定（对拍日志 vs 内禀约束）
3. 约束模板库的 owner 与入库流程（体感型那 20% 的转写人）
4. 是否引入程序化 hazard 生成层 —— 见 `docs/2026-11-03-Terra系列全景与PufferDrive落地路线.md` §4A/§6，本文不重复

---

## 9. 口径声明

- TerraZero 数字（94.19 / 70.87 / 560K–2.8M / 零演示）来自其一手指南页，2026-11-04 抓取；其评测协议为 nuPlan val14 与 InterPlan，**不等于部署分布**，迁移到 rino 数据上的结论未经证实。
- PufferDrive 320K SPS 来自本仓文档，口径为 tiny MLP + C 内核；接大模型后不成立。
- 21,174 个 Apollo map tile / 148 GB / 31 站点为 2026-11-04 实测（`/media/.../apollo_map/data/`）。
- 80/20 的违规/体感比例来自 owner 自陈，**尚未用数据湖统计验证**；本文按该比例分配人力，但 §3 的自动挖掘不依赖该比例成立。
- 本文不改任何代码；所有行号仅作定位锚点。
