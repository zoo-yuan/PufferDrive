# Terra 系列全景 × PufferDrive 落地路线

> 日期：2026-11-03
> 一手来源：Terra 系列四个项目页全部抓取核对 —— TerraZero `https://terra-applied.github.io/TerraZero/`（技术报告 arXiv 2607.13028）、TerraTransfer `.../TerraTransfer/`、TerraTransfer++ `.../TerraTransferPlusPlus/`、SPACeR `.../SPACeR/`（ICLR 2026）。
> PufferDrive 侧：分支 `dev/zoo_v3.0`，代码 HEAD `80a681b4`（与 `docs/2026-10-31-PufferDrive-vs-TerraZero-仿真器对比.md` 同基准；所有行号已对照当前文件核实）。
> 与 10-31 文档的关系：那份做**仿真器单轴**对比（数据哲学 / 吞吐 / 成绩口径）；本文扩展为**全系列四件工作**的成分因果分析 + 逐工作落地路线。**仍不做选型裁决**——对 STATE.md 三个开放问题给建议，裁决权在 owner。

## 0. 一句话

Terra 系列 = 同一个向量化 self-play 仿真栈的四种用法（RL-only ego planner → human-like sim agent → 学会看 → 学会规划）；其 RL-only SOTA 不是「算法突破」而是**五个环境成分的联合承重**——PufferDrive 的 gigaflow 模式已经继承了其中四项（含「日志降级为地图几何」哲学本身），唯一的结构性缺口是**长尾 hazard 程序化生成层**。

## 1. 流水线全景：四件工作的关系

```
TerraZero (系列起点, arXiv 2607.13028)   SPACeR (ICLR 2026)
  程序化仿真 + 零演示 self-play            human-like 锚定
  val14 94.19 (#1) / InterPlan 70.87      冻结 tokenized 参考模型 π_ref
  WOSAC 2024 realism 0.740                r += α·log π_ref(a|s) + PPO 损失加 β·KL
  2.8M SPS (8× 节点)                      65K 参数，比 SMART 小 50× / 快 10×
        │                                        │
        ▼ 共用同一仿真栈                          ▼ 解决「纯 RL 不像人」的另一条路
TerraTransfer (系列第二作)               TerraTransfer++ (2026-10)
  冻结 TerraZero 教师                     教师 rollouts → flow-matching 轨迹教师
  DINOv3+adapter 对齐隐空间               (8s 实现未来, 零日志轨迹)
  action-KL + 批关系低秩结构损失           再对齐 vision 学生 (三损失)
  HUGSim 0.490 HD-Score                   navhard 59.4 EPDMS (#1) / navtest 95.5 (#1)
                                          HUGSim 0.548 closed-loop
```

共同哲学：**把「学会开」和「学会看」彻底解耦**。驾驶能力全部来自向量化 self-play（8×10¹¹ 步 ≈ 23M 驾驶小时 ≈ 2535 年），视觉与规划只是事后对齐到冻结教师；任何阶段都不碰人类演示轨迹（paired (image, scene-state) 帧即可，无需 curated expert demonstrations）。

## 2. TerraZero 的成分因果链：为什么 RL-only 能到 SOTA（本文核心）

外部对比分析（DeepSeek）正确指出了关键成分清单：程序化场景生成器、异质 agent、零演示 recipe、3D 地图 + 真实交通规则、施工/横穿行人/抛锚车等逼真长尾环境。本节补上**承重机制**——每个成分缺了会发生什么。

### 2.1 五成分 × 承重机制

| # | 成分 | 承重机制（缺了会怎样） | 反证/证据 |
|---|---|---|---|
| 1 | **程序化场景生成器**（随机起终点 / 对手分布 / 信号配时 / 施工障碍） | RL 的学习信号来自碰撞、险情、恢复——这些恰恰是日志里最稀有的（"Real driving logs are overwhelmingly routine"）。程序化把长尾从**数据问题**变成**采样问题**：少量地图 = 无限场景 | TerraZero 拿 val14 #1 只用了少量真实地图（"Few Maps, Infinite Scenarios"） |
| 2 | **长尾 hazard 按需生成**（施工/静态障碍、横穿与 jaywalking 行人、parked 与 collided 车辆） | 成分 1 的特化深化：nudging、绕行、让行协商这类技能，正常分布的对手永远给不了训练信号 | InterPlan 70.87 vs SPDM(60) 63.66——长尾基准是这项能力的直接读数 |
| 3 | **异质道路使用者**（高保真卡车 + 行人 + 骑行者，多动力学模型） | 异质对手 = **对手分布多样性的来源**（卡车动力学差 → 轻型车必须学会与重车交互）；同时支撑 sim-agent 双重角色。注意精度点：TerraZero 页面脚注明示「**异质 agent 统一策略**（一个策略控车+卡车）是 *in preparation* 的新工作」——当前发布物是「多类型被仿真」而非「统一策略控异质」 | 联合控制版将另发 arXiv（页面 † 注） |
| 4 | **3D 地图 + 真实交通规则**（灯/停牌完整状态机） | 防止策略**钻仿真器空子**：TerraZero 原话 "policies learn genuine compliance rather than workarounds for scripted flags"。若规则是脚本旗标而非几何+配时的真实约束，RL 会学会利用旗标漏洞，成绩不可迁移；3D 保证立交/高差不产生虚假碰撞与虚假可行驶 | val14 分项 No AF-Coll 99.02 / TTC 95.62（安全项拉分） |
| 5 | **零演示 recipe + 规模**（8×10¹¹ 步，轻量策略多 GPU） | "trades sample efficiency for compute efficiency"：配方稳定性的前提是环境信号足够丰富且不可钻空子（= 成分 1–4），否则规模只会放大钻空子行为 | 每 episode 随机化 reward / 动力学 / 尺寸，策略无法过拟合单一目标 |

### 2.2 最硬的反证：缺成分的 RL-only 长什么样

SPACeR 论文自己提供了对照组——**只有任务奖励的纯 PPO**（同样的向量化仿真、同样的规模方法，缺 human-likeness 信号）：

| 方法 | WOSAC composite | minADE (m) | 行为学表现 |
|---|---|---|---|
| 纯 PPO（任务奖励） | 0.710 | **12.7** | stop-and-go 抖动、走错方向即停 |
| HR-PPO（BC 作 KL 正则） | 0.716 | 12.25 | 仍抖，交叉口让行突兀 |
| SPACeR（参考模型锚定） | 0.741 | 4.10 | 平滑、提前让行、交通合规 |
| SMART*（纯 IL，3M 参数） | 0.720 | 1.84 | 真实但慢 10×、闭环僵 |
| CAT-K*（IL SFT） | 0.766 | 1.47 | 分布分高但反应性差 |

### 2.3 由此浮出的路线对立：环境现实主义 vs 行为锚定主义

「RL-only 不像人」这个问题，Terra 系列内部有**两个对立解法**：

- **TerraZero = 环境现实主义**：把环境造得足够真（成分 1–4）+ 足够长尾，让 RL **凭环境信号自发收敛到人类式驾驶**（甚至零样本学出左侧通行）。human-likeness 不是目标函数，是涌现产物。
- **SPACeR = 行为锚定主义**：环境现实主义不彻底时（成本/保真有限），用冻结参考模型把策略**拉回人类分布**（似然奖励 + KL）。承认「RL-only 不像人」，用人类数据分布做锚。

**对本仓的含义**：PufferDrive 的 gigaflow 模式走的是 TerraZero 路线的地基（见 §3），但缺成分 2（长尾 hazard）；若补齐成本高，SPACeR 提供了**不改 C 内核的替代路径**（锚定而非造环境）。两条路不互斥——SPACeR 锚定 + hazard 生成可叠加。

## 3. PufferDrive 现状核对：五成分逐项对代码

核心发现：**「日志降级为地图几何」的哲学，PufferDrive gigaflow 模式已经在做**——比 10-31 文档的口径更接近 TerraZero。gigaflow 模式下 `.bin` 只贡献地图（lane 几何/交通控制/边界），agents 全部程序化重生成。

| TerraZero 成分 | PufferDrive 代码证据（已核实） | 判定 |
|---|---|---|
| 随机起终点/路线 | `spawn_agent()`（drive.h:2457）：lane 上随机撒点 + 碰撞/出路面检查（2393/2418）；route 随机游走（drive.h:1034）；`goal_source: map` 全图均匀采样 goal；`goal_regen_mode: finite/rolling` | ✅ |
| 随机 reward | `generate_reward_coefs()`（drive.h:2253）：18 项系数 per-agent 混合均匀采样（constants.h:254）；且 `reward_conditioning: true` 把系数喂进观测（drive.py:256，config:128）——策略知道自己在被什么奖励驱动，防过拟合单一目标 | ✅（GigaFlow 血统，比 TerraZero 页面公开的还细） |
| 随机信号配时 | drive.h:2342–2385：green 0.1–1× / yellow 0.5–0.75× / red 0.15–5× 默认时长采样 + **随机相位偏移**，GREEN→YELLOW→RED 状态机 | ✅ |
| 随机车辆尺寸 | spawn 时采样：车 2.0–5.5 × 1.5–2.5 m，非车 0.8–7.0 × 0.8–2.7 m（drive.h:2476–2481） | ✅ |
| 观测可观测性（第③轴） | `partner_blindness` / `phantom_braking`（概率触发 + 持续时长，drive.py:244–249） | ✅（TerraZero 页面未公开对应机制） |
| **3D 地图** | OBB 碰撞带 **z 轴早拒**（drive.h:1709–1716：先比 z 区间再 SAT）；partner 观测距离含 dz 的 3D 距离（drive.h:3840–3860）；lane 几何带 z | ✅（本次核实，修正 10-31 文档的模糊口径：立交/高差不产生虚假碰撞） |
| 真实交通规则 | 灯/停牌行为分支（`traffic_light_behavior/stop_sign_behavior: stop`）+ 随机配时状态机；**但 `stop_signs_enabled: false` 默认关** | ⚠️ 半开（配置项即得，见 §4-C） |
| 异质道路使用者 | `VEHICLE/PEDESTRIAN/CYCLIST` 三类（constants.h:28–30）+ 类型化尺寸采样；**无高保真卡车（挂车）**，行人/骑行者动力学简化 | ⚠️ 部分（类型骨架在，保真差距大；与 TerraZero 相同精度：统一策略控异质那边也还没发布） |
| **长尾 hazard 按需生成** | **无**：无 parked/collided vehicles、施工区、jaywalker、cut-in 制造者的程序化放置逻辑 | ❌ 唯一结构性缺口 |
| 零演示 recipe | PPO + V-trace（pufferl.py:600 `_compute_advantages` rho/c 剪裁）+ 优先采样 + advantage filtering；策略骨干与 TerraZero 同构（per-object MLP 编码 + masked max-pool ≈ DeepSets，torch.py:32 `_encode_and_pool`） | ✅ 配方在；规模与吞吐形态缺（320K SPS vs 1.3M，差距主要在推理侧 Python 往返） |

## 4. 缺口分析（按对 RL-only 成败的因果权重排序）

### A. 长尾 hazard 生成层（承重墙，缺了 InterPlan 类能力无从谈起）

TerraZero Figure 2 的清单即设计规格：construction & static obstacles / crossing & jaywalking pedestrians / parked & collided vehicles。落地设计要点：

- **静态型**：parked vehicles（沿 boundary 半占道）、collided vehicles（双车交叠姿态）、construction zone（polygon + 降速区）——`map_data.h` 已有静态 objects 数据结构，缺的只是**放置逻辑**；
- **行为型**：jaywalker（行人 agent + 横穿 goal 注入）、cut-in 制造者（激进 IDM 参数的 per-agent 变体）、phantom braker（突刹）——只需给 per-agent controller 加模式常量 + goal/参数注入，**不需要新子系统**；
- 复用 `check_spawn_collision/offroad` 同一套合法性检查；
- 按 AGENTS.md 信任边界：hazard 概率表是外部配置 → init 前校验；热路径（c_step/obs/collision）零 malloc；
- 估计：C 内核 ~600–1000 行 + config 项 + 单测。**这是唯一必须改 C 内核的缺口。**

### B. 推理吞吐形态（工程形态，不是能力成分）

只有奔 Terra 量级（8×10¹¹ 步）才必须改「C 引擎 CPU + GPU 零拷贝推理」；10⁹–10¹⁰ 步量级 320K SPS × 天级训练已够。**先 `profile.mode: sim` 量化推理占比再动**。改动集中在 `pufferl.py` 训练循环 + binding，不触仿真核心。

### C. 停牌默认关（一行配置 + 验证）

`stop_signs_enabled: false` → true 即得，需验证 WOMD 地图停牌覆盖率。TerraZero 明确列了 stop signs 为规则成分。

### D. 异质保真（建议降级）

对本仓目标（rino 数据湖 + Nova 契约，无 WOMD 卡车赛道需求）：高保真挂车动力学优先级低。建议只做尺寸/动力学参数分布扩展（把 spawn 采样表拉宽 + 类型条件化）。TerraZero 的「统一策略控异质」本身也还是 in preparation。

### E. 3D —— 已达标

本次核实结论：z 轴碰撞拒绝 + 3D 观测距离已在。比 10-31 文档口径更正面，无需动作。

## 5. 逐工作落地映射

| 工作 | PufferDrive 扮演的角色 | 改动面（文件级） | 工作量 | 优先级 |
|---|---|---|---|---|
| **TerraZero** | 复刻主体：gigaflow 模式即地基 | 缺口 A（drive.h hazard spawner + constants.h 模式常量 + config）+ 缺口 C（yaml 一行） | A: 1–2 周 C 开发；C: 即时 | **P0**（长尾能力 + E1b 压力测试共同前提） |
| **SPACeR** | RL 背景/sim-agent 的人味引擎 | `pufferl.py`：PPO 损失加 `β·KL(π_θ‖π_ref)`；reward 钩子加 `α·log π_ref(a|s)`（Python 侧批量算 logprob 可不动内核）；外部依赖：SMART 类参考模型 + **动作空间对齐**（PufferDrive 连续 jerk @0.3s vs SPACeR 200 token @5Hz，需重 tokenize——主要工程量在此） | 1–2 周，零 C 改动（logprob 走 Python） | **P1**（直接回应 E1b「RL 对手不像人」） |
| **TerraTransfer** | Phase 1 教师工厂（产冻结教师）；vision 对齐与本仓无关 | rollout 导出（`eval_replay.py`/viz 有基础，需加训练规模导出）；对齐管线在 nova 侧 | 导出: 数天；对齐: nova 仓 | P2（数据源 = rino paired frames，见下） |
| **TerraTransfer++** | 同上 + flow-matching 轨迹教师的 rollout 源 | 同上（每帧 8s 实现未来为目标；Terra 用 64×A100 一小时，数据量可控） | 同上 | P2 |

**TerraTransfer 与 ADR-0005 M1 的同构性（独立验证）**：TerraTransfer 的对齐数据 = paired (image, scene-state) 帧，无需演示轨迹——这正是 M1「复用 tfrecord 录制帧 re-anchor 到 live ego」的思路。Terra 系列的成功等于对 M1 路线的外部背书。更进一步：Nova 的 19 输入契约（含 `static_polygon_*` 四路）比 Terra 的 camera-only 更富，TerraTransfer 的「action-KL + 批关系低秩特征对齐」损失组合可原样移植为「Nova 学生 ↔ self-play 教师」——教师从 PufferDrive 状态直接编码，学生吃 19 输入。

## 6. 对 STATE.md 开放问题的建议（不代替裁决）

| 开放问题 | 本文建议 | 依据 |
|---|---|---|
| #1 程序化场景生成层 | **做，但范围收窄为 hazard spawner**（静态型 + 行为型），不重做场景/地图生成——gigaflow 地基已在（§3） | 成分 2 承重机制；改动量可控（§4-A） |
| #2 GPU 零拷贝推理 | **先 profile 再动**；10¹⁰ 步内不必须 | §4-B |
| #3 背景车 Σ | **采纳 SPACeR 第三路线**：reference-anchored self-play，替代 fraction / policy-pool 之争；顺带解决 E1b 对手质量问题 | §2.3、§5 |
| （未决）基线钉死 `dev/zoo_v3.0` | 支持：gigaflow + reward_conditioning + 观测噪声三件套已就位，差距集中在 hazard 层 | §3 |

## 7. 从 0 重写 / 其他方案的排除论证

- **从 0 重写：不成立。** 4700 行已验证 C 内核 + 2169 行 binding + Gym 封装 + V-trace PPO/优先采样/advantage filtering + WOSAC 工具链，全部在场且血统相同（GigaFlow 系）。重写无增量价值，只会引入新 bug 面。
- **其他开源替代：无可替代者。** MetaDrive/Highway-env 吞吐差约两个数量级（~10K SPS 级）；GigaFlow 官方库是本仓上游（能力子集）；CARLA/3D 仿真器渲染瓶颈与 Terra 哲学相反（TerraTransfer 正是为了绕开 photorealistic 渲染做 RL）。闭源者（TerraZero 本体、Applied Intuition 商业栈）无代码可借，只能借配方（10-31 文档结论不变）。
- **差异化竞争点**：TerraZero 没有真实数据湖——它用「无限程序化场景」弥补没有真实感知数据；本仓护城河 = 5000+ 车真实感知质量 + Apollo HD map。两条护城河在 **TerraTransfer 式 paired-frame 对齐**处汇合（§5）。

## 8. 口径声明

- val14 94.19 / InterPlan 70.87（全量 71.31）/ WOSAC 0.740 / HUGSim 0.548 / navhard 59.4 均为 Terra 系列官方页面自报，本仓未复现，不可与本仓 18 项指标直接对齐。
- TerraZero 的策略与观测张量契约未公开，不得假定与 Nova 19 输入同构（ADR-0006 边界不变）。
- WOSAC 同榜但仿真资产不同构（3D+卡车+行人 vs 以车为主），分数不代表同难度（承 10-31 文档 §3）。
- TerraZero 页面 WOSAC 2024 表列 SPACeR 0.741 / TerraZero 0.740；「matching SPACeR」为其自述。
- 吞吐口径：TerraZero 560K/1.3M/2.8M SPS 为其自报（"against object-level simulators that report comparable numbers"）；PufferDrive ~320K SPS 为本仓 2026-09 文档口径，两者测量条件未对齐，4× 差距是量级判断而非精确比较。
- 「异质统一策略」（车+卡车一个策略）为 TerraZero 页面标注的 *in preparation* 工作，本文按「未发布」处理。
