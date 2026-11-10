# 评测 harness 的陷阱清单：为什么「同一个 checkpoint」能测出 4 种行为

> 一句话：本仓的策略评测是**手搓**的（脚本自己拼 env、自己把 logits 变成动作、自己挑配置快照），
> 而训练是一条黑盒流水线（vecenv 打包观测 → sample_logits → 引擎解码）。手搓路径里**每一个
> 环节都是一个隐式契约**，"跑得通"不等于"等价"。本文件把已经踩过的 6 个坑、它们的检测方式、
> 以及"哪些数字因此作废"集中记在一处。
>
> 相关裁决：`docs/adr/0010-eval-harness-mirrors-training-contract.md`（动作选择 + dt）、
> `docs/adr/0011-training-dt-and-reward-scale-contract.md`（dt 与奖励尺度）、
> 事故现场记录：`docs/2026-11-05-canonical-spec-v0.md` §15。

---

## 0. 为什么这类 bug 特别贵

同一个 checkpoint，在三份不同的评测口径下读出的数字：

| 口径 | 中位车速 | 越道违规 | 撞 `lane_dist` 上限(4.0) |
|---|---|---|---|
| harness（argmax + dt 0.1）—— 作废 | **0.00 m/s** | 4/154（假"修复率 97.4%"） | 0 |
| harness 修好（mean + 训练 dt 0.3） | 2.00 m/s | **150/154** | 19 |
| 再换一份训练快照（v2a 的 coefs） | **4.62 m/s** | 154/154 | **131** |

三行说的都是 `checkpoints/lane_departure_roundC.pt` 这**同一份权重**。
这不是"精度差异"，是**结论方向相反**：第一行说"修好了 97.4%"，第二行说"基本没修"。
所以本仓的规矩是：**任何策略侧数字必须连同产生它的 harness 口径一起记录**，
口径不明的数字一律视为未测。

---

## 1. 坑 A：动作选择用 argmax（→ 0.00 m/s 自锁）

### 现场

`scripts/regression_retention.py` 早期用 `logits.argmax(-1)` 选动作。同一权重在训练里
`avg_speed_per_agent ≈ 6 m/s`，在 harness 里 154 个场景 **150 个中位速度 0.00 m/s**。

### 机制

策略是 **12 模态离散 categorical**（`JERK_LONG[4] × JERK_LAT[3]`），训练 rollout 是
**采样**，训练 eval 是 **`action_selection: mean`**（`discrete_probs_to_continuous_mean`：
物理动作表上的概率加权期望）。车在驻车起步时，策略分布的众数压在 **no-op 格（实测 ~63%）**：

```
argmax → no-op → 车不动 → 下一步观测还是"停着" → 分布还是压 no-op → 自锁
```

而 mean 输出的是期望值（有 24% 的质量在满加速格上）→ 车动起来。

更毒的一点：**带速度门限的谓词（`sim_speed > 0.5 m/s`）对静止车永远不成立**，
所以"趴窝"会被判据读成"没有越道" → 违规数从 150 塌到 4 → 凭空生成 97.4% 的修复率。

### 检测/防线

- 单一出口 `select_eval_action`（离散→mean 物理期望；连续→`Normal.loc`），
  `evaluate_scene` 与 `zeroshot_eval.run_policy_trace` 共用。
- `tests/unit_tests/test_policy_eval_action_contract.py::test_select_eval_action_is_prob_weighted_mean_not_argmax`
- **车动门**：评测必须报 `median_speed`，低于 1 m/s 的数字不能进任何结论
  （`baseline_check.py` / `regression_retention.py --check` 的硬判据）。

---

## 2. 坑 B：评测覆盖了训练用的 dt（→ 速度低估 8 倍）

### 现场

`build_env()` 早期强制 `dt=0.1`（判据侧 `DT_S`），而训练快照 `dt=0.3`。
同一个"选对动作"的权重，中位速度读成 **0.38 m/s**，正确口径下是 **2.44 m/s**。

### 机制

dt 同时是**积分步长**和**决策步长**。用 1/3 的 dt 去跑按 0.3 s 训练的权重：
速度、转角、每步奖励全部按 ~3 倍缩水（速度读出低估约 8 倍，因为航向/加速度的耦合）。
更糟的是它把 `episode_length` 的语义也换了（60 步 = 6 s 而不是 18 s）。

### 检测/防线

- `build_env` **不许覆盖 dt**，一律继承训练快照（只允许覆盖 map_dir/种子/控制者/agent 数）。
- `test_policy_env_matches_training_dt`（断言 `env.dt == snapshot.dt`）。
- 跨 dt 比较训练指标必须换算成**秒**：dt 0.3 → episode 16/60 步 = 4.8 s，
  dt 0.1 → 51/60 步 = 5.1 s，看起来"活了 3 倍"其实一样（ADR-0011）。

---

## 3. 坑 C：评测读到了**别轮**的配置快照（→ 行为 2.3 倍差异）

### 现场（2026-11-06 会话③实测）

`regression_retention.py` 的 `training_env_config()` 硬编码读 `experiments/config.yaml`，
所以拿它去评测另一轮实验的 checkpoint 时，喂进去的是**旧那一轮**的 env 段。
奖励系数是观测上下文的一部分，于是：

| 权重 | 在哪个快照下评测 | 中位车速 | 撞 `lane_dist` 上限 |
|---|---|---|---|
| roundC | 自己的（`randomization=true`, dt 0.3） | 2.00 m/s | 19/154 |
| roundC | v2a 的（`randomization=false`, `lane_center 0.04`） | **4.62 m/s** | **131/154** |
| v2a | v2a 自己的 | 4.54 m/s | 154/154 |

同一份权重换一份快照，行为差异 2.3 倍、越道上限命中数 7 倍。

### 机制

`context_dim = num_reward_coefs + goal_dim` —— **奖励系数是观测上下文的一部分**
（`write_reward_target_obs`）。所以"换个快照评测"不是改评分标准，而是**改策略的输入**：
网络会按另一组系数条件化，输出完全不同的动作分布。
同理，往 `reward_coefs` 数组**加一项**等于改观测维度，所有旧权重直接加载失败。

### 检测/防线

- `--run-config <data_dir>/config.yaml`（每轮实验一个 `train.data_dir`，快照随 checkpoint 走）；
  默认产物也写到该目录，避免覆盖上一轮的 `docs/measured/*`。
- CLI 会打印实际生效的快照路径（`[contract] env 快照 = …`），日志里可核对。
- 训练启动会**覆写** `data_dir/config.yaml`（基础配置 + 本轮 override）→ 多轮实验共用默认
  `experiments/` 会静默覆盖上一轮的契约（roundC 的快照就这么丢的，靠会话记录重建，
  重建件 `.scratch/config_roundC_reconstructed.yaml`）。

---

## 4. 坑 D：观测打包不一致（**这一条被证伪了，但过程本身是教训**）

### 现场

第一次面对"训练 6 m/s / harness 0.00 m/s"时，第一假设是**观测打包被手搓坏了**
（`compress_observations` / dtype / mask / zero-copy），于是逐位对拍 obs —— **完全一致**。

### 教训

- 假设要能证伪，对拍要做的第一步就是它；**但"obs 一致"不等于"推理路径等价"**。
  真正的偏离在两个没人当回事的地方（坑 A、坑 B）：动作选择和 dt。
- 事故记录写成"harness 观测不一致"（`docs/2026-11-05-canonical-spec-v0.md` §15），
  后来被 ADR-0010 更正。**保留这段错误结论是有意的**：它提醒"最先想到的那条缝"
  往往不是漏的那条。

---

## 5. 坑 E：参考零点与策略侧口径不对称（判据侧的合法偏差，但是个混淆项）

- replay 参考（录制司机）用 **dt 0.1、60 步 = 6 s**（保冻结判据数字 154/154）；
- 策略侧（在 dt 0.3 训练时）用 **dt 0.3、60 步 = 18 s**。

结果是**策略被评估的时间窗是参考的 3 倍**。在场景 bin 里，地图的车道覆盖是围绕切片
建的：车开出去十几二十米就可能离开映射车道集，`lane_dist` 撞 4.0 归一化上限 →
被判成"越道"。于是"开得快"本身就更容易违规，而 roundC 的"低违规 + 1.37 m/s"
很大一部分是**开得慢所以没开出地图**换来的（ADR-0011 把训练/评测统一到 dt 0.1 之后
这条对角线才被拉平）。

**规矩**：参考侧与策略侧的窗口必须一致；不一致必须在代码注释和数字旁边写清楚（现在是）。

---

## 6. 坑 F：`--limit` 在过滤之前生效（小，但会让人以为"场景没了"）

`scenes[:limit]` 先截断、再按 `scene_dir` 过滤 → 用 `--limit 2` 指一个只含另一批 bag 的
目录时，结果是"没有可评测场景"。已改成先过滤后截断。
同类的还有 `.get()` 默认值掩盖键名错误：诊断脚本里 `agent.get("steer", 0.0)`
读一个**不存在的**键 → 15 步全打印 `steer +0.000`，看起来像"策略从不打方向"。
真实键名是 `sim_steering`。**读 state 字典不要给 `.get` 兜底默认值**，宁可 KeyError。

---

## 7. 现状：这 6 条现在由什么挡住

| 坑 | 现在的挡板 |
|---|---|
| A argmax | `select_eval_action` 单一出口 + 3 个单测 + 车动门 |
| B dt | `build_env` 不覆盖 dt + `test_policy_env_matches_training_dt` + 换算成秒 |
| C 快照 | `--run-config` + 每轮独立 `train.data_dir` + 启动打印快照 |
| D 观测 | 逐位对拍（发现问题的正确第一步，只是当时结论方向错了） |
| E 窗口 | 训练/评测统一 dt 0.1（ADR-0011）；不对称必须注释 + 记录 |
| F 小坑 | 先过滤后截断；state 字典不给默认值 |

评测前自查三行：

```bash
python scripts/reward_scale_report.py --config <data_dir>/config.yaml --check   # 奖励契约
python -m pytest tests/unit_tests/test_policy_eval_action_contract.py -q        # 动作/dt 契约
python scripts/regression_retention.py --scene-dir .scratch/crossbag_scenes \
  --run-config <data_dir>/config.yaml --checkpoint <data_dir>/final_model.pt --check
```

`--check` 的两条硬判据永远是：**车在动**（中位车速门）与**参考零点没漂**（154/154）。

---

## 8. 附：受影响的数字台账

**作废**（建立在 argmax 趴窝 harness 上）：M7-4 修复率 97.4% / 回归保留率 100%、
`docs/html/regression_retention_v0.html` 的旧行、§14.5 的 328 场景对比、§14.6 的
safety-gap 惩罚结论、冻结基线 `lane_departure_roundC` 的"4/154"。

**仍然有效**（不依赖策略推理）：录制司机 154/154、IDM 0/168、M7-1/2/3
（route 推导、场景构建 replay 对拍 0.000 m、补对手 174/257）、训练管线自身日志里的指标。
