# 训练/评测的 dt 与奖励尺度是契约的一部分

Status: accepted 2026-11-06（会话③，owner 授权"奖励搜索迭代"；证据：IDM 对照实验 + 三轮训练 + 四道契约门）

## Context

P0 的原始计划是"一次性事件罚不够 → 加大罚金 / 减小速度奖励"。开训前做了一次量级审计，
三条结论把计划整个改道：

1. **配置里的奖励系数在训练中根本不读。** 训练快照 `reward_randomization: true` 时
   `generate_reward_coefs()` 用 `REWARD_BOUNDS` 抽样覆盖 `lane_center / lane_align /
   offroad / comfort …`，`reward_velocity` 更是硬编码 `2.5e-3`。原计划①
   （`reward_velocity 0.0025→0.0005`）会是个**空操作**。
2. **奖励尺度是断裂的。** 一集（60 步 · dt 0.3）可累积的量级：`velocity` ≤ 0.045、
   `lane_center(0.001)` = -0.027，而 `offroad` 终止事件 -1.5、一次性越道罚 -5.0
   —— 且后者在 154/154 个场景都触发，是**常数**。v1 的"velocity farming"归因在数值上
   不成立（0.045 付不起 5.0）；真实结构是密集项 1e-3、事件项 O(1) 且优势函数里没有信号。
3. **dt=0.3 让横向控制退化。** 横向动作是 3 档 jerk {-4,0,+4}，`a_lat += j_lat·dt`
   → 一步 ±1.2 m/s²、5 m/s 时约 13.7°/步，而握车道需要 ~0.01 rad/s。同一 IDM 控制器、
   同一场景、只改 dt：dt 0.3 → max\|lane_dist\| **4.0**（撞归一化上限，28 步后 offroad），
   dt 0.1 → **1.14–1.20**（52 步不越道）。另有两处不一致：replay 伙伴按 log 帧
   （0.1 s）推进，dt=0.3 时只走 1/3 速度；观测延迟 0.78 s 被量化成 0.9 s。

## Decision

1. **场景 bin 上的 replay 训练线用 `dt=0.1`**（= log 帧率 = 观测延迟量化基 = 参考零点窗口）。
   `dt=0.3` 上得到的奖励/策略结论一律视为被动作粒度污染。
2. **调参轮关 `reward_randomization`**；新 shaping 走独立 env 字段（不进 `reward_coefs`，
   因为它是观测上下文的一部分）。
3. **训练与评测必须同一份快照**：`scripts/regression_retention.py --run-config
   <data_dir>/config.yaml`。反例实测：同一份 roundC 权重，在自己的快照下评 2.00 m/s /
   19 个场景撞上限，在 v2a 快照下评 4.62 m/s / 131 个 —— 拿错快照能凭空造出 3 倍行为差异。
4. **四道奖励尺度门是开训前置条件**（`scripts/reward_scale_report.py --check`，
   `tests/unit_tests/test_reward_scale_contract.py` 钉住）：
   `live_coefs`（系数是否生效）· `lane_signal`（密集横向累积 ≥ 0.3×终止事件）·
   `event_dominance`（一次性罚 ≤ 4×密集累积）· `positive_channel`（正向累积 ≥ 0.25×越道累积）。
   第四道专门拦"趴着不动在构成本上最优"——`velocity_progress` 带 2.5 m/s 与车道关联门，
   所以抬它奖励的是"沿车道开起来"，站着和画圈都拿不到。

## Consequences

- 三轮实测（154 场景 `crossbag_scenes`，均未达冻结基线）：v2a（dt 0.3，修尺度）违规
  **154/154**、中位车速 4.54；v3（dt 0.1，只跑 300k）**141/154** 但中位车速 1.15（爬行，
  被 `positive_channel` 预测到）；v4（dt 0.1 + velocity 0.05）**153/154**，中位车速 2.34、
  中位 `lane_dist` 峰值 **1.445**（v2a 4.0 / v3 1.95 / roundC 1.007 在自己的快照下）。
- `episode_length` 从 dt 0.3 的 16/60 升到 dt 0.1 的 43–56/60，但**物理时长几乎没变**
  （4.8 s vs 4.4 s）——所以收益不是"活得更久"，而是横向控制精度。
- 未决：门限 0.9 m 与参考司机峰值中位 1.13 m 的余量只有 ~0.3 m，"avoidable"的可行性
  需要重新标定；离散 3 档横向 jerk 在 dt 0.1 下能否做到 <0.9 m 仍未证（候选：加细横向档
  / 连续动作头 / 更长训练）。
- `experiments/config.yaml`（`baseline_check.py` 依赖）仍是 dt 0.3 的旧快照；
  重冻基线时按新 contract 一并更新。
