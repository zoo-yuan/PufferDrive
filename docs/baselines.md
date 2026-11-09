# 冻结基线清单（Baseline manifest）

> ## ⛔ 状态：**已撤销（WITHDRAWN，2026-11-06）**
>
> 冻结后复核发现：`checkpoints/lane_departure_roundC.pt` 在当前代码下**中位速度 0.00 m/s**
> （154 个场景里 150 个不动）。带速度门限的 `lane_departure` 对静止车永远不成立，
> 所以"4/154 违规、修复率 97.4%"是**车没动**造成的假象，不是策略学会了。
>
> `python scripts/baseline_check.py --check` 现在会因"基线中位车速 0.00 m/s < 下限 1.0"而失败，
> 这是故意的 —— 防止任何人再引用这组数字。
>
> **根因**：评测 harness 手搓的推理路径与训练时 vecenv 的观测打包不一致（同一权重在训练里
> 6.4 m/s，在手搓 harness 里 0.00 m/s）。证据与修法见
> `docs/2026-11-05-canonical-spec-v0.md` §15。**M7-4 的全部策略侧数字在修复前不可引用。**
>
> 仍然有效的是**判据侧**的结论：录制司机 154/154 违规、IDM 0/154 违规 —— 场景确实含违规，
> 且几何上可合规通过（`docs/html/zeroshot_eval.html`）。

> 冻结 = 有一条命令能证明这些数字还能复现。**任何人不许再把"97.4%"当传说引用**，
> 要么跑 `python scripts/baseline_check.py --check`，要么不用这个数。

## 1. 冻结了什么

| 项 | 值 |
|---|---|
| 名称 | `lane_departure_roundC` |
| checkpoint | `checkpoints/lane_departure_roundC.pt`（30 MB，**不进 git**） |
| sha256 | `7339d8161c8efcd0572dbf6c0fc4ed30d1c1078b00b1190453f0ea674f46a6e5` |
| 清单文件 | `docs/measured/baseline_lane_departure_v0.json` |
| 场景集 | `.scratch/crossbag_scenes` 的 **154** 个 lane_departure 场景（3183:71 / 3123:50 / 3139:33） |
| 判据 | `lane_departure`：`sim_speed > 0.5 m/s` 且 `|lane_dist| > 0.9 m` |
| 训练预算 | 300k + 300k + 300k **agent-steps**（三轮续训） |

## 2. 数字

| 评测对象 | 违规场景 | 说明 |
|---|---|---|
| replay 参考（录制司机） | **154 / 154** | 场景确实含违规 |
| 训练前（随机初始化） | 141 | — |
| **基线 checkpoint** | **4 / 154** | 剩 4 条全在 3139 同一路口簇 |
| 修复率 | **97.4%** | 相对录制司机 |
| 回归保留率 | **100%** | 134 个正常场景未被改坏 |

## 3. 怎么验

```bash
python scripts/baseline_check.py --check
```

它会：核对 checkpoint 的 sha256 → 核对场景数 → 重跑参考零点与基线（约 45 秒）→
比对违规数与修复率，任何一项漂移都以非 0 退出。

期望输出：

```
reference 违规 : 154/154（清单 154）
基线违规       : 4/154（清单上限 6）
修复率         : 97.4%（清单 97%）
GATE      : PASS
```

## 4. 怎么从头重建

```bash
source scripts/puffer_env.sh

# 1) 三轮训练（约 15 分钟，GPU）
python -m pufferlib.pufferl train puffer_drive \
  env.map_dir=.scratch/crossbag_scenes env.num_maps=154 \
  env.simulation_mode=replay env.control_mode=control_sdc_only env.sdc_controller=policy \
  env.replay_expert_agents=0 env.goal_source=route env.num_agents=1 env.scenario_length=60 \
  vec.num_envs=16 vec.num_workers=8 train.bptt_horizon=32 train.minibatch_size=512 \
  train.total_timesteps=300000 train.device=cuda train.compile=false \
  train.data_dir=experiments/roundA
# 续训两轮：load_model_path=experiments/round{A,B}/final_model.pt，total_timesteps 依次 600000 / 900000 到 roundB / roundC

# 2) 评测
python scripts/regression_retention.py --scene-dir .scratch/crossbag_scenes \
  --baseline-checkpoint experiments/roundB/final_model.pt \
  --checkpoint          experiments/roundC/final_model.pt

# 3) 冻结
cp experiments/roundC/final_model.pt checkpoints/lane_departure_roundC.pt
sha256sum checkpoints/lane_departure_roundC.pt     # 写回 docs/measured/baseline_lane_departure_v0.json
```

## 5. 什么会让这个冻结失效

改下面任何一项，都必须重新训练并重新冻清单 —— 因为它们**改的是判据或观测契约**，不是超参：

| 变更 | 后果 | 本轮实例 |
|---|---|---|
| 观测向量维度（含 `num_reward_coefs`） | 旧 checkpoint 直接加载失败 | 我把 safety-gap 系数塞进 `reward_coefs`，context 27→28，全部旧权重报废；已改为不进数组 |
| 场景包络（`NEIGHBOR_LANE_RADIUS_M`） | 同一批场景的违规数会变 | 20 m → 40 m 让策略侧违规从 71 降到 1 |
| 约束谓词 / 阈值 / guard | 判据变了 | `lane_departure` 加 0.5 m/s guard；`cut_in_conflict` guard 1.0→0.9 m |
| 场景集（增删切片） | 分母变了 | 779 切片 → 82 场景的主键碰撞修复 |

## 6. 这个基线**不**代表什么

- **不是 SOTA**：900k agent-steps、3 条 bag、单一约束类型；
- **不是泛化证据**：所有场景都来自训练用的那 3 条 bag，没有 held-out bag；
- **不是交互能力**：补对手的场景仅用于评测，对手按 log 回放、不反应（见 `docs/partner-agents-explained.md`）；
- **不是舒适性结论**：comfort 类模板在 replay 下没有指标，未纳入。

上一个被**拒绝**的 checkpoint（328 场景 / 含 safety-gap 惩罚）记录在
`docs/2026-11-05-canonical-spec-v0.md` §14.5–§14.6，不进入基线。
