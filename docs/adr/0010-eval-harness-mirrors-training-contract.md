# 评测 harness 是训练契约的镜像：动作选择与 env 参数不许手搓

Status: accepted 2026-11-06（owner 批准"按推荐"；证据：`tests/unit_tests/test_policy_eval_action_contract.py` 红→绿三测 + `scripts/regression_retention.py` 修复前后对照）

## Context

手搓评测路径喂同一 roundC 权重：训练里车动（训练日志 m/s 量级），harness 里中位车速 **0.00 m/s**、60 步只有 2 种动作。逐位对拍证明 **obs 完全一致**——锅不在观测打包，而在两处手搓偏离训练契约：

1. **动作选择**：harness 用 `argmax`；训练 eval 用 `action_selection: mean`（物理动作表概率加权期望 `discrete_probs_to_continuous_mean`）。驻车起步时策略分布众数压在 no-op 格（实测 ~63%），argmax 自锁：停着→选 no-op→还停着。
2. **dt**：harness 强制 `dt=0.1`（判据侧 `DT_S`），训练 env `dt=0.3`。选对动作也把速度低估 ~8×（中位 2.44→0.38 m/s）。

## Decision

1. **策略评测的动作选择 = 训练 eval 契约**：`select_eval_action`（离散策略→`discrete_probs_to_continuous_mean`；连续策略→`Normal.loc`），**禁止 argmax**。单一出口，`evaluate_scene` 与 `zeroshot_eval.run_policy_trace` 共用。
2. **env 构造继承训练快照**（`experiments/config.yaml` env 段），只许覆盖 map_dir/种子/控制者/agent 数；**dt 不得覆盖**。
3. **口径不对称显式保留**：replay 参考零点维持 dt=0.1 + 位姿反推速度（保冻结判据数字 154/154）；策略侧用训练口径（60 步 × 0.3 s = 18 s）。不对称写进代码注释。
4. **验收 gate**：`median_speed_min`（车动门）+ 与训练代码路径（`load_env`+`vecenv.recv`）同量级对照；训练日志的聚合计（如 6.4 m/s）不作为 gate 基准（无法在训练路径复现）。

## Consequences

- roundC 诚实数字：违规 **150/154**、中位车速 1.37 m/s——旧 97.4% "修复率"全是趴窝假象，重训（事件级约束奖励）成为必经路。
- `num_reward_coefs` 属观测契约的既有约束不变：评测侧任何"看似无害"的重排/重选都要过训练契约对拍。
