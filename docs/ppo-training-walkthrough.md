# 当前实现怎么跑 PPO：从 `puffer train` 到一次梯度更新

> 面向"想知道代码里到底发生了什么"的读者。所有行号对应当前 HEAD 的
> `pufferlib/pufferl.py`、`pufferlib/ocean/torch.py`、`pufferlib/config/puffer_drive.yaml`。
>
> 一句话：**多进程 env 产数据 → 按 `bptt_horizon` 切成固定形状的段 → 算优势 → 按 |优势| 过滤 →
> 打乱做 minibatch → PPO clip 损失更新 → 记录/存档**。

---

## 1. 全景图

```
   ┌──────────────── 20 个 worker 进程（vec.backend=Multiprocessing）────────────────┐
   │ 每个 worker 里跑一份 C 仿真（pufferlib.ocean.drive.binding）+ 若干 agent          │
   └───────────────┬────────────────────────────────────────────────────────────────┘
                   │ recv(): obs, reward, done, trunc, info, env_id, mask
                   ▼
   ┌──────────────────────────── Trainer.evaluate()  pufferl.py:384 ────────────────┐
   │ 填满 self.segments 行 × horizon 列                                              │
   │   observations[seg, t, obs_dim]   ← 每个 agent 一列                             │
   │   actions / logprobs / values / rewards / terminals / masks                     │
   └───────────────┬────────────────────────────────────────────────────────────────┘
                   ▼
   _compute_advantages()  pufferl.py:646        →  V-trace/GAE 内核（C++/CUDA）
                   ▼
   优势过滤 adv_filter  pufferl.py:754+         →  只留 |A| ≥ 0.01×EMA(max|A|) 的 transition
                   ▼
   打乱 → minibatch → update_epochs 轮          pufferl.py:800+（非 RNN） / 666+（RNN 优先采样）
                   ▼
   _ppo_loss()  pufferl.py:564                  →  policy clip + value clip + entropy
                   ▼
   clip_grad_norm_(0.5) → optimizer.step()      pufferl.py:637
                   ▼
   mean_and_log / print_dashboard / save_checkpoint   pufferl.py:863 / 975 / 920
```

---

## 2. 数据形状：为什么是 "segments × horizon"

`Trainer.__init__`（`pufferl.py:176-196`）：

```python
if config["batch_size"] == "auto":
    config["batch_size"] = total_agents * config["bptt_horizon"]
segments = batch_size // horizon
self.observations = torch.zeros(segments, horizon, *obs_space.shape, ...)
```

代入本仓默认值（`bptt_horizon: 128`，`minibatch_size: 65536`）：

| 量 | 含义 | 例子（512 agents） |
|---|---|---|
| `total_agents` | 所有 worker 加起来同时被控的车 | 512 |
| `bptt_horizon` | 每个 agent 连续存多少步 | 128 |
| `batch_size` | 一次 update 用的 transition 总数 | 512 × 128 = 65 536 |
| `segments` | 有多少条"列"（row） | 65 536 / 128 = 512 |

**为什么不是"一个 episode 一行"**：PPO 不要求轨迹完整，它只需要固定形状的批。
把每个 agent 的连续 128 步当成一行，既能让 agent 之间被打散，又能保证时间维连续
（V-trace 要在时间维上倒着递推）。

一行满了就把该 agent 的**下一行**接着写。列被 `masks` 标记有效/无效：
episode 提前结束时该位置 `mask=False`，训练时被掩掉 —— 这就是日志里的 `masked_fraction`。

---

## 3. 一次更新里的 6 步

### ① 采样（`evaluate`，`pufferl.py:384`）

```python
while self.full_rows < self.segments:
    o, r, d, t, info, env_id, mask = self.vecenv.recv()
    logits, value = self.policy.forward_eval(o_device, state)
    action, logprob, _, _ = pufferlib.pytorch.sample_logits(logits, ...)
```

- 策略在 `torch.no_grad()` 下前向，动作从分布里采样（`sample_logits`，`pytorch.py:145`）；
- `r` 若开 `normalize_rewards` 会做 `sign(r)·log1p(|r|)` 压缩；
- 采样完 `vecenv.send(action)` 把动作发回 worker，形成流水线。

### ② 算优势（`_compute_advantages`，`pufferl.py:646`）

见 `docs/vtrace-explained.md`。默认配置（`rnn_name: null`）走的是 `ratio=1` 的路径，
等价于 GAE(γ=0.999, λ=0.95)。

### ③ 优势过滤（`adv_filter`，`pufferl.py:754+`）

```python
current_max = valid_abs_adv.max()
self.ema_max = beta * current_max + (1 - beta) * self.ema_max   # beta = 0.25
threshold = 0.01 * self.ema_max                                  # threshold_scale
keep_idx = valid_idx[valid_abs_adv >= threshold]
```

只训练"相对重要"的 transition。**这对 bad case 训练非常关键**：绝大多数帧是平稳跟车
（优势接近 0），真正教策略的是那 1% 的临界时刻。日志里 `kept_fraction` 就是这个比例。

### ④ 打乱 + minibatch（`pufferl.py:800+`）

```python
for _ in range(config["update_epochs"]):           # 默认 3 轮
    permutation = keep_idx[torch.randperm(keep_idx.numel())]
    for start in range(0, retained_transition_count, self.minibatch_size):
        ...  # 前向 + 反向，每 accumulate_minibatches 次 step 一次
```

**同一批数据被重复用 `update_epochs` 遍** —— 这正是需要 PPO 的 clip 与（可选）V-trace 折扣的原因：
用第二遍时策略已经变了。

### ⑤ PPO 损失（`_ppo_loss`，`pufferl.py:564`）

```python
ratio = (newlogprob - mb_logprobs).exp()
pg_loss1 = -mb_adv * ratio
pg_loss2 = -mb_adv * torch.clamp(ratio, 1 - clip_coef, 1 + clip_coef)
pg_loss  = torch.max(pg_loss1, pg_loss2).mean()          # clip_coef = 0.2

v_clipped = mb_values + clamp(newvalue - mb_values, ±vf_clip_coef)   # 默认 None，即不裁
v_loss    = 0.5 * max((newvalue-return)², (v_clipped-return)²).mean()

entropy_loss = entropy.mean()
loss = pg_loss - ent_coef * entropy_loss + vf_coef * v_loss          # 0.01 / 0.5
```

三个细节值得记：

- **优势在 minibatch 内部重新标准化**：`(adv - mean) / (std + 1e-8)`，让小批量之间的尺度一致；
- **`ratio` 用对数差再 `exp`**（`(newlogprob - oldlogprob).exp()`）而不是直接除，数值更稳；
- 诊断量 `approx_kl`、`clipfrac` 每轮都记 —— 训练崩之前通常先看到 `clipfrac` 冲到 0.3+ 或 KL 爆掉。

### ⑥ 梯度裁剪 + 更新（`pufferl.py:637`）

```python
grad_norm = torch.nn.utils.clip_grad_norm_(self.policy.parameters(), max_grad_norm)  # 0.5
self.optimizer.step()
```

---

## 4. RNN 路径不一样的两处（`use_rnn=True`）

虽然默认配置不走这里，但它是本仓"优先级采样"的实现地，值得知道：

```python
# pufferl.py:666+  _train_ppo_trajectory
anneal_beta = b0 + (1 - b0) * a * self.epoch / self.total_epochs
adv = advantages.abs().sum(axis=1)                    # 每段的总优势
prio_probs = (adv ** a) / (adv ** a).sum()            # 按 |A|^α 采样段
mb_prio = (self.segments * prio_probs[idx, None]) ** -anneal_beta   # 重要性校正
```

1. **按优势大小优先采样**（而不是均匀打乱），`beta` 随训练线性退火到 1 → 前期偏重难例、后期回归无偏；
2. **每轮重算 `ratio`** 并喂给 V-trace → 同一批轨迹多次复用也保持无偏。

---

## 5. 模型与观测（`pufferlib/ocean/torch.py`）

策略把所有实体按类型分槽，各自过一个 MLP 再 max-pool：

| 槽 | 内容 | 宽度参数 |
|---|---|---|
| ego | 自车状态 + 目标 | `ego_input_size: 128` |
| partner | 其他车辆 | `partner_input_size: 256` |
| lane | 车道中心线 | `lane_input_size` |
| boundary | 路沿/边界 | `boundary_input_size` |
| traffic_control | 红绿灯/停止线 | `traffic_control_input_size` |
| context | 全局上下文 | `context_input_size` |

```python
# pufferlib/ocean/torch.py:42-56
def _encode_and_pool(self, objects, valid_counts, encoder):
    valid_mask = torch.arange(objects.shape[1], device=...) < valid_counts.unsqueeze(1)
    encoded = encoder(objects)
    masked  = encoded.masked_fill(~valid_mask.unsqueeze(2), torch.finfo(...).min)
    pooled  = masked.max(dim=1).values            # 空槽位用 0 兜底
    return torch.where(valid_counts.unsqueeze(1) == 0, pooled.new_zeros(()), pooled)
```

**为什么用 mask + max-pool 而不是定长拼接**：partner 数量随场景变化，max-pool 让输出维度与数量无关；
`torch.compile` 又要求形状静态，所以用 `masked_fill` 而不是动态切片。
坏处是"数量信息"丢了 —— 需要的话得显式加特征。

`forward_eval`（推理）与 `forward`（训练）分开：前者不做 LSTM 状态更新、不需要梯度，
推理/评测走这条。

---

## 6. 存档与评测

`save_checkpoint`（`pufferl.py:920`）每 `checkpoint_interval` 个 update 存一次：

```python
state = {
    "policy_state_dict": ..., "optimizer_state_dict": ..., "scheduler_state_dict": ...,
    "global_step": ..., "epoch": ..., "ema_max": ..., "rng_state": ...,
}
```

- `experiments/models/model_<env>_<epoch>.pt`：只保留最新一个（自动删旧的）；
- `experiments/trainer_state.pt`：连优化器和 RNG 一起存，用于断点续训；
- `experiments/final_model.pt`：训练结束的最终权重；
- `ema_max` 也在里面 —— **优势过滤的阈值状态必须跟着存**，否则续训第一批的过滤会跳变。

`load_policy`（`pufferl.py:2785`）支持 `train.load_model_path=latest` / 具体路径，
并会用 `clean_policy_state_dict` 去掉 `torch.compile` 加的 `_orig_mod.` 前缀。

---

## 7. 关键超参一览（`puffer_drive.yaml`）

| 键 | 默认 | 影响 |
|---|---|---|
| `total_timesteps` | 5e11 | 总预算（本仓默认极大，实际靠外部中断） |
| `bptt_horizon` | 128 | 每行的步数；太短则 V-trace 看不到长期后果 |
| `minibatch_size` | 65536 | 每次梯度更新用多少 transition |
| `update_epochs` | 3 | 同一批数据复用几遍（PPO 的核心杠杆） |
| `clip_coef` | 0.2 | 策略比率裁剪范围 |
| `gamma` / `gae_lambda` | 0.999 / 0.95 | 折扣与优势平滑 |
| `ent_coef` | 0.01 | 熵奖励（探索） |
| `vf_coef` | 0.5 | 价值损失权重 |
| `max_grad_norm` | 0.5 | 梯度裁剪 |
| `adv_filter_*` | 0.25 / 0.01 / 0.0 | 优势过滤的 EMA、阈值、泄漏比例 |
| `anneal_lr` | true | 学习率线性退火 |

---

## 8. 从"跑通"到"训出东西"的排查顺序

按这个顺序看，能省掉大部分瞎调：

1. **`masked_fraction` 是否过高** → episode 太短或 `scenario_length` 与 `bptt_horizon` 不匹配；
2. **`kept_fraction` 是否接近 1** → 优势区分度不足（奖励信号太弱/太吵），过滤失去意义；
3. **`approx_kl` / `clipfrac`** → `clipfrac > 0.3` 说明步长太大，降 `lr` 或 `update_epochs`；
4. **`explained_variance`（价值拟合）** → 长期 < 0.3 说明 critic 没学会，先修观测/奖励再谈策略；
5. **`SPS`** → 采样是否成为瓶颈（GPU 利用率低往往意味着 env 侧慢）；
6. 再看 `score` / `dnf_rate` / `lane_center_rate` 这些任务指标。

---

## 9. 和 bad case 训练回路的关系（M7-4 要用的接口）

本仓已经把三件事做好了，正好对应"约束驱动的训练"：

| 需求 | 现成机制 |
|---|---|
| 只在临界时刻学 | `adv_filter_enabled` + `adv_filter_threshold_scale` |
| 难场景多采样 | RNN 路径的优先采样（`adv_sampling_prio_alpha/beta0`） |
| 场景可替换（换成 bad case 场景） | `env.map_dir` 指向任意 `.bin` 目录；`env.benchmark` 选内置数据集 |

也就是说 M7-4 **不需要改训练循环**：把 779 个场景（或其中"复现且可避"的 70 个）放进一个目录，
让 `map_dir` 指向它、`simulation_mode=replay` + `control_mode=control_sdc_only` + `sdc_controller=policy`，
策略就在这些 bad case 上学；再用引擎指标统计"同一批 bad case 是否还失败"，就是回归保留率。

---

## 10. 一句话总结

**PPO 在这份代码里不是一段公式，而是一条流水线**：
多进程 env 产固定形状的段 → V-trace/GAE 给每格打优势分 → 只留高分格 → 三遍 clip 更新 →
存档。改任何一个环节（奖励、优势、过滤、裁剪）都能单独验证，因为每一步都有对应的日志量。
