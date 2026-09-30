# 自博弈泛化：不确定消除与调研方案

> 日期：2026-09-11
> 本轮工作：读完 6 篇论文全文 + 本地 7 个仓库（`behavior-bench / gigapixel / GPUDrive-self-play / nocturne_lab / Pictura / PufferDrive / RAP-valeo-wod`），消除上一轮自标的"不确定"。
> 目标：为"另开一条 RL self-play 训练路径（脱离 BC）"给出**有证据的约束**与**可执行的调研方案**，不写实现。

---

## 1. 上一轮自标的"不确定"——现已全部消除

| 之前的不确定 | 结论（含出处） |
|---|---|
| GigaFlow 的 benchmark adapter 怎么把 nuPlan/Waymax 编码进观测？ | **Appendix E**：与每个 benchmark **共仿真**，把 benchmark 状态翻译进 GigaFlow 固定观测契约。nuPlan 每步需轨迹预测（用 GigaFlow 策略 rollout 0.3s）；CARLA 用 Jaeger et al. 的特权 expert 设置；Waymax 用 `IDMRoutePolicy`，goal = 专家轨迹终点 + 50m/100m 投影点。**三者的交通参与者都是 IDM / 规则 / 日志回放，从不是多样化的学习对手。** |
| Pictura / Alberti 的观测、动作、奖励？ | **4 相机（96×54，前后左右）**；对照的两个向量基线共享网络，唯一差别是 3 个"相机看不到"的槽位（他车速度、自车到车道中心距离、自车相对车道角）——`Vectorized` 填真值、`Vectorized†` 填 0。动作 = 4 长 × 3 横 jerk。奖励系数按 GigaFlow 分布随机。**50B steps / 35M km / 32×H100 13h。** |
| DriveZero 的方法细节？ | **DriveRL 特权 teacher**：5.70M 参数，结构化观测（ego 9D + actor 15D×5帧×≤96 + map 10D×≤256 token + 交通灯 4D + 2 个 goal anchor），对 jerk/steer-rate 输出 **Beta 分布**，**8 通道 critic**；奖励 = hard(碰撞/出路) + goal + 6 项**乘性**软指标（cross-lane/centerline/curb/comfort/TTC/overspeed）。**混合 agent**：日志回放 / IDM / 前车急刹 provider / 学习策略共存。DriveZero = DriveVFM(DINOv3+SigLIP2+SAM+DepthAnythingV2 蒸馏) + winner-takes-all 蒸馏 teacher。 |
| BehaviorBench 的 PPO 训练规模？ | **纯 self-play、~79,000 张真实 WOMD 地图**（`num_maps=79000`, `ppo_fraction=1.0`, `mix_traffic=False`, `control_mode=control_vehicles`）。**在这个规模下仍无法泛化到未见的交通 agent。** |

**本地资产**：`behavior-bench`（完整代码 + 权重 + 划分 + 评估框架）、`GPUDrive-self-play`、`nocturne_lab`、`RAP-valeo-wod` 为完整仓库；`gigapixel`、`Pictura` 仅有 README（代码 "coming soon"）。

---

## 2. 核心结论：泛化是三条**相互独立**的轴，靠三种不同机制

这是六篇工作+BahaviorBench 放在一起后最重要的规律，也是之前争论的根因：

| 轴 | 是否被 self-play 解决 | 正确机制 | 证据 |
|---|---|---|---|
| **① 地图/几何泛化** | ✅ 基本可以 | 地图几何扰动（rescale/shear/flip/reflect）+ 规模 | GigaFlow **只用 8 张地图**，zero-shot 到 nuPlan/WOMD |
| **② 交通行为泛化** | ❌ **不能** | **对手/行为多样性**（IDM / 回放 / 不同风格学习策略） | BehaviorBench：79k 真实地图 + 纯 self-play，换对手后 at-fault collision 显著上升；原话 "self-play RL agents trained at-scale fail to generalize to unseen traffic agents" |
| **③ 观测/感知泛化** | ❌ 不能 | 训练期观测可观测性与部署对齐（Pictura）或 perception adaptation（GigaPixel/DriveZero） | Pictura：特权向量 in-domain 更强、zero-shot 更差；去掉 3 个相机不可见特征就显著变差 |

**对"self-play/IDM/replay"的正确定性**：它们**不是**三种算法，而是**第 ② 轴的多样性旋钮**。behavior-bench 里就是显式 fraction（`mix_traffic / ppo_fraction / idm_fraction / expert_fraction`）与 `conditioned_mix`（按 agent 轮换 reward profile，产生异质车流）。

**因此**："有限地图 + 特权状态 + 海量 self-play ⇒ 强 planner 且零迁移到真实场景"——**第 ① 轴真，第 ②、③ 轴假**。你之前的怀疑成立；我上一轮"A 错⇒B 对"的逻辑错误已收回，文献同样不给单靠 self-play 的方案背书。

---

## 3. 从证据推出的、RL self-play 路径的硬约束

1. **观测契约必须"不比部署更特权"**（Pictura）：仿真里给的真值若 Apollo 感知给不出（精确速度、无遮挡），策略会依赖它。要么对齐可观测性，要么显式注入感知噪声（你有噪声统计——这点你之前已指出）。
2. **对手多样性是最大风险**（BehaviorBench）：训练与评估都必须混合 IDM / 回放 / 不同风格学习策略；只用一种对手=自欺。
3. **地图多样性靠扰动而非数量**：几何扰动 + 真实 Apollo 地图并用。
4. **奖励要"简单 + 随机化 + 条件化"**（GigaFlow/Pictura）：reward 权重随机并作为条件输入，用多样性换鲁棒；但简单奖励会导致 comfort/lane 指标差（BehaviorBench 观察到 PPO 三项行为指标都低）。
5. **动作空间要舒适有界**（Comfort by Construction）：否则安全指标被"违反舒适性"的躲避动作刷高，指标虚高且真车不可复现。
6. **规模/样本效率**：GigaFlow 用 advantage filtering（丢 ≤80% 低优势样本）；Pictura 用同一阈值(0.01×EMA)。若走 teacher-student，参考 GigaPixel(DAgger) / DriveZero(VFM 蒸馏)。
7. **评测必须跨对手、跨地图、跨初始化**，且 nuPlan Val14 太简单（可被"跟车道+碰撞检查"刷分）。

---

## 4. 调研方案（分阶段，先不写实现）

> 原则：**优先用 `behavior-bench` 现成资产把"泛化到底是哪条轴坏了"变成数字**，再决定训练配置。

### Phase 0 — 复现"换对手"泛化实验（最高优先，无需训练）
- **目的**：验证第 ② 轴，直接回答"你们要训的 self-play planner 会不会一换对手就塌"。
- **方法**：用 `behavior-bench` 的 `ppo_self_play.pt`（+ `conditioned_ppo.pt`），固定 ego=PPO，遍历 traffic ∈ {idm, ppo, smart, expert, conditioned_mix}；在 Interactive1k / Random1k 两个 split 上跑 `pufferlib/ocean/benchmark/eval.py`。
- **输出**：ego 的 at-fault collision / goal / comfort 随对手变化的曲线（即 BehaviorBench 表 1 的复现）。
- **成本**：无需训练，纯推理；本地 GPU 可跑。
- **风险**：低。

### Phase 1 — 量化"行为多样性"作为训练旋钮（低算力）
- **目的**：把第 ② 轴从"现象"变成"可控变量"。
- **方法**：用 `behavior-bench` 的 `mix_traffic=1` + `ppo_fraction/idm_fraction/expert_fraction` 训练若干短程 PPO（例如 10^8–10^9 steps 量级），对比：纯 self-play vs 混合对手；评估用 Phase 0 的换对手矩阵。
- **输出**："多样性比例 → 跨对手泛化"的曲线。
- **成本**：中（但远低于 10B steps；PufferDrive 单卡 ~30 万 SPS）。
- **风险**：中（配置/环境调试）。

### Phase 2 — 观测契约消融（在目标空间里做）
- **目的**：回答"Nova 17 张量里哪些特征值多少钱 / 哪些是特权泄漏"。
- **方法**：在 PufferDrive 侧构造 Nova 契约（17 张量的子集/全量）的观测，训练短程 PPO，消融：agent 类型、16 帧历史、occ、参考线、地图语义。对照 Pictura 的 `Vectorized` vs `Vectorized†` 思路：专门测"Apollo 感知给不出的槽位"的影响。
- **输出**：每个特征组的边际贡献 + 特权泄漏清单。
- **成本**：中（需要观测桥接的最小实现，可复用 Nova 的 feature builder 契约）。
- **风险**：中高（观测构建正确性）。

### Phase 3 — 地图/扰动消融
- **目的**：量化第 ① 轴，并评估 Apollo 地图的增量价值。
- **方法**：8 张 CARLA vs +WOMD vs +Apollo 地图 vs +几何扰动，训练短程 PPO，在留出地图上评。
- **成本**：中（依赖 Apollo→PufferDrive 几何转换，即之前讨论的 F1 最小几何层）。
- **风险**：中高（地图转换）。

### Phase 4 — 训练配方选型
- 基于 Phase 0–3，在 {直接 RL on Nova 契约} vs {特权 RL teacher + 蒸馏} vs {RL teacher + perception-only adaptation} 之间选型。
- 参考：GigaPixel（DAgger）、DriveZero（VFM 蒸馏 + 混合 agent）、Pictura（若目标是相机输入，则应从像素直接训）。

---

## 5. 需要你决策的点

1. **Phase 0 是否立即做**？（我认为是——最低成本、最高信息量，且完全用现成资产）
2. **最终观测契约**：Nova 17 张量全量，还是允许一个子集？（决定 Phase 2 的消融上限）
3. **对手策略**：训练期对手用哪几种？（决定 Phase 1 的旋钮范围）
4. **是否接受"训练期注入 Apollo 感知噪声"**（你之前已倾向用真实分布微调）——这直接决定第 ③ 轴。
5. **PufferDrive 版本基线**：以当前 `dev/zoo_v3.0` 为准？

## 6. 我仍不确定的（继续标注）

- `behavior-bench` 的 PPO 具体训练超参（学习率/熵/seed 数）是否与 GigaFlow 同量级；这影响"泛化失败"结论的**外推强度**（但结论方向已由 Phase 0 可直接复现，不依赖外推）。
- Pictura/Gigapixel 的**代码尚未开源**（本地只有 README），无法直接复用其渲染/DAgger 实现。
- 你们 Apollo 感知噪声与 sim 真值的分布差**具体多大**——这是第 ③ 轴的关键输入，需要你们的统计。
