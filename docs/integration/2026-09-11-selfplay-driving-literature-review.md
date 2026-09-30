# 自博弈驾驶文献补课：六篇工作的真实主张、证据与对 Nova-RL 路径的约束

> 日期：2026-09-11
> 阅读范围（全文，非摘要）：
> - **GigaFlow** — *Robust Autonomy Emerges from Self-Play*, Apple, arXiv:2502.03349
> - **GigaPixel** — *Scaling Self-Play for End-to-End Driving*, Mila + Emerge Lab, arXiv:2606.19641
> - **Pictura** — *Perspective-View Self-Play at Scale for Driving*, Valeo 等, arXiv:2607.26005
> - **DriveZero** — *End-to-End Driving Beyond Human Demonstrations*, Xiaomi, arXiv:2609.06055
> - **Comfort by Construction** — *Adaptive, Comfort-Bounded Action Spaces for Learned Driving Policies*, arXiv:2609.13011
> - **BehaviorBench** — *Beyond Self-Play and Scale: A Behavior Benchmark for Generalization in Autonomous Driving*, arXiv:2605.10034
>
> 目的：为"另开一条 RL self-play 训练路径（脱离 BC、海量自迭代场景）"提供有证据的设计约束，而不是转述摘要。

---

## 0. 一句话结论

**"在少量地图里用特权向量做 self-play，就能得到 map/scene-agnostic、zero-shot 迁移的强 planner"——这句话在文献里只对了一半：**

- **地图层面**的跨域迁移：**基本为真**（GigaFlow 只用 8 张地图，zero-shot 到 nuPlan/WOMD 真实地图）。
- **交通行为层面**的泛化：**被证伪**（BehaviorBench：大规模 self-play PPO 在换对手后 at-fault collision 显著上升，说明过拟合到 self-play 自己的行为）。
- **输入契约层面**的"零适配"：**为假**（每个 benchmark 都要把世界编码进模型固定的观测契约；GigaPixel/Pictura 明确否定了"向量→像素"的零适配）。

因此"荒诞感"的来源不是 self-play，而是**把"地图泛化"误当成"场景/行为泛化"**。

---

## 1. 六篇工作的机制与关键数字

| 工作 | 观测 | 训练范式 | 规模 / 吞吐 | 关键结论 | 关键限制 |
|---|---|---|---|---|---|
| **GigaFlow** | 特权向量（稀疏 lane 点 + 密集 boundary + stop line/信号灯 + 邻居 size/pos/orient/vel + 自车状态 + 条件 C） | 纯 self-play PPO（+ advantage filtering，过滤 ≤80% 低优势样本） | 6M 参数 Deep Sets；38,400 envs × ≤150 agents；1T transitions / 1.6B km / <10 天 8×A100 | 单一策略 zero-shot 超过 CARLA/nuPlan/Waymax 各自 SOTA；仿真内 3M km（17.5 年）一次事故 | 8 张合成地图；对他人 goal/acceleration **不可见**；benchmark 对手是 IDM/replay/rules |
| **GigaPixel** | 像素（透视渲染的简化 bbox 世界） | **Self-play DAgger**：特权 RL teacher → 像素 student 在线蒸馏；再仅适配 perception | 50K agent SPS / 单 GPU（PufferDrive + 透视渲染） | HUGSIM SOTA、NAVSIM-v2 有竞争力；**无人类轨迹监督**；规模↑收益↑ | 明确否定"像素直接 RL"的样本效率；需要 perception adaptation |
| **Pictura** | 像素（自研 CUDA 光栅化透视视图） | 直接从图像 self-play PPO，**无特权 teacher、无蒸馏** | 500K agent SPS / 2M images/s / 单 H100 | 特权向量"解决的是比部署更简单的问题"；图像策略**迁移到真实 WOMD 布局更好** | CARLA 合成地图；外观对齐留作未来工作 |
| **DriveZero** | 相机（DriveVFM 感知）+ 特权 RL teacher（DriveRL） | 解耦预训练（RL teacher + VFM 感知）→ winner-takes-all 蒸馏为 camera-only planner | DriveRL 5.70M 参数，输出 jerk/steer-rate 的 Beta 分布；PPO + value-guided test-time search | DriveRL nuPlan 三集均值 93.01（超 log-replay 专家），搜索后 93.57；DriveZero NAVSIMv1 94.8 PDMS = 人类；NAVSIMv2 navhard 57.1 EPDMS、HUGSIM 46.6 HD-Score SOTA；**无人类轨迹监督** | 需要感知预训练 + teacher 蒸馏；teacher 是特权状态 |
| **Comfort by Construction** | —（动作空间研究） | 分析 + 约束感知动作参数化 | 基于 PufferDrive | **无约束动作空间让策略用"违反舒适性"的躲避动作刷高安全指标**；安全指标测的是仿真器宽松度，不是策略质量 | 违反舒适性的动作在真车不可复现 |
| **BehaviorBench** | —（评测基准） | PufferDrive 上 2×1k WOMD 场景 + 多种交通 agent（IDM/PPO/SMART/aggressive/normal/cautious） | — | **大规模 self-play PPO 换对手后 at-fault collision 显著升高**；nuPlan Val14 可被"跟车道+碰撞检查"刷分；现有基准高估 planner | 证明"self-play 泛化"在**交通行为**维度上很弱 |

**共同点**：都用"特权/结构化状态 + 大规模并行 + PPO 类算法"，都强调"规模换鲁棒性"，都在**地图维度**观察到迁移。

**分歧点**：观测模态（向量 vs 像素）与是否允许特权 teacher：
- GigaFlow：特权向量，纯 self-play。
- GigaPixel：特权 teacher + 像素 student（DAgger）。
- Pictura：像素直接 self-play，**反对**特权优先。
- DriveZero：特权 teacher + 相机 student，但把**感知**单独用 VFM 预训练。

---

## 2. 逐条回答"荒诞感"里的每个疑问

### Q1. CARLA 地图够好吗？能模拟真实/复杂地图吗？
**不是"够不够"的问题，而是"泛化靠扰动而不是靠地图数量"**：
- GigaFlow 只用 **8 张**地图，靠 **rescale / shear / flip / reflection 随机扰动** + 随机 spawn + 均匀采样 goal，zero-shot 迁到 nuPlan/WOMD 真实地图。
- Pictura 同样只从 **CARLA 合成地图**训练，迁到 WOMD 布局。
- 所以"几张地图 = 一定过拟合"不成立；**几何扰动 + 规模**可以在很大程度上替代地图多样性。
- 但你们 Apollo HD 地图的价值在于**真实复杂度**（立交、潮汐、复杂路口）——这是合成地图扰动**不一定覆盖**的分布。导入它们是对的，但它是"扩分布"，不是"补数量"。

### Q2. zero-shot 迁到真实采集数据、还近乎 SOTA，是真的吗？输入不需要适配吗？
**分三层，别混：**
1. **模型权重 zero-shot**：GigaFlow 是真的（没在 benchmark 上训练/微调）。
2. **但世界必须被编码进模型的观测契约**：nuPlan/Waymax/CARLA 各自有 adapter，把环境状态映射成 GigaFlow 的 `(W,S,A,C)`。所以"zero-shot"指**不做模型适配**，不等于"任意输入直接可用"。
3. **跨模态（向量→像素）不 zero-shot**：GigaPixel 需要 DAgger + perception adaptation；Pictura 干脆从图像直接训练来绕开。

**对你最相关的**：Nova 的输入是**感知向量**（非像素），所以你在**观测模态**上更接近 GigaFlow 的范式，而不是 GigaPixel/Pictura。你需要的不是"perception 学出来的迁移"，而是**表示一致性**（仿真状态 → 17 张量 与 Apollo 感知 → 17 张量 同分布）。这是工程/数据问题，不是感知学习问题。

### Q3. "有限地图 + 特权状态 + 10B steps → 强 planner，还能跨地图 zero-shot"——是这样吗？
**地图层面：基本是。行为层面：不是。**
- GigaFlow 的 emergent skills 有训练步数曲线（merge/u-turn 在 10^9~10^12 steps 才可靠涌现），说明这是"规模 + 局部几何 + 稀疏奖励"的产物，不神秘。
- 但 **BehaviorBench** 直接测了这件事：self-play PPO **在自己当对手时 at-fault collision 最低，换成 IDM/保守/回放对手就显著变差**。论文结论原话是"self-play RL agents trained at-scale fail to generalize to unseen traffic agents"。
- 所以"与场景无关"是最弱的一环；"与地图无关"相对强。

### Q4. self-play 到底给了什么、没给什么？
- **给了**：对手课程、闭环纠错、交互鲁棒性、可无限扩张的状态覆盖。
- **没给**：跨观测契约的泛化（要 adapter/蒸馏）、跨对手行为的泛化（BehaviorBench 证伪）、感知鲁棒性（GigaFlow 自己承认只是"minimalistic noise modeling"；Pictura 认为特权向量会学到无法从自身视角解释的决策）。
- **另一个隐藏坑**（Comfort by Construction）：无约束动作空间下，策略会**用违反舒适性的动作刷高安全指标**——你看到的"强"可能有一部分是仿真器宽松度的假象。

---

## 3. 从文献推出来的、对你这条 RL 路径的硬约束

若你要"脱离 BC、单开 RL self-play、在 Nova 输入空间里、用海量自迭代场景训练"，文献给出如下**必须处理**的设计点：

1. **观测契约先定死，且要建模不确定性**（GigaFlow 的关键）：刻意让别人 goal/acceleration 不可见、加状态噪声、加部分可观测（遮挡）。否则策略会依赖部署时不存在的信息（Pictura 的核心批评）。
2. **交通行为多样性是最大风险**（BehaviorBench）：只用 IDM 当对手训练/评测 = 自欺。必须混合 IDM / replay / 不同风格 learned policies，且评测要在**异构对手**上做。
3. **把"地图多样性"当自变量，而不是地图数量**：几何扰动（rescale/shear/flip/reflect）+ 真实 Apollo 地图，二者都要。
4. **奖励简单 + 随机化 + 条件化**（GigaFlow）：reward 权重按 agent 随机并作为条件输入，用"多样性"换鲁棒；但 BehaviorBench 也指出简单奖励导致 comfort/lane 指标差，需要补行为项或后处理约束。
5. **动作空间要"舒适有界"**（Comfort by Construction）：否则指标虚高、真车不可复现。这也是你们必须和真车动力学对齐的地方。
6. **规模与采样效率**：GigaFlow 用 advantage filtering 过滤 ≤80% 低优势样本；DriveZero/GigaPixel 用 teacher 蒸馏来降低样本复杂度。你需要决定"直接 RL 17 张量"还是"特权 teacher 蒸馏"。
7. **评测必须跨对手、跨地图、跨初始化**：否则无法区分"能力"与"过拟合"。已有 `nuPlan Val14` 被证明太简单。

---

## 4. 对你判断的直接回应（我不再含糊）

- 你之前判断"不信任 A 能在 rino 数据上出竞争力结果"——**文献支持你**：跨观测契约的免费迁移不存在。
- 但我上一轮说"所以 B 一定对"——**那是逻辑错误，我收回**。文献同样不给 B 背书：**B 的成败取决于上面 7 条约束是否被处理**，尤其是对手多样性（BehaviorBench）和舒适动作空间（Comfort）。B 只是**必要条件**，不是充分条件。
- 你说的"另开一条 RL self-play 路径"是合理的**方向**，但文献表明它是一个**多变量工程**：观测契约、对手多样性、地图扰动、奖励设计、动作空间、规模，缺一个都会退化成"只在自家 benchmark 上好看"。
- 之前我提的"PufferDrive 只能当世界模拟器 + 对手"这个降级说法也偏了：在 B 里，PufferDrive 应该**同时是环境、对手来源、以及（可选）特权 teacher**；而 planner 的网络与观测必须按你的 17 张量契约重建。

---

## 5. 我仍然不确定的部分（明确标注）

- Pictura/Alberti 和 DriveZero 的**完整观测张量与奖励细节**（我读的是正文主干，未逐行核对附录）。
- GigaFlow 的 **benchmark adapter 具体如何把 nuPlan/Waymax 的 map/agent 编码成其观测**——这决定了"地图泛化"里有多少是通用几何、有多少是 adapter 工程。
- BehaviorBench 的 PPO 具体训练配置（是否与 GigaFlow 同量级），这影响"自博弈泛化失败"结论的外推强度。
- 你们 rino/Apollo 感知噪声与 sim 真值的**分布差**到底多大——这决定"表示一致性"要投入多少。

---

## 6. 下一步可验证动作（不靠信仰）

1. **复现 BehaviorBench 的"换对手"测试**：把任意一个 PufferDrive self-play checkpoint 在 IDM / replay / 不同风格对手上评一遍，看 at-fault collision 是否跳变。这是最快的"自博弈泛化真伪"自测。
2. **做地图扰动消融**：固定 8 张地图 vs 加入你们的 Apollo 地图 vs 加入几何扰动，看 nuPlan/WOMD 迁移分。
3. **做观测契约消融**：在 Nova 17 张量契约下，分别给/不给 agent 类型、历史、occ，测能力差——直接量化"富语义到底值多少"。
4. **舒适约束基线**：按 Comfort by Construction 重做动作空间，看指标掉多少——掉的部分就是"仿真器宽松度"贡献的虚分。

