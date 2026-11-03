# PufferDrive vs TerraZero：两种驾驶仿真哲学的对撞

> 日期：2026-10-31
> 一手来源：TerraZero 项目页 `https://terra-applied.github.io/TerraZero/` + 技术报告 **arXiv 2607.13028**（Applied Intuition + UC Berkeley）；PufferDrive 侧数字/接口以本仓 HEAD（`7a36d34a`）为准。
> 目的：给 Nova 集成路线的两个开放问题（场景多样性从哪来、训练吞吐瓶颈在哪）提供外部参照。**不做选型裁决**。

## 0. 一句话

同为「C 内核 + self-play 驾驶仿真」，两者**数据哲学相反**：PufferDrive 以日志为地基（回放真值 + 日志场景 + 失败挖掘）；TerraZero 把日志**降级为地图几何来源**，场景全部程序化生成，并以此拿到 nuPlan val14 第一（94.19）。

## 1. 逐轴对比

| 轴 | PufferDrive（本仓） | TerraZero |
|---|---|---|
| **组织/形态** | 开源仓库，可改 C 内核（`pufferlib/ocean/drive/drive.h`） | 工业界闭源（Applied Intuition + UC Berkeley），技术报告 + 在线 demo，无代码 |
| **场景来源** | 日志 `.bin`：agent 轨迹 + route + goal + 交通控制 + 静态物 + `scenario_id[128]`；`resample_frequency: 256000` 换图采样 | 日志**只提供地图几何**；每 episode 随机化路侧智能体、信号配时、动力学、reward、车辆尺寸 → 少量地图 = 无限长尾 |
| **智能体建模** | 车辆为主；per-agent controller ∈ {static, policy, replay, idm}；行人/骑行简化 | 车 + **高保真卡车** + 行人 + 骑行者，多动力学模型，完整交通规则（灯/停车牌），**3D 地图** |
| **观测** | 自有 ego-centric BEV 向量（~1125–1848 维） | 向量化，但页面/报告**未公开张量契约** |
| **吞吐/架构** | C 内核 + Python 多进程向量化；文档记 ~320K SPS（单 GPU，2026-09 口径） | C engine 跑 CPU、**策略推理放 GPU 走 zero-copy**；560K（消费卡）/ **1.3M（单 server GPU）** / 2.8M（8× 节点）→ 同口径约 4× |
| **训练配方** | PPO + V-trace + 优先采样 + advantage filtering；车流混合 log-replay + IDM + self-play + expert | **零演示纯 RL**：无人工演示、无模仿、无日志轨迹、推理期无 fallback planner；「用算力换样本效率」 |
| **成绩口径** | 18 项指标 + WOSAC + 种子化失败回放（#533）；不以 val14 为目标（val14 仅外部可比基准） | val14 **94.19**（#1；GigaFlow 93.8）、InterPlan 长尾 **70.87**（全量 71.31）、WOSAC 2024 realism 0.741（≈SPACeR 0.741）；**零样本跨城市跨数据集**，自发学会左侧通行 |
| **双重角色** | ego + 背景车对手混合，MARL 为主 | 同一个栈既做 ego planner，又做 **sim agent**（车/行人/骑行者联合控制） |

## 2. 对本项目的三条可迁移结论

1. **多样性来源可以不是数据湖**。TerraZero 用「日志只当地图」就把 val14 打到第一；我们手握 5000+ 车真实感知质量，却因场景受限于日志分布而担心长尾（ADR-0003 的第②/③轴）。**中间态候选**：保留 PufferDrive 的 replay 保真作「真实感锚点」，另加一层程序化 generator 扩多样性——但这要改 C 内核（`.bin` 场景结构与程序化场景不同源），成本不小。
2. **闭源 → 只能借思想**。可抄的是「日志降级 + 每 episode 随机化动力学/reward/尺寸」的配方，以及「规则化背景车 + 纯 RL ego」的分工；无可复用代码。
3. **瓶颈会从模拟挪到推理**。19 输入契约的张量比 TerraZero 自家契约重得多，320K SPS 的 Python 侧训练循环很可能先撞上策略推理带宽。TerraZero 的「C 引擎 CPU + GPU 零拷贝推理」是要抄的工程形态（对应本仓待办：`pufferl.py` 训练循环的 host↔device 往返）。

## 3. 不可比的部分（口径声明）

- val14 94.19、InterPlan 70.87、WOSAC 0.741 均为 **TerraZero 官方页面/报告自报**；与 GigaFlow / PDM 同表可比，但本仓未复现，也**不可**与本仓 18 指标直接对齐。
- TerraZero 的**策略与观测契约未公开**，不得假定其输入与 Nova 19 输入同构（见 `docs/adr/0006-nova-contract-19-in-4-out-static-polygon.md`）。
- WOSAC 双方在同一榜单（2023：TerraZero 0.632 vs GigaFlow 0.619；2024：0.741 ≈ SPACeR），但**仿真资产不同**（3D+卡车+行人 vs 2D 车辆为主），分数不代表同难度。

## 4. 对本仓的动作项（开放问题，待 owner 裁决）

| # | 问题 | 背景 | 影响面 |
|---|---|---|---|
| 1 | 是否给 PufferDrive 引入**程序化场景生成层**（TerraZero 路线） | 与 ADR-0003 的对手采样 Σ、以及 10-09 长尾分析直接相关；改 C 内核 | 高（新子系统） |
| 2 | 19 输入契约下**策略推理是否成为新瓶颈**，是否改「C 引擎 + GPU 零拷贝推理」形态 | 当前训练循环在 Python 侧（`pufferlib/pufferl.py`） | 中（架构改造） |
| 3 | 背景车多样性：沿用 PufferDrive 的 per-agent controller，还是移植 behavior-bench 的 **fraction 混合** | behavior-bench 才有 `mix_traffic/ppo_fraction/idm_fraction/expert_fraction`；PufferDrive 本仓**无** fraction 旋钮（2026-10-31 全分支核实） | 低（移植量小） |
