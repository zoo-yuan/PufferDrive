# bad case 是场景种子；反馈契约 = A-only 约束；训练接口模型无关

Status: accepted 2026-11-04（owner 本会话裁决；分析见 `docs/2026-11-04-badcase-as-scenario-seed-constraint-loop.md`）

## Context

目标已从"给 Nova 定制仿真"收敛为：用 PufferDrive 得到一个吃矢量化输入（agents / occ polygons / HD map / traffic lights）的 planner，**能持续吃实车 bad case 反馈迭代到上车**（1→100），而不是做一个 demo。

三个外部事实约束了方案形状：

1. **bad case 的到达形式是 `bag + 时间戳` + 可下载的 HD map**，信息完备、非卡点；但**只有约束，没有专家修正、没有同场景好样本、没有分类标签**（owner 明示 "only A"）。
2. 反馈构成约为**违规型 80% / 体感型 20%**（owner 自陈 2/8 原则，尚未用数据湖统计验证）。
3. **Nova 不是唯一实现**，只是"在我们的采集数据上实现 model planner 的一条算法路径"。

既有 ADR-0002 / ADR-0006 把训练锚在 Nova 部署契约（19 输入 / 4 输出）上。这在"只迭代 Nova"时正确，在"模型可替换 + 持续吃 bad case"时**锁死了模型代际**：每换一个 encoder 就要重写数据层，等于每半年重做一次 0→1。

## Decision

1. **反馈契约 = A-only**：反馈只提供约束，不提供专家轨迹。bad case 的定义从"标签"改为
   `(场景种子, 约束束)`；训练信号 = reward（进度/舒适）+ 硬约束，不做专家对齐。
2. **RL 是核心训练手段**；IL 降级为可选的 anchor / 初始化。依据：TerraZero 以零人类演示、零日志轨迹的纯 self-play 取得 nuPlan val14 94.19（#1）与 InterPlan 70.87（#1）（一手页面，2026-11-04）。
3. **canonical scene/feature spec 模型无关**，由本仓拥有；Nova 降级为**基线模型 + 参考实现**。部署契约仍是"上线验收门"，不再是"训练接口的唯一真值定义"。
4. **occ 是物理实体**：进入碰撞 / offroad 热路径，不是纯观测通道。
5. **走廊锚定与全图漫游两个合法性域都要支持**；裁判分别是"对拍日志"与"内禀约束（路网连通 / 无穿模 / 车道方向 / 信号相位 / 目标可达）"。
6. **舒适进动作空间硬约束**（Comfort by Construction），不得只做软惩罚——A-only 下没有专家轨迹约束，软惩罚会被 RL 用 jerk 换安全分。
7. **1→100 的判据是两个数**：约束覆盖率、回归保留率。闭环分数不作为验收依据。

## Considered Options

- **专家修正 / 人工标注期望轨迹（B/C 反馈）**：被否——owner 侧只有 A；且 A 让违规型回路可全自动（PufferDrive 的 18 项指标本身就是谓词）。
- **继续把 19 张量当训练接口（沿用 ADR-0006 的代际口径）**：被否——模型可替换是第一性要求；19 张量只保留为基线实例与上线验收门。
- **occ 只作观测通道**：被否——物理实体，撞不撞得上必须能被策略感知到后果。
- **只做走廊锚定**：被否——长尾覆盖需要漫游域；且 TerraZero 的证据表明"少量地图 × 程序化场景"是 SOTA 的必要成分。
- **纯 IL 路线**：被否——在 10⁸–10⁹ 步预算下 RL-only 已有 SOTA 先例；IL 仅在需要行为锚定时作为可选件。

## Consequences

- **L0 canonical spec 成为新的、不可替换的资产**：字段清单与版本策略必须先冻结（含 provenance / 感知版本 / 地图版本），否则 1→100 每次都要重做数据层。
- 语义外挂（ADR-0001）的**服务对象**从"仅 Nova"扩展为"任意 encoder"；Core 仍不感知任何模型的特征空间，该裁定不变。
- 新增"**约束工程**"角色：负责把 20% 体感型 bad case 转写成 `(模板, 参数)`；20 条模板库为可规模化的上限形态。
- 必须有**可行性裁判**：同一场景用 `IDM / REPLAY / STATIC` 也跑一遍；全挂则场景不可解，禁止进 RL。
- **Goodhart 三防线**：全 18 指标审计、留出一族约束不参与训练、人工抽检 rollout。
- **积分器一致性升为 P0**：模型侧的 Euler 积分与 PufferDrive 的"梯形 + 精确圆弧"不一致时，critic 基于一个模型"没以为自己到过"的状态打标，闭环信号失真且不报错。
- **新增 `ACTION_TYPE_CONTROL`**：现 action 是 jerk，且 `c_throttle/c_steer` 奖励条件化系数、零穿越吸附、`v_eff = max(|v|,1)` 三处会静默改写 `(acc, κ)`；直连控制模式必须跳过这三处，否则执行不可复现（分析 §6.1.1）。
- **不反转 ADR-0002 的核心论点**：ADR-0002 反对的是"在 PufferDrive 原生信息子集 obs 上训练 / 走特权教师蒸馏链"；本裁定的 canonical spec 仍是与部署同源的富语义，只是不把 Nova 的 19 张量布局写死。
- PPO 只能作用在**有似然**的动作上：Nova 的 90 维模态 categorical 可用；diffusion 连续精修头不可用，只能作给定模态后的确定性精修。

## 验证锚点

- 分析全文：`docs/2026-11-04-badcase-as-scenario-seed-constraint-loop.md`
- 指标谓词：`pufferlib/ocean/drive/datatypes.h:78-86`（`metrics_array`，18 项）
- 场景窗口/`scenario_id`：`pufferlib/ocean/drive/map_data.h:700+`
- 执行限幅（已标定 3799 车型）：`docs/vehicle_profile.md` §3–§4；源头 `pufferlib/ocean/drive/constants.h:117-133` / `drive.h:4341-4390`
- Nova 积分器与似然来源：`~/labs-x/agent_group/nova/adfm/models/nova_decoder/nova_planner_decoder/diffusion_head.py`（`_integrate_control` / `motion_prob`）
