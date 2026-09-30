# PufferDrive × Apollo × Nova 集成

将 L4 车队采集的 Apollo HD 地图与运营日志导入 PufferDrive 仿真，用于闭环评估、失败挖掘与迭代 Nova 两段式 planner 的集成上下文。

## Language

**PufferDrive Core**:
PufferDrive 的 C 仿真内核（`pufferlib/ocean/drive/*.h`）：几何、动力学、碰撞、路线、交通控制、观测与奖励的唯一权威。
_Avoid_: 用 "PufferDrive" 泛指集成层（Core 特指内核）

**语义外挂（Semantic Sidecar）**:
PufferDrive Core 之外的一层，按场景提供 Nova 需要的富语义（14ch 地图、红绿灯、参考线、occ）。Core 不感知 Nova 的特征空间。
_Avoid_: 后处理、前处理（都指代不清）

**Apollo HD Map**:
`apollo.hdmap.Map` protobuf（PointENU 曲线、Lane/Boundary/type/turn、Signal、Crosswalk、Road、Junction、Overlap、Header.Projection）。
_Avoid_: 直接叫"地图"（与 14ch 特征地图混淆）

**14ch 特征地图（data_feature Map）**:
Nova 的规范地图编码：`LanePolyline`(14ch) / `Hardboundary`(7ch) / `Crosswalk`(5ch) / `Stopline`(10ch)，定义于 `data_process/data_feature.proto::Map`。
_Avoid_: 把它当作"原始地图"

**生产侧（Producer）**:
`wt-apollo/modules/tools/record_exporter`，把 Apollo HD Map + 车辆 record 转成 `DataFeature`（含 14ch 地图与 agent 轨迹）。是 14ch 编码的唯一真值来源。
_Avoid_: 数据管线、数据准备

**M1 / M2（地图特征来源）**:
M1 = 复用 tfrecord 每帧已录制的 14ch 特征并 re-anchor 到 live ego；M2 = 在任意 ego 位姿下从 Apollo HD Map 现场重抽 14ch 特征。
_Avoid_: 静态地图、动态地图（歧义）

**Scene**:
一个 PufferDrive episode 的输入单元：地图二进制 + agents（轨迹、route、goal）+ traffic controls。
本项目约定：窗口 = `16 历史 + 80 未来`（t0 = 第 16 帧，与 Nova 96 帧一致）；goal 取 agent GT 轨迹终点；agent 集合先保真（只用感知轨迹 + 有效性过滤），密度作为独立可调变量。
_Avoid_: 场景（口语混用 clip/scene/t0）

**第一实验（E1a/E1b）**:
E1a = ego 由 Nova 控制、背景车 GT 回放（非反应基线）；E1b = ego 由 Nova 控制、背景车 IDM/RL（反应式压力）。两者差值即反应性信号。
_Avoid_: 把 E1b 当成完整评测（它只回答"反应性影响"）

## Relationships

- **Apollo HD Map** ──(Producer)──▶ **14ch 特征地图** + agent 轨迹 → `DataFeature`
- **14ch 特征地图** ──(语义外挂)──▶ Nova 的 17 张量输入；**不经过 PufferDrive Core**
- **Apollo HD Map** ──(几何转换)──▶ **PufferDrive Core** 的地图二进制（只含几何/可行驶/交通控制）
- **Scene** 同时喂给 PufferDrive Core 与语义外挂，两者以 `scenario_id` 对齐
- **PufferDrive Core** 的 agent 控制器 ∈ {STATIC, POLICY, REPLAY, IDM}；POLICY 槽位可挂 Nova

## Example dialogue

> **Dev:** "Nova 闭环里那张 14ch 地图，是 PufferDrive Core 算出来的吗？"
> **Domain expert:** "不是。Core 只认几何和路径。14ch 由**语义外挂**给，源头是**生产侧**的 `frame_builder.h`——管线即规格，不能自己重写。"

## Flagged ambiguities

- "地图"同时指 **Apollo HD Map** 与 **14ch 特征地图** —— 已拆分，必须限定词。
- deploy 的 **OnboardFrame** 自称"原始语义"，但其 `MapElements` 已是 14ch，并非 Apollo HD Map 曲线 —— 集成时勿把二者等同。
- PufferDrive 仓库在 2026-09 发生大版本演进（新增 IDM / 交通灯 / route-goal / per-agent controller），旧方案文档基于更早版本 —— 引用能力时须核对当前 `constants.h`。
