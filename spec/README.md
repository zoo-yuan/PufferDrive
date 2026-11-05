# spec/ —— L0 canonical spec 的唯一真值源

本目录只放一件事：**模型无关的 canonical scene/feature spec**（ADR-0007 decision 3）。
Nova 的 19 输入 / 4 输出是**上线验收门与基线实例**（ADR-0006），不是训练接口定义；它在字段表里以 `nova:` 列出现，仅作映射视图。

## 文件

| 文件 | 作用 |
|---|---|
| `canonical_v0.yaml` | 字段注册表（手工维护，唯一真值源） |
| `../scripts/spec_report.py` | 校验 + 生成可视报告（不手写表格） |
| `../docs/html/spec_v0_coverage.html` | 生成的覆盖率报告（M1 验收物） |

## 字段属性（全部必填，缺一个即 gate 失败）

`unit` / `frame` / `dtype` / `time` / `truth` / `sim` / `nova` / `status`

- `frame`：`global` | `ego_rfu` | `agent_rfu` | `-`。**坐标系混用是本项目历史第一大 bug 源**（CONTEXT: ego-centric RFU 直行 heading=π/2）。
- `sim`：`native`（内核已有）| `bridged`（桥接层要产出）| `absent`（两侧都没有 → 桥接层必做项）。
- `status`：`frozen`（可依赖）| `draft`（待 owner 确认）| `blocked`（依赖未决项）。

## 冻结协议

1. 改字段 → 只改 `canonical_v0.yaml`（散文里不再手写字段表）。
2. 跑 `python scripts/spec_report.py --check`，gate 必须过：
   - 字段数 ≥ 40
   - 必填属性缺失 = 0
   - frozen 字段占比 ≥ 60%
   - 每个组至少 1 个 frozen 字段
3. 跑报告器生成 HTML，把数字贴进会话或 PR 描述。
4. `schema_version` 只在**破坏性变更**时升（删字段/改 frame/改 unit）；新增字段不提版本。
5. 字段级语义变更要在 `docs/adr/` 留裁决，不在本文件里辩论。

## 三条已冻结的硬约束

1. **occ 是物理实体**，必须进碰撞/offroad 热路径，不得只做观测通道（ADR-0007 decision 4）。
2. **地图射程与障碍射程是两个字段**（`map_range_m=200` / `agent_range_m≈70`）——地图可看远，障碍物不能（vehicle_profile §4.2）。
3. **限值分四种性质**（硬动力学限 / 约束 / 指标 / 场景生成边界），混用会同时限制物理能力与堵死"超速后怎么收"的学习（vehicle_profile §4.1）。

## 与其他文档的边界

- `docs/vehicle_profile.md`：车辆参数与实测数字的一手来源 → 本 spec 的 `vehicle` / `ego` 组引它，不复制推导。
- `docs/integration/CONTEXT.md`：术语与关系（Scene / M1 / M2 / 生产侧）。
- `docs/adr/0006`：19 输入 / 4 输出的张量布局 → `nova:` 列的来源。
- 本 spec **不含**模型特征空间、不含张量 shape 决策、不含训练超参。
