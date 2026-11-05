# 地图拓扑编码：邻接留在 Core，turn/overlap 走 map 侧表

Status: accepted 2026-11-05（owner 裁决；证据 `docs/2026-11-05-canonical-spec-v0.md` §3.2 / §4.2）

## Context

L0 canonical spec 的 `map` 组有三个拓扑字段，冻结前必须先定它们落在哪一层。三源审计给出了实测事实：

- **Apollo HD Map 侧 100% 有拓扑**：3 张真地图（4672 / 6690 / 31186 lane）的 `predecessor_id` 与 `successor_id` 密度 ×1.05–1.08（存在一 lane 多后继），`turn` 精确 ×1.00，`overlap` ×0.98–1.12。
- **Core 侧已有邻接**：`RoadMapElement` 含 `num_entries/entry_lanes` 与 `num_exits/exit_lanes`（`pufferlib/ocean/drive/datatypes.h:145-148`），由 `map_data.h:526-544` 从 map bin 读入，`drive.h:700-730` 用于 route 采样。
- **Core 侧确实缺 turn 与 overlap**：无对应字段，且 `c_step` / route / 碰撞热路径都不消费它们。

早先 `docs/adr/0007` 的表述（"`RoadMapElement` 只有 type/entry/exit/speed_limit，没有 boundary_type/turn_type/overlap"）本来就准确——`spec/canonical_v0.yaml` 初版把 entry/exit 误读成"无拓扑"，本次一并修正。

## Decision

1. **邻接（predecessor / successor）不新增任何结构**：Core 的 `entry_lanes/exit_lanes` 就是它，`sim=native`。转换器只负责 Apollo `predecessor_id/successor_id`（字符串 id）→ 数组索引的映射。
2. **`turn_type` 走 map 侧表**：作为随 scenario 下发的 `lane_id → turn` 列，不进 `RoadMapElement`，不改 map bin 格式。`sim=bridged`。
3. **`overlap`（stop_line / crosswalk 关联）只在语义外挂侧消费**，同样不进 Core。
4. 因此**本裁决不触发 map bin 重建**（仓库内 40 个 `pufferlib/resources/drive/binaries/**/*.bin` 保持有效）。

## Consequences

- `map_turn_type` 从 `blocked` 转 `frozen`；spec 的 `blocked` 计数清零（原 3 → 0）。
- 改 Core 结构 + bin 版本升版的代价被规避：那会要求重建 40 个 bin，且 turn 不属热路径字段，收益为零。
- 成本上界：8997 地图 31186 lane × i32 ≈ 125 KB / 场景；随 scenario 文件一起走，不进 `c_step`。
- 代价：`turn` 数据与 map bin 不再是单一工件，二者必须按 `scenario_id` + `map_version` 对齐（已由 spec 的 `provenance` 组覆盖）。
- 走廊域"路网连通"裁判用 Core 邻接即可；若裁判需要"左转/右转/直行"语义，从 map 侧表取，不读 Core。

## Considered Options

- **扩 `RoadMapElement` 加 `turn_type`**：被否——要动 `datatypes.h` + `map_data.h` 读写 + bin 格式版本，并重建 40 个 bin，换来的只是把一个非热路径字段塞进热路径结构体。
- **turn 完全不入 spec（放语义外挂黑盒）**：被否——turn 是约束模板（"该车道禁止右转"/"丁字路口"）的判据来源，必须显式冻结在 L0，否则每次换模型都要重新约定。
- **同时扩 Core 和 sidecar（双份真值）**：被否——两份真值必然漂移。
