# Nova 部署契约代际切换：19 输入 / 4 输出，static_polygon_* 取代 occ 流

Status: accepted 2026-10-31（一手来源：nova 仓 `~/labs-x/agent_group/nova` 的 `docs/dataset/数据规范V1.1.3_0911修正版.md` §导出 ONNX 输入字段(19路) / §导出 ONNX 输出字段(4路)、`docs/adr/0004-static-polygon-contract-and-occ-stream-retirement.md`（accepted 2026-10-30）、`docs/adr/0003-refline-slot-geometry-and-zero-refline-ruling.md`（accepted 2026-10-14）、`docs/coord_description.md`、`docs/onnx_export.md`（自带 17→19 过时注记））

## Context

ADR-0002（proposed 2026-10-09）把"在 Nova 部署观测契约内训练"锚在 **2026-09-11 代际：17 输入 / 5 输出**（含 `occ_polygons[1,2048]` + `occ_polygon_len[1]`，图内 ESDF 算子）。该代际已于 2026-10-30 在 nova 侧废止，本仓库 5 处文档（STATE / CONTEXT / gap 分析 / 09-11 方案 / 09-11 调研）仍以"17 张量"为前提。

## Decision

现行部署契约 = **19 输入 / 4 输出**；本仓库所有"契约"陈述一律按此代际书写，并在引用处标注代际与日期。

- **静态障碍改四路定长**：`static_polygon_points[1,32,32,2]` f32（ego RFU，≤32 polygon × 32 等弧长边界点，pad 0）、`static_polygon_types[1,32]` i64（token 0–7，pad −1；raw ONNX 声明 int64，引擎侧经 `build_engine.py` 插 Cast 变 int32）、`static_polygon_heights[1,32]` f32（m，pad 0）、`static_polygon_mask[1,32]` bool（pad false）。`occ_polygons` **只留训练侧**建 ESDF / ESDF loss，不进导出图。cap 32 是**有损截断**（按 AABB 到 ego 原点距离升序取最近 32），丢弃数必须上报。
- **输出 4 路**：`pred_traffic_light_decision` 已随 TL decision 预测删除；规划/预测轨迹均为 60 帧 @10Hz（6s）、global 系。
- **参考线槽位**：数据侧 5 条（严格序 ego, left, right, left-left, right-right）→ 模型侧 **3 槽、几何序**（按最近有效点 |lat| 升序，slot0 = ego ≈ lat 0）。**三槽全零 = 无链占位帧 = 训练错误数据，必须跳过推理**，不得喂进模型。
- **坐标系**：除 `stop_line_info` 与全部输出轨迹为 **global** 外，输入特征一律 **ego-centric RFU**（x=右, y=前, 直行 heading=π/2）；`obs_label_real_smooth8s` 为 agent-centric RFU。换算 `p_l = R(π/2−θ)·(p_g − t)`。

## Consequences

- **G8** 的 bridge 目标由"17 张量"改为"19 输入 / 4 输出"，且必须产出 `static_polygon_*` 四路：PufferDrive 的 40 static objects → 32 cap 需同款"最近优先"截断，并按 nova 口径上报丢弃数（`BuildStats.static_polygons_dropped_by_cap`）。
- **G9**（静态/占据物暴露）改写为"静态障碍四路张量暴露"，不再是 occ 多边形流；occ 只在训练侧存在。
- **G3**（route → 参考线）除映射外新增两条硬约束：slot0 必须是真实 ego 几何；无真实 ego 线时该帧跳过推理。
- **M1**（ADR-0005）复用录制帧特征时必须按 ego-centric RFU + heading 制式重抽——0911 数据已把除 `obs_label_real_smooth8s` 外的坐标统一到 ego-centric RFU。
- **窗口口径**：训练/GT 仍是 16 历史 + 80 未来 @10Hz（8s），但部署输出只有 60 帧（6s）；supervision 供 8s、模型吐 6s，二者不可混用。

## Considered Options

- **保留 17 输入双代际分支**：nova 侧已否（两套前处理 + 两套 golden，维护面翻倍、误用风险高于收益；旧产物归档不删）。本仓同样不引双契约。
- **只改 STATE 不动 ADR**：ADR-0002 标题会继续宣称"17 张量"，新会话仍被误导。
- **改写 ADR-0002 正文**：抹掉 2026-09-11 的原始判断依据，违反 strata「不重写历史」；故只在 Status 行加代际指针。
