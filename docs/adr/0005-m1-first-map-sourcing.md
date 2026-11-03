# M1 先行：复用录制帧 14ch 地图特征；M2 原图重抽转换器单独立项

Status: accepted 2026-09-11（owner）；2026-10-31 补充见 `docs/adr/0006-nova-contract-19-in-4-out-static-polygon.md` —— "14ch" 指 data_feature proto 侧字段编码，模型侧地图输入已切 15 维统一元素 `map_vector_features[1,800,11,15]`；M1 re-anchor 须按 ego-centric RFU + heading 制式重抽（0911 数据已把除 `obs_label_real_smooth8s` 外的坐标统一到 ego-centric RFU）

Nova 的地图输入是 `data_feature.proto::Map` 的 14ch 编码，其生产侧代码不在本仓库（`wt-apollo/modules/tools/record_exporter/frame_builder.h`，从 `apollo.hdmap.Map` 构建）。M1（复用 tfrecord 每帧已录 14ch 特征、re-anchor 到 live ego）与训练分布**逐位同源**，先行采用；M2（在任意 ego 位姿从原始 Apollo HD map 现场重抽 14ch）解锁全空间覆盖，但需移植生产侧转换器，单独立项、以 M1 的"地图覆盖哨兵"触发为闸门。配套 Scene 契约（M1 期间）：窗口 96 帧 = 16 历史 + 80 未来（t0=第 16 帧，与 Nova 对齐）；goal 取 agent GT 轨迹终点；agent 集合保真优先（感知轨迹 + 有效性过滤），密度作为独立可控变量；首实验 E1a（GT 回放背景）→ E1b（IDM 反应式背景）取差值作为反应性信号。
