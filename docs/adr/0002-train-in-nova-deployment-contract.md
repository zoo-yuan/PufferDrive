# Planner 直接在 Nova 部署观测契约（17 张量）中训练，不用 PufferDrive 原生观测、不走特权教师蒸馏链

Status: proposed 2026-10-09（方向由 owner 于 2026-09-11 拍板为"分支 B"；最终验收待 owner）；2026-10-31 代际切换见 `docs/adr/0006-nova-contract-19-in-4-out-static-polygon.md` —— 下文"17 张量 / 5 输出"是 2026-09-11 代际的原始判断，现行 = 19 输入 / 4 输出（`static_polygon_*` 取代 occ 流）

PufferDrive 原生观测（~1125 维：ego/partner/lane/boundary/traffic-control/goal）只是 Nova 17 张量契约（~163k 维，含 agent 类型、16 帧历史、occ、15 维地图语义、参考线、红绿灯关联）的**信息子集（GCD）**——在原生观测上训练出的 policy 永远学不会使用 Nova 独有特征，而跨观测契约的 "zero-shot" 迁移在文献中不存在（GigaFlow 的 zero-shot 靠 per-benchmark 环境适配器把世界编进**同一固定契约**；Pictura 证明特权观测迁移更差）。因此：RL 训练在 17 张量空间内进行（由 ADR-0001 的语义外挂产出），并**跳过** GigaPixel/DriveZero 式"特权教师→蒸馏"链路——因为部署输入（Apollo 感知向量）本身已是向量化，无相机模态鸿沟。后果：PufferDrive 角色降级为世界仿真 + 对手 + 奖励/噪声源；策略网络必须按 17 张量输入重建。
