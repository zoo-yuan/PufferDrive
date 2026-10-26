# Lessons（可复用教训；同一教训第二次出现必须升级为结构/ADR）

- **文献里的 "zero-shot" ≠ 跨输入迁移**：self-play 驾驶论文的 zero-shot 指固定模型观测契约 + per-benchmark 环境适配器，且基准交通是 IDM/回放/规则，从不是多样化学习对手（GigaFlow App.E 核实）。读论文时先问"适配器在哪"。
- **self-play 不是闭环分数的效率来源**：CaRL 1B 样本 → nuPlan Val14-R 93.12，GigaFlow 1T 样本 → 93.8（1000× 换 0.7 分）；DriveZero A.5.2 篇内对照打平。它的价值要用"对手分布可控生成 + 全场安全测试"计价。
- **闭环安全分可能度量的是仿真器宽松度**：无约束动作网格让策略用违反舒适包络的机动刷安全分（jerk 越限至 4513%，Comfort-by-Construction）。报安全分必须同时审计实际运动学。
- **PufferDrive 能力断言必须按分支核实**：3.0 的噪声在 `origin/8-9_bernhard_dev`（`dev/zoo_v3.0` 无 pose-noise）；`mining_viz.py` 已被 #533 的种子化失败回放删除。引用能力前先 `git branch --contains` + 查当前树。
