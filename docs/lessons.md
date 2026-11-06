# Lessons（可复用教训；同一教训第二次出现必须升级为结构/ADR）

- **文献里的 "zero-shot" ≠ 跨输入迁移**：self-play 驾驶论文的 zero-shot 指固定模型观测契约 + per-benchmark 环境适配器，且基准交通是 IDM/回放/规则，从不是多样化学习对手（GigaFlow App.E 核实）。读论文时先问"适配器在哪"。
- **self-play 不是闭环分数的效率来源**：CaRL 1B 样本 → nuPlan Val14-R 93.12，GigaFlow 1T 样本 → 93.8（1000× 换 0.7 分）；DriveZero A.5.2 篇内对照打平。它的价值要用"对手分布可控生成 + 全场安全测试"计价。
- **闭环安全分可能度量的是仿真器宽松度**：无约束动作网格让策略用违反舒适包络的机动刷安全分（jerk 越限至 4513%，Comfort-by-Construction）。报安全分必须同时审计实际运动学。
- **PufferDrive 能力断言必须按分支核实**：3.0 的噪声在 `origin/8-9_bernhard_dev`（`dev/zoo_v3.0` 无 pose-noise）；`mining_viz.py` 已被 #533 的种子化失败回放删除。引用能力前先 `git branch --contains` + 查当前树。
- **引用外部部署契约必须带代际锚点**：“17 张量”曾无标记地散在本仓 5 处文档，上游 2026-10-30 切换代际后全线漂移（nova 侧 `onnx_export.md` 同样只能靠事后注记）。已升级为结构：ADR-0006 承载代际事实 + `CONTEXT.md` 新增“Nova 部署契约 / 参考线槽位 / ego-centric RFU”术语条 + STATE 记录 nova 一手锚点文件；引用契约一律写“代际 + 日期 + 权威文件”。
- **宣布"数据缺失"前先找索引文件**：我把 `base_map.catelog`（84/115 MB）+ 6,249/10,874 个 500 m tile 的大图判成"没有地图"，据此写出一条错误的"数据供给缺口"结论。大图从来不以整图 bin 交付，`sim_map.bin` 0 字节、`routing_map.bin` 只是路由图，都不能当日志里的"没有"。缺文件时先 `ls` 目录结构 + 找 `*.catelog`/`*.catalog`/分片目录，再谈缺口。
- **指标失败先问"这段该不该被度量"**：车道贴合度 p90 33.7° 的真相是该 bag 大半时间在场站里挪车（1.5–2.1 m/s，无车道线），不是匹配错。放阈值之前先按工况切分（行驶中 ≥2 m/s vs 场站），把"数据覆盖不足"与"算法错误"分开计。**放宽阈值永远是最后手段。**
- **"编译器太旧"先别升级系统**：`pufferlib._C` 编不过的真因是 torch 强制 `-std=c++20`（g++ 9.4 只认 `-std=c++2a` 且缺 `<compare>`）。把 gcc 13 装在 conda 的 build-only 环境里、用 `CC`/`CXX`/`PATH` 只影响当前 shell，就解决了，没有任何系统级改动。顺带发现两个隐藏坑：① 本机 CUDA toolkit 12.6 vs torch 13.2，CUDA 内核根本编不了（要 `PUFFER_BUILD_CPU_ONLY`）；② `torch.compile` 训练时会**再调一次编译器**，所以运行前也要 `source` 工具链，光编一次 `_C` 不够。
