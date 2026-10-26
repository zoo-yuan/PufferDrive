# 第③轴的 gap 是 track 级感知抖动，不是 i.i.d. 传感器噪声

Status: proposed 2026-10-09（源于 owner realization）

rino 日志携带的是**线上感知的 track 级时序不稳定**（CIPV 位置/速度/朝向抖动、间歇漏检、ID 跳变、时延、版本漂移），而 nuPlan 式输入近乎无噪；仿真 track（replay/IDM/self-play 生成）相对它是"特权观测"。裁决：噪声注入必须（1）**时间相关**（匹配自相关，非每帧独立），（2）对决策关键目标（CIPV）**加权**，（3）进入 **16 帧历史**（模型唯一的滤波接口），（4）跨感知版本**域随机化**并建立漂移监控。`origin/8-9_bernhard_dev` 上的 `pose_noise_*`/brownian 是必要但不充分的起点。前置裁决缺口：抖动过滤归属哪一层（上游感知平滑 / planner 自滤波 / 独立估计层）未定，它直接改变训练数据构造要求——见 `docs/integration/2026-10-09-perception-jitter-as-the-real-axis3.md` §4。
