# 混合智能体对手为必需；拒绝朴素 self-play 作为 planner 训练交通

Status: proposed 2026-10-09

纯共享策略自博弈会过拟合自身对手分布，且规模不解决：BehaviorBench 的 PPO（自博弈、79k 张真实 WOMD 地图）对自己近乎零碰撞、对 IDM 高一个数量级，10¹¹ 步仍不单调下降；Waymax 2023 早有同现象（对 IDM 训练的 DQN 在日志回放对手下碰撞 25.15% vs 对 IDM 6.50%）；DriveZero 附录 A.5.2 篇内对照中混合 self-play 与 log/IDM 教师打平（93.11 vs 93.01）。裁决：训练与评测的交通必须混合 log-replay + IDM + 特殊行为（前车急刹、phantom-braking、blindness）+ self-play，且评测必须跨对手。PufferDrive 3.0 已有旋钮（`mix_traffic`、`ppo/idm/expert_fraction`、per-agent controller）；开放设计空间是对手采样矩阵 Σ（policy pool / league），驾驶领域目前空白。
