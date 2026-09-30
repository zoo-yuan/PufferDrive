# PufferDrive Core 只负责几何与动力学，Nova 富语义走独立 Sidecar

## Status
proposed

## Context

要把 Apollo HD 地图（`apollo.hdmap.Map`）导入 PufferDrive 以迭代 Nova，存在两种耦合方式：把 Nova 的 15 维地图语义、红绿灯、参考线、occ 写进 PufferDrive 的 C 地图格式（C-native），或在 Core 之外用 Sidecar 提供这些语义。

C-native 会让 PufferDrive 地图二进制与 Nova 特征规范强绑定，Nova 语义演进（版本、字段）都要改 C 结构与内核，回退困难。而 PufferDrive Core 仿真实际只需要几何、可行驶区域、碰撞与交通控制，不需要 Nova 的特征空间。

## Decision

PufferDrive Core 的地图二进制只承载仿真必需的最小几何与交通控制；Nova 的全部富语义（14ch 地图、红绿灯、参考线、occ）由 **语义外挂（Sidecar）** 提供，以 `scenario_id` 与 Core 对齐。Core 不感知 Nova 的观测/特征契约。

## Considered Options

- **C-native 富格式**：把 15 维语义写入 PufferDrive 二进制 —— 高耦合、随 Nova 版本反复改 C、回退难。
- **Sidecar（选用）**：Core 保持稳定，语义层可独立演进，且能复用生产侧 14ch 管线保证与训练分布同源。

## Consequences

- PufferDrive Core 的 `.bin` 格式不随 Nova 语义变化，但需要维护一个 `scenario_id → sidecar` 的关联契约。
- Sidecar 成为 Nova 输入的单一真值来源；任何绕过它直接改 Core 观测的做法都会破坏同源性。
