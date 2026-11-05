"""桥接层：把规划器的 ``(acc, kappa)`` 序列转成 PufferDrive 的控制动作。

为什么这一层必须存在
--------------------
Nova 的推理在把轨迹"说"出来之前，会先把前 4 个加速度样本往当前加速度方向插值
（``_integrate_control`` 里的 wash-in）。仿真器不做这件事——它按给定的加速度原样积分。
如果桥接层忘了洗，critic 就会基于一个"模型从没打算去过"的状态打标签：

    实测 6 s 偏差 0.39–0.92 m（tests/unit_tests/test_control_mode_parity.py:
    test_raw_control_diverges_from_model_intent）

这个偏差不会报错，只会让闭环信号整体偏移。

责任边界
--------
- **桥接层**：wash-in（本模块）。属于"把模型意图翻译成仿真动作"的语义工作。
- **C 内核**：不做 wash-in。``ACTION_TYPE_CONTROL`` 就是"给什么积分什么"，
  这样执行保真可以被单独测试（见 ``INTEGRATION_SCHEME_MODEL_EULER``）。

公式来自 nova ``adfm/models/nova_decoder/nova_planner_decoder/diffusion_head.py``：
``target = 2 * mean(acc[:5] - a0) + a0``，前 4 个样本按 0.2/0.4/0.6/0.8 插值。
**不要"改进"这个公式**——它是模型侧的既有行为，改了就与模型不一致。
"""

from __future__ import annotations

import numpy as np

NOVA_ACC_WASH_IN_SAMPLES = 4
NOVA_ACC_WASH_IN_SCALE = np.array([0.2, 0.4, 0.6, 0.8], dtype=np.float32)
NOVA_ACC_WASH_IN_MEAN_WINDOW = 5


def apply_acc_wash_in(accel, current_accel):
    """返回 wash-in 之后的加速度序列（不修改入参）。

    accel: (N,) 规划器给出的纵向加速度 [m/s²]
    current_accel: 标量，执行时刻的实测纵向加速度 [m/s²]
    """
    accel = np.asarray(accel, dtype=np.float32)
    if accel.ndim != 1:
        raise ValueError(f"accel must be 1-D, got shape {accel.shape}")
    if accel.size == 0:
        return accel.copy()

    a0 = np.float32(current_accel)
    window = min(NOVA_ACC_WASH_IN_MEAN_WINDOW, accel.size)
    target = np.float32(2.0) * np.mean(accel[:window] - a0) + a0

    washed = accel.copy()
    samples = min(NOVA_ACC_WASH_IN_SAMPLES, accel.size)
    scale = NOVA_ACC_WASH_IN_SCALE[:samples]
    washed[:samples] = a0 + scale * (target - a0)
    return washed


def build_control_actions(accel, kappa, current_accel, apply_wash_in=True):
    """把 (acc, κ) 序列打成交给 ``ACTION_TYPE_CONTROL`` 的 (N, 2) 动作数组。

    apply_wash_in=False 只用于对照实验（复现"忘记洗"的偏差），生产路径必须是 True。
    """
    accel = np.asarray(accel, dtype=np.float32)
    kappa = np.asarray(kappa, dtype=np.float32)
    if accel.shape != kappa.shape:
        raise ValueError(f"accel/kappa length mismatch: {accel.shape} vs {kappa.shape}")
    if accel.size and accel.shape[0] < 1:
        raise ValueError("empty control sequence")

    longitudinal = apply_acc_wash_in(accel, current_accel) if apply_wash_in else accel.astype(np.float32)
    return np.stack([longitudinal, kappa], axis=1).astype(np.float32)
