"""Event-level lane-departure reward: fires once per episode, linear in the coefficient.

契约（ADR-0007 A-only 约束反馈 / canonical-spec §14.6 的正确形态）：
- 谓词与 constraints_v0 lane_departure 同源（|lane_dist| > 0.9 m 且 sim_speed > 0.5 m/s）；
- 每 episode 只罚一次，落在首次越道的那一步；
- 独立 env 字段（不进 reward_coefs 观测契约）；
- 系数 0（默认）= 完全关闭。

验证手段：必须用 **policy 控制器**驱动（roundC 在修好的 harness 里 150/154 违规，
必然触发）。不能用 replay 控制器：replay 贴 log 位姿、不积分动力学，engine 侧
`agent->sim_speed` 恒 0（M7-3 已知），速度 guard 永远不满足 —— 那不是本奖励的 bug。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest
import torch

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

SCENE_DIR = REPO_ROOT / ".scratch" / "crossbag_scenes"
CHECKPOINT = REPO_ROOT / "checkpoints" / "lane_departure_roundC.pt"
STEPS = 60

pytestmark = pytest.mark.skipif(
    not list(SCENE_DIR.glob("*.bin"))[:1] or not CHECKPOINT.exists(),
    reason="no crossbag scene bins or roundC checkpoint on this machine",
)


def _policy_episode_return(policy, scene_path: Path, device, reward_lane_departure: float) -> float:
    """policy 控制器跑完一集（与 evaluate_scene 同契约：mean 动作、训练 dt），返回奖励总和。"""
    from regression_retention import build_env, select_eval_action, training_env_config

    config = training_env_config()
    config["reward_lane_departure"] = reward_lane_departure
    env = build_env(scene_path, env_config=config)
    obs, _ = env.reset(seed=0)
    total = 0.0
    for _ in range(STEPS):
        obs_tensor = torch.as_tensor(np.asarray(obs), dtype=torch.float32, device=device)
        with torch.no_grad():
            logits, _value = policy.forward_eval(obs_tensor, {})
            action = select_eval_action(policy, logits)
        action = action.view(env.num_agents, -1).cpu().numpy().astype(np.float32)
        obs, *_ = env.step(action)
        total += float(np.asarray(env.rewards).sum())
    return total


def test_event_penalty_is_once_per_episode_and_linear():
    from regression_retention import build_env, build_policy, load_weights

    scene = sorted(SCENE_DIR.glob("*.bin"))[0]
    device = torch.device("cpu")
    policy = build_policy(build_env(scene), device)
    load_weights(policy, CHECKPOINT, device)

    return_zero = _policy_episode_return(policy, scene, device, 0.0)
    return_small = _policy_episode_return(policy, scene, device, 2.5)
    return_large = _policy_episode_return(policy, scene, device, 5.0)

    delta_small = return_zero - return_small
    delta_large = return_zero - return_large
    assert delta_large == pytest.approx(2 * delta_small, abs=1e-4), (
        "event must be one-shot: doubling the coefficient must double the penalty"
    )
    assert delta_small > 0.0, "roundC violates this scene; the event must have fired"
