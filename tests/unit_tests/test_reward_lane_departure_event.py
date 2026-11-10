"""Event-level lane-departure reward: fires once per episode, linear in the coefficient.

契约（ADR-0007 A-only 约束反馈 / canonical-spec §14.6 的正确形态）：
- 谓词与 constraints_v0 lane_departure 同源（|lane_dist| > 0.9 m 且 sim_speed > 0.5 m/s）；
- 每 episode 只罚一次，落在首次越道的那一步；
- 独立 env 字段（不进 reward_coefs 观测契约）；
- 系数 0（默认）= 完全关闭。

验证手段：必须用 **policy 控制器**驱动。不能用 replay 控制器：replay 贴 log 位姿、
不积分动力学，engine 侧 `agent->sim_speed` 恒 0（M7-3 已知），速度 guard 永远不满足
—— 那不是本奖励的 bug。

驱动者用**脚本策略**（固定全加速 + 满左舵）而不是真 checkpoint：动作表从 12 类改成
28 类之后（ADR-0011）旧的 12 类权重已不可加载，而这条测试要的只是"一集里必然越道"，
不需要任何真实策略行为。
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
STEPS = 60

pytestmark = pytest.mark.skipif(
    not list(SCENE_DIR.glob("*.bin"))[:1],
    reason="no crossbag scene bins on this machine",
)


class _ScriptedDepartingPolicy:
    """固定"全加速 + 满左舵"：几秒内必然把车开出车道，用来触发事件罚。"""

    def forward_eval(self, observations, state=None):
        batch = observations.shape[0]
        return torch.zeros(batch, 3), torch.zeros(batch, 1)

    def discrete_probs_to_continuous_mean(self, probs):
        # 满加速（long 末档）+ 满左舵（lat 首档）
        return torch.stack([torch.ones(probs.shape[0]), -torch.ones(probs.shape[0])], dim=-1)


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
    scene = sorted(SCENE_DIR.glob("*.bin"))[0]
    device = torch.device("cpu")
    policy = _ScriptedDepartingPolicy()

    return_zero = _policy_episode_return(policy, scene, device, 0.0)
    return_small = _policy_episode_return(policy, scene, device, 2.5)
    return_large = _policy_episode_return(policy, scene, device, 5.0)

    delta_small = return_zero - return_small
    delta_large = return_zero - return_large
    assert delta_large == pytest.approx(2 * delta_small, abs=1e-4), (
        "event must be one-shot: doubling the coefficient must double the penalty"
    )
    assert delta_small > 0.0, "the scripted policy drives off the lane; the event must have fired"
