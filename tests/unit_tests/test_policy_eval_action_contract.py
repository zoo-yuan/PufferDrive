"""Eval-harness action contract: the policy path must mirror training eval exactly.

2026-11-06 bug (see docs/2026-11-05-canonical-spec-v0.md §15): the hand-rolled
harness turned logits into actions with argmax. At a parked start the trained
policy's distribution puts ~63% on the no-op cell, so argmax self-locks
(parked -> no-op -> still parked) and every policy-side number read 0.00 m/s.
Training-side eval uses ``action_selection: mean`` (probability-weighted
physical-table expectation, ``discrete_probs_to_continuous_mean``) and the
training env dt — the harness must do both.

These tests lock the harness seam, not ``sample_logits`` itself (that is
covered by test_action_selection.py).
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

CHECKPOINT = REPO_ROOT / "checkpoints" / "lane_departure_roundC.pt"
SCENE_DIR = REPO_ROOT / ".scratch" / "retention_scenes"
RUN_CONFIG = REPO_ROOT / "experiments" / "config.yaml"

# The parked-start distribution that produced the 0.00 m/s self-lock:
# mode on the no-op cell, second-most mass on full acceleration.
# (Stub table: idx0 = no-op (0,0), idx1 = full brake (-1,0), idx3 = full accel (+1,0).)
PARKED_LOGITS = torch.tensor([[1.0, 0.05, 0.0, 0.9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])


class _StubDiscretePolicy:
    """Minimal discrete-policy stand-in: 12 jerk classes, no-op at index 0,
    full longitudinal jerk at index 3 (mirrors the real action table shape)."""

    is_continuous = False

    def __init__(self):
        table = torch.zeros(12, 2)
        table[1, 0] = -1.0
        table[3, 0] = 1.0
        table[2, 1] = -1.0
        table[4, 1] = 1.0
        self.action_table = table

    def discrete_actions_to_continuous(self, actions):
        return self.action_table[actions.long()]

    def discrete_probs_to_continuous_mean(self, probs):
        return probs @ self.action_table.to(probs.dtype)


def test_select_eval_action_is_prob_weighted_mean_not_argmax():
    """argmax returns the no-op row (long=0); mean must return the expectation
    (long>0 because 24% of the mass sits on full acceleration)."""
    from regression_retention import select_eval_action

    policy = _StubDiscretePolicy()
    action = select_eval_action(policy, PARKED_LOGITS)

    argmax_row = policy.discrete_actions_to_continuous(PARKED_LOGITS.argmax(dim=-1))
    assert float(argmax_row[0, 0]) == 0.0  # the self-lock: argmax is the no-op

    assert float(action[0, 0]) > 0.05, "mean selection must produce a nonzero push off standstill"
    probs = torch.softmax(PARKED_LOGITS, dim=-1)
    expected = policy.discrete_probs_to_continuous_mean(probs)
    assert torch.allclose(action, expected, atol=1e-6)


@pytest.mark.skipif(not CHECKPOINT.exists(), reason="roundC checkpoint not on this machine")
@pytest.mark.skipif(not (SCENE_DIR).exists() or not list(SCENE_DIR.glob("*.bin"))[:1],
                    reason="no retention scene bins")
def test_policy_env_matches_training_dt():
    """The policy env must inherit dt from the training snapshot (experiments/
    config.yaml, dt=0.3), not the 0.1 s judge cadence — stepping the trained
    policy at the wrong dt understates its speed ~8x."""
    import yaml

    from regression_retention import build_env

    train_env_cfg = yaml.safe_load(RUN_CONFIG.read_text(encoding="utf-8"))["env"]
    scene = sorted(SCENE_DIR.glob("*.bin"))[0]
    env = build_env(scene)
    assert env.dt == train_env_cfg["dt"]


@pytest.mark.skipif(not CHECKPOINT.exists(), reason="roundC checkpoint not on this machine")
@pytest.mark.skipif(not (SCENE_DIR).exists() or not list(SCENE_DIR.glob("*.bin"))[:1],
                    reason="no retention scene bins")
def test_policy_moves_off_standstill_in_harness():
    """The exact 2026-11-06 symptom: same checkpoint that logs ~m/s speeds in
    training must not read 0.00 m/s median in the eval harness."""
    import statistics

    from regression_retention import build_env, build_policy, evaluate_scene, load_weights
    from scene_feasibility import get_metric_names, load_template

    device = torch.device("cpu")
    metric_names, metric_index = get_metric_names()
    scene = sorted(SCENE_DIR.glob("*.bin"))[0]
    template = load_template("lane_departure")

    env = build_env(scene)
    policy = build_policy(env, device)
    load_weights(policy, CHECKPOINT, device)

    result = evaluate_scene(policy, scene, template, metric_names, metric_index, device)
    assert result["median_speed"] > 0.5, (
        f"harness median speed {result['median_speed']:.2f} m/s — policy is "
        "parked; action selection or env contract diverged from training again"
    )
    assert statistics.median([result["median_speed"]]) > 0.5  # keep the number a float
