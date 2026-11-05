"""Observation envelope: partner range and observation delay must actually reach the env.

Why this test exists
--------------------
`docs/vehicle_profile.md` §4.2 measured two numbers on six real records:

* the AD stack never perceives beyond ~70 m (per-bag per-message max p50 = 66.2 m,
  p100 median = 73.4 m, zero obstacles beyond 100 m across 89,304 samples);
* the control side acts on data that is 0.78 s old (median; range 0.41-1.21 s).

The simulator defaults were 200 m and 0 s. A zero-delay, 200 m simulator is
strictly better informed than the car, so policies learn to be more aggressive
than the vehicle can be - and the gap never shows up in simulation.

These tests pin the *mechanism* (delay ring semantics + range plumbing), not the
calibrated numbers.

Gates: delay is exact for t >= K (bitwise), warmup returns the reset observation,
and a zero delay is a no-op (no ring allocated, no copies).
"""

import numpy as np
import pytest
import yaml

from pufferlib.ocean.drive.drive import Drive

DT = 0.1
ENV_STEPS = 12


def make_env(seed, **overrides):
    kwargs = dict(
        map_dir="pufferlib/resources/drive/binaries/carla",
        num_maps=8,
        num_agents=1,
        min_agents_per_env=1,
        max_agents_per_env=1,
        action_type="control",
        integration_scheme="model_euler",
        skip_actuator_limits=True,
        dynamics_model="jerk",
        dt=DT,
        scenario_length=60,
        resample_frequency=0,
        simulation_mode="gigaflow",
        control_mode="control_vehicles",
        seed=seed,
    )
    kwargs.update(overrides)
    return Drive(**kwargs)


def rollout(env, seed):
    """Same seed + same action stream for every env, so raw trajectories are identical."""
    env.reset(seed=seed)
    rng = np.random.default_rng(0)
    observations = []
    for _ in range(ENV_STEPS):
        actions = rng.uniform(-1.0, 1.0, size=(env.num_agents, 2)).astype(np.float32)
        observations.append(env.step(actions)[0].copy())
    return np.array(observations)


def test_delay_zero_allocates_no_ring():
    env = make_env(7, observation_delay_s=0.0)
    assert env.observation_delay_steps == 0
    assert env._obs_delay_ring is None
    assert env.effective_observation_delay_s == 0.0


def test_delay_is_quantised_to_dt_and_reported():
    env = make_env(7, observation_delay_s=0.53)
    assert env.observation_delay_steps == 5  # round(0.53 / 0.1)
    assert env.effective_observation_delay_s == pytest.approx(0.5)


def test_delayed_observation_matches_older_fresh_observation():
    raw = rollout(make_env(7, observation_delay_s=0.0), seed=7)
    delayed_env = make_env(7, observation_delay_s=0.5)
    delayed = rollout(delayed_env, seed=7)
    delay_steps = delayed_env.observation_delay_steps
    assert delay_steps == 5

    # Warmup: the ring is seeded with the reset observation, so the policy sees the
    # initial state (never zeros, never a future state).
    for step in range(delay_steps):
        assert not np.array_equal(delayed[step], raw[0])

    # Steady state: bitwise equal to the observation that was fresh `delay_steps` ago.
    for step in range(delay_steps, ENV_STEPS):
        assert np.array_equal(delayed[step], raw[step - delay_steps]), f"step {step}"


def test_reset_rewarms_the_delay_ring():
    env = make_env(7, observation_delay_s=0.5)
    rollout(env, seed=7)
    first_after_reset = env.reset(seed=7)[0].copy()
    observations = [env.step(np.zeros((env.num_agents, 2), dtype=np.float32))[0].copy() for _ in range(3)]
    for observation in observations:
        assert np.array_equal(observation, first_after_reset)


def partner_block(env, observation):
    start = env.ego_features + env.num_reward_coefs + env.goal_dim
    end = start + env.obs_slots_partners_n * env.partner_features
    return observation[..., start:end]


def test_partner_range_override_reaches_the_env():
    # 8 agents per env so partners exist at all; 1 m cannot contain any of them.
    narrow = make_env(7, obs_range_partner_m=1.0, num_agents=8, min_agents_per_env=8, max_agents_per_env=8)
    wide = make_env(7, obs_range_partner_m=200.0, num_agents=8, min_agents_per_env=8, max_agents_per_env=8)
    assert narrow.obs_range_partner_m == 1.0
    assert wide.obs_range_partner_m == 200.0
    narrow_raw = rollout(narrow, seed=7)
    wide_raw = rollout(wide, seed=7)
    narrow_partners = partner_block(narrow, narrow_raw)
    wide_partners = partner_block(wide, wide_raw)
    assert not np.any(narrow_partners), "1 m partner range must hide every partner"
    assert np.any(wide_partners), "200 m partner range must expose at least one partner"


def test_puffer_drive_config_carries_the_measured_envelope():
    with open("pufferlib/config/puffer_drive.yaml", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    env_config = config["env"]
    assert env_config["obs_range_partner_m"] == 70.0
    assert env_config["observation_delay_s"] == pytest.approx(0.78)
