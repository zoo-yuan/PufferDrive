"""Parity between PufferDrive's ``model_euler`` control mode and nova's integrator.

Why this test exists
--------------------
`ACTION_TYPE_CONTROL` + `integration_scheme="model_euler"` exist so a
trajectory-level planner's control sequence ``(a_long, kappa)`` executes in the
simulator exactly as the planner integrated it. If the two drift, every critic
label (collision / offroad / comfort) is computed on a state the planner never
intended - a systematic bias that produces no error message.

Reference
---------
``nova/_integrate_control`` (``diffusion_head.py``) is a semi-implicit Euler at
dt = 0.1 s::

    v_k     = v_0 + sum(acc * dt)
    yaw_k   = yaw_0 + sum(kappa * v * dt)
    x_k     = x_0 + sum(v * cos(yaw) * dt)
    y_k     = y_0 + sum(v * sin(yaw) * dt)

It also applies an **acceleration wash-in at inference**: the first four
acceleration samples are interpolated from the current acceleration with weights
0.2/0.4/0.6/0.8 towards ``2 * mean(acc[:5] - a0) + a0``. That preprocessing is
NOT part of the simulator; the bridge must apply it before feeding the
simulator, and ``test_raw_control_diverges_from_model_intent`` measures the cost
of forgetting it.

Gates: the C side accumulates in float32, so the floor is ~1e-5 m over 6 s.
"""

import math

import numpy as np
import pytest

from pufferlib.ocean.drive.drive import Drive

DT = 0.1
NOVA_ACC_WASH_IN_SCALE = np.array([0.2, 0.4, 0.6, 0.8], dtype=np.float32)
PARITY_STEPS = 60
MAX_XY_ERROR_M = 1e-3
MAX_HEADING_ERROR_RAD = 1e-5
MAX_SPEED_ERROR_MPS = 1e-5


def nova_integrate_control(initial_state, control, inter_acc):
    """Float32 port of nova's ``_integrate_control`` (values only, no autograd)."""
    x0, y0, heading0, speed0, accel0 = (np.float32(value) for value in initial_state)
    accel = control[:, 0].astype(np.float32).copy()
    kappa = control[:, 1].astype(np.float32).copy()

    if inter_acc:
        target = np.float32(2.0) * np.mean(accel[:5] - accel0) + accel0
        accel[:4] = accel0 + NOVA_ACC_WASH_IN_SCALE * (target - accel0)

    speed = speed0 + np.cumsum(accel, dtype=np.float32) * np.float32(DT)
    heading = heading0 + np.cumsum(kappa * speed, dtype=np.float32) * np.float32(DT)
    x = x0 + np.cumsum(speed * np.cos(heading), dtype=np.float32) * np.float32(DT)
    y = y0 + np.cumsum(speed * np.sin(heading), dtype=np.float32) * np.float32(DT)
    return np.stack([x, y, heading, speed], axis=1), accel


def make_env(seed):
    return Drive(
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
        scenario_length=PARITY_STEPS + 50,
        resample_frequency=0,
        simulation_mode="gigaflow",
        control_mode="control_vehicles",
        seed=seed,
    )


def read_agent(env):
    state = env.get_state()
    scenario = state[0] if isinstance(state, list) else state
    agent = (scenario.get("agents") or [])[0]
    heading = agent["sim_heading"]
    signed_speed = agent["sim_vx"] * math.cos(heading) + agent["sim_vy"] * math.sin(heading)
    return (
        agent["sim_x"],
        agent["sim_y"],
        heading,
        signed_speed,
        agent["accel_long"],
        agent["accel_lat"],
    )


def run_controls(env, controls):
    states = []
    for step in range(len(controls)):
        env.step(controls[step].reshape(1, 2).astype(np.float32))
        x, y, heading, speed, _accel_long, _accel_lat = read_agent(env)
        states.append((x, y, heading, speed))
    return np.array(states, dtype=np.float64)


def assert_parity(got, reference):
    xy_error = np.hypot(got[:, 0] - reference[:, 0], got[:, 1] - reference[:, 1])
    heading_error = np.abs((got[:, 2] - reference[:, 2] + np.pi) % (2.0 * np.pi) - np.pi)
    speed_error = np.abs(got[:, 3] - reference[:, 3])
    assert xy_error.max() < MAX_XY_ERROR_M, f"max |dxy| = {xy_error.max():.3e} m"
    assert heading_error.max() < MAX_HEADING_ERROR_RAD, f"max |dheading| = {heading_error.max():.3e} rad"
    assert speed_error.max() < MAX_SPEED_ERROR_MPS, f"max |dv| = {speed_error.max():.3e} m/s"


def sample_controls(seed):
    rng = np.random.default_rng(seed)
    accel = rng.uniform(-2.0, 1.5, size=PARITY_STEPS).astype(np.float32)
    kappa = rng.uniform(-0.08, 0.08, size=PARITY_STEPS).astype(np.float32)
    return np.stack([accel, kappa], axis=1)


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_model_euler_matches_nova_integrator(seed):
    controls = sample_controls(seed)
    env = make_env(seed)
    env.reset()
    initial = read_agent(env)[:5]
    reference, _accel = nova_integrate_control(initial, controls, inter_acc=False)
    got = run_controls(env, controls)
    assert_parity(got, reference)


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_acceleration_wash_in_reproduces_model_intent(seed):
    controls = sample_controls(seed)
    env = make_env(seed)
    env.reset()
    initial = read_agent(env)[:5]
    reference, washed_accel = nova_integrate_control(initial, controls, inter_acc=True)
    got = run_controls(env, np.stack([washed_accel, controls[:, 1]], axis=1))
    assert_parity(got, reference)


def test_raw_control_diverges_from_model_intent():
    """Documents why the bridge must preprocess: raw acc is not what the model integrates."""
    controls = sample_controls(0)
    env = make_env(0)
    env.reset()
    initial = read_agent(env)[:5]
    reference, _washed = nova_integrate_control(initial, controls, inter_acc=True)
    got = run_controls(env, controls)
    xy_error = np.hypot(got[:, 0] - reference[:, 0], got[:, 1] - reference[:, 1])
    assert xy_error.max() > 0.1, (
        "raw acceleration unexpectedly matched the model intent; "
        "either nova dropped its wash-in or the bridge is applying it twice"
    )


# --- 桥接层（生产代码路径）--------------------------------------------------
# 上面的 test_acceleration_wash_in_reproduces_model_intent 用测试内的实现验证语义；
# 下面这几条验证 pufferlib/ocean/drive/control_bridge.py 就是那个实现，且边界行为明确。

def test_bridge_wash_in_reproduces_model_intent(seed=0):
    from pufferlib.ocean.drive.control_bridge import build_control_actions

    controls = sample_controls(seed)
    env = make_env(seed)
    env.reset()
    initial = read_agent(env)[:5]
    reference, _washed = nova_integrate_control(initial, controls, inter_acc=True)
    actions = build_control_actions(controls[:, 0], controls[:, 1], initial[4])
    got = run_controls(env, actions)
    assert_parity(got, reference)


def test_bridge_wash_in_touches_only_the_first_four_samples():
    from pufferlib.ocean.drive.control_bridge import apply_acc_wash_in

    accel = np.array([1.0, 2.0, 3.0, 4.0, 0.5, 0.6], dtype=np.float32)
    washed = apply_acc_wash_in(accel, 0.0)
    assert np.array_equal(washed[4:], accel[4:]), "wash-in must not touch samples past the first four"


def test_bridge_wash_in_is_identity_for_constant_acceleration():
    """全序列等于当前加速度时，target == a0，wash-in 不该改动任何东西。"""
    from pufferlib.ocean.drive.control_bridge import apply_acc_wash_in

    accel = np.full(6, -1.5, dtype=np.float32)
    washed = apply_acc_wash_in(accel, -1.5)
    assert np.allclose(washed, accel, atol=1e-7)


def test_bridge_wash_in_handles_short_sequences():
    """不足 4 个样本、或不足 5 个样本时不能崩，也不能读到越界。"""
    from pufferlib.ocean.drive.control_bridge import apply_acc_wash_in

    for length in (1, 2, 3):
        accel = np.arange(1, length + 1, dtype=np.float32)
        washed = apply_acc_wash_in(accel, 0.0)
        assert washed.shape == accel.shape
        assert np.all(np.isfinite(washed))


def test_build_control_actions_rejects_mismatched_lengths():
    import pytest as _pytest

    from pufferlib.ocean.drive.control_bridge import build_control_actions

    with _pytest.raises(ValueError):
        build_control_actions(np.zeros(5), np.zeros(4), 0.0)
