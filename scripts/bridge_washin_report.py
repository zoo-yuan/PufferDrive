#!/usr/bin/env python3
"""桥接层 acc wash-in 的验收图：洗 vs 不洗，偏差随时间的曲线。

为什么需要这张图
----------------
`(acc, κ)` 序列在喂进仿真之前必须按 nova 推理的方式洗前 4 个加速度样本，否则 critic
会基于"模型从没打算去过"的状态打标签。这个偏差不报错，只让闭环信号整体偏移 ——
所以要用曲线把它显式钉住，而不是靠一句话说明。

判据（gate）
------------
- 洗过：6 s / 60 步内 max |Δxy| < 1e-4 m（float32 累积噪声是地板）
- 不洗：max |Δxy| > 0.1 m（否则说明这条测试已经失去分辨力）

用法:
    python scripts/bridge_washin_report.py
    python scripts/bridge_washin_report.py --check
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from pufferlib.ocean.drive.control_bridge import (  # noqa: E402
    NOVA_ACC_WASH_IN_SCALE,
    build_control_actions,
)
from pufferlib.ocean.drive.drive import Drive  # noqa: E402

DEFAULT_OUT = REPO_ROOT / "docs" / "html" / "bridge_washin_report.html"
DEFAULT_JSON = REPO_ROOT / "docs" / "measured" / "bridge_washin_v0.json"

DT = 0.1
STEPS = 60
MAX_WASHED_ERROR_M = 1e-4
MIN_RAW_ERROR_M = 0.1


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
        scenario_length=STEPS + 50,
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
    return (
        agent["sim_x"],
        agent["sim_y"],
        heading,
        agent["sim_vx"] * math.cos(heading) + agent["sim_vy"] * math.sin(heading),
        agent["accel_long"],
    )


def nova_integrate(initial_state, control, wash_in):
    """nova `_integrate_control` 的 float32 移植（半隐式 Euler）。"""
    x0, y0, heading0, speed0, accel0 = (np.float32(value) for value in initial_state)
    accel = control[:, 0].astype(np.float32).copy()
    kappa = control[:, 1].astype(np.float32).copy()
    if wash_in:
        target = np.float32(2.0) * np.mean(accel[:5] - accel0) + accel0
        accel[:4] = accel0 + NOVA_ACC_WASH_IN_SCALE * (target - accel0)
    speed = speed0 + np.cumsum(accel, dtype=np.float32) * np.float32(DT)
    heading = heading0 + np.cumsum(kappa * speed, dtype=np.float32) * np.float32(DT)
    x = x0 + np.cumsum(speed * np.cos(heading), dtype=np.float32) * np.float32(DT)
    y = y0 + np.cumsum(speed * np.sin(heading), dtype=np.float32) * np.float32(DT)
    return np.stack([x, y, heading, speed], axis=1)


def run(env, actions):
    states = []
    for step in range(actions.shape[0]):
        env.step(actions[step].reshape(1, 2).astype(np.float32))
        states.append(read_agent(env)[:4])
    return np.array(states, dtype=np.float64)


def sample_controls(seed):
    rng = np.random.default_rng(seed)
    accel = rng.uniform(-2.0, 1.5, size=STEPS).astype(np.float32)
    kappa = rng.uniform(-0.08, 0.08, size=STEPS).astype(np.float32)
    return np.stack([accel, kappa], axis=1)


def measure(seed):
    controls = sample_controls(seed)
    raw_env = make_env(seed)
    raw_env.reset(seed=seed)
    initial = read_agent(raw_env)[:5]

    reference = nova_integrate(initial, controls, wash_in=True)

    raw_env2 = make_env(seed)
    raw_env2.reset(seed=seed)
    raw_states = run(raw_env2, controls)

    washed_env = make_env(seed)
    washed_env.reset(seed=seed)
    washed_actions = build_control_actions(controls[:, 0], controls[:, 1], initial[4])
    washed_states = run(washed_env, washed_actions)

    raw_error = np.hypot(raw_states[:, 0] - reference[:, 0], raw_states[:, 1] - reference[:, 1])
    washed_error = np.hypot(washed_states[:, 0] - reference[:, 0], washed_states[:, 1] - reference[:, 1])
    return {
        "seed": seed,
        "raw_error_m": raw_error.tolist(),
        "washed_error_m": washed_error.tolist(),
        "raw_max_m": float(raw_error.max()),
        "washed_max_m": float(washed_error.max()),
    }


def line_chart(series, width=1080, height=336):
    """两条 Δxy 曲线（对数纵轴），标出 1e-4 m 判据线。"""
    steps = len(series[0][1])
    left, right, top, bottom = 84, 26, 26, 46
    plot_w = width - left - right
    plot_h = height - top - bottom
    floor = 1e-6
    ceiling = max(max(values) for _name, values, _color in series) * 2

    def y_of(value):
        value = max(value, floor)
        return top + plot_h * (1 - (math.log10(value) - math.log10(floor)) / (math.log10(ceiling) - math.log10(floor)))

    parts = [f'<svg viewBox="0 0 {width} {height}" width="100%" role="img">']
    for exponent in range(-6, int(math.log10(ceiling)) + 1):
        value = 10 ** exponent
        y = y_of(value)
        if not (top <= y <= top + plot_h):
            continue
        parts.append(
            f'<line x1="{left}" y1="{y:.1f}" x2="{width - right}" y2="{y:.1f}" stroke="#eaeef2" />'
            f'<text x="{left - 8}" y="{y + 4:.1f}" font-size="10" fill="#57606a" text-anchor="end">'
            f"{value:g}</text>"
        )
    for name, values, color in series:
        points = " ".join(
            f"{left + plot_w * index / max(1, steps - 1):.1f},{y_of(value):.1f}"
            for index, value in enumerate(values)
        )
        parts.append(
            f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2.2" />'
        )
    criterion_y = y_of(MAX_WASHED_ERROR_M)
    parts.append(
        f'<line x1="{left}" y1="{criterion_y:.1f}" x2="{width - right}" y2="{criterion_y:.1f}" '
        f'stroke="#1a7f37" stroke-dasharray="6 4" />'
        f'<text x="{width - right - 4}" y="{criterion_y - 6:.1f}" font-size="10.5" fill="#1a7f37" '
        f'text-anchor="end">判据 1e-4 m</text>'
    )
    for index, (name, _values, color) in enumerate(series):
        parts.append(
            f'<rect x="{left + index * 190}" y="{height - 48}" width="10" height="10" fill="{color}" />'
            f'<text x="{left + index * 190 + 15}" y="{height - 39}" font-size="11" fill="#1f2328">{esc(name)}</text>'
        )
    parts.append(
        f'<text x="{left}" y="{height - 16}" font-size="10.5" fill="#57606a">'
        f"横轴 0–{steps} 步（{steps * DT:.1f} s），纵轴对数：与 nova 积分参考的 |Δxy|（m）</text>"
    )
    parts.append("</svg>")
    return "".join(parts)


def esc(text):
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def render(results, failures):
    gate_state = "PASS" if not failures else "FAIL"
    gate_color = "#1a7f37" if not failures else "#cf222e"
    washed_max = max(result["washed_max_m"] for result in results)
    raw_max = max(result["raw_max_m"] for result in results)
    raw_min = min(result["raw_max_m"] for result in results)
    series = []
    colors = ["#cf222e", "#0b6bcb", "#9a6700"]
    for index, result in enumerate(results):
        series.append((f"不洗（seed {result['seed']}）", result["raw_error_m"], colors[index % len(colors)]))
    series.append(("洗过（桥接层）", results[0]["washed_error_m"], "#1a7f37"))

    parts = [f"""<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>桥接层 acc wash-in 验收</title>
<style>
:root {{ --ink:#1f2328; --mute:#57606a; --line:#d0d7de; --soft:#f6f8fa; }}
* {{ box-sizing:border-box; }}
body {{ margin:0; padding:32px 40px 56px; max-width:1240px; color:var(--ink);
  font:15px/1.6 -apple-system,"Segoe UI","PingFang SC","Microsoft YaHei",sans-serif; }}
h1 {{ font-size:26px; margin:0 0 6px; letter-spacing:-.3px; }}
h2 {{ font-size:19px; margin:34px 0 10px; padding-bottom:8px; border-bottom:1px solid var(--line); }}
.sub {{ color:var(--mute); font-size:13px; margin:0 0 18px; }}
.cards {{ display:grid; grid-template-columns:repeat(4,1fr); gap:12px; margin:18px 0 6px; }}
.card {{ border:1px solid var(--line); border-radius:10px; padding:13px 15px; background:var(--soft); }}
.card .k {{ font-size:12px; color:var(--mute); }}
.card .v {{ font-size:25px; font-weight:650; }}
.card .n {{ font-size:11.5px; color:var(--mute); }}
table {{ border-collapse:collapse; width:100%; font-size:13px; }}
th,td {{ border:1px solid var(--line); padding:5px 9px; }}
th {{ background:var(--soft); font-size:12px; text-align:left; }}
td.c,th.c {{ text-align:center; }}
code {{ background:var(--soft); border:1px solid var(--line); border-radius:4px; padding:1px 4px; font-size:11.5px; }}
.gate {{ display:inline-block; padding:3px 12px; border-radius:6px; font-size:13px; font-weight:700; color:#fff; background:{gate_color}; }}
.fail {{ border-left:3px solid #cf222e; background:#fff5f5; padding:9px 13px; border-radius:0 6px 6px 0; margin:7px 0; font-size:13px; }}
.ok {{ border-left:3px solid #1a7f37; background:#f2fbf4; padding:9px 13px; border-radius:0 6px 6px 0; margin:7px 0; font-size:13px; }}
.warn {{ border-left:3px solid #9a6700; background:#fffbf0; padding:9px 13px; border-radius:0 6px 6px 0; margin:7px 0; font-size:13px; }}
.note {{ color:var(--mute); font-size:12.5px; }}
</style></head><body>"""]
    parts.append(f"""
<h1>桥接层 acc wash-in 验收</h1>
<p class="sub">{STEPS} 步 / {STEPS * DT:.1f} s · 3 个种子 · 与 nova 积分参考对比 ·
gate <span class="gate">{gate_state}</span></p>""")

    parts.append('<div class="cards">')
    for key, value, note in [
        ("洗过 max |Δxy|", f"{washed_max:.1e} m", f"判据 < {MAX_WASHED_ERROR_M:g} m"),
        ("不洗 max |Δxy|", f"{raw_max:.2f} m", f"最低 {raw_min:.2f} m"),
        ("差值倍数", f"{raw_max / max(washed_max, 1e-12):.0f}×", "不洗比洗过差多少"),
        ("覆盖", "3 种子 × 60 步", "float32 累积噪声是地板"),
    ]:
        parts.append(
            f'<div class="card"><div class="k">{key}</div><div class="v">{value}</div>'
            f'<div class="n">{note}</div></div>'
        )
    parts.append("</div>")

    if failures:
        for item in failures:
            parts.append(f'<div class="fail">{esc(item)}</div>')
    else:
        parts.append(
            '<div class="ok">gate 通过：桥接层洗过之后与模型意图一致到 float32 噪声；'
            "不洗则偏差三个数量级之上 —— 说明这条测试仍有分辨力。</div>"
        )

    parts.append("<h2>1. |Δxy| 随时间</h2>")
    parts.append(line_chart(series))
    parts.append(
        '<p class="note">不洗的曲线从第 1 步就开始偏离（前 4 个加速度样本被直接使用），'
        "洗过之后全程贴着 float32 地板。</p>"
    )

    parts.append("<h2>2. 逐种子数字</h2>")
    parts.append(
        "<table><thead><tr><th>seed</th><th class='c'>不洗 max |Δxy|</th>"
        "<th class='c'>洗过 max |Δxy|</th><th class='c'>终点 |Δxy|（不洗）</th></tr></thead><tbody>"
    )
    for result in results:
        parts.append(
            f'<tr><td class="c">{result["seed"]}</td>'
            f'<td class="c">{result["raw_max_m"]:.3f} m</td>'
            f'<td class="c">{result["washed_max_m"]:.2e} m</td>'
            f'<td class="c">{result["raw_error_m"][-1]:.3f} m</td></tr>'
        )
    parts.append("</tbody></table>")
    parts.append(
        '<p class="note">公式：<code>target = 2·mean(acc[:5] − a0) + a0</code>，'
        "前 4 个样本按 0.2/0.4/0.6/0.8 插值。这是 nova 侧的既有行为，桥接层只复刻不改进。</p>"
    )
    parts.append(
        '<p class="sub" style="margin-top:30px">由 <code>scripts/bridge_washin_report.py</code> 生成。'
        "实现 <code>pufferlib/ocean/drive/control_bridge.py</code>。</p></body></html>"
    )
    return "".join(parts)


def main():
    parser = argparse.ArgumentParser(description="桥接层 wash-in 验收")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--json-out", type=Path, default=DEFAULT_JSON)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    results = [measure(seed) for seed in (0, 1, 2)]
    for result in results:
        print(
            f"[seed {result['seed']}] 不洗 max|Δxy| {result['raw_max_m']:.3f} m · "
            f"洗过 max|Δxy| {result['washed_max_m']:.2e} m"
        )

    failures = []
    for result in results:
        if result["washed_max_m"] > MAX_WASHED_ERROR_M:
            failures.append(
                f"seed {result['seed']}: 洗过 max|Δxy| {result['washed_max_m']:.2e} > {MAX_WASHED_ERROR_M:g}"
            )
        if result["raw_max_m"] < MIN_RAW_ERROR_M:
            failures.append(
                f"seed {result['seed']}: 不洗 max|Δxy| {result['raw_max_m']:.3f} < {MIN_RAW_ERROR_M:g}（测试失去分辨力）"
            )

    for item in failures:
        print(f"GATE FAIL : {item}")
    if not failures:
        print("GATE      : PASS")

    if not args.check:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(render(results, failures), encoding="utf-8")
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(
            json.dumps({"results": results, "gate": failures}, ensure_ascii=False, indent=1),
            encoding="utf-8",
        )
        print(f"report    : {args.out}")
        print(f"json      : {args.json_out}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
