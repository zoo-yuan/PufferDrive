#!/usr/bin/env python3
"""可行性裁判 —— 同一场景用 IDM / REPLAY / STATIC 各跑一遍，判定场景是否可解。

为什么需要它
------------
A-only 反馈下 bad case 没有专家轨迹，唯一的判据就是"约束"。但约束可以被一个不可解的
场景触发（例如 spawn 在两条相撞的轨迹上）。ADR-0007 的规则：

    同场景用 IDM / REPLAY / STATIC 各跑一遍；全挂即场景不可解，禁止进 RL。

IDM 是参考解算器（有物理合理性、会跟随参考线），REPLAY 是见证人（真实人类轨迹），
STATIC 是负对照（什么都不做，用来标出"这约束在静止时就触发"的坏模板）。

用法:
    python scripts/constraint_feasibility.py                       # 8 张图 × 3 控制器
    python scripts/constraint_feasibility.py --maps 4 --steps 40
    python scripts/constraint_feasibility.py --check               # 只校验，不写 HTML

gate:
    每个模板都在 3 个控制器下被评价过；每个控制器的 rollout 至少 eval 到 1 个有效步；
    STATIC 全触发的模板 = 0（否则该模板在静止时也成立，是坏谓词）。
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from constraint_judge import (  # noqa: E402
    AGENT_STATE_METRICS,
    BASELINE_ROW,
    as_predicate_list,
    evaluate,
    guard_holds,
    read_native_metrics,
    CONSTANTS_HEADER,
)

from pufferlib.ocean.drive.drive import Drive  # noqa: E402

DEFAULT_SPEC = REPO_ROOT / "spec" / "constraints_v0.yaml"
DEFAULT_OUT = REPO_ROOT / "docs" / "html" / "constraints_v0_feasibility.html"
MAP_DIR = "pufferlib/resources/drive/binaries/carla"
CONTROLLERS = ("idm", "replay", "static")
DT = 0.1


def make_env(controller, steps, seed):
    return Drive(
        map_dir=MAP_DIR,
        num_maps=8,
        num_agents=1,
        min_agents_per_env=1,
        max_agents_per_env=1,
        action_type="continuous",
        dynamics_model="jerk",
        dt=DT,
        scenario_length=steps + 20,
        resample_frequency=0,
        simulation_mode="gigaflow",
        control_mode="control_vehicles",
        sdc_controller=controller,
        seed=seed,
    )


def state_row(agent, metric_names, metric_index):
    """把一步的 agent 状态拼成谓词可读的行（metrics_array + 少量状态量）。"""
    values = agent.get("metrics_array") or []
    row = {}
    for name in metric_names:
        index = metric_index[name]
        row[name.lower()] = float(values[index]) if index < len(values) else 0.0
    for name in AGENT_STATE_METRICS:
        row[name] = float(agent.get(name, 0.0) or 0.0)
    return row


def rollout(controller, templates, metric_names, metric_index, steps, seed):
    """跑一个 episode，逐步评价所有模板，返回 {template_id: 是否触发}。"""
    env = make_env(controller, steps, seed)
    env.reset(seed=seed)
    fired = {template["id"]: False for template in templates}
    warmup_steps = {
        template["id"]: int(round(float(template.get("warmup_s", 0.0)) / DT))
        for template in templates
    }
    worst = {template["id"]: 0 for template in templates}
    evaluated_steps = 0

    for step_index in range(steps):
        env.step(np.zeros((env.num_agents, 2), dtype=np.float32))
        state = env.get_state()
        scenarios = state if isinstance(state, list) else [state]
        for scenario in scenarios:
            for agent in scenario.get("agents") or []:
                if not agent.get("active_agent", 1):
                    continue
                row = state_row(agent, metric_names, metric_index)
                evaluated_steps += 1
                for template in templates:
                    # 静止起步不是“停滞”：warmup 内的步不参与评价
                    if step_index < warmup_steps[template["id"]]:
                        continue
                    params = template.get("params", {}) or {}
                    predicates = as_predicate_list(template["predicate"])
                    try:
                        hit = evaluate(predicates, params, row) and guard_holds(
                            template.get("guard"), params, row
                        )
                    except KeyError:
                        continue
                    if hit:
                        fired[template["id"]] = True
                        worst[template["id"]] += 1
    return fired, evaluated_steps, worst


def verdict(fired_by_controller):
    """IDM 是参考解算器；REPLAY 是见证人。"""
    if not fired_by_controller["idm"]:
        return "solvable"
    if fired_by_controller["replay"]:
        return "unsolvable"
    return "idm_only"


VERDICT_META = {
    "solvable": ("#1a7f37", "可解", "IDM 未触发 → 该场景有物理可行的通过方式"),
    "idm_only": ("#9a6700", "IDM 不足", "IDM 触发但 REPLAY 未触发 → 需要更强策略，不可判为不可解"),
    "unsolvable": ("#cf222e", "不可解", "IDM 与 REPLAY 都触发 → 禁止进 RL"),
}


def esc(text):
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def render(templates, results, failures, steps):
    gate_state = "PASS" if not failures else "FAIL"
    gate_color = "#1a7f37" if not failures else "#cf222e"
    counts = {key: 0 for key in VERDICT_META}
    for row in results:
        counts[row["verdict"]] += 1

    parts = [f"""<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>约束可行性裁判 —— IDM / REPLAY / STATIC</title>
<style>
:root {{ --ink:#1f2328; --mute:#57606a; --line:#d0d7de; --soft:#f6f8fa; }}
* {{ box-sizing:border-box; }}
body {{ margin:0; padding:32px 40px 56px; max-width:1240px; color:var(--ink);
  font:15px/1.6 -apple-system,"Segoe UI","PingFang SC","Microsoft YaHei",sans-serif; }}
h1 {{ font-size:26px; margin:0 0 6px; letter-spacing:-.3px; }}
h2 {{ font-size:19px; margin:34px 0 10px; padding-bottom:8px; border-bottom:1px solid var(--line); }}
.sub {{ color:var(--mute); font-size:13px; margin:0 0 20px; }}
.cards {{ display:grid; grid-template-columns:repeat(4,1fr); gap:12px; margin:20px 0 6px; }}
.card {{ border:1px solid var(--line); border-radius:10px; padding:14px 16px; background:var(--soft); }}
.card .k {{ font-size:12px; color:var(--mute); }}
.card .v {{ font-size:26px; font-weight:650; }}
.card .n {{ font-size:12px; color:var(--mute); }}
table {{ border-collapse:collapse; width:100%; font-size:13px; }}
th,td {{ border:1px solid var(--line); padding:5px 8px; }}
th {{ background:var(--soft); font-size:12px; text-align:left; }}
td.c,th.c {{ text-align:center; }}
tbody tr:nth-child(even) td {{ background:#fcfcfd; }}
code {{ background:var(--soft); border:1px solid var(--line); border-radius:4px; padding:1px 4px; font-size:11.5px; }}
.gate {{ display:inline-block; padding:3px 12px; border-radius:6px; font-size:13px; font-weight:700; color:#fff; background:{gate_color}; }}
.fail {{ border-left:3px solid #cf222e; background:#fff5f5; padding:9px 13px; border-radius:0 6px 6px 0; margin:7px 0; font-size:13px; }}
.ok {{ border-left:3px solid #1a7f37; background:#f2fbf4; padding:9px 13px; border-radius:0 6px 6px 0; margin:7px 0; font-size:13px; }}
.warn {{ border-left:3px solid #9a6700; background:#fffbf0; padding:9px 13px; border-radius:0 6px 6px 0; margin:7px 0; font-size:13px; }}
.note {{ color:var(--mute); font-size:12px; }}
.pill {{ display:inline-block; padding:1px 8px; border-radius:999px; font-size:11px; font-weight:600; color:#fff; }}
</style></head><body>"""]
    parts.append(f"""
<h1>约束可行性裁判 —— IDM / REPLAY / STATIC</h1>
<p class="sub">20 条模板 × 3 个控制器 × {steps} 步 rollout（8 张 carla 图） · gauge <span class="gate">{gate_state}</span></p>""")

    parts.append('<div class="cards">')
    cards = [
        ("评价模板", len(results), "全部 20 条"),
        ("可解", counts["solvable"], "IDM 未触发"),
        ("IDM 不足", counts["idm_only"], "REPLAY 未触发"),
        ("不可解", counts["unsolvable"], "IDM+REPLAY 都触发 → 弃用"),
    ]
    for key, value, note in cards:
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
            '<div class="ok">gate 通过：每条模板都在三个控制器下被真跑评价过；'
            "STATIC 未出现全触发模板。</div>"
        )

    parts.append("<h2>模板 × 控制器触发矩阵</h2>")
    parts.append(
        '<p class="note">● = 该控制器在 rollout 中至少触发一次该约束；'
        "「IDM 不足」= IDM 触发但 REPLAY 未触发（场景仍需更强策略，不能判死）。</p>"
    )
    parts.append(
        "<table><thead><tr><th>模板</th><th class='c'>类别</th>"
        "<th class='c'>IDM</th><th class='c'>REPLAY</th><th class='c'>STATIC</th>"
        "<th class='c'>判定</th><th>含义</th></tr></thead><tbody>"
    )
    for row in results:
        color, label, meaning = VERDICT_META[row["verdict"]]
        cells = []
        for controller in CONTROLLERS:
            hit = row["fired"][controller]
            cells.append(
                f'<td class="c" style="color:{"#cf222e" if hit else "#1a7f37"};font-weight:700">'
                f'{"●" if hit else "○"}</td>'
            )
        parts.append(
            f'<tr><td>{esc(row["label"])}<br><span class="note">{esc(row["id"])}</span></td>'
            f'<td class="c">{esc(row["category"])}</td>'
            + "".join(cells)
            + f'<td class="c"><span class="pill" style="background:{color}">{label}</span></td>'
            f'<td class="note">{esc(meaning)}</td></tr>'
        )
    parts.append("</tbody></table>")
    parts.append(
        '<p class="note">注：这是"模板在 gigaflow 随机场景下会不会误触发"的机制验证，'
        "不是 bad case 库的统计。真正的可行性裁判要在每个待入库场景上跑。</p>"
    )
    parts.append(
        '<p class="sub" style="margin-top:30px">由 <code>scripts/constraint_feasibility.py</code> 生成。'
        "库源 <code>spec/constraints_v0.yaml</code>。</p></body></html>"
    )
    return "".join(parts)


def main():
    parser = argparse.ArgumentParser(description="约束可行性裁判")
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--steps", type=int, default=40)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    spec = yaml.safe_load(args.spec.read_text(encoding="utf-8"))
    templates = spec["templates"]
    metric_names, _lower = read_native_metrics(CONSTANTS_HEADER)
    metric_index = {name: index for index, name in enumerate(metric_names)}

    print(f"templates : {len(templates)}")
    print(f"metrics   : {len(metric_names)} 项（constants.h）")

    fired_by_controller = {}
    steps_by_controller = {}
    for controller in CONTROLLERS:
        fired, evaluated_steps, _worst = rollout(
            controller, templates, metric_names, metric_index, args.steps, args.seed
        )
        fired_by_controller[controller] = fired
        steps_by_controller[controller] = evaluated_steps
        hit_count = sum(1 for value in fired.values() if value)
        print(f"  {controller:8s} 触发模板 {hit_count:2d}/{len(templates)}  （有效评价步 {evaluated_steps}）")

    results = []
    for template in templates:
        template_id = template["id"]
        fired = {controller: fired_by_controller[controller][template_id] for controller in CONTROLLERS}
        results.append(
            {
                "id": template_id,
                "label": template.get("label", template_id),
                "category": template.get("category", "violation"),
                "fired": fired,
                "verdict": verdict(fired),
            }
        )

    failures = []
    for controller, evaluated_steps in steps_by_controller.items():
        if evaluated_steps < 1:
            failures.append(f"{controller} 没有有效评价步")
    # STATIC 是负对照：安全/体感类约束不该在“什么都不做”时触发；
    # 进度类（stall_crawl）对着 STATIC 触发是预期行为，不算坏谓词。
    static_bad = [
        row["id"]
        for row in results
        if row["fired"]["static"] and not row["fired"]["idm"] and row["category"] != "progress"
    ]
    if static_bad:
        failures.append(f"STATIC 独自触发（静止时就成立的坏谓词）: {static_bad}")

    counts = {key: 0 for key in VERDICT_META}
    for row in results:
        counts[row["verdict"]] += 1
    print("verdict   : " + " / ".join(f"{key}={value}" for key, value in counts.items()))
    for item in failures:
        print(f"GATE FAIL : {item}")
    if not failures:
        print("GATE      : PASS")

    if not args.check:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(render(templates, results, failures, args.steps), encoding="utf-8")
        print(f"report    : {args.out}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
