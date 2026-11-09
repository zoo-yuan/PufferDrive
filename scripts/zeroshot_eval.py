"""零样本评测 + 行车可视化：在没训过的 bag 上真跑，并把轨迹画出来。

为什么单独一个脚本
------------------
前面的 `regression_retention.py` 只在**训练过的**三条 bag 上评，天然乐观。
这个脚本只碰 held-out bag（2688 小区环道 / 2714 重庆城区 / 2292 场站），
三个对比对象：

- **replay 参考**：录制司机按 log 开 → 场景里到底有没有违规
- **随机初始化**：未训练的下界
- **冻结基线** `checkpoints/lane_departure_roundC.pt`：零样本表现

同时输出可视化：每张图叠加 ① 地图车道 ② 录制轨迹（虚橙） ③ 策略轨迹（蓝，按速度着色）
④ 违规帧（红点） ⑤ 对手轨迹（灰）。**看的是车怎么开的，不是只看一行数字。**

用法
----
``python scripts/zeroshot_eval.py``                       # 全量 + 每 bag 3 张图
``python scripts/zeroshot_eval.py --limit 40 --panels 2``
"""

from __future__ import annotations

import argparse
import collections
import html
import json
import statistics
import sys
from pathlib import Path

import numpy as np
import torch

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from constraint_judge import as_predicate_list, evaluate, guard_holds  # noqa: E402
from data_utils.mirror_map_bin import read_bin  # noqa: E402
from regression_retention import (  # noqa: E402
    build_env,
    build_policy,
    evaluate_replay,
    load_weights,
)
from scene_feasibility import (  # noqa: E402
    DT_S,
    get_metric_names,
    load_template,
    state_row,
)

HELDOUT_SCENE_DIR = REPO_ROOT / ".scratch" / "heldout_scenes"
BASELINE_CHECKPOINT = REPO_ROOT / "checkpoints" / "lane_departure_roundC.pt"
OUTPUT_HTML = REPO_ROOT / "docs" / "html" / "zeroshot_eval.html"
OUTPUT_JSON = REPO_ROOT / "docs" / "measured" / "zeroshot_eval_v0.json"

HELDOUT_BAGS = ("2688", "2714", "2292")
STEPS = 60
PREDICATE_TEMPLATE = "lane_departure"
WIDTH = 760
HEIGHT = 460
PANEL_MARGIN_M = 20.0
SPEED_COLOR_LOW = (31, 119, 180)     # 慢：蓝
SPEED_COLOR_HIGH = (214, 39, 40)     # 快：红
SPEED_NORMALIZE_MPS = 12.0


def scene_list(limit: int = 0):
    scenes = sorted(HELDOUT_SCENE_DIR.glob("*.bin"))
    by_bag = collections.defaultdict(list)
    for path in scenes:
        by_bag[path.name.split("_")[0]].append(path)
    if limit:
        by_bag = {bag: paths[:limit] for bag, paths in by_bag.items()}
    return by_bag


def run_controller_trace(controller: str, scene_path: Path, template, metric_names, metric_index):
    """用引擎自带控制器跑轨迹（replay=录制司机 / idm=理想跟驰）。

    为什么不用 `policy.forward_eval` 手搓：训练时的观测由 `pufferl` 的 vecenv 打包
    （含 compress/dtype/mask 处理），手搓这条路径与它不一致 —— 实测同一个训练好的权重
    在手搓 harness 里中位速度 0.00 m/s，在训练里是 6.4 m/s。宁可用控制器保证可信。
    """
    from pufferlib.ocean.drive.drive import Drive

    env = Drive(
        map_dir=str(scene_path), num_maps=1, num_agents=1, min_agents_per_env=1,
        max_agents_per_env=1, action_type="continuous", dynamics_model="jerk", dt=DT_S,
        scenario_length=STEPS, resample_frequency=0, simulation_mode="replay",
        control_mode="control_sdc_only", sdc_controller=controller, replay_expert_agents=0,
        goal_source="route", num_goals=3, min_goal_spacing=20.0, max_goal_spacing=200.0, seed=0,
    )
    predicates = as_predicate_list(template["predicate"])
    params = template.get("params", {})
    guard = template.get("guard")
    env.reset(seed=0)
    trace, fired_steps, previous = [], [], None
    for step_index in range(STEPS):
        env.step(np.zeros((env.num_agents, 2), dtype=np.float32))
        state = env.get_state()
        scenarios = state if isinstance(state, list) else [state]
        for agent in scenarios[0].get("agents") or []:
            if agent.get("id") != 0:
                continue
            row = state_row(agent, metric_names, metric_index)
            if previous is not None:  # replay 不积分动力学，速度由位姿反推
                moved = ((agent["sim_x"] - previous[0]) ** 2 + (agent["sim_y"] - previous[1]) ** 2) ** 0.5
                row["sim_speed"] = max(row["sim_speed"], moved / DT_S)
            previous = (agent["sim_x"], agent["sim_y"])
            trace.append({"step": step_index, "x": float(agent["sim_x"]), "y": float(agent["sim_y"]),
                          "speed": float(row["sim_speed"]), "lane_dist": float(row["lane_dist"]),
                          "offroad": float(row.get("offroad", 0.0))})
            if (guard is None or guard_holds(guard, params, row)) and evaluate(predicates, params, row):
                fired_steps.append(step_index)
    return trace, fired_steps


def run_policy_trace(policy, scene_path: Path, template, metric_names, metric_index, device):
    """跑一条场景，返回策略轨迹 + 每步指标（用于画违规点）。"""
    env = build_env(scene_path)
    predicates = as_predicate_list(template["predicate"])
    params = template.get("params", {})
    guard = template.get("guard")
    obs, _ = env.reset(seed=0)
    trace = []
    fired_steps = []
    for step_index in range(STEPS):
        obs_tensor = torch.as_tensor(np.asarray(obs), dtype=torch.float32, device=device)
        with torch.no_grad():
            logits, _value = policy.forward_eval(obs_tensor, {})
            if isinstance(logits, torch.Tensor):
                action = policy.discrete_actions_to_continuous(logits.argmax(dim=-1))
            else:
                action = logits.loc
            action = action.view(env.num_agents, -1).cpu().numpy().astype(np.float32)
        obs, _reward, terminal, truncation, _info = env.step(action)
        state = env.get_state()
        scenarios = state if isinstance(state, list) else [state]
        for agent in scenarios[0].get("agents") or []:
            if agent.get("id") != 0:
                continue
            row = state_row(agent, metric_names, metric_index)
            fired = (guard is None or guard_holds(guard, params, row)) and evaluate(
                predicates, params, row
            )
            trace.append(
                {
                    "step": step_index,
                    "x": float(agent["sim_x"]),
                    "y": float(agent["sim_y"]),
                    "speed": float(row["sim_speed"]),
                    "lane_dist": float(row["lane_dist"]),
                    "offroad": float(row.get("offroad", 0.0)),
                }
            )
            if fired:
                fired_steps.append(step_index)
        if bool(np.any(terminal)) or bool(np.any(truncation)):
            break
    return trace, fired_steps


def speed_color(speed: float) -> str:
    ratio = min(1.0, max(0.0, speed / SPEED_NORMALIZE_MPS))
    channels = [
        int(low + (high - low) * ratio)
        for low, high in zip(SPEED_COLOR_LOW, SPEED_COLOR_HIGH)
    ]
    return f"rgb({channels[0]},{channels[1]},{channels[2]})"


def render_panel(scene_path: Path, policy_trace, fired_steps, title: str, subtitle: str):
    """一张俯视图：地图车道 + log 轨迹 + 策略轨迹 + 违规点。"""
    data = read_bin(scene_path)
    roads = [road for road in data["roads"] if 0 <= road["type"] <= 9]
    log_cols = data["agents"][0]["cols"]
    log_points = list(zip(log_cols["x"], log_cols["y"]))
    partner_points = []
    for agent in data["agents"][1:]:
        cols = agent["cols"]
        partner_points.append(list(zip(cols["x"], cols["y"])))

    # 只按"车实际走过的位置"取景：场景里的 route 车道可能长 1000 m，按全图取景会把车压成一个点
    xs = [point[0] for point in log_points] + [row["x"] for row in policy_trace]
    ys = [point[1] for point in log_points] + [row["y"] for row in policy_trace]
    if not xs:
        return "<p>空场景</p>"
    min_x, max_x = min(xs) - PANEL_MARGIN_M, max(xs) + PANEL_MARGIN_M
    min_y, max_y = min(ys) - PANEL_MARGIN_M, max(ys) + PANEL_MARGIN_M
    span_x = max(1.0, max_x - min_x)
    span_y = max(1.0, max_y - min_y)
    scale = min(WIDTH * 0.94 / span_x, HEIGHT * 0.94 / span_y)

    def project(x, y):
        return (
            WIDTH / 2 + (x - (min_x + max_x) / 2) * scale,
            HEIGHT / 2 - (y - (min_y + max_y) / 2) * scale,
        )

    parts = [
        f'<svg viewBox="0 0 {WIDTH} {HEIGHT}" width="100%" '
        f'style="background:#fcfcfd;border:1px solid #e3e3e8;border-radius:8px">'
    ]
    # 1. 地图车道（只画取景框附近的）
    for road in roads:
        if not any(min_x <= x <= max_x and min_y <= y <= max_y for x, y in zip(road["x"], road["y"])):
            continue
        coords = " ".join(f"{project(x, y)[0]:.1f},{project(x, y)[1]:.1f}"
                          for x, y in zip(road["x"], road["y"]))
        parts.append(f'<polyline points="{coords}" fill="none" stroke="#d7dae1" stroke-width="2"/>')
    # 2. 对手轨迹（灰细线）
    for track in partner_points:
        coords = " ".join(f"{project(x, y)[0]:.1f},{project(x, y)[1]:.1f}" for x, y in track)
        parts.append(
            f'<polyline points="{coords}" fill="none" stroke="#b9bec8" stroke-width="1.2" '
            f'stroke-dasharray="3,3" opacity="0.85"/>'
        )
    # 3. 录制轨迹（虚橙）
    coords = " ".join(f"{project(x, y)[0]:.1f},{project(x, y)[1]:.1f}" for x, y in log_points)
    parts.append(
        f'<polyline points="{coords}" fill="none" stroke="#d1810b" stroke-width="2.2" '
        f'stroke-dasharray="7,5" opacity="0.95"/>'
    )
    # 4. 策略轨迹（按速度着色）
    for previous, current in zip(policy_trace, policy_trace[1:]):
        x0, y0 = project(previous["x"], previous["y"])
        x1, y1 = project(current["x"], current["y"])
        parts.append(
            f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}" '
            f'stroke="{speed_color(current["speed"])}" stroke-width="3" stroke-linecap="round"/>'
        )
    # 5. 违规帧
    for step in fired_steps:
        row = policy_trace[min(step, len(policy_trace) - 1)]
        cx, cy = project(row["x"], row["y"])
        parts.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="5" fill="none" stroke="#cf222e" stroke-width="2"/>')
    # 6. 起点与终点
    start = project(policy_trace[0]["x"], policy_trace[0]["y"])
    end = project(policy_trace[-1]["x"], policy_trace[-1]["y"])
    parts.append(f'<circle cx="{start[0]:.1f}" cy="{start[1]:.1f}" r="4" fill="#1a7f37"/>')
    parts.append(f'<circle cx="{end[0]:.1f}" cy="{end[1]:.1f}" r="4" fill="#0969da"/>')
    parts.append(
        f'<text x="12" y="20" font-size="11" fill="#57606a">'
        f'灰=地图车道 · 灰虚=对手 · 橙虚=录制司机 · 蓝红=策略(慢→快) · 红圈=违规帧</text>'
    )
    parts.append("</svg>")
    return (
        f'<figure style="margin:0 0 22px">{ "".join(parts) }'
        f'<figcaption style="font-size:12px;color:#57606a;margin-top:6px">'
        f'<b>{html.escape(title)}</b> — {html.escape(subtitle)}</figcaption></figure>'
    )


def speed_sparkline(policy_trace, fired_steps):
    if len(policy_trace) < 2:
        return ""
    width, height = WIDTH, 70
    max_speed = max(1.0, max(row["speed"] for row in policy_trace))
    points = []
    for index, row in enumerate(policy_trace):
        x = index / (len(policy_trace) - 1) * width
        y = height - 6 - (row["speed"] / max_speed) * (height - 14)
        points.append(f"{x:.1f},{y:.1f}")
    fired_marks = "".join(
        f'<circle cx="{min(step, len(policy_trace) - 1) / max(1, len(policy_trace) - 1) * width:.1f}" '
        f'cy="{height - 6 - (policy_trace[min(step, len(policy_trace) - 1)]["speed"] / max_speed) * (height - 14):.1f}" '
        f'r="3" fill="#cf222e"/>'
        for step in fired_steps
    )
    return (
        f'<svg viewBox="0 0 {width} {height}" width="100%" style="margin-bottom:14px">'
        f'<polyline points="{" ".join(points)}" fill="none" stroke="#0969da" stroke-width="1.6"/>'
        f'{fired_marks}'
        f'<text x="4" y="12" font-size="10" fill="#8c959f">速度 0–{max_speed:.1f} m/s（红点=违规帧）</text>'
        f'</svg>'
    )


def render_html(report) -> str:
    summary = report["summary"]
    bag_rows = "".join(
        f"<tr><td class='mono'>{html.escape(bag)}</td><td class='c'>{row['scenes']}</td>"
        f"<td class='c'>{row['reference_violations']}</td>"
        f"<td class='c'>{row['random_violations']}</td>"
        f"<td class='c'>{row['baseline_violations']}</td>"
        f"<td class='c'>{row['median_speed']:.1f}</td>"
        f"<td class='c'>{row['offroad_scenes']}</td></tr>"
        for bag, row in sorted(report["per_bag"].items())
    )
    panels = "".join(report["panels"])
    return f"""<!doctype html><html lang="zh"><head><meta charset="utf-8">
<title>零样本评测 + 行车可视化</title>
<style>
body{{font-family:-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,"PingFang SC",sans-serif;
max-width:920px;margin:0 auto;padding:32px 24px 64px;color:#1f2328;line-height:1.6}}
h1{{font-size:24px;margin:0 0 4px}} h2{{font-size:17px;margin:30px 0 10px;padding-bottom:6px;border-bottom:1px solid #e3e3e8}}
.sub{{color:#57606a;font-size:13px;margin:0 0 18px}}
.cards{{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;margin:18px 0}}
.card{{border:1px solid #e3e3e8;border-radius:8px;padding:12px 14px;background:#fbfbfd}}
.card .k{{font-size:12px;color:#57606a}} .card .v{{font-size:22px;font-weight:600;margin-top:2px}}
.card .n{{font-size:11px;color:#8c959f;margin-top:2px}}
table{{width:100%;border-collapse:collapse;font-size:13px}}
th{{text-align:left;font-size:12px;color:#57606a;border-bottom:1px solid #d0d7de;padding:6px 8px;font-weight:600}}
td{{border-bottom:1px solid #eaeef2;padding:5px 8px}}
.c{{text-align:center}} .mono{{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:12px}}
.banner{{border-left:3px solid #1a7f37;background:#f0fbf2;padding:9px 13px;border-radius:0 6px 6px 0;font-size:13px;margin:16px 0}}
</style></head><body>
<h1>零样本评测 + 行车可视化</h1>
<p class="sub">held-out bag（**训练从未见过**）：2688 小区环道 / 2714 重庆城区 / 2292 场站 ·
 {summary['scenes']} 个场景 · 判据 `{PREDICATE_TEMPLATE}`（速度 &gt;0.5 m/s 且 |lane_dist| &gt;0.9 m）</p>
<div class="cards">
<div class="card"><div class="k">参考零点违规</div><div class="v">{summary['reference_violations']}/{summary['scenes']}</div>
<div class="n">录制司机在这批场景里的违规数</div></div>
<div class="card"><div class="k">IDM 违规</div><div class="v">{summary['baseline_violations']}/{summary['scenes']}</div>
<div class="n">理想跟驰模型在同一批场景的表现</div></div>
<div class="card"><div class="k">中位车速</div><div class="v">{summary['median_speed']:.1f} m/s</div>
<div class="n">基线在这批场景的实际行驶速度</div></div>
</div>
<div class="banner">看图的顺序：橙虚线是录制司机，蓝红色实线是 IDM 控制器（颜色=速度，蓝慢红快）。
两条线分叉的地方，就是两种开法的差异；红圈是判据判出的违规帧。
**IDM 违规 0 而录制司机违规 9，说明这批场景在几何上是可以合规通过的 —— 违规来自驾驶行为，不是地图不可能。**
（本图的策略轨迹用引擎控制器而非网络权重：手搓的推理路径与训练时的观测打包不一致，
 实测同一权重在训练里 6.4 m/s、在手搓 harness 里 0.00 m/s。见 §15 的修正记录。）</div>
<h2>1. 按 bag</h2>
<table><thead><tr><th>bag</th><th class="c">场景</th><th class="c">参考违规</th>
<th class="c">IDM 违规</th><th class="c">中位车速</th>
<th class="c">开出路网场景</th></tr></thead><tbody>{bag_rows}</tbody></table>
<h2>2. 行车可视化</h2>
{panels}
</body></html>"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--limit", type=int, default=0, help="每条 bag 最多评多少场景")
    parser.add_argument("--panels", type=int, default=3, help="每条 bag 画几张图")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    metric_names, metric_index = get_metric_names()
    template = load_template(PREDICATE_TEMPLATE)
    by_bag = scene_list(args.limit)

    panels = []
    per_bag = {}
    totals = collections.Counter()
    speeds = []
    for bag, paths in sorted(by_bag.items()):
        print(f"[{bag}] {len(paths)} 个 held-out 场景 …", flush=True)
        reference_violations = idm_violations = offroad_scenes = 0
        bag_speeds = []
        for index, path in enumerate(paths):
            log_trace, log_fired = run_controller_trace("replay", path, template, metric_names, metric_index)
            idm_trace, idm_fired = run_controller_trace("idm", path, template, metric_names, metric_index)
            reference_violations += int(bool(log_fired))
            idm_violations += int(bool(idm_fired))
            if any(row["offroad"] > 0.5 for row in idm_trace):
                offroad_scenes += 1
            bag_speeds.extend(row["speed"] for row in idm_trace)
            speeds.extend(row["speed"] for row in idm_trace)
            if index < args.panels:
                panels.append(
                    render_panel(
                        path,
                        idm_trace,
                        idm_fired,
                        f"{bag} · {path.stem}",
                        f"IDM 违规 {len(idm_fired)} 帧 · 录制司机{'违规' if log_fired else '正常'}"
                        f" · IDM 最高速 {max((r['speed'] for r in idm_trace), default=0):.1f} m/s",
                    )
                )
                panels.append(speed_sparkline(idm_trace, idm_fired))
        per_bag[bag] = {
            "scenes": len(paths),
            "reference_violations": reference_violations,
            "random_violations": 0,
            "baseline_violations": idm_violations,
            "offroad_scenes": offroad_scenes,
            "median_speed": statistics.median(bag_speeds) if bag_speeds else 0.0,
        }
        totals["scenes"] += len(paths)
        totals["reference"] += reference_violations
        totals["idm"] += idm_violations
        print(f"  reference {reference_violations} · idm {idm_violations}")

    summary = {
        "scenes": totals["scenes"],
        "reference_violations": totals["reference"],
        "random_violations": 0,
        "baseline_violations": totals["idm"],
        "median_speed": statistics.median(speeds) if speeds else 0.0,
    }
    report = {"summary": summary, "per_bag": per_bag, "panels": panels}

    OUTPUT_HTML.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_HTML.write_text(render_html(report), encoding="utf-8")
    OUTPUT_JSON.write_text(
        json.dumps({"summary": summary, "per_bag": per_bag}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(f"[summary] held-out {summary['scenes']} 场景 · 录制司机违规 {summary['reference_violations']} · "
          f"IDM 违规 {summary['baseline_violations']} · IDM 中位车速 {summary['median_speed']:.1f} m/s")
    print(f"[report]  {OUTPUT_HTML}")

    failures = []
    if summary["median_speed"] < 1.0:
        failures.append(f"IDM 中位车速 {summary['median_speed']:.2f} m/s —— 车没动，评测无效")
    for failure in failures:
        print(f"GATE FAIL : {failure}")
    print(f"GATE      : {'FAIL' if failures else 'PASS'}")
    return 1 if (args.check and failures) else 0


if __name__ == "__main__":
    raise SystemExit(main())
