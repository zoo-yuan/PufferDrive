"""场景级可行性裁判（M7-3）：bag 侧抓到的 bad case，在仿真里还成立吗？

三层判定，逐切片做
------------------
1. **REPLAY**：ego 按 log 轨迹播。引擎自己的指标能不能把这条违规判出来？
   判不出 = bag 侧探测器与引擎指标口径不一致（这类切片不能用作训练信号）。
2. **IDM**：同一条 route 上换成理想跟驰模型。IDM 也违规 = 这个场景本身就逼人违规；
   IDM 不违规 = 违规是"这个驾驶员的选择"，场景可解。
3. **STATIC**：不给动作。STATIC 都触发 = 谓词写错了（与运动无关），必须回炉。

判定口径来自 `spec/constraints_v0.yaml`（与 M4 共用同一套谓词实现），不另写一套。

用法
----
``python scripts/scene_feasibility.py``                       # 默认 2714 的 80 条
``python scripts/scene_feasibility.py --bag 3183 --limit 20``
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
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from constraint_judge import (  # noqa: E402
    AGENT_STATE_METRICS,
    CONSTANTS_HEADER,
    as_predicate_list,
    evaluate,
    guard_holds,
    read_native_metrics,
)
from data_utils.mirror_map_bin import write_bin  # noqa: E402
from scene_from_slice import (  # noqa: E402
    OUTPUT_DIR,
    build_scene,
    load_bag_context,
)

MINING_JSON = REPO_ROOT / "docs" / "measured" / "badcase_mining_v0.json"
CONSTRAINT_SPEC = REPO_ROOT / "spec" / "constraints_v0.yaml"
OUTPUT_HTML = REPO_ROOT / "docs" / "html" / "scene_feasibility_v0.html"
OUTPUT_JSON = REPO_ROOT / "docs" / "measured" / "scene_feasibility_v0.json"

CONTROLLERS = ("replay", "idm", "static")
DT_S = 0.1
MAX_STEPS = 80
# 与挖掘契约的场景窗口一致：核心违规在 warmup 之后、horizon 之内
WARMUP_STEPS = 16
# 这些指标由动力学积分给出；replay 模式直接贴 log 位姿、不积分，所以恒为 0。
# 它们不能判成"未复现"，只能判成"replay 下不可测"。
DYNAMICS_ONLY_METRICS = {"accel_long", "accel_lat", "jerk_long", "jerk_lat"}
# 这些指标必须有别的交通参与者才可能非平凡。M7-2 的场景只有自车，所以它们判不出是**场景缺件**，
# 不是探测器或引擎的问题。
PARTNER_DEPENDENT_METRICS = {"ttc", "distance_to_collision", "collision", "at_fault_collision"}


def load_template(template_id):
    spec = yaml.safe_load(CONSTRAINT_SPEC.read_text(encoding="utf-8"))
    for template in spec["templates"]:
        if template["id"] == template_id:
            return template
    raise KeyError(f"{template_id}: not in constraints_v0.yaml")


def get_metric_names():
    """constants.h 给的是大写名，state_row 里用小写键 —— 索引表必须按小写建。

    这里曾经按大写建表、按小写查，KeyError 被下面的 except 吞掉，整批判成"未复现"。
    """
    names, _lower = read_native_metrics(CONSTANTS_HEADER)
    return names, {name.lower(): index for index, name in enumerate(names)}


def state_row(agent, metric_names, metric_index):
    values = agent.get("metrics_array") or []
    row = {}
    for name in metric_names:
        index = metric_index[name.lower()]
        row[name.lower()] = float(values[index]) if index < len(values) else 0.0
    for name in AGENT_STATE_METRICS:
        row[name] = float(agent.get(name, 0.0) or 0.0)
    return row


def run_controller(scene_path: Path, controller: str, steps: int, template, metric_names, metric_index):
    """跑一个 controller，返回触发情况（是否触发 / 首次触发步 / 峰值指标）。"""
    from pufferlib.ocean.drive.drive import Drive

    env = Drive(
        map_dir=str(scene_path),
        num_maps=1,
        num_agents=1,
        min_agents_per_env=1,
        max_agents_per_env=1,
        action_type="continuous",
        dynamics_model="jerk",
        dt=DT_S,
        scenario_length=steps,
        resample_frequency=0,
        simulation_mode="replay",
        control_mode="control_sdc_only",
        sdc_controller=controller,
        num_goals=3,
        min_goal_spacing=20.0,
        max_goal_spacing=200.0,
        seed=0,
    )
    env.reset(seed=0)
    # evaluate() 吃的是谓词清单；直接传 dict 会被当成可迭代的键（str），报一个很迷惑的类型错
    predicates = as_predicate_list(template["predicate"])
    metric_name = predicates[0]["metric"]
    fired_step = None
    peak = 0.0
    previous_positions = {}
    for step_index in range(steps):
        env.step(np.zeros((env.num_agents, 2), dtype=np.float32))
        state = env.get_state()
        scenarios = state if isinstance(state, list) else [state]
        for scenario in scenarios:
            for agent in scenario.get("agents") or []:
                row = state_row(agent, metric_names, metric_index)
                if controller == "replay":
                    # replay 直接贴 log 位姿，引擎的 sim_speed/accel/jerk 全是 0。
                    # 速度只能由相邻位姿反推，否则带移动门限的谓词在 replay 下永远不成立。
                    key = agent.get("id", 0)
                    previous = previous_positions.get(key)
                    if previous is not None:
                        moved = (
                            (agent["sim_x"] - previous[0]) ** 2 + (agent["sim_y"] - previous[1]) ** 2
                        ) ** 0.5
                        row["sim_speed"] = moved / DT_S
                    previous_positions[key] = (agent["sim_x"], agent["sim_y"])
                peak = max(peak, abs(row.get(metric_name, 0.0)))
                if step_index < WARMUP_STEPS:
                    continue
                if guard_holds(template.get("guard"), template.get("params", {}), row) and evaluate(
                    predicates, template.get("params", {}), row
                ):
                    if fired_step is None:
                        fired_step = step_index
    return {"fired": fired_step is not None, "fired_step": fired_step, "peak": peak}


def render_html(rows, summary) -> str:
    verdict_labels = {
        "reproduced_avoidable": "复现且可避（IDM 不违规）",
        "reproduced_unavoidable": "复现但 IDM 也违规",
        "not_reproduced": "引擎指标判不出",
        "metric_absent_in_replay": "replay 下该指标恒为 0（需动力学积分）",
        "needs_partners": "场景缺背景交通（M7-2 只有自车）",
        "static_fires": "STATIC 也触发（谓词与运动无关）",
    }
    verdict_rows = "".join(
        f"<tr><td>{html.escape(verdict_labels.get(name, name))}</td><td class='c'>{count}</td></tr>"
        for name, count in summary["verdicts"].most_common()
    )
    detail_rows = "".join(
        f"<tr><td class='mono'>{html.escape(row['scenario_id'])}</td>"
        f"<td class='c'>{row['replay']['peak']:.2f}</td>"
        f"<td class='c'>{'✓' if row['replay']['fired'] else '—'}</td>"
        f"<td class='c'>{'✓' if row['idm']['fired'] else '—'}</td>"
        f"<td class='c'>{'✓' if row['static']['fired'] else '—'}</td>"
        f"<td class='note'>{html.escape(verdict_labels.get(row['verdict'], row['verdict']))}</td></tr>"
        for row in rows[:400]
    )
    return f"""<!doctype html><html lang="zh"><head><meta charset="utf-8">
<title>场景级可行性裁判 · M7-3</title>
<style>
body{{font-family:-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,"PingFang SC",sans-serif;
max-width:1000px;margin:0 auto;padding:32px 24px 64px;color:#1f2328;line-height:1.6}}
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
.note{{color:#57606a;font-size:12px}}
.banner{{border-left:3px solid #1a7f37;background:#f0fbf2;padding:9px 13px;border-radius:0 6px 6px 0;font-size:13px;margin:16px 0}}
</style></head><body>
<h1>场景级可行性裁判（M7-3）</h1>
<p class="sub">{summary['template']} · {summary['total']} 个不同场景（{summary['rows_in_bag']} 条切片）·
 3 个 controller × {MAX_STEPS} 步 ·
 谓词与阈值来自 <code>spec/constraints_v0.yaml</code>（与 M4 同一套实现）</p>
<div class="cards">
<div class="card"><div class="k">复现率（可测切片）</div><div class="v">{summary['reproduced']}/{summary['testable']}</div>
<div class="n">{summary['reproduced'] / max(1, summary['testable']) * 100:.0f}% 在引擎里判得出；
 {summary['total'] - summary['testable']} 条不可测（comfort 无指标 / 缺对手）</div></div>
<div class="card"><div class="k">IDM 也违规</div><div class="v">{summary['idm_fires']}</div>
<div class="n">这些场景对理想驾驶员也难</div></div>
<div class="card"><div class="k">STATIC 触发（违规/体感）</div><div class="v">{summary['static_fires_motion']}</div>
<div class="n">&gt;0 说明谓词与运动无关；进度型另计 {summary['static_fires']}</div></div>
<div class="card"><div class="k">引擎判不出</div><div class="v">{summary['verdicts'].get('not_reproduced', 0)}</div>
<div class="n">探测器与引擎指标口径不一致，需逐条看</div></div>
</div>
<div class="banner">判定口径：REPLAY 判不出 = bag 侧探测器与引擎指标不一致，该切片不能当训练信号；
 IDM 不违规 = 违规来自驾驶员选择，场景可解；IDM 也违规 = 场景本身逼人违规。STATIC 触发属于谓词缺陷。</div>
<h2>1. 判定分布</h2>
<table><thead><tr><th>判定</th><th class="c">场景</th></tr></thead><tbody>{verdict_rows}</tbody></table>
<h2>2. 逐场景（前 400 条）</h2>
<table><thead><tr><th>scenario</th><th class="c">REPLAY 峰值</th><th class="c">REPLAY 触发</th>
<th class="c">IDM 触发</th><th class="c">STATIC 触发</th><th>判定</th></tr></thead>
<tbody>{detail_rows}</tbody></table>
</body></html>"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", default="3183")
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--steps", type=int, default=MAX_STEPS)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    mining = json.loads(MINING_JSON.read_text(encoding="utf-8"))
    bag_slices = mining["slices"][args.bag]
    # 同一 scenario_id 会被多个模板复用（id = bag#int(t_start*10)），去重后才是"场景"数
    unique_slices = []
    seen_scenarios = set()
    for slice_info in bag_slices:
        if slice_info["scenario_id"] in seen_scenarios:
            continue
        seen_scenarios.add(slice_info["scenario_id"])
        unique_slices.append(slice_info)
    bag_slices = unique_slices
    if args.limit:
        bag_slices = bag_slices[: args.limit]
    metric_names, metric_index = get_metric_names()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    rows = []
    errors = collections.Counter()
    verdicts = collections.Counter()
    context_cache = {}
    for slice_info in bag_slices:
        template = load_template(slice_info["template"])
        predicates = as_predicate_list(template["predicate"])
        threshold = template["params"][predicates[0]["threshold_param"]]["value"]

        scene_path = OUTPUT_DIR / f"{slice_info['scenario_id'].replace('#', '_')}.bin"
        if not scene_path.exists():
            context = context_cache.setdefault(args.bag, load_bag_context(args.bag))
            scene, _slice_context = build_scene({**slice_info, "bag": args.bag}, context)
            write_bin(scene, scene_path)

        result = {"scenario_id": slice_info["scenario_id"], "template": slice_info["template"]}
        for controller in CONTROLLERS:
            try:
                result[controller] = run_controller(
                    scene_path, controller, args.steps, template, metric_names, metric_index
                )
            except Exception as error:  # 单个场景跑不起来不该拖垮整批，但必须计数
                errors[f"{type(error).__name__}: {str(error)[:70]}"] += 1
                result[controller] = {"fired": False, "fired_step": None, "peak": 0.0,
                                      "error": f"{type(error).__name__}: {error}"}

        metric_name = as_predicate_list(template["predicate"])[0]["metric"]
        if result["static"]["fired"]:
            verdict = "static_fires"
        elif not result["replay"]["fired"] and metric_name in DYNAMICS_ONLY_METRICS and result["replay"]["peak"] == 0.0:
            verdict = "metric_absent_in_replay"
        elif not result["replay"]["fired"] and metric_name in PARTNER_DEPENDENT_METRICS:
            verdict = "needs_partners"
        elif not result["replay"]["fired"]:
            verdict = "not_reproduced"
        elif result["idm"]["fired"]:
            verdict = "reproduced_unavoidable"
        else:
            verdict = "reproduced_avoidable"
        result["verdict"] = verdict
        result["category"] = template["category"]
        verdicts[verdict] += 1
        rows.append(result)
        print(f"  {result['scenario_id']:>22} {result['template']:>22} replay={'✓' if result['replay']['fired'] else '·'} "
              f"idm={'✓' if result['idm']['fired'] else '·'} "
              f"static={'✓' if result['static']['fired'] else '·'}  peak={result['replay']['peak']:.2f} "
              f"→ {verdict}", flush=True)

    peaks = [row["replay"]["peak"] for row in rows]
    summary = {
        "bag": args.bag,
        "template": (
            rows[0]["template"]
            if len({row["template"] for row in rows}) == 1
            else f"混合 {len({row['template'] for row in rows})} 种模板"
        ),
        "total": len(rows),
        "rows_in_bag": len(mining["slices"][args.bag]),
        "reproduced": sum(1 for row in rows if row["replay"]["fired"]),
        "idm_fires": sum(1 for row in rows if row["idm"]["fired"]),
        "static_fires": sum(1 for row in rows if row["static"]["fired"]),
        # 进度型模板对"原地不动"的 STATIC 基线天然成立（没起步≠停滞），单独计，不算谓词缺陷
        "static_fires_motion": sum(
            1 for row in rows if row["static"]["fired"] and row["category"] in ("violation", "comfort")
        ),
        "median_peak": statistics.median(peaks) if peaks else 0.0,
        "threshold": threshold if rows else 0.0,
        "verdicts": verdicts,
        # 分母只能算"该测得出"的切片：comfort 在 replay 下无指标、ttc 类需要对手
        "testable": len(rows)
        - verdicts.get("metric_absent_in_replay", 0)
        - verdicts.get("needs_partners", 0),
        "errors": dict(errors),
    }

    OUTPUT_HTML.write_text(render_html(rows, summary), encoding="utf-8")
    OUTPUT_JSON.write_text(
        json.dumps(
            {"summary": {k: v for k, v in summary.items() if k != "verdicts"},
             "verdicts": dict(verdicts), "rows": rows},
            ensure_ascii=False, indent=2,
        ),
        encoding="utf-8",
    )
    print(f"[summary] 复现 {summary['reproduced']}/{summary['testable']}（可测）· 共 {summary['total']} 切片 · "
          f"IDM 违规 {summary['idm_fires']} · "
          f"STATIC 触发 {summary['static_fires']} · 峰值中位 {summary['median_peak']:.2f} m")
    print(f"[verdict] {dict(verdicts.most_common())}")
    print(f"[report]  {OUTPUT_HTML}")

    failures = []
    if errors:
        failures.append(f"controller 跑挂 {sum(errors.values())} 次：{dict(errors)}")
    if summary["static_fires_motion"]:
        failures.append(
            f"STATIC 触发 {summary['static_fires_motion']} 条违规/体感模板（谓词与运动无关，需回炉）"
        )
    if summary["reproduced"] < 0.8 * max(1, summary["testable"]):
        failures.append(f"复现率 {summary['reproduced']}/{summary['testable']} < 80%（可测切片）")
    for failure in failures:
        print(f"GATE FAIL : {failure}")
    print(f"GATE      : {'FAIL' if failures else 'PASS'}")
    return 1 if (args.check and failures) else 0


if __name__ == "__main__":
    raise SystemExit(main())
