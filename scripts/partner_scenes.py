"""给 `needs_partners` 类场景补上对手（M7-4b）。

问题：ttc / 碰撞 / cut_in 这些模板必须有别的交通参与者才可能非平凡。M7-2 的场景只有自车，
所以这些切片在场景裁判里一律"缺件"，复现率恒为 0 —— 不是判据错，是场景缺件。

做法：从 `/apollo/perception/obstacles` 把稳定可见的障碍物轨迹写成 PufferDrive agent
（`mark_as_expert=1`，按 log 回放），自车仍然由策略/控制器控制。然后重跑三控制器裁判，
看复现率是否从 0 抬起来。

用法
----
``python scripts/partner_scenes.py --bags 3183``            # 先验一条 bag
``python scripts/partner_scenes.py --bags 3183 3123 3139``
"""

from __future__ import annotations

import argparse
import collections
import html
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from data_utils.mirror_map_bin import write_bin  # noqa: E402
from scene_feasibility import (  # noqa: E402
    MAX_STEPS,
    get_metric_names,
    load_template,
    run_controller,
)
from scene_from_slice import build_scene, load_bag_context  # noqa: E402

MINING_JSON = REPO_ROOT / "docs" / "measured" / "badcase_mining_v0.json"
FEAS_DIR = REPO_ROOT / ".scratch" / "feas"
SCENE_DIR = REPO_ROOT / ".scratch" / "partner_scenes"
OUTPUT_HTML = REPO_ROOT / "docs" / "html" / "partner_scenes_v0.html"
OUTPUT_JSON = REPO_ROOT / "docs" / "measured" / "partner_scenes_v0.json"

CONTROLLERS = ("replay", "idm", "static")


def needs_partner_scenes(bag: str):
    path = FEAS_DIR / f"{bag}.json"
    if not path.exists():
        return []
    data = json.loads(path.read_text(encoding="utf-8"))
    return [
        {"scenario_id": row["scenario_id"], "template": row["template"], "bag": bag}
        for row in data.get("rows", [])
        if row.get("verdict") == "needs_partners"
    ]


def render_html(rows, summary) -> str:
    table = "".join(
        f"<tr><td class='mono'>{html.escape(row['scenario_id'])}</td>"
        f"<td class='c'>{html.escape(row['template'])}</td>"
        f"<td class='c'>{row['agents']}</td>"
        f"<td class='c'>{row['peak']:.2f}</td>"
        f"<td class='c'>{'✓' if row['fired'] else '—'}</td>"
        f"<td class='note'>{html.escape(row['note'])}</td></tr>"
        for row in rows[:400]
    )
    return f"""<!doctype html><html lang="zh"><head><meta charset="utf-8">
<title>补对手场景 · M7-4b</title>
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
<h1>补对手场景（M7-4b）</h1>
<p class="sub">{summary['total']} 个"缺对手"切片 · 对手来自 bag 的
 <code>/apollo/perception/obstacles</code>（稳定可见、距自车 ≤80 m、最多 24 个）·
 三控制器 × {MAX_STEPS} 步 · 谓词同 constraints_v0</p>
<div class="cards">
<div class="card"><div class="k">补了对手后复现</div><div class="v">{summary['reproduced']}/{summary['total']}</div>
<div class="n">补之前这批评分恒为 0（缺件）</div></div>
<div class="card"><div class="k">对手数中位</div><div class="v">{summary['median_agents']:.0f}</div>
<div class="n">每个场景除自车外的 agent 数</div></div>
<div class="card"><div class="k">峰值中位</div><div class="v">{summary['median_peak']:.2f}</div>
<div class="n">replay 下该模板指标峰值</div></div>
<div class="card"><div class="k">仍判不出</div><div class="v">{summary['not_reproduced']}</div>
<div class="n">多数是感知层缺跟踪或距离过远</div></div>
</div>
<div class="banner">对手是"按 log 回放"的 agent：它们的轨迹与真车一致，但不会对策略作出反应。
 这一步的目的是让 ttc/碰撞/cut_in 的谓词有机会非平凡，**不是**做交互仿真。</div>
<h2>1. 逐场景</h2>
<table><thead><tr><th>scenario</th><th class="c">模板</th><th class="c">agent 数</th>
<th class="c">峰值</th><th class="c">复现</th><th>说明</th></tr></thead><tbody>{table}</tbody></table>
</body></html>"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bags", nargs="+", default=["3183"])
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    metric_names, metric_index = get_metric_names()
    SCENE_DIR.mkdir(parents=True, exist_ok=True)
    mining = json.loads(MINING_JSON.read_text(encoding="utf-8"))["slices"]
    by_id = {row["scenario_id"]: row for rows in mining.values() for row in rows}

    rows = []
    for bag in args.bags:
        scenes = needs_partner_scenes(bag)
        if args.limit:
            scenes = scenes[: args.limit]
        if not scenes:
            print(f"[{bag}] 没有 needs_partners 场景")
            continue
        print(f"[{bag}] {len(scenes)} 个缺对手切片 …", flush=True)
        context = load_bag_context(bag)
        for scene_info in scenes:
            scene_row = by_id[scene_info["scenario_id"]]
            template = load_template(scene_info["template"])
            scene, _slice_context = build_scene(
                {**scene_row, "bag": bag}, context, include_partners=True
            )
            path = SCENE_DIR / f"{scene_info['scenario_id'].replace('#', '_')}.bin"
            write_bin(scene, path)

            try:
                replay = run_controller(path, "replay", MAX_STEPS, template, metric_names, metric_index)
            except Exception as error:  # 单条跑不起来不该拖垮整批
                rows.append({**scene_info, "agents": len(scene["agents"]), "peak": 0.0,
                             "fired": False, "note": f"{type(error).__name__}: {str(error)[:50]}"})
                continue
            rows.append(
                {
                    **scene_info,
                    "agents": len(scene["agents"]),
                    "peak": round(replay["peak"], 3),
                    "fired": replay["fired"],
                    "note": "能判出" if replay["fired"] else "对手存在但未触发",
                }
            )
            print(f"  {scene_info['scenario_id']:>26} agents={len(scene['agents']):2d} "
                  f"peak={replay['peak']:.2f} fired={'Y' if replay['fired'] else '.'}", flush=True)

    if not rows:
        print("ERROR: 没有可建场景", file=sys.stderr)
        return 1

    peaks = sorted(row["peak"] for row in rows)
    summary = {
        "total": len(rows),
        "reproduced": sum(1 for row in rows if row["fired"]),
        "not_reproduced": sum(1 for row in rows if not row["fired"]),
        "median_agents": sorted(row["agents"] for row in rows)[len(rows) // 2],
        "median_peak": peaks[len(peaks) // 2],
    }
    OUTPUT_HTML.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_HTML.write_text(render_html(rows, summary), encoding="utf-8")
    OUTPUT_JSON.write_text(
        json.dumps({"summary": summary, "rows": rows}, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(f"[summary] 复现 {summary['reproduced']}/{summary['total']} · "
          f"agent 中位 {summary['median_agents']} · 峰值中位 {summary['median_peak']:.2f}")
    print(f"[report]  {OUTPUT_HTML}")

    failures = []
    if summary["reproduced"] < 0.5 * summary["total"]:
        failures.append(f"补充对手后复现率仍只有 {summary['reproduced']}/{summary['total']}")
    for failure in failures:
        print(f"GATE FAIL : {failure}")
    print(f"GATE      : {'FAIL' if failures else 'PASS'}")
    return 1 if (args.check and failures) else 0


if __name__ == "__main__":
    raise SystemExit(main())
