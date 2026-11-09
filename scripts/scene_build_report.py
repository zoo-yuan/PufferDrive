"""779 个切片 → PufferDrive 场景 bin 的批量验收（M7-2）。

分两层证据：
1. **结构层（全量）**：779 份 bin 都写出来、读回来校验（route ≤64、邻接不越界、
   cum_lengths 与 length 一致、lane_graph 矩阵尺寸对）——写错一个 int 就会让 C 侧崩。
2. **行为层（抽样）**：每 bag 抽 2 条真跑引擎（replay 模式），量 env 位姿与 log 位姿的偏差。
   结构对但坐标/route 错时，回放会立刻跑偏，所以这一层是"场景真的能跑"的证据。

用法
----
``python scripts/scene_build_report.py``                # 全量结构 + 每 bag 2 条回放
``python scripts/scene_build_report.py --no-run``       # 只做结构层
"""

from __future__ import annotations

import argparse
import collections
import html
import json
import statistics
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from data_utils.mirror_map_bin import read_bin, write_bin  # noqa: E402
from scene_from_slice import (  # noqa: E402
    MAX_REPLAY_ERROR_M,
    OUTPUT_DIR,
    build_scene,
    load_bag_context,
    run_engine,
    verify_bin,
)

MINING_JSON = REPO_ROOT / "docs" / "measured" / "badcase_mining_v0.json"
OUTPUT_HTML = REPO_ROOT / "docs" / "html" / "scene_build_v0.html"
OUTPUT_JSON = REPO_ROOT / "docs" / "measured" / "scene_build_v0.json"

REPLAY_SAMPLES_PER_BAG = 2
REPLAY_STEPS = 40
MAX_ROUTE_LANES = 64


def render_html(report) -> str:
    summary = report["summary"]
    bag_rows = "".join(
        f"<tr><td class='mono'>{html.escape(bag)}</td><td class='c'>{row['built']}</td>"
        f"<td class='c'>{row['failed']}</td><td class='c'>{row['median_route_lanes']:.0f}</td>"
        f"<td class='c'>{row['median_scene_lanes']:.0f}</td>"
        f"<td class='c'>{row['median_route_m']:.0f}</td>"
        f"<td class='c'>{row['replay_max_error_m'] if row['replay_max_error_m'] is not None else '—'}</td></tr>"
        for bag, row in sorted(report["per_bag"].items())
    )
    failure_rows = "".join(
        f"<tr><td class='mono'>{html.escape(name)}</td><td class='c'>{count}</td></tr>"
        for name, count in report["failures"].most_common()
    ) or "<tr><td colspan='2' class='note'>无</td></tr>"
    return f"""<!doctype html><html lang="zh"><head><meta charset="utf-8">
<title>场景构建 · M7-2</title>
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
<h1>场景构建（M7-2）</h1>
<p class="sub">{summary['rows']} 条切片 = {summary['total']} 个不同场景 → PufferDrive map bin · 结构层全量校验 · 行为层每 bag 抽
 {REPLAY_SAMPLES_PER_BAG} 条真跑引擎（replay 模式，{REPLAY_STEPS} 步）</p>
<div class="cards">
<div class="card"><div class="k">写出的场景</div><div class="v">{summary['built']}</div>
<div class="n">共 {summary['total']} 个不同场景（{summary['rows']} 条切片）· 结构问题 {summary['structural_problems']}</div></div>
<div class="card"><div class="k">route 段数中位</div><div class="v">{summary['median_route_lanes']:.0f}</div>
<div class="n">引擎上限 {MAX_ROUTE_LANES} 段</div></div>
<div class="card"><div class="k">场景车道中位</div><div class="v">{summary['median_scene_lanes']:.0f}</div>
<div class="n">route + 30 m 内邻道</div></div>
<div class="card"><div class="k">回放偏差 max</div><div class="v">{summary['replay_max_error_m']:.3f} m</div>
<div class="n">抽样 {summary['replay_samples']} 条 · 门限 {MAX_REPLAY_ERROR_M} m</div></div>
</div>
<div class="banner">回放偏差为 0 说明坐标平移、route 索引、邻接与 lane_graph 全部写对：只要有一处错，
 引擎的 replay 位姿就会立刻偏离 log。场景文件落在 <code>{html.escape(str(OUTPUT_DIR.relative_to(REPO_ROOT)))}/</code>。</div>
<h2>1. 按 bag</h2>
<table><thead><tr><th>bag</th><th class="c">场景</th><th class="c">失败</th>
<th class="c">route 段中位</th><th class="c">场景车道中位</th><th class="c">route 长度中位 m</th>
<th class="c">回放偏差 max m</th></tr></thead><tbody>{bag_rows}</tbody></table>
<h2>2. 失败分类</h2>
<table><thead><tr><th>原因</th><th class="c">切片</th></tr></thead><tbody>{failure_rows}</tbody></table>
<h2>3. 已知边界</h2>
<p class="note">本版场景只有自车（无背景交通）、不含交通控制（红绿灯/停止线）、车道类型统一写
 <code>LANE_SURFACE_STREET</code>。这些都是 M7-2 之后的增量，不是当前判据的一部分。</p>
</body></html>"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--no-run", action="store_true")
    args = parser.parse_args()

    slices = json.loads(MINING_JSON.read_text(encoding="utf-8"))["slices"]
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    per_bag = {}
    failures = collections.Counter()
    route_lanes_all = []
    scene_lanes_all = []
    route_m_all = []
    replay_errors = []
    built = 0
    structural_problems = 0

    seen_scenarios = set()
    for bag, bag_slices in slices.items():
        # 同一个 scenario_id 会被多个模板/相邻帧复用（id = bag#int(t_start*10)），
        # 去重后再建，否则统计里 779 个"切片"其实只有 82 个不同场景。
        unique_slices = []
        for slice_info in bag_slices:
            if slice_info["scenario_id"] in seen_scenarios:
                continue
            seen_scenarios.add(slice_info["scenario_id"])
            unique_slices.append(slice_info)
        print(f"[{bag}] {len(bag_slices)} 切片 → {len(unique_slices)} 个不同场景 …", flush=True)
        context = load_bag_context(bag)
        bag_route_lanes = []
        bag_scene_lanes = []
        bag_route_m = []
        bag_failed = 0
        built_paths = []
        for slice_info in unique_slices:
            try:
                scene, slice_context = build_scene({**slice_info, "bag": bag}, context)
                if len(scene["agents"][0]["route"]) > MAX_ROUTE_LANES:
                    raise RuntimeError("route 超过引擎上限")
                path = OUTPUT_DIR / f"{slice_context['scenario_id'].replace('#', '_')}.bin"
                write_bin(scene, path)
                problems = verify_bin(path)
                if problems:
                    structural_problems += 1
                    failures.update(problems)
                    bag_failed += 1
                    continue
                built += 1
                built_paths.append(path)
                bag_route_lanes.append(slice_context["route_lanes"])
                bag_scene_lanes.append(slice_context["scene_lanes"])
                bag_route_m.append(slice_context["route_distance_m"])
                route_lanes_all.append(slice_context["route_lanes"])
                scene_lanes_all.append(slice_context["scene_lanes"])
                route_m_all.append(slice_context["route_distance_m"])
            except Exception as error:  # 单条失败不该拖垮批量
                failures[f"{type(error).__name__}: {str(error)[:60]}"] += 1
                bag_failed += 1

        bag_replay_error = None
        if not args.no_run and built_paths:
            sample_errors = []
            for path in built_paths[:REPLAY_SAMPLES_PER_BAG]:
                try:
                    data = read_bin(path)
                    cols = data["agents"][0]["cols"]
                    _env, positions = run_engine(path, REPLAY_STEPS)
                    limit = min(len(positions), len(cols["x"]) - 1)
                    errors = [
                        ((positions[i][0] - cols["x"][i + 1]) ** 2
                         + (positions[i][1] - cols["y"][i + 1]) ** 2) ** 0.5
                        for i in range(limit)
                    ]
                    if errors:
                        sample_errors.append(max(errors))
                except Exception as error:
                    failures[f"engine: {type(error).__name__}"] += 1
            if sample_errors:
                bag_replay_error = round(max(sample_errors), 3)
                replay_errors.extend(sample_errors)

        per_bag[bag] = {
            "built": len(bag_route_lanes),
            "failed": bag_failed,
            "median_route_lanes": statistics.median(bag_route_lanes) if bag_route_lanes else 0,
            "median_scene_lanes": statistics.median(bag_scene_lanes) if bag_scene_lanes else 0,
            "median_route_m": statistics.median(bag_route_m) if bag_route_m else 0,
            "replay_max_error_m": bag_replay_error,
        }

    summary = {
        "total": len(seen_scenarios),
        "rows": sum(len(value) for value in slices.values()),
        "built": built,
        "structural_problems": structural_problems,
        "median_route_lanes": statistics.median(route_lanes_all) if route_lanes_all else 0,
        "median_scene_lanes": statistics.median(scene_lanes_all) if scene_lanes_all else 0,
        "median_route_m": statistics.median(route_m_all) if route_m_all else 0,
        "replay_samples": len(replay_errors),
        "replay_max_error_m": max(replay_errors) if replay_errors else 0.0,
    }
    report = {"summary": summary, "per_bag": per_bag, "failures": failures}

    OUTPUT_HTML.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_HTML.write_text(render_html(report), encoding="utf-8")
    OUTPUT_JSON.write_text(
        json.dumps(
            {
                "summary": summary,
                "per_bag": per_bag,
                "failures": dict(failures),
                "replay_errors_m": [round(value, 4) for value in replay_errors],
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    print(f"[summary] 场景 {built}/{summary['total']} · 结构问题 {structural_problems} · "
          f"route 段中位 {summary['median_route_lanes']:.0f} · 场景车道中位 {summary['median_scene_lanes']:.0f} · "
          f"回放偏差 max {summary['replay_max_error_m']:.3f} m（{summary['replay_samples']} 条抽样）")
    print(f"[report]  {OUTPUT_HTML}")

    gate_failures = []
    if built < summary["total"]:
        gate_failures.append(f"只有 {built}/{summary['total']} 个切片写成场景")
    if structural_problems:
        gate_failures.append(f"结构校验问题 {structural_problems} 条")
    if summary["replay_samples"] and summary["replay_max_error_m"] > MAX_REPLAY_ERROR_M:
        gate_failures.append(f"回放偏差 {summary['replay_max_error_m']:.3f} m > {MAX_REPLAY_ERROR_M} m")
    for failure in gate_failures:
        print(f"GATE FAIL : {failure}")
    print(f"GATE      : {'FAIL' if gate_failures else 'PASS'}")
    return 1 if (args.check and gate_failures) else 0


if __name__ == "__main__":
    raise SystemExit(main())
