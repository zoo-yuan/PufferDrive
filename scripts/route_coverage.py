"""779 个切片能不能串出 route？—— M7-1 的批量验收。

单条切片看图（``scripts/route_preview.py``）只能说明"口径对不对"，这个脚本回答"能覆盖多少"：
逐切片匹配自车车道 → 沿 ``successor_id`` 走到 ``ROUTE_TARGET_DISTANCE_M``，统计成功率与
失败原因分类（未匹配 / 断链 / 长度不足 / 换道缺失）。

性能要点：每条 bag 的 pose 只读一次、每张地图的索引只建一次（分块地图按整条 bag 的 bbox 选片），
切片之间复用，所以 779 条切片是分钟级而不是小时级。

用法
----
``python scripts/route_coverage.py``          # 生成报告
``python scripts/route_coverage.py --check``  # 失败退出码非 0
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

from data_utils.apollo_lane_index import TileLoader, load_lane_index_for_map_dir  # noqa: E402
from route_preview import (  # noqa: E402
    OUTPUT_JSON as PREVIEW_JSON,
    ROUTE_TARGET_DISTANCE_M,
    map_dir_for_bag,
    match_poses,
    read_all_poses,
    select_window,
    walk_route,
)

MINING_JSON = REPO_ROOT / "docs" / "measured" / "badcase_mining_v0.json"
OUTPUT_HTML = REPO_ROOT / "docs" / "html" / "route_coverage.html"
OUTPUT_JSON = REPO_ROOT / "docs" / "measured" / "route_coverage_v0.json"

# 判定"能跑":route 至少走到目标距离的这个比例。走不满说明拓扑在这段断了，
# 场景要么缩短目标距离，要么换 goal_source（PufferDrive 已有 GT_MAP）。
MIN_ROUTE_LENGTH_RATIO = 0.5
# 引擎侧 MAX_ROUTE_LENGTH = 64 段，3139 的车道平均只有 6.6 m（路口碎段），64 段只能走 425 m。
# 场景把 goal 设在 route 末端即可，所以 300 m 就算"能跑"，但两个口径分开报，不混在一起。
MIN_USABLE_DISTANCE_M = 300.0
MAX_UNMATCHED_RATE = 0.10


def load_slices():
    return json.loads(MINING_JSON.read_text(encoding="utf-8"))["slices"]


def bag_context(bag: str):
    """一条 bag 的 pose + 地图索引，切片之间复用。"""
    _bag_path, poses = read_all_poses(bag)
    map_dir = map_dir_for_bag(bag)
    bbox = (
        min(pose["x"] for pose in poses),
        min(pose["y"] for pose in poses),
        max(pose["x"] for pose in poses),
        max(pose["y"] for pose in poses),
    )
    index, source = load_lane_index_for_map_dir(map_dir, bbox)
    loader = TileLoader(map_dir) if source.startswith("base_map.catelog") else None
    return {"poses": poses, "index": index, "loader": loader, "source": source,
            "map_name": map_dir.name}


def analyze_slice(slice_info, context):
    poses, anchor_offset = select_window(
        context["poses"],
        slice_info["t_start_s"],
        int(slice_info["warmup_steps"]),
        int(slice_info["horizon_steps"]),
    )
    if len(poses) < 2:
        return {"status": "too_few_frames", "detail": f"{len(poses)} frames", "distance_m": 0.0}

    matched = match_poses(poses, context["index"])
    lane_ids = []
    for item in matched:
        if item["match"] is None:
            continue
        if not lane_ids or lane_ids[-1] != item["match"]["lane_id"]:
            lane_ids.append(item["match"]["lane_id"])
    unmatched_rate = sum(1 for item in matched if item["match"] is None) / len(matched)
    if not lane_ids:
        return {"status": "no_lane_match", "detail": "整窗未匹配", "distance_m": 0.0,
                "unmatched_rate": unmatched_rate}

    start_lane_id = lane_ids[0]
    start_lane = context["index"].by_id.get(start_lane_id)
    if start_lane is None:
        return {"status": "lane_missing_in_index", "detail": start_lane_id, "distance_m": 0.0}

    chain, steps, distance_m, stop_reason, branches = walk_route(
        start_lane, context["index"], context["loader"]
    )
    if distance_m >= ROUTE_TARGET_DISTANCE_M:
        status = "ok"
    elif distance_m >= MIN_USABLE_DISTANCE_M:
        status = "usable_short"
    else:
        status = "short_route"
    return {
        "status": status,
        "detail": stop_reason,
        "distance_m": round(distance_m, 1),
        "lane_count": len(steps),
        "branch_count": branches,
        "matched_lane_count": len(lane_ids),
        "unmatched_rate": round(unmatched_rate, 3),
        "anchor_offset": anchor_offset,
        "goal": {"x": float(chain[-1].points[-1][0]), "y": float(chain[-1].points[-1][1])},
    }


def render_html(rows, summary, per_template, per_bag) -> str:
    template_rows = "".join(
        f"<tr><td class='mono'>{html.escape(name)}</td><td class='c'>{total}</td>"
        f"<td class='c'>{ok}</td><td class='c'>{ok / total * 100:.0f}%</td>"
        f"<td class='c'>{total - ok}</td></tr>"
        for name, (total, ok) in sorted(per_template.items(), key=lambda item: -item[1][0])
    )
    bag_rows = "".join(
        f"<tr><td class='mono'>{html.escape(name)}</td><td class='c'>{total}</td>"
        f"<td class='c'>{ok}</td><td class='c'>{ok / total * 100:.0f}%</td>"
        f"<td class='c'>{median_distance:.0f}</td></tr>"
        for name, (total, ok, median_distance) in sorted(per_bag.items())
    )
    reason_rows = "".join(
        f"<tr><td class='mono'>{html.escape(name)}</td><td class='c'>{count}</td></tr>"
        for name, count in summary["reasons"].most_common()
    )
    return f"""<!doctype html><html lang="zh"><head><meta charset="utf-8">
<title>route 覆盖率 · M7-1</title>
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
.banner{{border-left:3px solid #1a7f37;background:#f0fbf2;padding:9px 13px;border-radius:0 6px 6px 0;font-size:13px;margin:16px 0}}
</style></head><body>
<h1>route 覆盖率（M7-1）</h1>
<p class="sub">{summary['total']} 个 bad case 切片 · 判定"能跑" = route 连续 ≥{MIN_USABLE_DISTANCE_M:.0f} m
 （引擎 <code>ROUTE_TARGET_DISTANCE</code> 是 {ROUTE_TARGET_DISTANCE_M:.0f} m，够不到时 goal 放 route 末端）
 · 未匹配帧 ≤{MAX_UNMATCHED_RATE:.0%}</p>
<div class="cards">
<div class="card"><div class="k">可跑切片</div><div class="v">{summary['ok']}</div>
<div class="n">共 {summary['total']} 条 · {summary['ok'] / max(1, summary['total']) * 100:.0f}%</div></div>
<div class="card"><div class="k">route 长度中位</div><div class="v">{summary['median_distance_m']:.0f} m</div>
<div class="n">p10 {summary['p10_distance_m']:.0f} m / p90 {summary['p90_distance_m']:.0f} m</div></div>
<div class="card"><div class="k">分支遭遇</div><div class="v">{summary['branch_slices']}</div>
<div class="n">切片 route 里遇到过分叉</div></div>
<div class="card"><div class="k">未匹配帧</div><div class="v">{summary['unmatched_rate']:.1%}</div>
<div class="n">超半径或航向不符 → 不出判定</div></div>
</div>
<div class="banner">结论：{'这批复用 route 口径能覆盖绝大多数切片' if summary['ok'] / max(1, summary['total']) > 0.8 else '覆盖率不足 80%，场景化前要先处理失败分类'}。
 失败原因分布见第 3 节 —— 每类都对应一个具体处置动作，不是笼统的"数据不够"。</div>
<h2>1. 按模板</h2>
<table><thead><tr><th>模板</th><th class="c">切片</th><th class="c">可跑</th><th class="c">覆盖率</th><th class="c">失败</th></tr></thead>
<tbody>{template_rows}</tbody></table>
<h2>2. 按 bag</h2>
<table><thead><tr><th>bag</th><th class="c">切片</th><th class="c">可跑</th><th class="c">覆盖率</th><th class="c">route 中位 m</th></tr></thead>
<tbody>{bag_rows}</tbody></table>
<h2>3. route 停止原因（含通过项）</h2>
<p class="sub">`ok_ok` = 走满 {ROUTE_TARGET_DISTANCE_M:.0f} m；`ok_usable_short` = 连续但短于目标（引擎 64 段上限或地图环道），
 场景把 goal 放在 route 末端即可；其余为失败。</p>
<table><thead><tr><th>原因</th><th class="c">切片</th></tr></thead><tbody>{reason_rows}</tbody></table>
</body></html>"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    slices = load_slices()
    rows = []
    per_template = collections.defaultdict(lambda: [0, 0])
    per_bag = collections.defaultdict(lambda: [0, 0, []])
    reasons = collections.Counter()
    distances = []
    branch_slices = 0
    unmatched_total = 0
    unmatched_frames = 0

    for bag, bag_slices in slices.items():
        print(f"[{bag}] {len(bag_slices)} 切片 …", flush=True)
        context = bag_context(bag)
        for slice_info in bag_slices:
            result = analyze_slice(slice_info, context)
            rows.append({"scenario_id": slice_info["scenario_id"], "bag": bag,
                         "template": slice_info["template"], **result})
            ok = (
                result["status"] in ("ok", "usable_short")
                and result.get("unmatched_rate", 0.0) <= MAX_UNMATCHED_RATE
            )
            per_template[slice_info["template"]][0] += 1
            per_template[slice_info["template"]][1] += int(ok)
            per_bag[bag][0] += 1
            per_bag[bag][1] += int(ok)
            per_bag[bag][2].append(result["distance_m"])
            reasons["ok_" + result["status"] if ok else result["detail"]] += 1
            distances.append(result["distance_m"])
            branch_slices += int(result.get("branch_count", 0) > 0)
            unmatched_total += 1
            unmatched_frames += result.get("unmatched_rate", 0.0)

    distances.sort()
    ok_count = sum(1 for row in rows if row["status"] in ("ok", "usable_short")
                   and row.get("unmatched_rate", 0.0) <= MAX_UNMATCHED_RATE)
    full_count = sum(1 for row in rows if row["status"] == "ok")
    summary = {
        "total": len(rows),
        "ok": ok_count,
        "full": full_count,
        "median_distance_m": distances[len(distances) // 2] if distances else 0.0,
        "p10_distance_m": distances[int(0.1 * (len(distances) - 1))] if distances else 0.0,
        "p90_distance_m": distances[int(0.9 * (len(distances) - 1))] if distances else 0.0,
        "branch_slices": branch_slices,
        "unmatched_rate": unmatched_frames / max(1, unmatched_total),
        "reasons": reasons,
    }
    per_bag_median = {
        bag: (total, ok, sorted(values)[len(values) // 2])
        for bag, (total, ok, values) in per_bag.items()
    }

    OUTPUT_HTML.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_HTML.write_text(render_html(rows, summary, per_template, per_bag_median), encoding="utf-8")
    OUTPUT_JSON.write_text(
        json.dumps(
            {
                "summary": {k: v for k, v in summary.items() if k != "reasons"},
                "reasons": dict(reasons),
                "per_template": {k: {"total": v[0], "ok": v[1]} for k, v in per_template.items()},
                "per_bag": {k: {"total": v[0], "ok": v[1], "median_distance_m": v[2]}
                            for k, v in per_bag_median.items()},
                "rows": rows,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    print(f"[summary] 可跑 {ok_count}/{len(rows)} = {ok_count / max(1, len(rows)) * 100:.0f}% · "
          f"route 中位 {summary['median_distance_m']:.0f} m · 分支切片 {branch_slices} · "
          f"未匹配帧 {summary['unmatched_rate']:.2%}")
    print(f"[reasons] {dict(reasons.most_common(6))}")
    print(f"[report]  {OUTPUT_HTML}")

    failures = []
    if ok_count < 0.8 * len(rows):
        failures.append(f"可跑 {ok_count}/{len(rows)} < 80%")
    if summary["unmatched_rate"] > MAX_UNMATCHED_RATE:
        failures.append(f"未匹配帧 {summary['unmatched_rate']:.1%} > {MAX_UNMATCHED_RATE:.0%}")
    for failure in failures:
        print(f"GATE FAIL : {failure}")
    print(f"GATE      : {'FAIL' if failures else 'PASS'}")
    return 1 if (args.check and failures) else 0


if __name__ == "__main__":
    raise SystemExit(main())
