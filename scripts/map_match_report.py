#!/usr/bin/env python3
"""地图匹配质量报告 —— bag 里的自车到底贴不贴得上 Apollo 车道。

为什么先做这张报告
------------------
15 条约束里有 8 条需要"我在哪条车道上"。匹配层如果悄悄错（贴到对向车道、或者车道
中心线本身偏移），后面所有 lane 相关的挖掘结果都是错的，而且不会报错。所以先用图把
匹配质量钉住：匹配率、横向偏移的**符号与分布**、航向差、以及轨迹上色。

口径
----
- ``lateral_m``：有符号横向偏移（左正右负）。**系统性偏差**（整条轨迹恒偏一侧）
  要么是地图中心线与实际行车线的偏差，要么是坐标系差异 —— 报告里单独量化，不做静默校正。
- 无 ``base_map.bin`` 的 bag 直接标为不可匹配，不猜测（有的地图只有 GB 级 txt）。

用法:
    python scripts/map_match_report.py
    python scripts/map_match_report.py --check     # 只校验，不写 HTML

gate:
    有地图的 bag 匹配率 >= 95%；|heading_diff| p90 < 10°；
    报告的每一条 bag 都有明确状态（匹配 / 无地图 / 无 bag）。
"""

import argparse
import math
import statistics
import sys
from collections import Counter
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from data_utils.apollo_lane_index import load_lane_index  # noqa: E402
from data_utils.apollo_record import decode_localization_pose, iter_messages  # noqa: E402

DEFAULT_DATA_ROOT = Path("/media/zoo/da1510ce-bbc7-4b3e-9d3b-e5f2137af995")
DEFAULT_OUT = REPO_ROOT / "docs" / "html" / "map_match_report.html"
DEFAULT_JSON = REPO_ROOT / "docs" / "measured" / "map_match_v0.json"
MAX_POSES = 600
MIN_MATCH_RATE = 0.95
MAX_HEADING_P90_DEG = 10.0

LATERAL_BUCKETS = [
    (0.25, "#1a7f37", "|lat| < 0.25 m"),
    (0.5, "#6aa84f", "0.25–0.5 m"),
    (0.9, "#9a6700", "0.5–0.9 m"),
    (1.5, "#d46b08", "0.9–1.5 m"),
    (float("inf"), "#cf222e", "> 1.5 m"),
]


def find_map_for(record_dir, map_root):
    for link in record_dir.iterdir():
        if not link.is_symlink():
            continue
        map_bin = map_root / link.name / "base_map.bin"
        if map_bin.exists():
            return link.name, map_bin
    return None, None


def collect(record_dir, map_root, max_poses=MAX_POSES):
    """返回 (lane_index 或 None, 每帧匹配结果列表, bag 路径, 地图名, 状态)。"""
    bag = next(record_dir.glob("*.bag"), None)
    map_name, map_bin = find_map_for(record_dir, map_root)
    if bag is None:
        return None, [], None, map_name, "missing_bag"
    if map_bin is None:
        return None, [], bag, None, "missing_map_bin"

    index = load_lane_index(map_bin)
    matches = []
    for _channel, _timestamp, payload in iter_messages(bag, channels={"/apollo/localization/pose"}):
        pose = decode_localization_pose(payload)
        match = index.match(pose["x_m"], pose["y_m"], pose["heading_rad"])
        matches.append({"x": pose["x_m"], "y": pose["y_m"], "match": match})
        if len(matches) >= max_poses:
            break
    return index, matches, bag, map_name, "ok"


def summarize(matches):
    matched = [item for item in matches if item["match"]]
    if not matched:
        return None
    signed = [item["match"]["lateral_m"] for item in matched]
    heading = [abs(math.degrees(item["match"]["heading_diff_rad"])) for item in matched]
    lanes = [item["match"]["lane_id"] for item in matched]
    transitions = sum(1 for a, b in zip(lanes, lanes[1:]) if a != b)
    absolute = sorted(abs(value) for value in signed)
    heading_sorted = sorted(heading)

    def percentile(values, fraction):
        return values[min(len(values) - 1, int(fraction * (len(values) - 1)))]

    return {
        "match_rate": len(matched) / len(matches),
        "signed_median_m": statistics.median(signed),
        "abs_p50_m": percentile(absolute, 0.5),
        "abs_p90_m": percentile(absolute, 0.9),
        "abs_p99_m": percentile(absolute, 0.99),
        "abs_max_m": absolute[-1],
        "heading_p50_deg": percentile(heading_sorted, 0.5),
        "heading_p90_deg": percentile(heading_sorted, 0.9),
        "heading_max_deg": heading_sorted[-1],
        "lane_transitions": transitions,
        "distinct_lanes": len(set(lanes)),
        "over_limit_frames": sum(
            1
            for item in matched
            if item["match"]["speed_limit_mps"] > 0
        ),
    }


def esc(text):
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def trajectory_svg(matches, width=1080, height=380):
    """俯视轨迹，按 |lateral| 分色；未匹配帧画红叉。"""
    points = [(item["x"], item["y"]) for item in matches]
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max(max_x - min_x, 1e-6)
    span_y = max(max_y - min_y, 1e-6)
    scale = min((width - 40) / span_x, (height - 50) / span_y)
    offset_x = 20 + ((width - 40) - span_x * scale) / 2
    offset_y = 26 + ((height - 50) - span_y * scale) / 2

    def project(x, y):
        return offset_x + (x - min_x) * scale, offset_y + (max_y - y) * scale

    parts = [f'<svg viewBox="0 0 {width} {height}" width="100%" role="img">']
    for index in range(0, len(matches) - 1, 1):
        current = matches[index]
        following = matches[index + 1]
        x0, y0 = project(current["x"], current["y"])
        x1, y1 = project(following["x"], following["y"])
        match = current["match"]
        if match is None:
            parts.append(
                f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}" '
                f'stroke="#cf222e" stroke-width="2.5" stroke-dasharray="3 3" />'
            )
            continue
        magnitude = abs(match["lateral_m"])
        color = next(color for limit, color, _label in LATERAL_BUCKETS if magnitude < limit)
        parts.append(
            f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}" '
            f'stroke="{color}" stroke-width="2.4" stroke-linecap="round" />'
        )
    parts.append(
        f'<text x="20" y="{height - 8}" font-size="10.5" fill="#57606a">'
        f"轨迹长度 {span_x * scale:.0f}×{span_y * scale:.0f} px；"
        "颜色 = |横向偏移|（绿→红），红色虚线 = 未匹配帧</text>"
    )
    parts.append("</svg>")
    return "".join(parts)


def histogram_svg(values, width=520, height=160, bins=41, span=1.5):
    if not values:
        return ""
    counts = [0] * bins
    for value in values:
        position = (value + span) / (2 * span)
        index = min(bins - 1, max(0, int(position * bins)))
        counts[index] += 1
    peak = max(counts) or 1
    plot_w, plot_h = width - 60, height - 52
    parts = [f'<svg viewBox="0 0 {width} {height}" width="100%" role="img">']
    parts.append(
        f'<text x="0" y="12" font-size="11.5" fill="#1f2328">有符号横向偏移分布（左正右负，m）</text>'
    )
    for index, count in enumerate(counts):
        if not count:
            continue
        x = 40 + plot_w * index / bins
        h = plot_h * count / peak
        center = (index + 0.5) / bins * 2 * span - span
        color = "#0b6bcb" if abs(center) < 0.9 else "#cf222e"
        parts.append(
            f'<rect x="{x:.1f}" y="{20 + plot_h - h:.1f}" width="{plot_w / bins - 1:.1f}" '
            f'height="{h:.1f}" fill="{color}" opacity="0.8" />'
        )
    for threshold in (-0.9, 0.9):
        x = 40 + plot_w * (threshold + span) / (2 * span)
        parts.append(
            f'<line x1="{x:.1f}" y1="20" x2="{x:.1f}" y2="{20 + plot_h}" stroke="#cf222e" '
            f'stroke-dasharray="4 3" />'
        )
    parts.append(
        f'<text x="40" y="{height - 6}" font-size="10" fill="#57606a">'
        f"−{span:.1f} m ─── 0 ─── +{span:.1f} m；虚线 = 车道偏离阈值 ±0.9 m</text>"
    )
    parts.append("</svg>")
    return "".join(parts)


def render(results, failures):
    gate_state = "PASS" if not failures else "FAIL"
    gate_color = "#1a7f37" if not failures else "#cf222e"
    ok_rows = [row for row in results if row["status"] == "ok" and row["summary"]]
    table_rows = [row for row in results if row["status"] != "missing_bag"]
    no_map = [row for row in table_rows if row["status"] == "missing_map_bin"]
    skipped_dirs = [row["bag"] for row in results if row["status"] == "missing_bag"]

    parts = [f"""<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>地图匹配质量报告</title>
<style>
:root {{ --ink:#1f2328; --mute:#57606a; --line:#d0d7de; --soft:#f6f8fa; }}
* {{ box-sizing:border-box; }}
body {{ margin:0; padding:32px 40px 56px; max-width:1320px; color:var(--ink);
  font:15px/1.6 -apple-system,"Segoe UI","PingFang SC","Microsoft YaHei",sans-serif; }}
h1 {{ font-size:26px; margin:0 0 6px; letter-spacing:-.3px; }}
h2 {{ font-size:19px; margin:34px 0 10px; padding-bottom:8px; border-bottom:1px solid var(--line); }}
h3 {{ font-size:13px; margin:18px 0 6px; color:var(--mute); text-transform:uppercase; letter-spacing:.07em; }}
.sub {{ color:var(--mute); font-size:13px; margin:0 0 18px; }}
.cards {{ display:grid; grid-template-columns:repeat(4,1fr); gap:12px; margin:18px 0 6px; }}
.card {{ border:1px solid var(--line); border-radius:10px; padding:13px 15px; background:var(--soft); }}
.card .k {{ font-size:12px; color:var(--mute); }}
.card .v {{ font-size:25px; font-weight:650; }}
.card .n {{ font-size:11.5px; color:var(--mute); }}
table {{ border-collapse:collapse; width:100%; font-size:12.5px; }}
th,td {{ border:1px solid var(--line); padding:5px 8px; }}
th {{ background:var(--soft); font-size:12px; text-align:left; }}
td.c,th.c {{ text-align:center; }}
tbody tr:nth-child(even) td {{ background:#fcfcfd; }}
code {{ background:var(--soft); border:1px solid var(--line); border-radius:4px; padding:1px 4px; font-size:11px; }}
.gate {{ display:inline-block; padding:3px 12px; border-radius:6px; font-size:13px; font-weight:700; color:#fff; background:{gate_color}; }}
.fail {{ border-left:3px solid #cf222e; background:#fff5f5; padding:9px 13px; border-radius:0 6px 6px 0; margin:7px 0; font-size:13px; }}
.ok {{ border-left:3px solid #1a7f37; background:#f2fbf4; padding:9px 13px; border-radius:0 6px 6px 0; margin:7px 0; font-size:13px; }}
.warn {{ border-left:3px solid #9a6700; background:#fffbf0; padding:9px 13px; border-radius:0 6px 6px 0; margin:7px 0; font-size:13px; }}
.note {{ color:var(--mute); font-size:12px; }}
.grid2 {{ display:grid; grid-template-columns:1fr 1fr; gap:22px; }}
</style></head><body>"""]
    parts.append(f"""
<h1>地图匹配质量报告</h1>
<p class="sub">Apollo <code>base_map.bin</code> ↔ bag 自车姿态 · {len(ok_rows)} 条可匹配 / {len(results)} 条录包 ·
gate <span class="gate">{gate_state}</span></p>""")

    parts.append('<div class="cards">')
    for key, value, note in [
        ("可匹配 bag", len(ok_rows), f"共 {len(results)} 条"),
        ("无 base_map.bin", len(no_map), "只有 GB 级 txt，无法解析"),
        (
            "平均匹配率",
            f"{statistics.mean([row['summary']['match_rate'] for row in ok_rows]) * 100:.0f}%" if ok_rows else "—",
            f"门槛 {MIN_MATCH_RATE:.0%}",
        ),
        (
            "最大 |lateral|",
            f"{max(row['summary']['abs_max_m'] for row in ok_rows):.2f} m" if ok_rows else "—",
            "行驶中距车道中心线",
        ),
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
            '<div class="ok">gate 通过：可匹配 bag 的匹配率达标、航向差 p90 达标；'
            "无地图的 bag 已显式标注而不是猜。</div>"
        )

    parts.append("<h2>1. 逐 bag 匹配质量</h2>")
    parts.append(
        "<table><thead><tr><th>bag</th><th>地图</th><th class='c'>匹配率</th>"
        "<th class='c'>lateral 中位</th><th class='c'>|lat| p90</th><th class='c'>|lat| max</th>"
        "<th class='c'>|heading| p90</th><th class='c'>经过车道</th><th class='c'>换道次数</th>"
        "</tr></thead><tbody>"
    )
    for row in table_rows:
        summary = row["summary"]
        if row["status"] != "ok" or summary is None:
            reason = "无 base_map.bin（仅 txt）" if row["status"] == "missing_map_bin" else "无 bag"
            parts.append(
                f'<tr><td><code>{esc(row["bag"])}</code></td>'
                f'<td class="note">{esc(row["map"] or "—")}</td>'
                f'<td class="c" colspan="7" style="color:#cf222e">{esc(reason)}</td></tr>'
            )
            continue
        parts.append(
            f'<tr><td><code>{esc(row["bag"])}</code></td>'
            f'<td class="note">{esc(row["map"])}</td>'
            f'<td class="c">{summary["match_rate"] * 100:.0f}%</td>'
            f'<td class="c">{summary["signed_median_m"]:+.2f} m</td>'
            f'<td class="c">{summary["abs_p90_m"]:.2f} m</td>'
            f'<td class="c">{summary["abs_max_m"]:.2f} m</td>'
            f'<td class="c">{summary["heading_p90_deg"]:.1f}°</td>'
            f'<td class="c">{summary["distinct_lanes"]}</td>'
            f'<td class="c">{summary["lane_transitions"]}</td></tr>'
        )
    parts.append("</tbody></table>")
    if skipped_dirs:
        parts.append(
            f'<p class="note">另有 {len(skipped_dirs)} 个目录只有地图没有 bag（'
            + "、".join(skipped_dirs)
            + "），不参与匹配统计。</p>"
        )
    parts.append(
        '<p class="note">「lateral 中位」的<b>符号</b>是关键：整条轨迹恒偏一侧说明地图中心线与'
        "实际行车线有系统偏差（不是匹配错误）。这个偏差必须显式量化，否则车道偏离阈值会被它吃掉一半。</p>"
    )

    for row in ok_rows:
        parts.append(f'<h2>2.{ok_rows.index(row) + 1} {esc(row["bag"])} —— 轨迹上色</h2>')
        parts.append(trajectory_svg(row["matches"]))
        signed = [item["match"]["lateral_m"] for item in row["matches"] if item["match"]]
        parts.append('<div class="grid2">')
        parts.append(histogram_svg(signed))
        parts.append(
            '<div class="note">'
            f'<b>匹配率</b> {row["summary"]["match_rate"] * 100:.0f}% ｜ '
            f'<b>有符号中位</b> {row["summary"]["signed_median_m"]:+.2f} m ｜ '
            f'<b>|lat| p50/p90/p99</b> '
            f'{row["summary"]["abs_p50_m"]:.2f} / {row["summary"]["abs_p90_m"]:.2f} / '
            f'{row["summary"]["abs_p99_m"]:.2f} m<br>'
            f'<b>|heading| p50/p90</b> {row["summary"]["heading_p50_deg"]:.1f}° / '
            f'{row["summary"]["heading_p90_deg"]:.1f}° ｜ '
            f'<b>经过车道</b> {row["summary"]["distinct_lanes"]} 条 ｜ '
            f'<b>换道</b> {row["summary"]["lane_transitions"]} 次'
            '</div>'
        )
        parts.append("</div>")

    parts.append("<h2>3. 这批数据的地图覆盖缺口</h2>")
    parts.append(
        '<div class="warn">'
        f"{len(no_map)} 条 bag 没有可解析的 <code>base_map.bin</code>"
        "（对应地图目录只有 21 GB / 34 GB 的 <code>base_map.txt</code>）。"
        "这些 bag 的 8 条车道类约束<b>无法挖掘</b>，只能等对应地图的 bin 落地。"
        "这不是匹配算法的问题，是数据供给问题 —— 但会直接决定哪些 bag 能进 bad case 库。</div>"
    )
    parts.append(
        '<p class="note">补 bin 的命令（在 Apollo 地图目录执行）：<code>protoc --encode=apollo.hdmap.Map '
        "hdmap_map.proto &lt; base_map.txt &gt; base_map.bin</code>，或从 Apollo 工具链导出。</p>"
    )
    parts.append(
        '<p class="sub" style="margin-top:30px">由 <code>scripts/map_match_report.py</code> 生成。'
        "匹配层 <code>data_utils/apollo_lane_index.py</code>。</p></body></html>"
    )
    return "".join(parts)


def main():
    parser = argparse.ArgumentParser(description="地图匹配质量报告")
    parser.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--json-out", type=Path, default=DEFAULT_JSON)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    map_root = args.data_root / "apollo_map" / "data"
    results = []
    import json

    for record_dir in sorted(args.data_root.glob("rx-*")):
        index, matches, bag, map_name, status = collect(record_dir, map_root)
        summary = summarize(matches) if matches else None
        name = record_dir.name.replace("rx-", "")
        print(
            f"[{name:5s}] {status:16s} "
            + (
                f"匹配 {summary['match_rate'] * 100:3.0f}%  lateral {summary['signed_median_m']:+.2f} m  "
                f"|heading| p90 {summary['heading_p90_deg']:.1f}°  车道 {summary['distinct_lanes']}"
                if summary
                else "（不可匹配）"
            )
        )
        results.append(
            {
                "bag": name,
                "map": map_name,
                "status": status,
                "summary": summary,
                "matches": matches,
            }
        )

    ok_rows = [row for row in results if row["status"] == "ok" and row["summary"]]
    failures = []
    if not ok_rows:
        failures.append("没有任何 bag 能匹配（检查地图 bin 与姿态坐标系）")
    for row in ok_rows:
        if row["summary"]["match_rate"] < MIN_MATCH_RATE:
            failures.append(
                f"{row['bag']}: 匹配率 {row['summary']['match_rate']:.0%} < {MIN_MATCH_RATE:.0%}"
            )
        if row["summary"]["heading_p90_deg"] > MAX_HEADING_P90_DEG:
            failures.append(
                f"{row['bag']}: |heading| p90 {row['summary']['heading_p90_deg']:.1f}° > {MAX_HEADING_P90_DEG}°"
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
            json.dumps(
                {
                    "bags": [
                        {
                            "bag": row["bag"],
                            "map": row["map"],
                            "status": row["status"],
                            "summary": row["summary"],
                        }
                        for row in results
                    ],
                    "gate": failures,
                },
                ensure_ascii=False,
                indent=1,
            ),
            encoding="utf-8",
        )
        print(f"report    : {args.out}")
        print(f"json      : {args.json_out}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
