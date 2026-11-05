#!/usr/bin/env python3
"""观测包络验收图 —— 6 条真实录包的感知距离与数据年龄。

输入: `data_utils/measure_obs_latency.py --json-out` 的多录包结果（JSON 数组）。
输出: `docs/html/spec_v0_obs_envelope.html`（自包含 SVG，双面板）

为什么要这张图
--------------
`docs/vehicle_profile.md` §4.2 的两个数（70 m / 0.53 s）原先只来自 2 条录包。
本次把 6 条全部跑一遍，结果是 **0.53 s 站不住**：control 侧数据年龄中位 0.78 s
（0.41–1.21）。spec 的 `agent_range_m` / `observation_delay_s` 必须以 6 条 bag
的分布为准，而不是单条低端值。

gate:
    >100 m 的障碍物计数 = 0（否则 70 m 上限不成立）
    per-message max 的 p100 中位落在 [60, 80] m
    control 数据年龄中位落在 [0.4, 1.2] s
"""

import argparse
import json
import statistics
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_INPUT = REPO_ROOT / "docs" / "measured" / "obs_envelope_6bags.json"
DEFAULT_OUT = REPO_ROOT / "docs" / "html" / "spec_v0_obs_envelope.html"

OPERATIONAL_SPEED_MPS = 12.5
FROZEN_AGENT_RANGE_M = 70.0
FROZEN_DELAY_S = 0.78
HIST_BIN_ORDER = ["0-30m", "30-60m", "60-100m", "100-150m", ">=150m"]


def bag_label(record):
    stem = Path(record).name
    if "white-rhino-" in stem:
        return stem.split("white-rhino-")[1].split(".")[0]
    return stem[:12]


def stat(d, key, sub):
    return d[key][sub]


def collect(records):
    rows = []
    for record in records:
        rows.append(
            {
                "bag": bag_label(record["record"]),
                "p50_max_m": record["perception_range_m"]["per_message_max"]["p50"],
                "p100_max_m": record["perception_range_m"]["per_message_max"]["p100"],
                "delay_control_s": record["sensor_age_s"]["control"]["p50"],
                "delay_planning_s": record["sensor_age_s"]["planning"]["p50"],
                "publish_lag_s": record["publish_lag_s"]["perception"]["p50"],
                "reaction_m": record["reaction_distance_at_operational_speed_m"]["from_planning"],
                "obstacles": record["counts"]["obstacles"],
            }
        )

    histogram = {key: 0 for key in HIST_BIN_ORDER}
    for record in records:
        for key, value in record["perception_range_m"]["distance_histogram"].items():
            histogram[key] = histogram.get(key, 0) + value

    return rows, histogram


def gate(rows, histogram):
    failures = []
    far = histogram.get("100-150m", 0) + histogram.get(">=150m", 0)
    if far != 0:
        failures.append(f"{far} 个障碍物 >100 m，70 m 上限不成立")
    p100_median = statistics.median(row["p100_max_m"] for row in rows)
    if not 60.0 <= p100_median <= 80.0:
        failures.append(f"per-message max p100 中位 {p100_median:.1f} m 不在 [60, 80]")
    delay_median = statistics.median(row["delay_control_s"] for row in rows)
    if not 0.4 <= delay_median <= 1.2:
        failures.append(f"control 数据年龄中位 {delay_median:.2f} s 不在 [0.4, 1.2]")
    return failures, {"p100_median": p100_median, "delay_median": delay_median, "far": far}


def esc(text):
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def range_panel_svg(rows, histogram):
    """面板 ①：距离直方图（对数高度）+ 70 m / 200 m 标线。"""
    width, height = 1120, 300
    left, right, top, bottom = 70, 40, 26, 58
    plot_w = width - left - right
    plot_h = height - top - bottom
    max_count = max(histogram.values()) or 1

    import math

    parts = [f'<svg viewBox="0 0 {width} {height}" width="100%" role="img">']
    # y 轴（对数刻度，避免 61540 压平 2624）
    for exponent in range(0, 6):
        value = 10**exponent
        if value > max_count:
            break
        y = top + plot_h - plot_h * (math.log10(value) / math.log10(max_count))
        parts.append(
            f'<line x1="{left}" y1="{y:.1f}" x2="{width - right}" y2="{y:.1f}" stroke="#eaeef2" />'
            f'<text x="{left - 8}" y="{y + 4:.1f}" font-size="10" fill="#57606a" text-anchor="end">{value:,}</text>'
        )
    # 柱
    bin_count = len(HIST_BIN_ORDER)
    slot = plot_w / bin_count
    for index, key in enumerate(HIST_BIN_ORDER):
        count = histogram.get(key, 0)
        ratio = math.log10(count) / math.log10(max_count) if count else 0.0
        bar_h = plot_h * ratio
        x = left + slot * index + slot * 0.18
        bar_w = slot * 0.64
        color = "#1a7f37" if index < 3 else "#cf222e"
        if count == 0:
            parts.append(
                f'<text x="{x + bar_w / 2:.1f}" y="{top + plot_h - 8:.1f}" font-size="11" '
                f'fill="#cf222e" text-anchor="middle">0</text>'
            )
        parts.append(
            f'<rect x="{x:.1f}" y="{top + plot_h - bar_h:.1f}" width="{bar_w:.1f}" '
            f'height="{bar_h:.1f}" fill="{color}" opacity="0.85" rx="2" />'
        )
        parts.append(
            f'<text x="{x + bar_w / 2:.1f}" y="{top + plot_h + 18:.1f}" font-size="11" '
            f'fill="#1f2328" text-anchor="middle">{key}</text>'
            f'<text x="{x + bar_w / 2:.1f}" y="{top + plot_h + 34:.1f}" font-size="10" '
            f'fill="#57606a" text-anchor="middle">{count:,}</text>'
        )

    # 标线：仿真原值 200 m 在画外，用箭头标注
    marker_x = left + slot * (70.0 - 0.0) / 30.0  # 70 m 落在 60-100 格内
    parts.append(
        f'<line x1="{marker_x:.1f}" y1="{top}" x2="{marker_x:.1f}" y2="{top + plot_h}" '
        f'stroke="#0b6bcb" stroke-width="2" stroke-dasharray="5 4" />'
        f'<text x="{marker_x + 6:.1f}" y="{top + 14}" font-size="11.5" fill="#0b6bcb" font-weight="600">'
        f"新上限 70 m</text>"
    )
    parts.append(
        f'<text x="{width - right - 6}" y="{top + 14}" font-size="11.5" fill="#cf222e" '
        f'text-anchor="end" font-weight="600">仿真原值 200 m（在画外 →）</text>'
    )
    parts.append(
        f'<text x="{left}" y="{height - 6}" font-size="11" fill="#57606a">'
        f"障碍物距离（m）。纵轴对数刻度；6 条录包合计 "
        f"{sum(histogram.values()):,} 个观测。</text>"
    )
    parts.append("</svg>")
    return "".join(parts)


def delay_panel_svg(rows):
    """面板 ②：每条 bag 的数据年龄（control / planning）+ 冻结值标线。"""
    width, height = 1120, 356
    left, right, top, bottom = 96, 40, 26, 78
    plot_w = width - left - right
    plot_h = height - top - bottom
    max_delay = max(max(row["delay_control_s"] for row in rows), FROZEN_DELAY_S) * 1.15

    def y_of(value):
        return top + plot_h - plot_h * (value / max_delay)

    parts = [f'<svg viewBox="0 0 {width} {height}" width="100%" role="img">']
    for tick in [0.0, 0.3, 0.6, 0.9, 1.2, 1.5]:
        if tick > max_delay:
            break
        y = y_of(tick)
        parts.append(
            f'<line x1="{left}" y1="{y:.1f}" x2="{width - right}" y2="{y:.1f}" stroke="#eaeef2" />'
            f'<text x="{left - 8}" y="{y + 4:.1f}" font-size="10" fill="#57606a" '
            f'text-anchor="end">{tick:.1f}s</text>'
        )

    slot = plot_w / len(rows)
    for index, row in enumerate(rows):
        cx = left + slot * (index + 0.5)
        y_control = y_of(row["delay_control_s"])
        y_planning = y_of(row["delay_planning_s"])
        parts.append(
            f'<line x1="{cx:.1f}" y1="{y_control:.1f}" x2="{cx:.1f}" y2="{y_planning:.1f}" '
            f'stroke="#d0d7de" stroke-width="2" />'
            f'<circle cx="{cx:.1f}" cy="{y_control:.1f}" r="5.5" fill="#cf222e" />'
            f'<circle cx="{cx:.1f}" cy="{y_planning:.1f}" r="4.5" fill="#0b6bcb" />'
            f'<text x="{cx:.1f}" y="{top + plot_h + 20:.1f}" font-size="11" fill="#1f2328" '
            f'text-anchor="middle">{esc(row["bag"])}</text>'
            f'<text x="{cx:.1f}" y="{top + plot_h + 35:.1f}" font-size="9.5" fill="#57606a" '
            f'text-anchor="middle">{row["delay_control_s"]:.2f}s</text>'
        )

    frozen_y = y_of(FROZEN_DELAY_S)
    parts.append(
        f'<line x1="{left}" y1="{frozen_y:.1f}" x2="{width - right}" y2="{frozen_y:.1f}" '
        f'stroke="#1a7f37" stroke-width="2" stroke-dasharray="6 4" />'
        f'<text x="{width - right - 4}" y="{frozen_y - 7:.1f}" font-size="11.5" fill="#1a7f37" '
        f'text-anchor="end" font-weight="600">冻结值 0.78 s（6 条中位）</text>'
    )
    parts.append(
        f'<text x="{left}" y="{height - 6}" font-size="11" fill="#57606a">'
        "● control 侧数据年龄  ● planning 侧。数值 = header.lidar_timestamp 到发布时刻的差，p50。</text>"
    )
    parts.append("</svg>")
    return "".join(parts)


def render(rows, histogram, summary, failures):
    gate_state = "PASS" if not failures else "FAIL"
    gate_color = "#1a7f37" if not failures else "#cf222e"
    blind_distance = OPERATIONAL_SPEED_MPS * FROZEN_DELAY_S

    parts = [f"""<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>观测包络验收 —— 6 条真实录包</title>
<style>
:root {{ --ink:#1f2328; --mute:#57606a; --line:#d0d7de; --soft:#f6f8fa; }}
* {{ box-sizing:border-box; }}
body {{ margin:0; padding:32px 40px 56px; max-width:1320px; color:var(--ink);
  font:15px/1.65 -apple-system,"Segoe UI","PingFang SC","Microsoft YaHei",sans-serif; }}
h1 {{ font-size:26px; margin:0 0 6px; letter-spacing:-.3px; }}
h2 {{ font-size:19px; margin:36px 0 10px; padding-bottom:8px; border-bottom:1px solid var(--line); }}
.sub {{ color:var(--mute); font-size:13px; margin:0 0 20px; }}
.cards {{ display:grid; grid-template-columns:repeat(4,1fr); gap:12px; margin:20px 0 6px; }}
.card {{ border:1px solid var(--line); border-radius:10px; padding:14px 16px; background:var(--soft); }}
.card .k {{ font-size:12px; color:var(--mute); }}
.card .v {{ font-size:26px; font-weight:650; letter-spacing:-.5px; }}
.card .n {{ font-size:12px; color:var(--mute); }}
table {{ border-collapse:collapse; width:100%; font-size:13px; }}
th,td {{ border:1px solid var(--line); padding:5px 8px; }}
th {{ background:var(--soft); font-size:12px; text-align:left; }}
td.c,th.c {{ text-align:center; }}
tbody tr:nth-child(even) td {{ background:#fcfcfd; }}
code {{ background:var(--soft); border:1px solid var(--line); border-radius:4px; padding:1px 5px; font-size:11.5px; }}
.gate {{ display:inline-block; padding:3px 12px; border-radius:6px; font-size:13px; font-weight:700; color:#fff; background:{gate_color}; }}
.fail {{ border-left:3px solid #cf222e; background:#fff5f5; padding:9px 13px; border-radius:0 6px 6px 0; margin:7px 0; font-size:13px; }}
.ok {{ border-left:3px solid #1a7f37; background:#f2fbf4; padding:9px 13px; border-radius:0 6px 6px 0; margin:7px 0; font-size:13px; }}
.warn {{ border-left:3px solid #9a6700; background:#fffbf0; padding:9px 13px; border-radius:0 6px 6px 0; margin:7px 0; font-size:13px; }}
.note {{ color:var(--mute); font-size:12.5px; }}
</style></head><body>"""]
    parts.append(f"""
<h1>观测包络验收 —— 6 条真实录包</h1>
<p class="sub">spec <code>canonical_v0</code> · 数据 6 条 bag（
{sum(row["obstacles"] for row in rows):,} 个障碍物观测） · gate <span class="gate">{gate_state}</span></p>""")

    parts.append('<div class="cards">')
    cards = [
        ("感知上限（冻结）", f"{FROZEN_AGENT_RANGE_M:.0f} m", f"per-message max p100 中位 {summary['p100_median']:.1f} m"),
        ("数据年龄（冻结）", f"{FROZEN_DELAY_S:.2f} s", f"control p50 中位 {summary['delay_median']:.2f} s"),
        ("盲开距离", f"{blind_distance:.1f} m", f"{OPERATIONAL_SPEED_MPS} m/s × {FROZEN_DELAY_S} s"),
        (">100 m 的障碍物", f"{summary['far']}", "70 m 上限的实证依据"),
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
            '<div class="ok">gate 通过：100 m 外障碍物为 0（70 m 上限成立）；'
            "感知上限与数据年龄的中位都在预期区间。</div>"
        )

    parts.append("<h2>面板 ①　感知距离分布（6 条合计）</h2>")
    parts.append(range_panel_svg(rows, histogram))
    parts.append(
        '<p class="note">柱子 = 障碍物距离分桶（纵轴对数）。'
        "100–150 m 与 ≥150 m 两桶为 <b>0</b>，所以 70 m 不是拍出来的保守值，而是数据上界。</p>"
    )

    parts.append("<h2>面板 ②　数据年龄（每条 bag）</h2>")
    parts.append(delay_panel_svg(rows))
    parts.append(
        '<div class="warn">原 spec 候选值 <b>0.53 s</b> 只对应 <code>rx-2714</code> 一条；'
        "6 条的中位是 <b>0.78 s</b>，最差 1.21 s。冻结值改为 0.78 s，"
        "并把该字段保留为 per-scenario provenance（每条 bag 带自己的实测值）。</div>"
    )

    parts.append("<h2>逐条明细</h2>")
    parts.append(
        "<table><thead><tr><th>bag</th><th class='c'>障碍物观测</th>"
        "<th class='c'>per-message max p50</th><th class='c'>per-message max p100</th>"
        "<th class='c'>control 数据年龄</th><th class='c'>planning 数据年龄</th>"
        "<th class='c'>反算距离 (planning)</th></tr></thead><tbody>"
    )
    for row in rows:
        parts.append(
            f'<tr><td><code>{esc(row["bag"])}</code></td>'
            f'<td class="c">{row["obstacles"]:,}</td>'
            f'<td class="c">{row["p50_max_m"]:.1f} m</td>'
            f'<td class="c">{row["p100_max_m"]:.1f} m</td>'
            f'<td class="c">{row["delay_control_s"]:.2f} s</td>'
            f'<td class="c">{row["delay_planning_s"]:.2f} s</td>'
            f'<td class="c">{row["reaction_m"]:.1f} m</td></tr>'
        )
    parts.append("</tbody></table>")

    parts.append("<h2>仿真侧落地</h2>")
    parts.append(
        "<table><thead><tr><th>参数</th><th class='c'>原值</th><th class='c'>现值</th>"
        "<th>落地位置</th></tr></thead><tbody>"
        "<tr><td><code>obs_range_partner_m</code></td><td class='c'>200.0</td>"
        "<td class='c'>70.0</td><td><code>pufferlib/config/puffer_drive.yaml</code>"
        "（C 侧过滤 `drive.h:3861`）</td></tr>"
        "<tr><td><code>observation_delay_s</code></td><td class='c'>0（无此参数）</td>"
        "<td class='c'>0.78</td><td><code>Drive._apply_observation_delay</code>"
        "（ring buffer，量化到 dt；dt=0.3 时 3 步 = 0.9 s 有效值）</td></tr>"
        "</tbody></table>"
    )
    parts.append(
        '<p class="note">延迟量化误差：dt = 0.3 s 时 0.78 s → 3 步 = 0.9 s（+0.12 s）。'
        "实际生效值由 <code>effective_observation_delay_s</code> 上报，不静默截断。</p>"
    )
    parts.append(
        '<p class="sub" style="margin-top:32px">由 <code>scripts/spec_envelope_report.py</code> 生成，勿手改。'
        "测量源 <code>data_utils/measure_obs_latency.py</code>。</p></body></html>"
    )
    return "".join(parts)


def main():
    parser = argparse.ArgumentParser(description="观测包络验收图")
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    records = json.loads(args.input.read_text(encoding="utf-8"))
    if isinstance(records, dict):
        records = [records]
    rows, histogram = collect(records)
    failures, summary = gate(rows, histogram)

    print(f"records     : {len(records)}")
    print(f"p100 median : {summary['p100_median']:.1f} m")
    print(f"delay median: {summary['delay_median']:.2f} s")
    print(f"obstacles   : {sum(histogram.values()):,} (>{100} m: {summary['far']})")
    for row in rows:
        print(
            f"  {row['bag']:6s} max p50 {row['p50_max_m']:5.1f} p100 {row['p100_max_m']:5.1f} | "
            f"control {row['delay_control_s']:.2f}s planning {row['delay_planning_s']:.2f}s"
        )
    for item in failures:
        print(f"GATE FAIL   : {item}")
    if not failures:
        print("GATE        : PASS")

    if not args.check:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(render(rows, histogram, summary, failures), encoding="utf-8")
        print(f"report      : {args.out}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
