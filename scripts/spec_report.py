#!/usr/bin/env python3
"""canonical spec 覆盖率报告器 —— 校验字段注册表并生成可视报告。

用法:
    python scripts/spec_report.py                     # 打印统计 + 写 HTML
    python scripts/spec_report.py --check             # 只校验，gate 失败退出码 1
    python scripts/spec_report.py --out path.html     # 指定输出

gate（spec/README.md §冻结协议）:
    字段数 >= MIN_FIELDS；必填属性缺失 = 0；frozen 占比 >= MIN_FROZEN_RATIO；
    每组至少 1 个 frozen 字段；枚举值合法。
"""

import argparse
import sys
from collections import OrderedDict
from pathlib import Path

import yaml

REQUIRED_ATTRS = (
    "unit",
    "frame",
    "dtype",
    "time",
    "truth",
    "sim",
    "nova",
    "status",
)
VALID_FRAME = {"global", "ego_rfu", "agent_rfu", "screen", "-"}
VALID_SIM = {"native", "bridged", "absent"}
VALID_STATUS = {"frozen", "draft", "blocked"}
VALID_TIME = {"current", "history16", "future80", "future60", "static"}

MIN_FIELDS = 40
MIN_FROZEN_RATIO = 0.60

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SPEC = REPO_ROOT / "spec" / "canonical_v0.yaml"
DEFAULT_OUT = REPO_ROOT / "docs" / "html" / "spec_v0_coverage.html"

STATUS_COLOR = {"frozen": "#1a7f37", "draft": "#9a6700", "blocked": "#cf222e"}
SIM_COLOR = {"native": "#1a7f37", "bridged": "#9a6700", "absent": "#cf222e"}


def load_spec(path):
    with open(path, encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def iter_fields(spec):
    """按组顺序产出 (group_id, group_label, field)。"""
    for group in spec["groups"]:
        for field in group["fields"]:
            yield group["id"], group.get("label", group["id"]), field


def find_problems(spec):
    """返回 (字段级问题, 全局问题)。字段问题 = 必填属性缺失或枚举非法。"""
    field_problems = []
    global_problems = []

    if not spec.get("schema_version"):
        global_problems.append("缺 schema_version")
    if not spec.get("owner_decision"):
        global_problems.append("缺 owner_decision（必须指向 ADR）")

    seen_ids = {}
    for group_id, _, field in iter_fields(spec):
        field_id = field.get("id", "<无 id>")
        if field_id == "<无 id>":
            field_problems.append((group_id, field_id, "缺字段 id"))
            continue
        if field_id in seen_ids:
            field_problems.append((group_id, field_id, f"字段 id 重复（首次见 {seen_ids[field_id]}）"))
        seen_ids[field_id] = group_id

        missing = [attr for attr in REQUIRED_ATTRS if field.get(attr) in (None, "")]
        if missing:
            field_problems.append((group_id, field_id, "缺属性: " + ", ".join(missing)))

        checks = (
            ("frame", VALID_FRAME),
            ("sim", VALID_SIM),
            ("status", VALID_STATUS),
            ("time", VALID_TIME),
        )
        for attr, allowed in checks:
            value = field.get(attr)
            if value is not None and value not in allowed:
                field_problems.append((group_id, field_id, f"{attr} 非法: {value}"))

    return field_problems, global_problems


def compute_stats(spec):
    groups = OrderedDict()
    total = 0
    status_counts = {key: 0 for key in VALID_STATUS}
    sim_counts = {key: 0 for key in VALID_SIM}

    for group in spec["groups"]:
        group_id = group["id"]
        fields = group["fields"]
        frozen = sum(1 for f in fields if f.get("status") == "frozen")
        attr_filled = {attr: 0 for attr in REQUIRED_ATTRS}
        for field in fields:
            for attr in REQUIRED_ATTRS:
                if field.get(attr) not in (None, ""):
                    attr_filled[attr] += 1
            status = field.get("status")
            if status in status_counts:
                status_counts[status] += 1
            sim = field.get("sim")
            if sim in sim_counts:
                sim_counts[sim] += 1
        groups[group_id] = {
            "label": group.get("label", group_id),
            "field_count": len(fields),
            "frozen_count": frozen,
            "frozen_ratio": frozen / len(fields) if fields else 0.0,
            "attr_filled": attr_filled,
        }
        total += len(fields)

    frozen_total = status_counts["frozen"]
    return {
        "groups": groups,
        "total_fields": total,
        "frozen_total": frozen_total,
        "frozen_ratio": frozen_total / total if total else 0.0,
        "status_counts": status_counts,
        "sim_counts": sim_counts,
    }


def evaluate_gate(spec, stats, field_problems, global_problems):
    failures = []
    if stats["total_fields"] < MIN_FIELDS:
        failures.append(f"字段数 {stats['total_fields']} < {MIN_FIELDS}")
    if field_problems:
        failures.append(f"字段级问题 {len(field_problems)} 条（要求 0）")
    if global_problems:
        failures.append(f"全局问题 {len(global_problems)} 条（要求 0）")
    if stats["frozen_ratio"] < MIN_FROZEN_RATIO:
        failures.append(
            f"frozen 占比 {stats['frozen_ratio']:.1%} < {MIN_FROZEN_RATIO:.0%}"
        )
    for group_id, group in stats["groups"].items():
        if group["frozen_count"] < 1:
            failures.append(f"组 {group_id} 无 frozen 字段")
    return failures


def bar(ratio, color):
    width = max(0.0, min(1.0, ratio)) * 100.0
    return (
        f'<div class="bar"><div class="bar-fill" '
        f'style="width:{width:.1f}%;background:{color}"></div></div>'
    )


def html_escape(text):
    return (
        str(text)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
    )


def render_html(spec, stats, failures):
    total = stats["total_fields"]
    frozen = stats["frozen_total"]
    status_counts = stats["status_counts"]
    sim_counts = stats["sim_counts"]
    gate_state = "PASS" if not failures else "FAIL"
    gate_color = "#1a7f37" if not failures else "#cf222e"

    parts = []
    parts.append(
        f"""<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>canonical spec v0 覆盖率报告</title>
<style>
:root {{ --ink:#1f2328; --mute:#57606a; --line:#d0d7de; --bg:#ffffff; --soft:#f6f8fa; }}
* {{ box-sizing:border-box; }}
body {{ margin:0; padding:32px 40px 64px; font:15px/1.65 -apple-system,"Segoe UI","PingFang SC","Hiragino Sans GB","Microsoft YaHei",sans-serif;
       color:var(--ink); background:var(--bg); max-width:1280px; }}
h1 {{ font-size:26px; margin:0 0 6px; letter-spacing:-.3px; }}
h2 {{ font-size:19px; margin:40px 0 12px; padding-bottom:8px; border-bottom:1px solid var(--line); }}
h3 {{ font-size:15px; margin:22px 0 8px; color:var(--mute); text-transform:uppercase; letter-spacing:.06em; }}
.sub {{ color:var(--mute); font-size:13px; margin:0 0 24px; }}
.cards {{ display:grid; grid-template-columns:repeat(5,1fr); gap:12px; margin:24px 0 8px; }}
.card {{ border:1px solid var(--line); border-radius:10px; padding:14px 16px; background:var(--soft); }}
.card .k {{ font-size:12px; color:var(--mute); letter-spacing:.04em; }}
.card .v {{ font-size:28px; font-weight:650; letter-spacing:-.5px; margin-top:2px; }}
.card .n {{ font-size:12px; color:var(--mute); }}
table {{ border-collapse:collapse; width:100%; font-size:13.5px; }}
th, td {{ border:1px solid var(--line); padding:6px 9px; text-align:left; vertical-align:top; }}
th {{ background:var(--soft); font-weight:600; font-size:12.5px; }}
tr:nth-child(even) td {{ background:#fcfcfd; }}
code {{ background:var(--soft); border:1px solid var(--line); border-radius:4px; padding:1px 5px; font-size:12px;
        font-family:ui-monospace,SFMono-Regular,Menlo,monospace; }}
.bar {{ width:100%; height:8px; background:#eaeef2; border-radius:4px; overflow:hidden; min-width:60px; }}
.bar-fill {{ height:100%; }}
.mx td.c {{ text-align:center; width:78px; font-variant-numeric:tabular-nums; }}
.pill {{ display:inline-block; padding:1px 8px; border-radius:999px; font-size:11.5px; font-weight:600; color:#fff; }}
.gate {{ display:inline-block; padding:3px 12px; border-radius:6px; font-size:13px; font-weight:700; color:#fff; background:{gate_color}; }}
.fail {{ border-left:3px solid #cf222e; background:#fff5f5; padding:10px 14px; border-radius:0 6px 6px 0; margin:8px 0; font-size:13.5px; }}
.ok {{ border-left:3px solid #1a7f37; background:#f2fbf4; padding:10px 14px; border-radius:0 6px 6px 0; margin:8px 0; font-size:13.5px; }}
.note {{ color:var(--mute); font-size:12.5px; }}
.legend {{ display:flex; gap:18px; flex-wrap:wrap; font-size:12.5px; color:var(--mute); margin:8px 0 0; }}
.sw {{ display:inline-block; width:10px; height:10px; border-radius:2px; margin-right:5px; vertical-align:middle; }}
</style></head><body>"""
    )

    parts.append(
        f"""
<h1>canonical spec v0 —— 字段覆盖率报告</h1>
<p class="sub">
schema <code>{html_escape(spec.get('schema_version'))}</code> ·
as-of {html_escape(spec.get('as_of'))} ·
baseline <code>{html_escape(spec.get('baseline_commit'))}</code> ·
裁决 <code>{html_escape(Path(str(spec.get('owner_decision'))).name)}</code> ·
gate <span class="gate">{gate_state}</span>
</p>"""
    )

    parts.append('<div class="cards">')
    cards = [
        ("字段总数", total, f"下限 {MIN_FIELDS}"),
        ("组数", len(stats["groups"]), "ego/agents/occ/map/traffic/route/prov/vehicle"),
        ("frozen", f"{frozen}", f"{stats['frozen_ratio']:.1%}（下限 {MIN_FROZEN_RATIO:.0%}）"),
        ("draft", status_counts["draft"], "待 owner 确认"),
        ("blocked", status_counts["blocked"], "依赖未决项"),
    ]
    for key, value, note in cards:
        parts.append(
            f'<div class="card"><div class="k">{key}</div>'
            f'<div class="v">{value}</div><div class="n">{note}</div></div>'
        )
    parts.append("</div>")

    # ---- gate 明细
    parts.append("<h2>1. Gate 明细</h2>")
    if failures:
        for item in failures:
            parts.append(f'<div class="fail">{html_escape(item)}</div>')
    else:
        parts.append(
            '<div class="ok">全部 gate 通过：字段数、必填属性、枚举合法性、'
            "frozen 占比、每组 frozen 均达标。</div>"
        )

    # ---- 组 × 属性矩阵
    parts.append("<h2>2. 覆盖率矩阵（组 × 必填属性）</h2>")
    parts.append(
        '<p class="note">单元格 = 该组内已填该属性的字段比例。100% 为合格线；'
        "低于 100% 说明注册表里有半成品字段。</p>"
    )
    parts.append("<table><thead><tr><th>组</th><th>字段数</th>")
    for attr in REQUIRED_ATTRS:
        parts.append(f"<th>{attr}</th>")
    parts.append("<th>frozen</th></tr></thead><tbody>")
    for group_id, group in stats["groups"].items():
        parts.append(
            f'<tr><td><code>{group_id}</code><br><span class="note">'
            f'{html_escape(group["label"])}</span></td>'
            f'<td class="c">{group["field_count"]}</td>'
        )
        for attr in REQUIRED_ATTRS:
            filled = group["attr_filled"][attr]
            ratio = filled / group["field_count"] if group["field_count"] else 0.0
            color = "#1a7f37" if ratio >= 1.0 else ("#9a6700" if ratio >= 0.8 else "#cf222e")
            parts.append(
                f'<td class="c"><span style="color:{color};font-weight:600">'
                f"{ratio:.0%}</span>{bar(ratio, color)}</td>"
            )
        frozen_ratio = group["frozen_ratio"]
        color = "#1a7f37" if frozen_ratio >= MIN_FROZEN_RATIO else "#9a6700"
        parts.append(
            f'<td class="c">{group["frozen_count"]}/{group["field_count"]}'
            f"{bar(frozen_ratio, color)}</td></tr>"
        )
    parts.append("</tbody></table>")

    # ---- 状态分布
    parts.append("<h2>3. 状态分布</h2>")
    parts.append("<table><thead><tr><th>维度</th><th>取值</th><th>字段数</th><th>占比</th><th></th></tr></thead><tbody>")
    for dimension, counts, palette in (
        ("status", status_counts, STATUS_COLOR),
        ("sim", sim_counts, SIM_COLOR),
    ):
        for value, count in counts.items():
            ratio = count / total if total else 0.0
            parts.append(
                f"<tr><td>{dimension}</td>"
                f'<td><span class="pill" style="background:{palette[value]}">{value}</span></td>'
                f'<td class="c">{count}</td><td class="c">{ratio:.1%}</td>'
                f"<td>{bar(ratio, palette[value])}</td></tr>"
            )
    parts.append("</tbody></table>")
    parts.append(
        '<div class="legend">'
        '<span><span class="sw" style="background:#1a7f37"></span>native / frozen —— 可直接依赖</span>'
        '<span><span class="sw" style="background:#9a6700"></span>bridged / draft —— 桥接层待产出</span>'
        '<span><span class="sw" style="background:#cf222e"></span>absent / blocked —— 两侧都缺，阻塞项</span>'
        "</div>"
    )

    # ---- 字段明细
    parts.append("<h2>4. 字段明细</h2>")
    for group in spec["groups"]:
        group_stats = stats["groups"][group["id"]]
        parts.append(
            f'<h3>{group["id"]} · {html_escape(group.get("label", ""))} '
            f'<span class="note">({group_stats["frozen_count"]}/{group_stats["field_count"]} frozen)</span></h3>'
        )
        parts.append(
            "<table><thead><tr><th>字段</th><th>unit</th><th>frame</th><th>dtype</th>"
            "<th>time</th><th>sim</th><th>nova</th><th>status</th><th>真值来源 / 备注</th></tr></thead><tbody>"
        )
        for field in group["fields"]:
            status = field.get("status", "-")
            sim = field.get("sim", "-")
            parts.append(
                f'<tr><td><code>{html_escape(field.get("id"))}</code></td>'
                f'<td>{html_escape(field.get("unit", "-"))}</td>'
                f'<td>{html_escape(field.get("frame", "-"))}</td>'
                f'<td>{html_escape(field.get("dtype", "-"))}</td>'
                f'<td>{html_escape(field.get("time", "-"))}</td>'
                f'<td><span class="pill" style="background:{SIM_COLOR.get(sim, "#57606a")}">{sim}</span></td>'
                f'<td>{html_escape(field.get("nova", "-"))}</td>'
                f'<td><span class="pill" style="background:{STATUS_COLOR.get(status, "#57606a")}">{status}</span></td>'
                f'<td>{html_escape(field.get("truth", ""))}'
            )
            note = field.get("note")
            if note:
                parts.append(f'<br><span class="note">{html_escape(note)}</span>')
            parts.append("</td></tr>")
        parts.append("</tbody></table>")

    # ---- 阻塞清单
    blocked = [
        (group_id, field)
        for group_id, _, field in iter_fields(spec)
        if field.get("status") == "blocked"
    ]
    parts.append("<h2>5. 阻塞字段（spec 层未解）</h2>")
    if blocked:
        parts.append("<table><thead><tr><th>组</th><th>字段</th><th>说明</th></tr></thead><tbody>")
        for group_id, field in blocked:
            parts.append(
                f'<tr><td><code>{group_id}</code></td>'
                f'<td><code>{html_escape(field.get("id"))}</code></td>'
                f'<td>{html_escape(field.get("note", ""))}</td></tr>'
            )
        parts.append("</tbody></table>")
    else:
        parts.append('<div class="ok">无阻塞字段。</div>')

    parts.append(
        '<p class="sub" style="margin-top:40px">由 <code>scripts/spec_report.py</code> 生成，'
        "勿手改。数据源 <code>spec/canonical_v0.yaml</code>。</p></body></html>"
    )
    return "".join(parts)


def main():
    parser = argparse.ArgumentParser(description="canonical spec 覆盖率报告器")
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--check", action="store_true", help="只校验，不写 HTML")
    args = parser.parse_args()

    spec = load_spec(args.spec)
    field_problems, global_problems = find_problems(spec)
    stats = compute_stats(spec)
    failures = evaluate_gate(spec, stats, field_problems, global_problems)

    print(f"spec        : {args.spec}")
    print(f"fields      : {stats['total_fields']} (groups={len(stats['groups'])})")
    print(
        "status      : "
        + " / ".join(f"{key}={value}" for key, value in stats["status_counts"].items())
    )
    print(
        "sim         : "
        + " / ".join(f"{key}={value}" for key, value in stats["sim_counts"].items())
    )
    print(f"frozen ratio: {stats['frozen_ratio']:.1%} (gate >= {MIN_FROZEN_RATIO:.0%})")
    for group_id, group in stats["groups"].items():
        print(
            f"  - {group_id:10s} fields={group['field_count']:3d} "
            f"frozen={group['frozen_count']:3d}"
        )

    for item in field_problems:
        print(f"PROBLEM [{item[0]}] {item[1]}: {item[2]}")
    for item in global_problems:
        print(f"PROBLEM [spec] {item}")

    if failures:
        for item in failures:
            print(f"GATE FAIL   : {item}")
    else:
        print("GATE        : PASS")

    if not args.check:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(render_html(spec, stats, failures), encoding="utf-8")
        print(f"report      : {args.out}")

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
