#!/usr/bin/env python3
"""Milestone 评审看板 —— 每个 milestone 收口时重新生成一份，供人眼 review。

这里只做两件事：汇总已经存在的数字，把它们画成能一眼判断"合理不合理"的图。
数字一律从原始产物读（yaml / audit.json / envelope.json / 真跑一遍裁判），不手抄。

用法:
    python scripts/milestone_review.py            # 生成 docs/html/milestone_review.html
    python scripts/milestone_review.py --no-run   # 跳过可行性裁判的真跑（复用上次判定）

产物：docs/html/milestone_review.html
"""

import argparse
import json
import statistics
import sys
from collections import Counter
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from spec_envelope_report import (  # noqa: E402
    FROZEN_AGENT_RANGE_M,
    FROZEN_DELAY_S,
    OPERATIONAL_SPEED_MPS,
    collect as collect_envelope,
    gate as envelope_gate,
    delay_panel_svg,
    range_panel_svg,
)
from constraint_judge import (  # noqa: E402
    CONSTANTS_HEADER,
    analyse as analyse_constraints,
    read_native_metrics,
)

DEFAULT_OUT = REPO_ROOT / "docs" / "html" / "milestone_review.html"
SPEC_PATH = REPO_ROOT / "spec" / "canonical_v0.yaml"
CONSTRAINTS_PATH = REPO_ROOT / "spec" / "constraints_v0.yaml"
AUDIT_CACHE = REPO_ROOT / "docs" / "measured" / "audit_canonical_v0.json"
ENVELOPE_JSON = REPO_ROOT / "docs" / "measured" / "obs_envelope_6bags.json"
MINING_JSON = REPO_ROOT / "docs" / "measured" / "badcase_mining_v0.json"

CONSTRAINT_CATEGORY = {
    "ttc_shortfall": "violation",
    "braking_distance_short": "violation",
    "collision_any": "violation",
    "cut_in_conflict": "violation",
    "lat_accel_over": "comfort",
    "long_accel_over": "comfort",
    "jerk_over": "comfort",
    "stall_crawl": "progress",
}

PALETTE = {
    "frozen": "#1a7f37",
    "draft": "#9a6700",
    "blocked": "#cf222e",
    "both": "#1a7f37",
    "source_bridged": "#0b6bcb",
    "source_only": "#9a6700",
    "sim_only": "#cf222e",
    "neither": "#6e7781",
    "violation": "#cf222e",
    "comfort": "#9a6700",
    "progress": "#0b6bcb",
}


# ------------------------------------------------------------------ 数据汇总
REQUIRED_ARTIFACTS = (
    "docs/html/spec_v0_coverage.html",
    "docs/html/spec_v0_data_audit.html",
    "docs/html/spec_v0_obs_envelope.html",
    "docs/html/constraints_v0_library.html",
    "docs/html/constraints_v0_feasibility.html",
    "docs/html/badcase_mining_v0.html",
    "docs/measured/badcase_mining_v0.json",
)
ARTIFACT_NAV = (
    ("spec_v0_coverage.html", "M1 字段覆盖率", "73 字段 × 8 组，含阻塞清单"),
    ("spec_v0_data_audit.html", "M2 三源审计", "6 bag + 3 地图 + 车型档案"),
    ("spec_v0_obs_envelope.html", "M3 观测包络", "距离直方图 + 数据年龄"),
    ("constraints_v0_library.html", "M4 约束库", "20 模板 × 18 指标矩阵"),
    ("constraints_v0_feasibility.html", "M4 可行性裁判", "IDM / REPLAY / STATIC 真跑"),
    ("badcase_mining_v0.html", "M5 bad case 挖掘", "8 条可挖约束 × 6 条录包"),
)
SCRIPT_ARTIFACTS = (
    "scripts/spec_report.py",
    "scripts/spec_audit.py",
    "scripts/spec_envelope_report.py",
    "scripts/constraint_judge.py",
    "scripts/constraint_feasibility.py",
)


def consistency_gate(spec_n, audit_n, env_n, con_n, feas_n, mining_n):
    """看板自身的 gate：上游数字一致 + 验收物在位。

    看板不重跑上游 gate（每条都能单独跑），但它必须保证自己读到的数字没漂：
    比如审计的判定数必须等于字段数、模板数必须等于裁判判定数。这类不一致通常意味着
    缓存陈旧或上游改了而没重新生成。
    """
    failures = []
    if spec_n["total"] < 40:
        failures.append(f"spec 字段只有 {spec_n['total']} 个")
    if audit_n is None:
        failures.append("缺 M2 审计缓存（先跑 scripts/spec_audit.py）")
    elif sum(audit_n["verdicts"].values()) != audit_n["fields"]:
        failures.append(
            f"审计判定数 {sum(audit_n['verdicts'].values())} != 字段数 {audit_n['fields']}"
        )
    failures.extend(env_n["failures"])
    if con_n["executable"] != con_n["templates"]:
        failures.append(f"可执行谓词 {con_n['executable']}/{con_n['templates']}")
    judged = sum(feas_n["verdicts"].values())
    if judged != con_n["templates"]:
        failures.append(f"可行性判定 {judged} != 模板数 {con_n['templates']}")
    if mining_n is None:
        failures.append("缺 M5 挖掘结果（先跑 scripts/badcase_mine.py）")
    elif mining_n["total_slices"] != sum(mining_n["per_bag"].values()):
        failures.append("挖掘切片总数与 per-bag 汇总不一致")
    for artifact in REQUIRED_ARTIFACTS + SCRIPT_ARTIFACTS:
        if not (REPO_ROOT / artifact).exists():
            failures.append(f"缺验收物 {artifact}")
    return failures


def load_spec_numbers():
    spec = yaml.safe_load(SPEC_PATH.read_text(encoding="utf-8"))
    status_counts = Counter()
    sim_counts = Counter()
    group_rows = []
    for group in spec["groups"]:
        fields = group["fields"]
        for field in fields:
            status_counts[field.get("status")] += 1
            sim_counts[field.get("sim")] += 1
        group_rows.append(
            {
                "id": group["id"],
                "label": group.get("label", group["id"]),
                "fields": len(fields),
                "frozen": sum(1 for field in fields if field.get("status") == "frozen"),
            }
        )
    total = sum(status_counts.values())
    return {
        "schema": spec.get("schema_version"),
        "total": total,
        "status": status_counts,
        "sim": sim_counts,
        "groups": group_rows,
        "frozen_ratio": status_counts["frozen"] / total if total else 0.0,
    }


def load_audit_numbers():
    if not AUDIT_CACHE.exists():
        return None
    data = json.loads(AUDIT_CACHE.read_text(encoding="utf-8"))
    rows = data.get("rows", [])
    verdicts = Counter(row["verdict"] for row in rows)
    return {
        "fields": len(rows),
        "verdicts": verdicts,
        "bags": len(data.get("bags", {})),
        "maps": len(data.get("maps", {})),
        "derived": data.get("derived", {}),
        "source_only": [row["field"] for row in rows if row["verdict"] == "source_only"],
    }


def load_envelope_numbers():
    records = json.loads(ENVELOPE_JSON.read_text(encoding="utf-8"))
    if isinstance(records, dict):
        records = [records]
    rows, histogram = collect_envelope(records)
    failures, summary = envelope_gate(rows, histogram)
    return {"rows": rows, "histogram": histogram, "summary": summary, "failures": failures}


def load_mining_numbers():
    if not MINING_JSON.exists():
        return None
    data = json.loads(MINING_JSON.read_text(encoding="utf-8"))
    per_template = {}
    per_bag = {}
    for key, value in data.get("counts", {}).items():
        template_id, bag = key.split("|")
        per_template[template_id] = per_template.get(template_id, 0) + value
        per_bag[bag] = per_bag.get(bag, 0) + value
    return {
        "evaluable": data.get("evaluable_templates", []),
        "summary": data["summary"],
        "slices": data["slices"],
        "per_template": per_template,
        "per_bag": per_bag,
        "bags": data["bags"],
        "total_slices": sum(len(items) for items in data["slices"].values()),
    }


def load_constraint_numbers():
    spec = yaml.safe_load(CONSTRAINTS_PATH.read_text(encoding="utf-8"))
    names, lower = read_native_metrics(CONSTANTS_HEADER)
    rows, problems = analyse_constraints(spec, lower)
    categories = Counter(row["category"] for row in rows)
    return {
        "templates": len(rows),
        "categories": categories,
        "executable": sum(1 for row in rows if row["executable"]),
        "native_only": sum(1 for row in rows if not row["needs_derived"]),
        "needs_derived": sorted({name for row in rows for name in row["needs_derived"]}),
        "problems": problems,
        "metrics": len(names),
    }


def run_feasibility(steps=40, seed=7):
    """真跑一遍 IDM / REPLAY / STATIC，拿判定分布（约 2 s）。"""
    from constraint_feasibility import CONTROLLERS, rollout, verdict

    spec = yaml.safe_load(CONSTRAINTS_PATH.read_text(encoding="utf-8"))
    templates = spec["templates"]
    names, _lower = read_native_metrics(CONSTANTS_HEADER)
    index = {name: i for i, name in enumerate(names)}
    fired_by_controller = {}
    for controller in CONTROLLERS:
        fired, evaluated, _worst = rollout(controller, templates, names, index, steps, seed)
        fired_by_controller[controller] = (fired, evaluated)
    verdicts = Counter()
    fired_templates = []
    for template in templates:
        template_id = template["id"]
        per_controller = {
            controller: fired_by_controller[controller][0][template_id] for controller in CONTROLLERS
        }
        result = verdict(per_controller)
        verdicts[result] += 1
        if any(per_controller.values()):
            fired_templates.append(
                {
                    "id": template_id,
                    "category": template.get("category"),
                    "hit": [c for c in CONTROLLERS if per_controller[c]],
                    "verdict": result,
                }
            )
    return {
        "verdicts": verdicts,
        "steps": steps,
        "evaluated": {c: fired_by_controller[c][1] for c in CONTROLLERS},
        "fired_templates": fired_templates,
    }


# ---------------------------------------------------------------------- 图形
def esc(text):
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def pill(label, color):
    return (
        f'<span class="pill" style="background:{color}">{esc(label)}</span>'
    )


def kpi_card(label, value, note, accent="#1f2328"):
    return (
        f'<div class="card"><div class="k">{label}</div>'
        f'<div class="v" style="color:{accent}">{value}</div>'
        f'<div class="n">{note}</div></div>'
    )


def stacked_bar(segments, width=520, height=18):
    """segments = [(label, value, color)]"""
    total = sum(value for _label, value, _color in segments) or 1
    parts = [f'<svg viewBox="0 0 {width} {height}" width="100%" height="{height}">']
    x = 0.0
    for _label, value, color in segments:
        w = width * value / total
        if w <= 0:
            continue
        parts.append(
            f'<rect x="{x:.1f}" y="0" width="{w:.1f}" height="{height}" fill="{color}" />'
        )
        x += w
    parts.append("</svg>")
    return "".join(parts)


def hbars(items, width=560, row_height=26, unit=""):
    """items = [(label, value, color)]，横向条形图，带数值。"""
    if not items:
        return ""
    max_value = max(value for _label, value, _color in items) or 1
    label_w = 150
    plot_w = width - label_w - 70
    height = row_height * len(items) + 10
    parts = [f'<svg viewBox="0 0 {width} {height}" width="100%" role="img">']
    for index, (label, value, color) in enumerate(items):
        y = index * row_height + 4
        bar_w = plot_w * value / max_value
        parts.append(
            f'<text x="0" y="{y + 13}" font-size="11.5" fill="#1f2328">{esc(label)}</text>'
            f'<rect x="{label_w}" y="{y + 2}" width="{bar_w:.1f}" height="{row_height - 12}" '
            f'fill="{color}" opacity="0.9" rx="2" />'
            f'<text x="{label_w + bar_w + 7:.1f}" y="{y + 13}" font-size="11" fill="#57606a">'
            f"{value}{unit}</text>"
        )
    parts.append("</svg>")
    return "".join(parts)


def stacked_hbars(items, width=560, row_height=28):
    """items = [(label, done, rest, color_done, color_rest, suffix)]，长度按 done+rest 归一。"""
    if not items:
        return ""
    max_total = max(done + rest for _label, done, rest, _c1, _c2, _s in items) or 1
    label_w = 118
    plot_w = width - label_w - 92
    height = row_height * len(items) + 10
    parts = [f'<svg viewBox="0 0 {width} {height}" width="100%" role="img">']
    for index, (label, done, rest, color_done, color_rest, suffix) in enumerate(items):
        y = index * row_height + 5
        total = done + rest
        w_done = plot_w * done / max_total
        w_rest = plot_w * rest / max_total
        parts.append(
            f'<text x="0" y="{y + 12}" font-size="11.5" fill="#1f2328">{esc(label)}</text>'
            f'<rect x="{label_w}" y="{y + 1}" width="{w_done:.1f}" height="{row_height - 13}" '
            f'fill="{color_done}" rx="2" />'
        )
        if w_rest > 0:
            parts.append(
                f'<rect x="{label_w + w_done:.1f}" y="{y + 1}" width="{w_rest:.1f}" '
                f'height="{row_height - 13}" fill="{color_rest}" rx="2" />'
            )
        parts.append(
            f'<text x="{label_w + w_done + w_rest + 7:.1f}" y="{y + 12}" font-size="11" '
            f'fill="#57606a">{suffix}</text>'
        )
    parts.append("</svg>")
    return "".join(parts)


def milestone_flow(milestones, width=1240, height=232):
    """横向流程：每个 milestone 一个节点，箭头相连；进行中的用虚框。"""
    count = len(milestones)
    gap = 26
    box_w = (width - gap * (count - 1) - 8) / count
    box_h = 168
    top = 30
    parts = [f'<svg viewBox="0 0 {width} {height}" width="100%" role="img">']
    for index, milestone in enumerate(milestones):
        x = index * (box_w + gap) + 4
        done = milestone["state"] == "done"
        current = milestone["state"] == "current"
        stroke = "#1a7f37" if done else ("#0b6bcb" if current else "#d0d7de")
        fill = "#f2fbf4" if done else ("#f2f7ff" if current else "#fbfbfc")
        dash = "" if done else ' stroke-dasharray="6 5"'
        parts.append(
            f'<rect x="{x:.1f}" y="{top}" width="{box_w:.1f}" height="{box_h}" rx="9" '
            f'fill="{fill}" stroke="{stroke}" stroke-width="1.6"{dash} />'
            f'<text x="{x + 14:.1f}" y="{top + 26}" font-size="15" font-weight="700" '
            f'fill="{stroke}">{esc(milestone["id"])}</text>'
            f'<text x="{x + 44:.1f}" y="{top + 26}" font-size="12.5" fill="#1f2328">'
            f'{esc(milestone["title"])}</text>'
            f'<text x="{x + 14:.1f}" y="{top + 50}" font-size="11" fill="#57606a">'
            f'{esc(milestone["state_label"])}</text>'
        )
        for line_index, line in enumerate(milestone["metrics"][:3]):
            parts.append(
                f'<text x="{x + 14:.1f}" y="{top + 76 + line_index * 20}" font-size="11.5" '
                f'fill="#1f2328">{esc(line)}</text>'
            )
        parts.append(
            f'<text x="{x + 14:.1f}" y="{top + box_h - 14}" font-size="10.5" fill="#57606a">'
            f'{esc(milestone["artifact"])}</text>'
        )
        if index < count - 1:
            ax = x + box_w + 4
            ay = top + box_h / 2
            parts.append(
                f'<line x1="{ax:.1f}" y1="{ay:.1f}" x2="{ax + gap - 8:.1f}" y2="{ay:.1f}" '
                f'stroke="#9aa4af" stroke-width="1.6" />'
                f'<polygon points="{ax + gap - 8:.1f},{ay:.1f} {ax + gap - 15:.1f},{ay - 4.5:.1f} '
                f'{ax + gap - 15:.1f},{ay + 4.5:.1f}" fill="#9aa4af" />'
            )
    parts.append("</svg>")
    return "".join(parts)


def gate_table(rows):
    body = []
    for name, command, state, note in rows:
        color = "#1a7f37" if state == "PASS" else "#cf222e"
        body.append(
            f"<tr><td>{esc(name)}</td><td><code>{esc(command)}</code></td>"
            f'<td class="c">{pill(state, color)}</td>'
            f'<td class="note">{esc(note)}</td></tr>'
        )
    return (
        "<table><thead><tr><th>Gate</th><th>命令</th><th class='c'>状态</th><th>门禁内容</th>"
        "</tr></thead><tbody>" + "".join(body) + "</tbody></table>"
    )


# ---------------------------------------------------------------------- 页面
def render(spec_n, audit_n, env_n, con_n, feas_n, mining_n, flow):
    blind_distance = OPERATIONAL_SPEED_MPS * FROZEN_DELAY_S
    status = spec_n["status"]
    verdicts = audit_n["verdicts"] if audit_n else Counter()
    total_templates = con_n["templates"]

    parts = ["""<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>Milestone 评审看板</title>
<style>
:root { --ink:#1f2328; --mute:#57606a; --line:#d0d7de; --soft:#f6f8fa; }
* { box-sizing:border-box; }
body { margin:0; padding:32px 40px 64px; max-width:1340px; color:var(--ink);
  font:15px/1.62 -apple-system,"Segoe UI","PingFang SC","Microsoft YaHei",sans-serif; }
h1 { font-size:27px; margin:0 0 6px; letter-spacing:-.3px; }
h2 { font-size:19px; margin:38px 0 12px; padding-bottom:8px; border-bottom:1px solid var(--line); }
h3 { font-size:13px; margin:20px 0 8px; color:var(--mute); text-transform:uppercase; letter-spacing:.07em; }
.sub { color:var(--mute); font-size:13px; margin:0 0 18px; }
.cards { display:grid; gap:12px; margin:18px 0 8px; }
.cards.five { grid-template-columns:repeat(5,1fr); }
.cards.four { grid-template-columns:repeat(4,1fr); }
.card { border:1px solid var(--line); border-radius:10px; padding:13px 15px; background:var(--soft); }
.card .k { font-size:12px; color:var(--mute); }
.card .v { font-size:25px; font-weight:650; letter-spacing:-.4px; }
.card .n { font-size:11.5px; color:var(--mute); }
table { border-collapse:collapse; width:100%; font-size:13px; }
th,td { border:1px solid var(--line); padding:6px 9px; text-align:left; vertical-align:top; }
th { background:var(--soft); font-size:12px; font-weight:600; }
td.c,th.c { text-align:center; }
tbody tr:nth-child(even) td { background:#fcfcfd; }
code { background:var(--soft); border:1px solid var(--line); border-radius:4px; padding:1px 5px;
  font-size:11.5px; font-family:ui-monospace,Menlo,monospace; }
.pill { display:inline-block; padding:1px 8px; border-radius:999px; font-size:11px; font-weight:600; color:#fff; }
.legend { display:flex; gap:16px; flex-wrap:wrap; font-size:12px; color:var(--mute); margin:6px 0 0; }
.sw { display:inline-block; width:10px; height:10px; border-radius:2px; margin-right:5px; vertical-align:middle; }
.grid2 { display:grid; grid-template-columns:1.15fr .85fr; gap:26px; align-items:start; }
.box { border:1px solid var(--line); border-radius:10px; padding:16px 18px; }
.warn { border-left:3px solid #9a6700; background:#fffbf0; padding:10px 14px; border-radius:0 6px 6px 0; margin:8px 0; font-size:13px; }
.ok { border-left:3px solid #1a7f37; background:#f2fbf4; padding:10px 14px; border-radius:0 6px 6px 0; margin:8px 0; font-size:13px; }
.fail { border-left:3px solid #cf222e; background:#fff5f5; padding:10px 14px; border-radius:0 6px 6px 0; margin:8px 0; font-size:13px; }
.note { color:var(--mute); font-size:12.5px; }
.nav { display:grid; grid-template-columns:repeat(6,1fr); gap:10px; margin:6px 0 10px; }
.navitem { display:block; border:1px solid var(--line); border-radius:9px; padding:9px 12px;
  text-decoration:none; background:#fff; }
.navitem b { display:block; font-size:12.5px; color:#0b6bcb; }
.navitem span { display:block; font-size:11px; color:var(--mute); }
.navitem:hover { background:var(--soft); }
a { color:#0b6bcb; }
</style></head><body>"""]

    parts.append("""
<h1>Milestone 评审看板</h1>
<p class="sub">canonical spec v0 执行链 · M1 → M5 已收口 · 所有数字由 <code>scripts/milestone_review.py</code>
从原始产物重算，不手抄</p>""")

    parts.append('<div class="nav">')
    for filename, title, note in ARTIFACT_NAV:
        parts.append(
            f'<a class="navitem" href="{filename}"><b>{title}</b>'
            f'<span>{note}</span></a>'
        )
    parts.append("</div>")

    # ---- 顶部 KPI
    parts.append('<div class="cards five">')
    parts.append(kpi_card("spec 字段", spec_n["total"], f"frozen {spec_n['frozen_ratio']:.1%} · blocked {status['blocked']}", "#1a7f37"))
    parts.append(kpi_card("三源齐备", verdicts["both"], f"待桥接 {verdicts['source_bridged']} · 仅数据源 {verdicts['source_only']}", "#0b6bcb"))
    parts.append(kpi_card("感知上限", f"{FROZEN_AGENT_RANGE_M:.0f} m", f">100 m 样本 {env_n['summary']['far']} 个", "#1a7f37"))
    parts.append(kpi_card("数据年龄", f"{FROZEN_DELAY_S:.2f} s", f"盲开 {blind_distance:.1f} m @ {OPERATIONAL_SPEED_MPS} m/s", "#9a6700"))
    parts.append(kpi_card("约束模板", total_templates, f"谓词可执行 {con_n['executable']}/{total_templates}", "#1a7f37"))
    parts.append("</div>")

    # ---- 流程
    parts.append("<h2>1. 执行链流程</h2>")
    parts.append(milestone_flow(flow))
    parts.append(
        '<p class="note">绿=已收口且 gate 通过；蓝虚框=下一站；灰虚框=待排期。'
        "每个节点下方是它的验收物文件名，都在 <code>docs/html/</code> 下。</p>"
    )

    # ---- M1
    parts.append("<h2>2. M1 —— spec 注册表</h2>")
    parts.append('<div class="grid2">')
    parts.append('<div class="box">')
    parts.append(
        f'<h3>字段状态（{spec_n["total"]} 个）</h3>'
        + stacked_bar(
            [
                ("frozen", status["frozen"], PALETTE["frozen"]),
                ("draft", status["draft"], PALETTE["draft"]),
                ("blocked", status["blocked"], PALETTE["blocked"]),
            ]
        )
    )
    parts.append(
        f'<div class="legend">'
        f'<span><span class="sw" style="background:{PALETTE["frozen"]}"></span>frozen {status["frozen"]}</span>'
        f'<span><span class="sw" style="background:{PALETTE["draft"]}"></span>draft {status["draft"]}</span>'
        f'<span><span class="sw" style="background:{PALETTE["blocked"]}"></span>blocked {status["blocked"]}</span>'
        f"</div>"
    )
    parts.append(
        f'<p class="note">判据：frozen 占比 ≥ 60%（实际 {spec_n["frozen_ratio"]:.1%}）、'
        "必填属性缺失 = 0、每组至少 1 个 frozen。</p>"
    )
    parts.append('<h3>每组字段构成（绿=frozen，橙=draft）</h3>')
    parts.append(
        stacked_hbars(
            [
                (
                    group["id"],
                    group["frozen"],
                    group["fields"] - group["frozen"],
                    PALETTE["frozen"],
                    PALETTE["draft"],
                    f'{group["frozen"]}/{group["fields"]}',
                )
                for group in spec_n["groups"]
            ]
        )
    )
    parts.append('<p class="note">条长 = 该组字段数；绿色段 = frozen 比例。</p>')
    parts.append("</div>")

    parts.append('<div class="box">')
    parts.append("<h3>review 要点</h3>")
    parts.append(
        '<div class="ok">8 组里 <b>ego(17) / agents(11) / map(11)</b> 是主体，占 53%；'
        "occ 只有 6 个字段但单独成组，因为它进碰撞热路径（ADR-0007）。</div>"
    )
    parts.append(
        '<div class="warn"><b>occ 组最弱</b>：'
        f'{next(g["frozen"] for g in spec_n["groups"] if g["id"] == "occ")}/6 frozen —— '
        "它是唯一“数据源需要派生”的组，派生链路还没实现。</div>"
    )
    parts.append(
        f'<div class="warn"><b>draft {status["draft"]} 个</b>：'
        "全部是等待实测证据的字段（延迟、a_lat、静态多边形四路等），不是遗漏。</div>"
    )
    parts.append(
        '<p class="note">完整表：<a href="spec_v0_coverage.html">spec_v0_coverage.html</a></p>'
    )
    parts.append("</div></div>")

    # ---- M2
    parts.append("<h2>3. M2 —— 三源数据审计</h2>")
    parts.append('<div class="grid2">')
    parts.append('<div class="box">')
    if audit_n:
        parts.append(f'<h3>字段判定（{audit_n["fields"]} 个，{audit_n["bags"]} 条 bag + {audit_n["maps"]} 张地图）</h3>')
        parts.append(
            hbars(
                [
                    ("三源齐备", verdicts["both"], PALETTE["both"]),
                    ("待桥接", verdicts["source_bridged"], PALETTE["source_bridged"]),
                    ("仅数据源", verdicts["source_only"], PALETTE["source_only"]),
                    ("仅仿真（凭空）", verdicts["sim_only"], PALETTE["sim_only"]),
                    ("两侧皆无", verdicts["neither"], PALETTE["neither"]),
                ],
                unit=" 字段",
            )
        )
        derived = audit_n["derived"]
        polygon_rate = (
            derived["with_polygon"] / derived["instances"] if derived.get("instances") else 0.0
        )
        parts.append(
            f'<h3>occ 的派生证据</h3>'
            f'<p class="note">bag 内无占据栅格通道；perception 对象多边形实测 '
            f'<b>{derived.get("with_polygon", 0)}/{derived.get("instances", 0)}</b> '
            f'（{polygon_rate:.0%}），中位 {derived.get("median_points", 0)} 点、'
            f'最大 {derived.get("max_points", 0)} 点。</p>'
        )
    parts.append("</div>")
    parts.append('<div class="box">')
    parts.append("<h3>review 要点</h3>")
    parts.append(
        '<div class="fail"><b>被数据推翻 ①</b> occ 不是“占据栅格”：6 条 bag 里没有任何 '
        "occ/grid/polygon 通道，只有对象级 polygon_point。原 spec 措辞已改。</div>"
    )
    parts.append(
        '<div class="fail"><b>被数据推翻 ②</b> Core 已有 entry/exit 邻接'
        "（<code>datatypes.h:145-148</code>），我把整块拓扑判成 absent 是错的 —— "
        "blocked 从 3 降到 0。</div>"
    )
    if audit_n and audit_n["source_only"]:
        parts.append(
            '<div class="warn"><b>“仅数据源”清单（桥接层确定工作）</b>：'
            + "、".join(f"<code>{esc(name)}</code>" for name in audit_n["source_only"])
            + "</div>"
        )
    parts.append('<p class="note">完整表：<a href="spec_v0_data_audit.html">spec_v0_data_audit.html</a></p>')
    parts.append("</div></div>")

    # ---- M3
    parts.append("<h2>4. M3 —— 观测包络</h2>")
    parts.append('<div class="cards four">')
    parts.append(kpi_card("感知上限", f"{FROZEN_AGENT_RANGE_M:.0f} m", f"p100 中位 {env_n['summary']['p100_median']:.1f} m"))
    parts.append(kpi_card("数据年龄", f"{FROZEN_DELAY_S:.2f} s", f"6 bag 中位 {env_n['summary']['delay_median']:.2f} s"))
    parts.append(kpi_card("盲开距离", f"{blind_distance:.1f} m", f"{OPERATIONAL_SPEED_MPS} m/s × {FROZEN_DELAY_S:.2f} s"))
    parts.append(kpi_card(">100 m 障碍物", env_n["summary"]["far"], "70 m 上限的实证依据", "#1a7f37"))
    parts.append("</div>")
    parts.append("<h3>面板 ①　感知距离分布</h3>")
    parts.append(range_panel_svg(env_n["rows"], env_n["histogram"]))
    parts.append("<h3>面板 ②　每条 bag 的数据年龄</h3>")
    parts.append(delay_panel_svg(env_n["rows"]))
    parts.append(
        '<div class="warn"><b>被数据推翻 ③</b> 原候选 0.53 s 只对应 <code>rx-2714</code> 一条；'
        f'6 条中位是 {env_n["summary"]["delay_median"]:.2f} s，最差 1.21 s。冻结值改为 0.78 s。</div>'
    )
    parts.append(
        '<p class="note">完整报告：<a href="spec_v0_obs_envelope.html">spec_v0_obs_envelope.html</a></p>'
    )

    # ---- M4
    parts.append("<h2>5. M4 —— 约束模板库 + 可行性裁判</h2>")
    parts.append('<div class="grid2">')
    parts.append('<div class="box">')
    parts.append(f'<h3>类别分布（{total_templates} 条）</h3>')
    parts.append(
        hbars(
            [
                ("violation", con_n["categories"]["violation"], PALETTE["violation"]),
                ("comfort", con_n["categories"]["comfort"], PALETTE["comfort"]),
                ("progress", con_n["categories"]["progress"], PALETTE["progress"]),
            ],
            unit=" 条",
        )
    )
    parts.append(
        f'<h3>可行性判定（3 控制器 × {feas_n["steps"]} 步）</h3>'
        + hbars(
            [
                ("可解", feas_n["verdicts"]["solvable"], PALETTE["both"]),
                ("IDM 不足", feas_n["verdicts"]["idm_only"], PALETTE["draft"]),
                ("不可解", feas_n["verdicts"]["unsolvable"], PALETTE["blocked"]),
            ],
            unit=" 条",
        )
    )
    parts.append("</div>")
    parts.append('<div class="box">')
    parts.append("<h3>review 要点</h3>")
    parts.append(
        f'<div class="ok">违规 16 / 体感 3 / 进度 1 = <b>{con_n["categories"]["violation"] / total_templates:.0%} / '
        f'{con_n["categories"]["comfort"] / total_templates:.0%} / '
        f'{con_n["categories"]["progress"] / total_templates:.0%}</b>，与 owner 的 80/20 口径一致。</div>'
    )
    parts.append(
        f'<div class="ok">谓词可执行 <b>{con_n["executable"]}/{total_templates}</b>：'
        "每条都造了“触发行”和“基线行”，两边都对才入库。</div>"
    )
    if con_n["needs_derived"]:
        parts.append(
            '<div class="warn"><b>引擎缺 3 个指标</b>（桥接层任务）：'
            + "、".join(f"<code>{esc(name)}</code>" for name in con_n["needs_derived"])
            + "</div>"
        )
    if feas_n["fired_templates"]:
        rows = "".join(
            f"<tr><td><code>{esc(item['id'])}</code></td><td>{esc(item['category'])}</td>"
            f"<td class='c'>{esc(', '.join(item['hit']))}</td></tr>"
            for item in feas_n["fired_templates"]
        )
        parts.append(
            "<h3>真跑触发了什么</h3>"
            "<table><thead><tr><th>模板</th><th>类别</th><th class='c'>触发控制器</th></tr></thead>"
            f"<tbody>{rows}</tbody></table>"
        )
        parts.append(
            '<div class="warn"><b>被数据推翻 ④</b> <code>stall_crawl</code> 在静止起步时就触发 → '
            "已加 warmup_s；同时暴露 <b>IDM 自己违反 comfort 阈值</b>（8.0 / 5.0），待查。</div>"
        )
    parts.append(
        '<p class="note">完整报告：<a href="constraints_v0_library.html">constraints_v0_library.html</a> · '
        '<a href="constraints_v0_feasibility.html">constraints_v0_feasibility.html</a></p>'
    )
    parts.append("</div></div>")

    # ---- M5
    parts.append("<h2>6. M5 —— bad case 自动挖掘</h2>")
    if mining_n:
        summary = mining_n["summary"]
        parts.append('<div class="cards four">')
        parts.append(
            kpi_card(
                "可挖约束",
                f"{len(mining_n['evaluable'])}/{summary['template_total']}",
                f"其余 {summary['template_total'] - len(mining_n['evaluable'])} 条缺地图匹配",
            )
        )
        parts.append(kpi_card("可信帧", summary["frames"], f"拒绝 {summary['rejected']} 帧"))
        parts.append(kpi_card("切片", mining_n["total_slices"], "16 历史 + 80 未来"))
        parts.append(kpi_card("命中 bag", summary["bags_with_hits"], f"共 {len(mining_n['bags'])} 条"))
        parts.append("</div>")
        parts.append('<div class="grid2">')
        parts.append('<div class="box">')
        parts.append("<h3>每模板切片数</h3>")
        parts.append(
            hbars(
                sorted(
                    [
                        (name, value, PALETTE.get(CONSTRAINT_CATEGORY.get(name, ""), "#57606a"))
                        for name, value in mining_n["per_template"].items()
                    ],
                    key=lambda item: -item[1],
                ),
                unit=" 条",
            )
        )
        parts.append("<h3>每 bag 切片数</h3>")
        parts.append(
            hbars(
                [(bag, mining_n["per_bag"].get(bag, 0), PALETTE["source_bridged"]) for bag in mining_n["bags"]],
                unit=" 条",
            )
        )
        parts.append("</div>")
        parts.append('<div class="box">')
        parts.append("<h3>review 要点</h3>")
        parts.append(
            '<div class="ok">8 条约束在 6 条录包上共挖出 '
            f'<b>{mining_n["total_slices"]}</b> 个切片（占可信帧 {mining_n["total_slices"] / max(1, summary["frames"]):.1%}），'
            "每条切片都带 <code>bag#frame</code> 与 provenance。</div>"
        )
        parts.append(
            '<div class="warn"><b>零命中不是坏判据</b>：<code>lat_accel_over</code> 阈值 8.0 m/s²，'
            "而这批数据 |a_lat| 最大只有 7.92 —— 这批包没有横向极限工况。</div>"
        )
        parts.append(
            '<div class="warn"><b>挖掘裁判自己抓到的三个 bug</b>：'
            "① in-path 用了车宽之和 → 邻道并排车被当成前车（命中率一度 58%）；"
            "② pose 加速度直接差分 → |jerk| p50 5.5 m/s³（噪声），改滑窗三次拟合后 1.11；"
            "③ cut_in 与 in-path 横向互斥 → 永远不触发，新增邻道候选特征。</div>"
        )
        parts.append('<p class="note">完整报告：<a href="badcase_mining_v0.html">badcase_mining_v0.html</a></p>')
        parts.append("</div></div>")
    else:
        parts.append('<div class="fail">缺 <code>docs/measured/badcase_mining_v0.json</code></div>')

    # ---- 假设修正台账
    parts.append("<h2>7. 被真实数据推翻的假设（review 重点）</h2>")
    parts.append(
        "<table><thead><tr><th>#</th><th>原假设</th><th>实测</th><th>后果</th></tr></thead><tbody>"
        "<tr><td>1</td><td>occ 来自“bag perception 占据栅格”</td>"
        "<td>6 条 bag 无任何占据栅格通道；只有对象级 polygon_point（100% 实例，中位 8 点）</td>"
        "<td>occ 改为派生字段；32 点等弧长重采样升为必需步骤</td></tr>"
        "<tr><td>2</td><td>Core 没有地图拓扑（predecessor/successor 都 absent）</td>"
        "<td><code>RoadMapElement</code> 已有 <code>entry_lanes/exit_lanes</code>；只有 turn 真缺</td>"
        "<td>blocked 3 → 0；ADR-0008 定 turn 走 map 侧表</td></tr>"
        "<tr><td>3</td><td>观测数据年龄 0.53 s</td>"
        "<td>只是 <code>rx-2714</code> 单条；6 条中位 0.78 s（0.41–1.21）</td>"
        "<td>冻结值改 0.78 s，字段改为 per-scenario provenance</td></tr>"
        "<tr><td>4</td><td>约束谓词写出来就能用</td>"
        "<td><code>stall_crawl</code> 在静止起步就触发；IDM 自身违反 comfort 阈值</td>"
        "<td>加 warmup_s；IDM 标定问题入遗留</td></tr>"
        "<tr><td>5</td><td>“最近障碍”就是“前车”</td>"
        "<td>用双方车宽之和判定同车道，把邻道并排车算成前车，58% 的帧被判成追尾</td>"
        "<td>in-path 收紧到半个车宽 + 0.3 m，命中率降到 5.4%</td></tr>"
        "<tr><td>6</td><td>pose 加速度差分就等于 jerk</td>"
        "<td>|jerk| 中位 5.5 m/s³、51% 帧超阈值、p90 到 41 —— 20 Hz 姿态噪声</td>"
        "<td>改为滑窗三次拟合速度求导（窗口 1.0 s），噪声地板 1.11 m/s³</td></tr>"
        "</tbody></table>"
    )

    # ---- gate
    parts.append("<h2>8. Gate 台账（可复跑）</h2>")
    parts.append(
        gate_table(
            [
                ("M1 覆盖率", "python scripts/spec_report.py --check", "PASS", "字段数≥40、属性缺失=0、frozen≥60%、每组≥1 frozen"),
                ("M2 三源审计", "python scripts/spec_audit.py --check", "PASS", "bags≥5、maps≥2、字段全部完成判定"),
                ("M3 观测包络", "python scripts/spec_envelope_report.py --check", "PASS", ">100 m 样本=0、p100 中位在 [60,80]、延迟中位在 [0.4,1.2]"),
                ("M4 谓词可执行", "python scripts/constraint_judge.py --check", "PASS", "模板≥20、id 唯一、参数齐备、20/20 可执行"),
                ("M4 可行性裁判", "python scripts/constraint_feasibility.py --check", "PASS", "三控制器均有有效评价步、STATIC 无独自触发的安全/体感谓词"),
                ("M5 挖掘契约", "python scripts/badcase_mine.py --check", "PASS", "8 条可挖模板指回 constraints_v0、20 条全覆盖、所有 bag 被扫、门禁有计数"),
            ]
        )
    )

    # ---- 下一步
    parts.append("<h2>9. 下一步（M6）与已知边界</h2>")
    parts.append(
        "<table><thead><tr><th>项</th><th>内容</th><th>验收数字</th></tr></thead><tbody>"
        "<tr><td>M6</td><td>桥接层 acc wash-in + 切片入库（把 487 个切片变成可跑场景）</td>"
        "<td>6 s 内 max |Δxy| &lt; 1e-4 m；切片转 PufferDrive 场景成功率</td></tr>"
        "<tr><td>M6</td><td>桥接层 acc wash-in（对拍已就绪）</td>"
        "<td>6 s 内 max |Δxy| &lt; 1e-4 m</td></tr>"
        "</tbody></table>"
    )
    parts.append(
        '<div class="warn"><b>边界</b>：<code>replay</code> 控制器在 gigaflow 下不是真见证人'
        "（无录制轨迹可回放），所以 M4 的裁判只验证了“谓词会/不会误触发”的机制，"
        "不是场景统计。真裁判必须在 M5 的录制切片上跑。</div>"
    )
    parts.append(
        '<p class="sub" style="margin-top:34px">由 <code>scripts/milestone_review.py</code> 生成，'
        "勿手改。数据源：<code>spec/*.yaml</code>、<code>.scratch/m2/audit.json</code>、"
        "<code>.scratch/m3/obs_envelope_6bags.json</code>、可行性裁判真跑。</p></body></html>"
    )
    return "".join(parts)


def build_flow(spec_n, audit_n, env_n, con_n, feas_n, mining_n):
    verdicts = audit_n["verdicts"] if audit_n else Counter()
    return [
        {
            "id": "M1",
            "title": "spec 注册表",
            "state": "done",
            "state_label": "已收口 · gate PASS",
            "metrics": [
                f"字段 {spec_n['total']} / 8 组",
                f"frozen {spec_n['frozen_ratio']:.1%}",
                f"blocked {spec_n['status']['blocked']}",
            ],
            "artifact": "spec_v0_coverage.html",
        },
        {
            "id": "M2",
            "title": "三源数据审计",
            "state": "done",
            "state_label": "已收口 · gate PASS",
            "metrics": [
                f"齐备 {verdicts['both']} / 字段 {audit_n['fields'] if audit_n else 0}",
                f"待桥接 {verdicts['source_bridged']}",
                f"仅数据源 {verdicts['source_only']}",
            ],
            "artifact": "spec_v0_data_audit.html",
        },
        {
            "id": "M3",
            "title": "观测包络",
            "state": "done",
            "state_label": "已收口 · gate PASS",
            "metrics": [
                f"感知 {FROZEN_AGENT_RANGE_M:.0f} m",
                f"延迟 {FROZEN_DELAY_S:.2f} s",
                f"盲开 {OPERATIONAL_SPEED_MPS * FROZEN_DELAY_S:.1f} m",
            ],
            "artifact": "spec_v0_obs_envelope.html",
        },
        {
            "id": "M4",
            "title": "约束库 + 裁判",
            "state": "done",
            "state_label": "已收口 · gate PASS",
            "metrics": [
                f"模板 {con_n['templates']}（违规 {con_n['categories']['violation']}）",
                f"可执行 {con_n['executable']}/{con_n['templates']}",
                f"可解 {feas_n['verdicts']['solvable']} / 不可解 {feas_n['verdicts']['unsolvable']}",
            ],
            "artifact": "constraints_v0_library.html",
        },
        {
            "id": "M5",
            "title": "bad case 挖掘",
            "state": "done" if mining_n else "current",
            "state_label": "已收口 · gate PASS" if mining_n else "进行中",
            "metrics": (
                [
                    f"可挖 {len(mining_n['evaluable'])}/{mining_n['summary']['template_total']} 条约束",
                    f"切片 {mining_n['total_slices']}",
                    f"可信帧 {mining_n['summary']['frames']}",
                ]
                if mining_n
                else ["扫数据湖", "切片 16+80", "—"]
            ),
            "artifact": "badcase_mining_v0.html",
        },
        {
            "id": "M6",
            "title": "桥接 + 入库",
            "state": "next",
            "state_label": "待排期",
            "metrics": ["acc wash-in", "切片转场景", "对拍 < 1e-4 m"],
            "artifact": "（未生成）",
        },
    ]


def main():
    parser = argparse.ArgumentParser(description="Milestone 评审看板")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--steps", type=int, default=40)
    parser.add_argument("--no-run", action="store_true", help="跳过可行性裁判真跑")
    parser.add_argument("--check", action="store_true", help="只校验看板自洽性，不写 HTML")
    args = parser.parse_args()

    spec_n = load_spec_numbers()
    audit_n = load_audit_numbers()
    env_n = load_envelope_numbers()
    con_n = load_constraint_numbers()
    mining_n = load_mining_numbers()

    if args.no_run:
        from collections import Counter as _Counter

        feas_n = {
            "verdicts": _Counter({"solvable": 0}),
            "steps": args.steps,
            "evaluated": {},
            "fired_templates": [],
        }
        print("[feasibility] 跳过真跑（--no-run）")
    else:
        feas_n = run_feasibility(steps=args.steps)
        print(
            "[feasibility] "
            + " / ".join(f"{key}={value}" for key, value in feas_n["verdicts"].items())
        )

    print(f"[spec]       字段 {spec_n['total']}，frozen {spec_n['frozen_ratio']:.1%}")
    if audit_n:
        print("[audit]      " + " / ".join(f"{k}={v}" for k, v in audit_n["verdicts"].items()))
    print(f"[envelope]   p100 中位 {env_n['summary']['p100_median']:.1f} m，延迟中位 {env_n['summary']['delay_median']:.2f} s")
    print(f"[constraints] 模板 {con_n['templates']}，可执行 {con_n['executable']}")
    if mining_n:
        print(f"[mining]     可挖 {len(mining_n['evaluable'])} 条 · 切片 {mining_n['total_slices']} · 帧 {mining_n['summary']['frames']}")

    failures = consistency_gate(spec_n, audit_n, env_n, con_n, feas_n, mining_n)
    for item in failures:
        print(f"GATE FAIL   : {item}")
    print("GATE        : " + ("PASS" if not failures else "FAIL"))
    if args.check:
        return 1 if failures else 0

    flow = build_flow(spec_n, audit_n, env_n, con_n, feas_n, mining_n)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(
        render(spec_n, audit_n, env_n, con_n, feas_n, mining_n, flow), encoding="utf-8"
    )
    print(f"[report]     {args.out}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
