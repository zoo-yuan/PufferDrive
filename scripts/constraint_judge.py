#!/usr/bin/env python3
"""约束模板库 v0 的可执行性裁判 —— 20 条模板必须同时能"触发"和"不触发"。

为什么需要这个脚本
------------------
A-only 反馈下，bad case 只有约束、没有专家轨迹，所以"这条约束"本身就是唯一的监督信号。
一个写不进可执行谓词的模板等于没有入库：它既不能自动挖 case，也不能自动回归。

本脚本做三件事：
1. 把 `spec/constraints_v0.yaml` 的谓词对着**真实指标词表**解析（词表从
   `pufferlib/ocean/drive/constants.h` 的 `*_IDX` 定义读出来，不手抄）。
2. 对每条模板合成两行数据——一行必须触发、一行必须不触发——证明谓词可执行。
3. 生成 20×18 指标矩阵报告，并列出"谓词需要但引擎没有"的指标（= 桥接层工作清单）。

用法:
    python scripts/constraint_judge.py            # 校验 + 写 HTML + 打印
    python scripts/constraint_judge.py --check    # 只校验

gate:
    模板数 >= 20；id 唯一；谓词引用参数全部存在；20/20 可执行；
    violation 占比落在 [0.70, 0.86]；每个模板都有 reference_solver；
    至少 80% 模板用引擎已有指标即可评价。
"""

import argparse
import re
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SPEC = REPO_ROOT / "spec" / "constraints_v0.yaml"
DEFAULT_OUT = REPO_ROOT / "docs" / "html" / "constraints_v0_library.html"
CONSTANTS_HEADER = REPO_ROOT / "pufferlib" / "ocean" / "drive" / "constants.h"

MIN_TEMPLATES = 20
VIOLATION_RATIO_RANGE = (0.70, 0.86)
MIN_NATIVE_ONLY_RATIO = 0.80
VALID_SOLVERS = {"idm", "replay", "static", "policy"}

# Agent 结构里可被谓词直接读取的非 metrics 状态量（datatypes.h）
AGENT_STATE_METRICS = {
    "sim_speed": "m/s",
    "accel_long": "m/s²",
    "accel_lat": "m/s²",
    "jerk_long": "m/s³",
    "jerk_lat": "m/s³",
    "sim_heading": "rad",
    "yaw_rate": "rad/s",
}

OPERATORS = {
    "ge": lambda value, threshold: value >= threshold,
    "gt": lambda value, threshold: value > threshold,
    "le": lambda value, threshold: value <= threshold,
    "lt": lambda value, threshold: value < threshold,
    "eq": lambda value, threshold: value == threshold,
    "ne": lambda value, threshold: value != threshold,
    "abs_gt": lambda value, threshold: abs(value) > threshold,
    "abs_lt": lambda value, threshold: abs(value) < threshold,
}

# 合成行的基线：不触发任何模板的"正常行驶"状态
BASELINE_ROW = {
    "collision": 0.0,
    "offroad": 0.0,
    "red_light": 0.0,
    "stop_sign": 0.0,
    "reached_goal": 0.0,
    "lane_dist": 0.0,
    "lane_angle": 1.0,
    "comfort_violation": 0.0,
    "velocity_progress": 1.0,
    "speed_limit": 0.0,
    "avg_displacement_error": 0.0,
    "progression": 1.0,
    "at_fault_collision": 0.0,
    "ttc": 5.0,
    "distance_to_collision": 50.0,
    "progress_ratio": 1.0,
    "multi_lane_time": 0.0,
    "multi_lane_score": 0.0,
    "sim_speed": 0.0,
    "accel_long": 0.0,
    "accel_lat": 0.0,
    "jerk_long": 0.0,
    "jerk_lat": 0.0,
    "sim_heading": 0.0,
    "yaw_rate": 0.0,
}


def read_native_metrics(header_path):
    """从 constants.h 的 `*_IDX n` 定义里读出指标的 C 名称顺序。"""
    text = header_path.read_text(encoding="utf-8")
    matches = re.findall(r"^#define\s+([A-Z0-9_]+)_IDX\s+(\d+)\s*$", text, re.MULTILINE)
    indexed = {}
    for name, index in matches:
        indexed.setdefault(int(index), name)
    count = re.search(r"^#define\s+NUM_METRICS\s+(\d+)", text, re.MULTILINE)
    num_metrics = int(count.group(1)) if count else len(indexed)
    if sorted(indexed) != list(range(num_metrics)):
        raise ValueError(f"{header_path}: *_IDX 定义不连续 0..{num_metrics - 1}: {sorted(indexed)}")
    names = [indexed[i] for i in range(num_metrics)]
    return names, {name.lower() for name in names}


def as_predicate_list(predicate):
    if isinstance(predicate, list):
        return predicate
    return [predicate]


def evaluate(predicates, params, row):
    """any-of 语义：任一谓词成立即触发；guard 全部成立才评价。"""
    for predicate in predicates:
        threshold = params[predicate["threshold_param"]]["value"]
        value = row[predicate["metric"]]
        if OPERATORS[predicate["op"]](value, threshold):
            return True
    return False


def guard_holds(guard, params, row):
    if not guard:
        return True
    threshold = params[guard["threshold_param"]]["value"]
    return OPERATORS[guard["op"]](row[guard["metric"]], threshold)


def trip_value(op, threshold):
    """造一个必然满足该算子的值（阈值可能为 0 或负数，不能简单乘 2）。"""
    margin = max(1.0, abs(threshold))
    if op in ("gt", "ge"):
        return threshold + margin
    if op in ("lt", "le"):
        return threshold - margin
    if op == "abs_gt":
        return abs(threshold) + margin
    if op == "abs_lt":
        return threshold * 0.5
    return threshold


def trip_row(base, predicates, guard, params):
    """构造一行必然触发这些谓词的数据（guard 也要满足）。"""
    row = dict(base)
    if guard:
        threshold = params[guard["threshold_param"]]["value"]
        row[guard["metric"]] = trip_value(guard["op"], threshold)
    for predicate in predicates:
        threshold = params[predicate["threshold_param"]]["value"]
        row[predicate["metric"]] = trip_value(predicate["op"], threshold)
    return row


def build_baseline(derived):
    """正常行驶的基线行：派生指标用它自己的“安全”值（如无行人 = 大间距）。"""
    base = dict(BASELINE_ROW)
    for name, meta in (derived or {}).items():
        base[name] = float(meta.get("baseline_safe", 0.0))
    return base


def analyse(spec, native_metrics):
    derived = spec.get("derived_metrics", {}) or {}
    known = set(native_metrics) | set(AGENT_STATE_METRICS) | set(derived)
    problems = []
    rows = []

    for template in spec["templates"]:
        template_id = template["id"]
        params = template.get("params", {}) or {}
        predicates = as_predicate_list(template["predicate"])
        guard = template.get("guard")

        used = [p["metric"] for p in predicates] + ([guard["metric"]] if guard else [])
        missing_metrics = [name for name in used if name not in known]
        missing_params = []
        for predicate in predicates + ([guard] if guard else []):
            if predicate["threshold_param"] not in params:
                missing_params.append(predicate["threshold_param"])
            if predicate["op"] not in OPERATORS:
                missing_params.append(f"未知算子 {predicate['op']}")
        if missing_metrics:
            problems.append(f"{template_id}: 未知指标 {missing_metrics}")
        if missing_params:
            problems.append(f"{template_id}: 缺参数/算子 {missing_params}")

        needs_derived = [name for name in used if name in derived]
        executable = False
        if not missing_metrics and not missing_params:
            base = build_baseline(derived)
            # any-of 谓词：逐条造候选行，只要有一条能触发就算可执行
            candidates = [trip_row(base, [predicate], guard, params) for predicate in predicates]
            try:
                trips = any(
                    evaluate(predicates, params, candidate) and guard_holds(guard, params, candidate)
                    for candidate in candidates
                )
                quiet_on_baseline = not (
                    evaluate(predicates, params, base) and guard_holds(guard, params, base)
                )
                executable = trips and quiet_on_baseline
            except KeyError as error:
                problems.append(f"{template_id}: 合成行缺键 {error}")
                executable = False
        if not executable:
            problems.append(f"{template_id}: 谓词不可执行（无法同时表达触发/不触发）")

        rows.append(
            {
                "id": template_id,
                "label": template.get("label", template_id),
                "category": template.get("category", "violation"),
                "status": template.get("status", "draft"),
                "solver": template.get("reference_solver", ""),
                "metrics": used,
                "needs_derived": needs_derived,
                "executable": executable,
                "params": {name: meta for name, meta in params.items()},
                "predicate_text": format_predicate(predicates, guard, params),
                "note": template.get("note", ""),
            }
        )
    return rows, problems


def format_predicate(predicates, guard, params):
    def one(predicate):
        threshold = params.get(predicate["threshold_param"], {}).get("value", "?")
        unit = params.get(predicate["threshold_param"], {}).get("unit", "")
        return f"{predicate['metric']} {predicate['op']} {threshold} {unit}".strip()

    text = " 或 ".join(one(p) for p in predicates)
    if guard:
        text += f"  [当 {one(guard)}]"
    return text


def gate(spec, rows, problems):
    failures = list(problems)
    if len(rows) < MIN_TEMPLATES:
        failures.append(f"模板数 {len(rows)} < {MIN_TEMPLATES}")
    ids = [row["id"] for row in rows]
    if len(set(ids)) != len(ids):
        failures.append("模板 id 重复")
    violation = sum(1 for row in rows if row["category"] == "violation")
    ratio = violation / len(rows) if rows else 0.0
    low, high = VIOLATION_RATIO_RANGE
    if not low <= ratio <= high:
        failures.append(f"violation 占比 {ratio:.0%} 不在 [{low:.0%}, {high:.0%}]")
    bad_solver = [row["id"] for row in rows if row["solver"] not in VALID_SOLVERS]
    if bad_solver:
        failures.append(f"reference_solver 非法: {bad_solver}")
    native_only = sum(1 for row in rows if not row["needs_derived"])
    native_ratio = native_only / len(rows) if rows else 0.0
    if native_ratio < MIN_NATIVE_ONLY_RATIO:
        failures.append(f"引擎可直接评价的模板 {native_ratio:.0%} < {MIN_NATIVE_ONLY_RATIO:.0%}")
    if sum(1 for row in rows if row["executable"]) != len(rows):
        failures.append("存在不可执行谓词")
    return failures, {"violation_ratio": ratio, "native_ratio": native_ratio, "violation": violation}


def esc(text):
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


CATEGORY_COLOR = {"violation": "#cf222e", "comfort": "#9a6700", "progress": "#0b6bcb"}


def render(spec, rows, native_metrics_raw, stats, failures):
    native_metrics = [name.lower() for name in native_metrics_raw]
    gate_state = "PASS" if not failures else "FAIL"
    gate_color = "#1a7f37" if not failures else "#cf222e"
    derived = spec.get("derived_metrics", {}) or {}
    derived_rows = [row for row in rows if row["needs_derived"]]

    parts = [f"""<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>约束模板库 v0 —— 可执行性裁判</title>
<style>
:root {{ --ink:#1f2328; --mute:#57606a; --line:#d0d7de; --soft:#f6f8fa; }}
* {{ box-sizing:border-box; }}
body {{ margin:0; padding:32px 40px 56px; max-width:1340px; color:var(--ink);
  font:15px/1.6 -apple-system,"Segoe UI","PingFang SC","Microsoft YaHei",sans-serif; }}
h1 {{ font-size:26px; margin:0 0 6px; letter-spacing:-.3px; }}
h2 {{ font-size:19px; margin:36px 0 10px; padding-bottom:8px; border-bottom:1px solid var(--line); }}
.sub {{ color:var(--mute); font-size:13px; margin:0 0 20px; }}
.cards {{ display:grid; grid-template-columns:repeat(5,1fr); gap:12px; margin:20px 0 6px; }}
.card {{ border:1px solid var(--line); border-radius:10px; padding:14px 16px; background:var(--soft); }}
.card .k {{ font-size:12px; color:var(--mute); }}
.card .v {{ font-size:26px; font-weight:650; letter-spacing:-.5px; }}
.card .n {{ font-size:12px; color:var(--mute); }}
table {{ border-collapse:collapse; width:100%; font-size:12.5px; }}
th,td {{ border:1px solid var(--line); padding:4px 7px; }}
th {{ background:var(--soft); font-size:11.5px; text-align:left; }}
td.c,th.c {{ text-align:center; }}
tbody tr:nth-child(even) td {{ background:#fcfcfd; }}
code {{ background:var(--soft); border:1px solid var(--line); border-radius:4px; padding:1px 4px; font-size:11px; }}
.gate {{ display:inline-block; padding:3px 12px; border-radius:6px; font-size:13px; font-weight:700; color:#fff; background:{gate_color}; }}
.fail {{ border-left:3px solid #cf222e; background:#fff5f5; padding:9px 13px; border-radius:0 6px 6px 0; margin:7px 0; font-size:13px; }}
.ok {{ border-left:3px solid #1a7f37; background:#f2fbf4; padding:9px 13px; border-radius:0 6px 6px 0; margin:7px 0; font-size:13px; }}
.warn {{ border-left:3px solid #9a6700; background:#fffbf0; padding:9px 13px; border-radius:0 6px 6px 0; margin:7px 0; font-size:13px; }}
.note {{ color:var(--mute); font-size:12px; }}
.pill {{ display:inline-block; padding:1px 7px; border-radius:999px; font-size:11px; font-weight:600; color:#fff; }}
.mx td.c {{ width:30px; }}
.mx th.c {{ writing-mode:vertical-rl; text-orientation:mixed; height:104px; vertical-align:bottom; font-size:10px; }}
.mx tbody td.hit {{ background:#1a7f37; color:#fff; font-weight:700; }}
.mx tbody td.guard {{ background:#9a6700; color:#fff; font-weight:700; }}
.mx tbody td.derived {{ background:#cf222e; color:#fff; font-weight:700; }}
</style></head><body>"""]

    parts.append(f"""
<h1>约束模板库 v0 —— 可执行性裁判</h1>
<p class="sub">spec <code>constraints_v0</code> · 指标词表来自 <code>constants.h</code>（{len(native_metrics)} 项，不手抄） · gate <span class="gate">{gate_state}</span></p>""")

    parts.append('<div class="cards">')
    cards = [
        ("模板总数", len(rows), f"下限 {MIN_TEMPLATES}"),
        ("违规型", stats["violation"], f"{stats['violation_ratio']:.0%}（owner 口径 80%）"),
        ("体感/进度型", len(rows) - stats["violation"], "20% 一侧"),
        ("谓词可执行", sum(1 for row in rows if row["executable"]), "合成触发/不触发两行验证"),
        ("引擎直接可评", f"{stats['native_ratio']:.0%}", "其余需桥接新指标"),
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
            '<div class="ok">gate 通过：20 条模板 id 唯一、参数齐备、谓词全部可执行'
            "（每条都能同时表达触发与不触发）、违规/体感比例与引擎覆盖度达标。</div>"
        )

    # ---- 矩阵
    parts.append("<h2>1. 模板 × 引擎指标矩阵</h2>")
    parts.append(
        '<p class="note">绿 = 谓词直接读取该指标；橙 = 仅作为 guard 条件；'
        "红列 = 引擎 `metrics_array` 里没有、需桥接层提供的派生指标。</p>"
    )
    parts.append('<table class="mx"><thead><tr><th>模板</th>')
    for name in native_metrics:
        parts.append(f'<th class="c">{esc(name)}</th>')
    for name in derived:
        parts.append(f'<th class="c" style="color:#cf222e">{esc(name)}</th>')
    parts.append('<th class="c">类别</th></tr></thead><tbody>')
    for row in rows:
        predicate_metrics = set()
        guard_metrics = set()
        for template in spec["templates"]:
            if template["id"] != row["id"]:
                continue
            for predicate in as_predicate_list(template["predicate"]):
                predicate_metrics.add(predicate["metric"])
            if template.get("guard"):
                guard_metrics.add(template["guard"]["metric"])
        color = CATEGORY_COLOR.get(row["category"], "#57606a")
        parts.append(f'<tr><td>{esc(row["label"])}<br><code>{esc(row["id"])}</code></td>')
        for name in native_metrics:
            if name in predicate_metrics:
                parts.append('<td class="c hit">●</td>')
            elif name in guard_metrics:
                parts.append('<td class="c guard">○</td>')
            else:
                parts.append('<td class="c"></td>')
        for name in derived:
            parts.append('<td class="c derived">●</td>' if name in predicate_metrics else '<td class="c"></td>')
        parts.append(
            f'<td class="c"><span class="pill" style="background:{color}">{esc(row["category"])}</span></td></tr>'
        )
    parts.append("</tbody></table>")

    # ---- 明细
    parts.append("<h2>2. 模板明细（谓词 + 参数）</h2>")
    parts.append(
        "<table><thead><tr><th>模板</th><th>谓词</th><th>参数</th>"
        "<th class='c'>裁判参考</th><th class='c'>状态</th><th class='c'>可执行</th></tr></thead><tbody>"
    )
    for row in rows:
        params = "<br>".join(
            f'<code>{esc(name)}</code> = {meta["value"]} {esc(meta["unit"])}'
            for name, meta in row["params"].items()
        )
        color = CATEGORY_COLOR.get(row["category"], "#57606a")
        parts.append(
            f'<tr><td>{esc(row["label"])}<br><span class="note">{esc(row["id"])}</span></td>'
            f'<td><code>{esc(row["predicate_text"])}</code></td>'
            f"<td>{params}</td>"
            f'<td class="c">{esc(row["solver"])}</td>'
            f'<td class="c"><span class="pill" style="background:'
            f'{"#1a7f37" if row["status"] == "frozen" else "#9a6700"}">{esc(row["status"])}</span></td>'
            f'<td class="c" style="color:{"#1a7f37" if row["executable"] else "#cf222e"};font-weight:700">'
            f'{"✓" if row["executable"] else "✗"}</td></tr>'
        )
    parts.append("</tbody></table>")

    # ---- 缺口
    parts.append("<h2>3. 引擎指标缺口（桥接层工作清单）</h2>")
    if derived_rows:
        parts.append(
            f'<div class="warn">{len(derived_rows)} 条模板的谓词依赖 metrics_array 里<b>不存在</b>的指标，'
            "这些指标必须由桥接层提供，否则约束无法自动挖 case。</div>"
        )
        parts.append("<table><thead><tr><th>缺失指标</th><th>被哪些模板使用</th><th>说明</th></tr></thead><tbody>")
        for name, meta in derived.items():
            users = [row["label"] for row in derived_rows if name in row["needs_derived"]]
            if not users:
                continue
            parts.append(
                f'<tr><td><code>{esc(name)}</code></td><td>{esc("、".join(users))}</td>'
                f'<td class="note">{esc(meta.get("note", ""))}</td></tr>'
            )
        parts.append("</tbody></table>")
    else:
        parts.append('<div class="ok">无缺口：全部模板都可用引擎现有指标评价。</div>')
    parts.append(
        '<p class="note">注：谓词可执行 ≠ 场景可解。可行性裁判还要用 IDM / REPLAY / STATIC '
        "在同一场景各跑一遍；全挂即场景不可解，禁止进 RL（ADR-0007）。</p>"
    )

    parts.append(
        '<p class="sub" style="margin-top:32px">由 <code>scripts/constraint_judge.py</code> 生成，勿手改。'
        "库源 <code>spec/constraints_v0.yaml</code>。</p></body></html>"
    )
    return "".join(parts)


def main():
    parser = argparse.ArgumentParser(description="约束模板库可执行性裁判")
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    spec = yaml.safe_load(args.spec.read_text(encoding="utf-8"))
    native_names, native_set = read_native_metrics(CONSTANTS_HEADER)
    rows, problems = analyse(spec, native_set)
    failures, stats = gate(spec, rows, problems)

    print(f"metrics     : {len(native_names)} 项（来自 constants.h）")
    print(f"templates   : {len(rows)}")
    by_category = {}
    for row in rows:
        by_category[row["category"]] = by_category.get(row["category"], 0) + 1
    print("categories  : " + " / ".join(f"{k}={v}" for k, v in sorted(by_category.items())))
    print(f"executable  : {sum(1 for row in rows if row['executable'])}/{len(rows)}")
    print(f"native-only : {stats['native_ratio']:.0%}")
    for row in rows:
        mark = "✓" if row["executable"] else "✗"
        derived = f" [需 {','.join(row['needs_derived'])}]" if row["needs_derived"] else ""
        print(f"  {mark} {row['id']:28s} {row['category']:9s} {row['predicate_text']}{derived}")

    for item in failures:
        print(f"GATE FAIL   : {item}")
    if not failures:
        print("GATE        : PASS")

    if not args.check:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(render(spec, rows, native_names, stats, failures), encoding="utf-8")
        print(f"report      : {args.out}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
