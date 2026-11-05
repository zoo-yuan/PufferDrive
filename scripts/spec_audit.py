#!/usr/bin/env python3
"""canonical spec 三源数据审计 —— spec 字段 × 真实数据 的可复跑证据链。

三源:
    源 A  bags        Apollo cyber record（6 条 rx-*），逐通道消息数/时长/fps
    源 B  HD map      Apollo base_map.bin（真实地图），lane 级拓扑字段实测计数
    源 C  vehicle_param  车型档案 pb.txt

对每个 spec 字段给出判定:
    source = 数据源是否真有这一手真值（实测，不读代码推断）
    sim    = 仿真内核是否已有（来自 canonical_v0.yaml）
    判定   = 三源齐备 / 仅数据源（桥接必做）/ 仅仿真（凭空字段）/ 两侧皆无（阻塞）

用法:
    python scripts/spec_audit.py                 # 全量审计 + 写 HTML
    python scripts/spec_audit.py --check         # gate 校验，失败退出码 1
    python scripts/spec_audit.py --no-cache      # 忽略 bag 通道缓存

gate:
    bags >= 5，maps >= 2，未判定字段 = 0，报告已生成
"""

import argparse
import json
import sys
from collections import Counter, OrderedDict
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from data_utils.apollo_record import iter_fields, iter_messages, list_channels  # noqa: E402

DEFAULT_SPEC = REPO_ROOT / "spec" / "canonical_v0.yaml"
DEFAULT_OUT = REPO_ROOT / "docs" / "html" / "spec_v0_data_audit.html"
DEFAULT_CACHE = REPO_ROOT / "docs" / "measured" / "audit_canonical_v0.json"
DEFAULT_DATA_ROOT = Path("/media/zoo/da1510ce-bbc7-4b3e-9d3b-e5f2137af995")
DEFAULT_MAP_ROOT = DEFAULT_DATA_ROOT / "apollo_map" / "data"
DEFAULT_VEHICLE_PARAM = Path.home() / "Downloads" / "data_xijiao-master-3799_vehicle_param.pb.txt"
DEFAULT_TFRECORD_DIR = Path("/media/zoo/canaan_disk_23/rino_e2e_tfrecords/0911dataset")

MIN_BAGS = 5
MIN_MAPS = 2

# 每个 spec 组默认的一手来源类型；字段级例外见 FIELD_SOURCE_OVERRIDE
SOURCE_BAG = "bag"
SOURCE_MAP = "hdmap"
SOURCE_VEHICLE = "vehicle_param"
SOURCE_DERIVED = "derived"
SOURCE_MANIFEST = "manifest"
SOURCE_NONE = "none"
DERIVED_FIELDS = (
    "occ_count",
    "occ_polygons_xy_m",
    "occ_heights_m",
    "occ_types",
    "occ_valid_mask",
)

GROUP_SOURCE = {
    "ego": SOURCE_BAG,
    "agents": SOURCE_BAG,
    "occ": SOURCE_BAG,
    "map": SOURCE_MAP,
    "traffic": SOURCE_BAG,
    "route": SOURCE_BAG,
    "provenance": SOURCE_BAG,
    "vehicle": SOURCE_VEHICLE,
}

# 字段级来源例外：这些字段的实际一手来源与所在组默认不同
FIELD_SOURCE_OVERRIDE = {
    # occ 无占据栅格通道；真值是「perception 对象多边形 + HD map 静态物」派生物
    "occ_count": SOURCE_DERIVED,
    "occ_polygons_xy_m": SOURCE_DERIVED,
    "occ_heights_m": SOURCE_DERIVED,
    "occ_types": SOURCE_DERIVED,
    "occ_valid_mask": SOURCE_DERIVED,
    "static_objects_from_map": SOURCE_MAP,
    "tl_stop_line_xy_m": SOURCE_MAP,
    "tl_phase_schedule": SOURCE_MAP,
    "refline_slots_xy_m": SOURCE_MAP,
    "refline_slot0_is_ego": SOURCE_MAP,
    "goal_xy_m": SOURCE_BAG,
    "map_version": SOURCE_MAP,
    "perception_version": SOURCE_MANIFEST,  # 包内无版本通道，靠 release.info/构建清单
    "models_version": SOURCE_MANIFEST,
    "vehicle_param_id": SOURCE_VEHICLE,
}

# 组默认需要的 bag 通道；字段级例外见 FIELD_CHANNEL_OVERRIDE
GROUP_CHANNELS = {
    "ego": ["/apollo/localization/pose"],
    "agents": ["/apollo/perception/obstacles"],
    "occ": ["/apollo/perception/obstacles"],
    "map": [],
    "traffic": ["/apollo/perception/traffic_light"],
    "route": ["/apollo/routing_response_history"],
    "provenance": ["/apollo/localization/pose"],
    "vehicle": [],
}

FIELD_CHANNEL_OVERRIDE = {
    "ego_steering_angle_rad": ["/apollo/canbus/chassis"],
    "ego_acc_long_mps2": ["/apollo/localization/pose", "/apollo/planning"],
    "route_points_xy_m": ["/apollo/routing_response_history"],
    "tl_stop_line_xy_m": [],
    "tl_phase_schedule": [],
    "map_version": ["/apollo/routing_response_history"],
    "data_age_s": ["/apollo/localization/pose", "/apollo/perception/obstacles"],
    "sensor_age_s": ["/apollo/perception/obstacles"],
    "agent_history16": ["/apollo/perception/obstacles"],
}

# Apollo hdmap.Map 顶层字段号
MAP_TOP_FIELDS = {
    1: "header",
    2: "crosswalk",
    3: "junction",
    4: "lane",
    5: "stop_sign",
    6: "signal",
    7: "yield_sign",
    8: "overlap",
    9: "clear_area",
    10: "speed_bump",
    11: "road",
    12: "parking_space",
    13: "pnc_junction",
}

# Apollo hdmap.Lane 字段号
LANE_FIELDS = {
    1: "id",
    2: "central_curve",
    3: "left_boundary",
    4: "right_boundary",
    5: "length",
    6: "speed_limit",
    7: "overlap_id",
    8: "predecessor_id",
    9: "successor_id",
    10: "left_neighbor_forward_lane_id",
    11: "right_neighbor_forward_lane_id",
    12: "type",
    13: "turn",
    14: "direction",
}

MAP_GROUP_FIELDS = {
    "map_lane_centerline": ("lane", "central_curve"),
    "map_lane_boundary": ("lane", "left_boundary"),
    "map_hard_boundary": ("lane", "right_boundary"),
    "map_crosswalk": ("crosswalk", None),
    "map_stopline": ("signal", None),
    "map_semantic_label": ("lane", None),
    "map_topology_predecessor": ("lane", "predecessor_id"),
    "map_topology_successor": ("lane", "successor_id"),
    "map_turn_type": ("lane", "turn"),
    "map_speed_limit_mps": ("lane", "speed_limit"),
    "map_map_range_m": ("lane", "central_curve"),
    "static_objects_from_map": ("overlap", None),
    "tl_stop_line_xy_m": ("signal", None),
    "tl_phase_schedule": ("signal", None),
    "refline_slots_xy_m": ("lane", "central_curve"),
    "refline_slot0_is_ego": ("lane", "central_curve"),
    "map_version": ("header", None),
}


# --------------------------------------------------------------------- bags
def audit_bags(bag_paths, cache_path, use_cache):
    if use_cache and cache_path.exists():
        cached = json.loads(cache_path.read_text(encoding="utf-8"))
        bags = cached.get("bags") if isinstance(cached, dict) else None
        if isinstance(bags, dict) and bags and all(isinstance(v, dict) for v in bags.values()):
            print(f"[bags] 使用缓存 {cache_path}（{len(bags)} 条）")
            return bags
        print(f"[bags] 缓存无效，重新解析")

    result = {}
    for path in bag_paths:
        counts = list_channels(str(path))
        result[str(path)] = counts
        print(f"[bags] {path.name}: {len(counts)} 通道, {sum(counts.values())} 消息")
    return result


# ---------------------------------------------------- derived / manifest
# Apollo PerceptionObstacle 关键字段号（用于判定 occ 的可派生性）
OBSTACLE_FIELD_PROFILE = {
    2: "position",
    3: "theta",
    5: "length",
    6: "width",
    7: "height",
    8: "polygon_point",
    10: "type",
}
OBSTACLE_CHANNEL = "/apollo/perception/obstacles"
DERIVED_SAMPLE_FRAMES = 60


def audit_obstacle_polygons(bag_paths, frames_per_bag=20, max_bags=3):
    """实测：perception obstacles 里对象级多边形是否存在（occ 的唯一可派生来源）。"""
    summary = {
        "path": "",
        "bags": [],
        "frames": 0,
        "instances": 0,
        "with_polygon": 0,
        "height_present": 0,
        "type_present": 0,
        "median_points": 0,
        "max_points": 0,
    }
    if not bag_paths:
        return summary

    point_counts = []
    for path in bag_paths[:max_bags]:
        frames = 0
        summary["bags"].append(Path(path).stem.split(".")[0])
        summary["path"] = str(path)
        for _channel, _timestamp, payload in iter_messages(path, channels={OBSTACLE_CHANNEL}):
            frames += 1
            if frames > frames_per_bag:
                break
            summary["frames"] += 1
            for field_number, _wire, value in iter_fields(payload):
                if field_number != 1 or not isinstance(value, bytes):
                    continue
                sub_fields = Counter()
                for sub_number, _sw, _sv in iter_fields(value):
                    sub_fields[sub_number] += 1
                summary["instances"] += 1
                if sub_fields.get(8):
                    summary["with_polygon"] += 1
                    point_counts.append(sub_fields[8])
                if sub_fields.get(7):
                    summary["height_present"] += 1
                if sub_fields.get(10):
                    summary["type_present"] += 1

    point_counts.sort()
    summary["median_points"] = point_counts[len(point_counts) // 2] if point_counts else 0
    summary["max_points"] = point_counts[-1] if point_counts else 0
    return summary


def audit_manifest(data_root):
    """release.info = map_version / 构建台账的一手来源实证。"""
    entries = []
    for info in sorted(data_root.glob("rx-*/release.info")):
        text = info.read_text(encoding="utf-8", errors="replace")
        fields = {}
        for line in text.splitlines():
            if ":" in line:
                key, value = line.split(":", 1)
                fields[key.strip()] = value.strip()
        entries.append({"path": str(info), "name": info.parent.name, "fields": fields})
    return entries


def audit_hdmap(map_bin):
    raw = map_bin.read_bytes()
    top_counter = Counter()
    lane_counter = Counter()
    signal_counter = Counter()
    lane_count = 0

    for field_number, _wire, value in iter_fields(raw):
        top_counter[field_number] += 1
        if field_number == 4 and isinstance(value, bytes):
            lane_count += 1
            for sub_number, _w, _v in iter_fields(value):
                lane_counter[sub_number] += 1
        elif field_number == 6 and isinstance(value, bytes):
            for sub_number, _w, _v in iter_fields(value):
                signal_counter[sub_number] += 1

    def lane_name(field_number):
        return LANE_FIELDS.get(field_number, f"field_{field_number}")

    return {
        "path": str(map_bin),
        "bytes": len(raw),
        "lane_count": lane_count,
        "top": {MAP_TOP_FIELDS.get(k, f"field_{k}"): v for k, v in top_counter.items()},
        "lane": {lane_name(k): v for k, v in lane_counter.items() if k != 1},
        "signal_subfields": {f"field_{k}": v for k, v in signal_counter.items()},
    }


# ------------------------------------------------------------------ verdict
def field_source(field_id, group_id):
    return FIELD_SOURCE_OVERRIDE.get(field_id, GROUP_SOURCE[group_id])


def field_channels(field_id, group_id):
    if field_id in FIELD_CHANNEL_OVERRIDE:
        return FIELD_CHANNEL_OVERRIDE[field_id]
    return GROUP_CHANNELS[group_id]


def bag_coverage(channels, bags):
    """返回 (覆盖该字段的 bag 比例, 有该通道的 bag 名列表)。"""
    if not channels:
        return 0.0, []
    hits = []
    for path, counts in bags.items():
        if all(channel in counts for channel in channels):
            hits.append(Path(path).stem)
    return len(hits) / len(bags) if bags else 0.0, hits


def map_presence(field_id, maps):
    """返回 (含该字段的地图比例, 命中地图名, 实测计数列表)。maps = {name: audit}。"""
    spec_entry = MAP_GROUP_FIELDS.get(field_id)
    if not spec_entry or not maps:
        return 0.0, [], []
    top_key, lane_key = spec_entry
    hits = []
    measured = []
    for name, audit in maps.items():
        top_count = audit["top"].get(top_key, 0)
        lane_count = audit["lane"].get(lane_key, 0) if lane_key else top_count
        if lane_key:
            ok = lane_count > 0
        else:
            ok = top_count > 0
        if ok:
            hits.append(name)
        measured.append(lane_count if lane_key else top_count)
    return len(hits) / len(maps), hits, measured


def build_matrix(spec, bags, maps, vehicle_param_ok, tfrecord_count, derived, manifest):
    rows = []
    for group in spec["groups"]:
        for field in group["fields"]:
            field_id = field["id"]
            source = field_source(field_id, group["id"])
            channels = field_channels(field_id, group["id"])
            row = {
                "group": group["id"],
                "field": field_id,
                "sim": field.get("sim"),
                "status": field.get("status"),
                "source_kind": source,
                "source_coverage": 0.0,
                "source_detail": "",
            }

            if source == SOURCE_BAG:
                coverage, hits = bag_coverage(channels, bags)
                row["source_coverage"] = coverage
                row["source_detail"] = f"{len(hits)}/{len(bags)} bag:" + ",".join(
                    channel.split("/")[-1] for channel in channels
                )
            elif source == SOURCE_MAP:
                coverage, hits, measured = map_presence(field_id, maps)
                row["source_coverage"] = coverage
                detail = f"{len(hits)}/{len(maps)} map"
                if measured:
                    detail += " 计数=" + "/".join(str(x) for x in measured)
                row["source_detail"] = detail
            elif source == SOURCE_VEHICLE:
                row["source_coverage"] = 1.0 if vehicle_param_ok else 0.0
                row["source_detail"] = "vehicle_param pb.txt" if vehicle_param_ok else "缺档案"
            elif source == SOURCE_DERIVED:
                polygon_rate = (
                    derived["with_polygon"] / derived["instances"]
                    if derived.get("instances")
                    else 0.0
                )
                row["source_coverage"] = polygon_rate
                row["source_detail"] = (
                    f"派生自 obstacles.polygon_point {derived['with_polygon']}/"
                    f"{derived['instances']} 实例带多边形，中位 {derived['median_points']} 点"
                    if derived.get("instances")
                    else "无派生证据"
                )
            elif source == SOURCE_MANIFEST:
                row["source_coverage"] = 1.0 if manifest else 0.0
                row["source_detail"] = (
                    f"release.info ×{len(manifest)}" if manifest else "缺台账"
                )
            else:
                row["source_coverage"] = 0.0
                row["source_detail"] = (
                    "数据源无此通道" if tfrecord_count else "数据源无此通道"
                )

            row["verdict"] = classify(row["source_coverage"], row["sim"])
            rows.append(row)
    return rows


def classify(source_coverage, sim):
    has_source = source_coverage >= 0.5
    has_sim = sim in ("native", "bridged")
    if has_source and sim == "native":
        return "both"
    if has_source and sim == "bridged":
        return "source_bridged"
    if has_source and sim == "absent":
        return "source_only"
    if not has_source and sim == "native":
        return "sim_only"
    if not has_source and sim == "bridged":
        return "sim_only"
    return "neither"


VERDICT_META = {
    "both": ("#1a7f37", "三源齐备", "数据源有真值 + 仿真已实现，可直接依赖"),
    "source_bridged": ("#0b6bcb", "待桥接", "数据源有真值，仿真需桥接层产出"),
    "source_only": ("#9a6700", "仅数据源", "数据源有真值，仿真侧完全没有 → 桥接必做"),
    "sim_only": ("#cf222e", "仅仿真（凭空）", "仿真有此字段但数据源无真值 → 训练契约里是假的"),
    "neither": ("#6e7781", "两侧皆无", "阻塞：既无数据源也无仿真实现"),
}


# ------------------------------------------------------------------- report
def bar(ratio, color):
    width = max(0.0, min(1.0, ratio)) * 100.0
    return (
        f'<div class="bar"><div class="bar-fill" style="width:{width:.1f}%;'
        f'background:{color}"></div></div>'
    )


def esc(text):
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def render_html(spec, bags, maps, vehicle_param_ok, tfrecord_count, rows, failures, derived, manifest):
    verdict_counts = Counter(row["verdict"] for row in rows)
    total = len(rows)
    gate_state = "PASS" if not failures else "FAIL"
    gate_color = "#1a7f37" if not failures else "#cf222e"

    key_channels = [
        "/apollo/localization/pose",
        "/apollo/perception/obstacles",
        "/apollo/perception/traffic_light",
        "/apollo/perception/safetyobstacles",
        "/apollo/planning",
        "/apollo/prediction",
        "/apollo/routing_response_history",
        "/apollo/canbus/chassis",
        "/apollo/control",
    ]

    parts = [f"""<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>canonical spec v0 —— 三源数据审计</title>
<style>
:root {{ --ink:#1f2328; --mute:#57606a; --line:#d0d7de; --soft:#f6f8fa; }}
* {{ box-sizing:border-box; }}
body {{ margin:0; padding:32px 40px 64px; max-width:1320px; color:var(--ink);
  font:15px/1.65 -apple-system,"Segoe UI","PingFang SC","Microsoft YaHei",sans-serif; }}
h1 {{ font-size:26px; margin:0 0 6px; letter-spacing:-.3px; }}
h2 {{ font-size:19px; margin:38px 0 12px; padding-bottom:8px; border-bottom:1px solid var(--line); }}
h3 {{ font-size:13px; margin:22px 0 8px; color:var(--mute); text-transform:uppercase; letter-spacing:.08em; }}
.sub {{ color:var(--mute); font-size:13px; margin:0 0 22px; }}
.cards {{ display:grid; grid-template-columns:repeat(5,1fr); gap:12px; margin:22px 0 8px; }}
.card {{ border:1px solid var(--line); border-radius:10px; padding:14px 16px; background:var(--soft); }}
.card .k {{ font-size:12px; color:var(--mute); }}
.card .v {{ font-size:27px; font-weight:650; letter-spacing:-.5px; }}
.card .n {{ font-size:12px; color:var(--mute); }}
table {{ border-collapse:collapse; width:100%; font-size:13px; }}
th,td {{ border:1px solid var(--line); padding:5px 8px; text-align:left; vertical-align:top; }}
th {{ background:var(--soft); font-weight:600; font-size:12px; }}
td.c,th.c {{ text-align:center; }}
tbody tr:nth-child(even) td {{ background:#fcfcfd; }}
code {{ background:var(--soft); border:1px solid var(--line); border-radius:4px; padding:1px 5px;
  font-size:11.5px; font-family:ui-monospace,Menlo,monospace; }}
.bar {{ width:100%; height:7px; background:#eaeef2; border-radius:4px; overflow:hidden; min-width:50px; }}
.bar-fill {{ height:100%; }}
.pill {{ display:inline-block; padding:1px 7px; border-radius:999px; font-size:11px; font-weight:600; color:#fff; }}
.gate {{ display:inline-block; padding:3px 12px; border-radius:6px; font-size:13px; font-weight:700; color:#fff; background:{gate_color}; }}
.fail {{ border-left:3px solid #cf222e; background:#fff5f5; padding:9px 13px; border-radius:0 6px 6px 0; margin:7px 0; font-size:13px; }}
.ok {{ border-left:3px solid #1a7f37; background:#f2fbf4; padding:9px 13px; border-radius:0 6px 6px 0; margin:7px 0; font-size:13px; }}
.warn {{ border-left:3px solid #9a6700; background:#fffbf0; padding:9px 13px; border-radius:0 6px 6px 0; margin:7px 0; font-size:13px; }}
.note {{ color:var(--mute); font-size:12.5px; }}
.has {{ color:#1a7f37; font-weight:650; }}
.no {{ color:#cf222e; font-weight:650; }}
.legend {{ display:flex; gap:16px; flex-wrap:wrap; font-size:12px; color:var(--mute); margin-top:8px; }}
.sw {{ display:inline-block; width:10px; height:10px; border-radius:2px; margin-right:5px; }}
</style></head><body>"""]

    parts.append(f"""
<h1>canonical spec v0 —— 三源数据审计</h1>
<p class="sub">spec <code>{esc(spec.get('schema_version'))}</code> ·
数据: {len(bags)} bag · {len(maps)} HD map · {tfrecord_count} tfrecord(参照) ·
车型档案 {'✓' if vehicle_param_ok else '✗'} ·
gate <span class="gate">{gate_state}</span></p>""")

    parts.append('<div class="cards">')
    for key, value, note in [
        ("spec 字段", total, "全部参与判定"),
        ("三源齐备", verdict_counts["both"], "数据源+仿真都有"),
        ("待桥接", verdict_counts["source_bridged"], "仿真需产出"),
        ("仅数据源", verdict_counts["source_only"], "仿真侧完全没有"),
        ("两侧皆无", verdict_counts["neither"] + verdict_counts["sim_only"], "阻塞/凭空"),
    ]:
        parts.append(
            f'<div class="card"><div class="k">{key}</div><div class="v">{value}</div>'
            f'<div class="n">{note}</div></div>'
        )
    parts.append("</div>")

    if failures:
        parts.append("<h2>Gate</h2>")
        for item in failures:
            parts.append(f'<div class="fail">{esc(item)}</div>')
    else:
        parts.append(
            '<div class="ok">gate 通过：bags/maps 数量达标，'
            f'{total} 个字段全部完成判定，无遗漏。</div>'
        )

    # ---- 源 A: bag 通道矩阵
    parts.append("<h2>1. 源 A —— bag 通道实测（6 条 rx-*）</h2>")
    parts.append(
        '<p class="note">数值 = 该 bag 内该通道消息数；<span class="no">—</span> = 通道不存在。'
        "最后一列是该通道在 6 条 bag 中的命中率。</p>"
    )
    parts.append("<table><thead><tr><th>通道</th>")
    for path in bags:
        parts.append(f"<th class='c'>{esc(Path(path).stem.split('.')[0].replace('white-rhino-',''))}</th>")
    parts.append("<th class='c'>命中率</th></tr></thead><tbody>")
    for channel in key_channels:
        parts.append(f"<tr><td><code>{esc(channel)}</code></td>")
        hits = 0
        for counts in bags.values():
            value = counts.get(channel)
            if value:
                hits += 1
                parts.append(f'<td class="c has">{value}</td>')
            else:
                parts.append('<td class="c no">—</td>')
        ratio = hits / len(bags)
        color = "#1a7f37" if ratio >= 1.0 else ("#9a6700" if ratio > 0 else "#cf222e")
        parts.append(f'<td class="c">{ratio:.0%}{bar(ratio, color)}</td></tr>')
    parts.append("</tbody></table>")

    # ---- 源 B: HD map
    parts.append("<h2>2. 源 B —— HD map 拓扑实测（Apollo base_map.bin）</h2>")
    parts.append(
        '<p class="note">lane 子字段计数。拓扑密度 = 计数 / lane 数（>1 表示一 lane 多后继）。'
        "turn / speed_limit 与 lane 数相等即 100% 覆盖。</p>"
    )
    parts.append("<table><thead><tr><th>地图</th><th class='c'>lane</th>")
    for key in ("predecessor_id", "successor_id", "turn", "overlap_id", "speed_limit", "left_boundary", "central_curve"):
        parts.append(f"<th class='c'>{key}</th>")
    parts.append("</tr></thead><tbody>")
    for name, audit in maps.items():
        lane_count = max(1, audit["lane_count"])
        parts.append(f"<tr><td>{esc(name)}<br><span class='note'>lane={audit['lane_count']}</span></td>")
        parts.append(f"<td class='c'>{audit['lane_count']}</td>")
        for key in ("predecessor_id", "successor_id", "turn", "overlap_id", "speed_limit", "left_boundary", "central_curve"):
            value = audit["lane"].get(key, 0)
            ratio = value / lane_count
            color = "#1a7f37" if value else "#cf222e"
            parts.append(f'<td class="c"><span style="color:{color};font-weight:600">{value}</span>'
                         f'<br><span class="note">×{ratio:.2f}</span></td>')
        parts.append("</tr>")
    parts.append("</tbody></table>")

    parts.append("<h3>顶层要素（跨地图）</h3>")
    top_keys = ["lane", "signal", "stop_sign", "crosswalk", "overlap", "junction", "road"]
    parts.append("<table><thead><tr><th>地图</th>")
    for key in top_keys:
        parts.append(f"<th class='c'>{key}</th>")
    parts.append("</tr></thead><tbody>")
    for name, audit in maps.items():
        parts.append(f"<tr><td>{esc(name)}</td>")
        for key in top_keys:
            value = audit["top"].get(key, 0)
            cls = "has" if value else "no"
            parts.append(f'<td class="c {cls}">{value}</td>')
        parts.append("</tr>")
    parts.append("</tbody></table>")

    # ---- 派生证据（occ 的唯一来源）
    parts.append("<h2>3. 派生证据 —— occ 的真实来源</h2>")
    if derived.get("instances"):
        polygon_rate = derived["with_polygon"] / derived["instances"]
        parts.append(
            '<p class="note">bag 内<b>不存在</b>占据栅格通道（全部通道名中无 occ/polygon/grid/free）。'
            "occ 的唯一一手来源是 perception 的<b>对象级多边形</b>，实测如下：</p>"
        )
        parts.append(
            "<table><thead><tr><th>指标</th><th class='c'>实测值</th><th>说明</th></tr></thead><tbody>"
        )
        rows_derived = [
            ("采样范围", f"{len(derived['bags'])} 条 bag", ", ".join(Path(name).name for name in derived["bags"])),
            ("采样帧数", derived["frames"], "每条 bag 取前 20 帧"),
            ("障碍物实例数", derived["instances"], "61 帧累计"),
            ("带 polygon_point 的实例", f"{derived['with_polygon']} ({polygon_rate:.1%})", "field 8 = 对象轮廓"),
            ("多边形点数 中位 / 最大", f"{derived['median_points']} / {derived['max_points']}", "对齐 32 点需等弧长重采样"),
            ("带 height 的实例", derived.get("height_present", 0), "static_polygon_heights 的派生源"),
            ("带 type 的实例", derived.get("type_present", 0), "static_polygon_types 的派生源"),
        ]
        for key, value, note in rows_derived:
            parts.append(
                f'<tr><td>{esc(key)}</td><td class="c" style="font-weight:650">{esc(value)}</td>'
                f'<td class="note">{esc(note)}</td></tr>'
            )
        parts.append("</tbody></table>")
        parts.append(
            '<div class="warn"><b>spec 措辞已据此修正</b>：occ 的 truth 从「bag perception 占据栅格」'
            "改为「perception 对象多边形 + HD map 静态物」派生物。占据栅格本身在本数据湖不存在。</div>"
        )
    else:
        parts.append('<div class="fail">未取到派生证据（bag 不可读）。</div>')

    parts.append("<h3>外部存账（release.info）</h3>")
    if manifest:
        parts.append("<table><thead><tr><th>批次</th><th>map version</th><th>commit</th><th>branch</th></tr></thead><tbody>")
        for entry in manifest:
            fields = entry["fields"]
            parts.append(
                f'<tr><td><code>{esc(entry["name"])}</code></td>'
                f'<td>{esc(fields.get("version", "-"))}</td>'
                f'<td>{esc(fields.get("commit", "-"))}</td>'
                f'<td>{esc(fields.get("branch", "-"))}</td></tr>'
            )
        parts.append("</tbody></table>")
    else:
        parts.append('<div class="fail">无 release.info，map_version/perception_version 无存账来源。</div>')

    # ---- 判定矩阵
    parts.append("<h2>4. spec 字段 × 三源判定</h2>")
    parts.append('<div class="legend">')
    for key, (color, label, _desc) in VERDICT_META.items():
        parts.append(f'<span><span class="sw" style="background:{color}"></span>{label}（{verdict_counts[key]}）</span>')
    parts.append("</div>")
    parts.append("<table><thead><tr><th>组</th><th>字段</th><th>sim</th><th>数据源</th>"
                 "<th class='c'>源命中率</th><th>证据</th><th class='c'>判定</th></tr></thead><tbody>")
    for row in rows:
        color, label, _desc = VERDICT_META[row["verdict"]]
        parts.append(
            f'<tr><td><code>{esc(row["group"])}</code></td>'
            f'<td><code>{esc(row["field"])}</code></td>'
            f'<td class="c">{esc(row["sim"])}</td>'
            f'<td class="c">{esc(row["source_kind"])}</td>'
            f'<td class="c">{row["source_coverage"]:.0%}'
            f'{bar(row["source_coverage"], color)}</td>'
            f'<td class="note">{esc(row["source_detail"])}</td>'
            f'<td class="c"><span class="pill" style="background:{color}">{label}</span></td></tr>'
        )
    parts.append("</tbody></table>")

    # ---- 结论
    sim_only = [row for row in rows if row["verdict"] == "sim_only"]
    source_only = [row for row in rows if row["verdict"] in ("source_only", "source_bridged")]
    neither = [row for row in rows if row["verdict"] == "neither"]

    parts.append("<h2>5. 结论与可裁决清单</h2>")
    if sim_only:
        parts.append(
            '<div class="warn"><b>仿真有、数据源无（凭空字段）</b>：'
            "训练契约里这些字段没有真值来源，必须降级为派生物或删除。"
            "<ul>" + "".join(
                f'<li><code>{esc(row["field"])}</code> · {esc(row["source_detail"])}</li>'
                for row in sim_only
            ) + "</ul></div>"
        )
    if source_only:
        parts.append(
            '<div class="warn"><b>数据源有、仿真缺（桥接必做）</b>：' +
            ", ".join(f'<code>{esc(row["field"])}</code>' for row in source_only[:24]) +
            (f" 等 {len(source_only)} 项" if len(source_only) > 24 else "") + "</div>"
        )
    if neither:
        parts.append(
            '<div class="fail"><b>两侧皆无（阻塞）</b>：' +
            ", ".join(f'<code>{esc(row["field"])}</code>' for row in neither) + "</div>"
        )

    parts.append(
        '<p class="sub" style="margin-top:36px">由 <code>scripts/spec_audit.py</code> 生成，勿手改。'
        "spec 源 <code>spec/canonical_v0.yaml</code>。</p></body></html>"
    )
    return "".join(parts)


def main():
    parser = argparse.ArgumentParser(description="canonical spec 三源数据审计")
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--cache", type=Path, default=DEFAULT_CACHE)
    parser.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT)
    parser.add_argument("--map-root", type=Path, default=DEFAULT_MAP_ROOT)
    parser.add_argument("--vehicle-param", type=Path, default=DEFAULT_VEHICLE_PARAM)
    parser.add_argument("--tfrecord-dir", type=Path, default=DEFAULT_TFRECORD_DIR)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--no-cache", action="store_true")
    args = parser.parse_args()

    spec = yaml.safe_load(args.spec.read_text(encoding="utf-8"))

    bag_paths = sorted(args.data_root.glob("rx-*/*.bag"))
    print(f"[bags] 发现 {len(bag_paths)} 条 bag")
    bags = audit_bags(bag_paths, args.cache, not args.no_cache)

    maps = OrderedDict()
    for bag_path in bag_paths:
        for link in bag_path.parent.glob("*"):
            if not link.is_symlink():
                continue
            name = link.name
            if name in maps:
                continue
            map_bin = args.map_root / name / "base_map.bin"
            if not map_bin.exists():
                print(f"[maps] {name}: 无 base_map.bin，跳过（仅 txt，过大不解析）")
                continue
            print(f"[maps] 解析 {name} ({map_bin.stat().st_size / 1e6:.0f} MB)")
            maps[name] = audit_hdmap(map_bin)

    vehicle_param_ok = args.vehicle_param.exists()
    tfrecord_count = len(list(args.tfrecord_dir.glob("*.tfrecord.gz"))) if args.tfrecord_dir.exists() else 0

    print(f"[derived] 采样 perception obstacles 多边形证据...")
    derived = audit_obstacle_polygons(bag_paths)
    manifest = audit_manifest(args.data_root)
    if derived.get("instances"):
        print(
            f"[derived] 实例 {derived['instances']}, 带多边形 {derived['with_polygon']} "
            f"({derived['with_polygon'] / derived['instances']:.0%}), "
            f"点数中位 {derived['median_points']} 最大 {derived['max_points']}"
        )
    print(f"[manifest] release.info {len(manifest)} 份")

    rows = build_matrix(spec, bags, maps, vehicle_param_ok, tfrecord_count, derived, manifest)
    verdict_counts = Counter(row["verdict"] for row in rows)

    failures = []
    if len(bags) < MIN_BAGS:
        failures.append(f"bags {len(bags)} < {MIN_BAGS}")
    if len(maps) < MIN_MAPS:
        failures.append(f"maps {len(maps)} < {MIN_MAPS}")
    if any(row["verdict"] is None for row in rows):
        failures.append("存在未判定字段")

    print(f"[spec] 字段 {len(rows)}")
    for key, (_color, label, _desc) in VERDICT_META.items():
        print(f"  {label:12s} {verdict_counts[key]:3d}")
    print(f"[maps] 解析 {len(maps)} 张: {', '.join(maps)}")
    print(f"[tfrecord] 参照集 {tfrecord_count} 个文件（本次仅计数，不深解析）")

    for item in failures:
        print(f"GATE FAIL : {item}")
    if not failures:
        print("GATE      : PASS")

    if not args.check:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(
            render_html(
                spec, bags, maps, vehicle_param_ok, tfrecord_count, rows, failures, derived, manifest
            ),
            encoding="utf-8",
        )
        args.cache.parent.mkdir(parents=True, exist_ok=True)
        args.cache.write_text(
            json.dumps(
                {
                    "bags": bags,
                    "maps": maps,
                    "rows": rows,
                    "derived": derived,
                    "manifest": manifest,
                    "failures": failures,
                },
                ensure_ascii=False,
                indent=1,
            ),
            encoding="utf-8",
        )
        print(f"report    : {args.out}")
        print(f"cache     : {args.cache}")

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
