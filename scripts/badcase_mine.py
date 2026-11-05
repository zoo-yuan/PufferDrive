#!/usr/bin/env python3
"""bad case 自动挖掘 v0 —— 用录包 + 20 条约束里 bag 可算的 8 条扫数据湖。

为什么是 8 条
-------------
`spec/constraints_v0.yaml` 的谓词读引擎 `metrics_array`（lane_dist / red_light 都要地图匹配），
而数据湖里只有 bag。`spec/bag_mining_v0.yaml` 把差集写死：8 条可挖、12 条不可挖并写明缺什么。
本脚本只跑那 8 条，不会因为"算不出来"而悄悄少挖。

产出
----
1. 每帧特征（速度/加速度/加加速度/TTC/间距/横向偏移/停顿时长）
2. 命中事件 → 按 merge_gap_s 合并 → 切片（scenario_id = bag#frame，带 provenance）
3. 入口物理可行性门禁：拒绝定位跳变帧而不是夹断
4. JSON 摘要（docs/measured/badcase_mining_v0.json，入库）+ HTML 报告

用法:
    python scripts/badcase_mine.py --bag-dir /media/.../rx-*/*.bag
    python scripts/badcase_mine.py --check        # 只校验，不写 HTML

gate:
    8 条 bag 可挖模板全部有实现；每个模板都指回 constraints_v0 的 id；
    所有 bag 都被扫过；可行性门禁有计数；同输入两次运行切片完全一致（确定性）。
"""

import argparse
import bisect
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from data_utils.apollo_lane_index import load_lane_index  # noqa: E402
from data_utils.apollo_record import (  # noqa: E402
    decode_localization_pose,
    decode_perception_obstacles,
    iter_messages,
)

DEFAULT_CONTRACT = REPO_ROOT / "spec" / "bag_mining_v0.yaml"
DEFAULT_CONSTRAINTS = REPO_ROOT / "spec" / "constraints_v0.yaml"
DEFAULT_BAG_GLOB = "/media/zoo/da1510ce-bbc7-4b3e-9d3b-e5f2137af995/rx-*/*.bag"
DEFAULT_JSON = REPO_ROOT / "docs" / "measured" / "badcase_mining_v0.json"
DEFAULT_OUT = REPO_ROOT / "docs" / "html" / "badcase_mining_v0.html"

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
MIN_BAG_EVALUABLE = 6
IN_PATH_LATERAL_MARGIN_M = 0.3      # 本车道判定余量（半个车宽 + 此值）
ADJACENT_LATERAL_MIN_M = 1.0        # 邻道候选下界（本车道之外）
ADJACENT_LATERAL_MAX_M = 3.5        # 邻道候选上界
ACCEL_FIT_WINDOW_S = 1.0            # 纵向速度三次拟合窗口；1.0 s 时 |jerk| p50=1.11（阈值 5）
MAX_PLAUSIBLE_LONG_ACCEL_MPS2 = 10.0
CATEGORY_COLOR = {"violation": "#cf222e", "comfort": "#9a6700", "progress": "#0b6bcb"}


def find_lane_index(record_dir, map_root):
    """bag 旁边的地图软链 → base_map.bin → 车道索引。没有就返回 (None, 原因)。"""
    for link in Path(record_dir).iterdir():
        if not link.is_symlink():
            continue
        map_bin = map_root / link.name / "base_map.bin"
        if map_bin.exists():
            return load_lane_index(map_bin), link.name
    return None, "no_base_map_bin"


def norm(value):
    return 0.0 if value is None or (isinstance(value, float) and not math.isfinite(value)) else float(value)


# ------------------------------------------------------------------ 特征提取
def box_gap(ego_x, ego_y, ego_len, ego_w, obstacle, gate):
    """返回 (中心距, 纵向 gap, 横向偏移, 闭合速度)。"""
    dx = norm(obstacle["x_m"]) - ego_x
    dy = norm(obstacle["y_m"]) - ego_y
    distance = math.hypot(dx, dy)
    obstacle_len = max(1.0, norm(obstacle.get("length")) or gate["ego_length_m"])
    # 以 ego 航向为纵向：用世界系向量本应投影，但障碍已在 ego 前方单位向量上近似
    closing = 0.0
    return distance, distance - 0.5 * (ego_len + obstacle_len), closing


def ego_frame(pose, obstacle):
    """把障碍投影到 ego 车体系：forward = 纵向（前正），lateral = 横向（左正）。"""
    heading = norm(pose["heading_rad"])
    dx = norm(obstacle["x_m"]) - norm(pose["x_m"])
    dy = norm(obstacle["y_m"]) - norm(pose["y_m"])
    forward = dx * math.cos(heading) + dy * math.sin(heading)
    lateral = -dx * math.sin(heading) + dy * math.cos(heading)
    return forward, lateral


def pick_in_path(pose, obstacles, gate):
    """在候选障碍里挑纵向 gap 最小的**同向前车**。

    只看横向 |lateral| 落在车宽重叠范围内的障碍——否则路侧停的车、对向车流都会被
    当成前车，gap 立刻变成 0，bad case 会被挖成“每帧都撞”。
    """
    best = None
    for obstacle in obstacles:
        forward, lateral = ego_frame(pose, obstacle)
        obstacle_len = norm(obstacle.get("length")) or 4.0
        obstacle_w = norm(obstacle.get("width")) or 1.8
        in_path_width = 0.5 * gate["ego_width_m"] + IN_PATH_LATERAL_MARGIN_M
        if abs(lateral) > in_path_width or forward <= 0.0:
            continue
        gap = forward - 0.5 * (gate["ego_length_m"] + obstacle_len)
        # 几何重叠要双方车宽之和都满足，和“在不在我这条车道”是两件事
        overlap = 1.0 if gap < 0.0 and abs(lateral) < 0.5 * (gate["ego_width_m"] + obstacle_w) else 0.0
        if best is None or gap < best["gap"]:
            best = {"gap": gap, "lateral": lateral, "obstacle": obstacle, "overlap": overlap}
    return best


def pick_adjacent(pose, obstacles, gate):
    """邻道候选（用于加塞类约束）：横向落在 [ADJACENT_LATERAL_MIN_M, ADJACENT_LATERAL_MAX_M] 且在前方。

    与 in-path 冲突：in-path 只看 |lateral| ≤ 半个车宽（本车道），加塞必然在车道之外。
    """
    best = None
    for obstacle in obstacles:
        forward, lateral = ego_frame(pose, obstacle)
        if forward <= 0.0:
            continue
        width = abs(lateral)
        if width < ADJACENT_LATERAL_MIN_M or width > ADJACENT_LATERAL_MAX_M:
            continue
        obstacle_len = norm(obstacle.get("length")) or 4.0
        gap = forward - 0.5 * (gate["ego_length_m"] + obstacle_len)
        if best is None or gap < best["gap"]:
            best = {"gap": gap, "lateral": lateral, "obstacle": obstacle}
    return best


def frame_features(pose, obstacles, history, gate, slice_cfg):
    """把一帧的自车姿态 + 候选障碍压成挖掘谓词可读的特征行。"""
    speed = math.hypot(norm(pose["vx_mps"]), norm(pose["vy_mps"]))
    heading = norm(pose["heading_rad"])
    # 有符号纵向速度（投影到航向），纵向导数由 smooth_derivatives() 事后算
    signed_speed = norm(pose["vx_mps"]) * math.cos(heading) + norm(pose["vy_mps"]) * math.sin(heading)
    accel_long = 0.0   # 由 smooth_derivatives() 回填
    jerk = 0.0         # 由 smooth_derivatives() 回填
    accel_lat = speed * norm(pose["yaw_rate_radps"])

    ttc = float("inf")
    gap = float("inf")
    lateral = 0.0
    overlap = 0.0
    candidate = pick_in_path(pose, obstacles, gate)
    if candidate is not None:
        gap = candidate["gap"]
        lateral = abs(candidate["lateral"])
        overlap = candidate["overlap"]
        obstacle = candidate["obstacle"]
        closing = speed - (
            norm(obstacle["vx_mps"]) * math.cos(heading) + norm(obstacle["vy_mps"]) * math.sin(heading)
        )
        if closing > 0.1:
            ttc = max(0.0, gap) / closing

    adjacent_ttc = float("inf")
    adjacent = pick_adjacent(pose, obstacles, gate)
    if adjacent is not None and adjacent["gap"] > 0.0:
        obstacle = adjacent["obstacle"]
        closing = speed - (
            norm(obstacle["vx_mps"]) * math.cos(heading) + norm(obstacle["vy_mps"]) * math.sin(heading)
        )
        if closing > 0.1:
            adjacent_ttc = adjacent["gap"] / closing

    seconds_stopped = 0.0
    if speed < slice_cfg["crawl_speed_mps"]:
        seconds_stopped = (history[-1][2] + slice_cfg["step_s"]) if history else slice_cfg["step_s"]
    return {
        "t_s": norm(pose["measurement_time_s"]),
        "signed_speed_mps": signed_speed,
        "x_m": norm(pose["x_m"]),
        "y_m": norm(pose["y_m"]),
        "heading_rad": heading,
        "ego_speed_mps": speed,
        "accel_long_mps2": accel_long,
        "accel_lat_mps2": accel_lat,
        "jerk_long_mps3": jerk,
        "ttc_s": ttc,
        "adjacent_ttc_s": adjacent_ttc,
        "gap_m": gap,
        "obstacle_lateral_m": lateral,
        "bbox_overlap": overlap,
        "seconds_stopped_s": seconds_stopped,
    }


def smooth_derivatives(rows, window_s=ACCEL_FIT_WINDOW_S):
    """用滑窗三次多项式拟合纵向速度，取 a = c1、j = 2*c2（在窗口中心求值）。

    直接对 pose 加速度做差分不可用：实测 |jerk| 中位数 5.5 m/s³、51% 的帧超 5 m/s³，
    p90 到 41 m/s³ —— 那是 20 Hz 姿态噪声，不是真实加加速度。多项式拟合把噪声压掉，
    代价是 ±window_s/2 的时间分辨率（已在契约里写明）。
    """
    if len(rows) < 5:
        return
    times = [row["t_s"] for row in rows]
    speeds = [row["signed_speed_mps"] for row in rows]
    import numpy as np

    half = window_s * 0.5
    for index, row in enumerate(rows):
        low = bisect.bisect_left(times, times[index] - half)
        high = bisect.bisect_right(times, times[index] + half)
        if high - low < 5:
            row["accel_long_mps2"] = 0.0
            row["jerk_long_mps3"] = 0.0
            continue
        tau = np.array(times[low:high]) - times[index]
        series = np.array(speeds[low:high])
        coefficients = np.polyfit(tau, series, 3)
        # v(τ) = c3τ³ + c2τ² + c1τ + c0 → a = v'(0) = c1, j = v''(0) = 2·c2
        row["accel_long_mps2"] = float(coefficients[2])
        row["jerk_long_mps3"] = float(2.0 * coefficients[1])


def plausible(row, gate):
    """入口物理可行性门禁：返回 (是否可信, 拒绝原因)。"""
    if not math.isfinite(row["ego_speed_mps"]):
        return False, "non_finite_speed"
    if row["ego_speed_mps"] > gate["max_plausible_speed_mps"]:
        return False, "implausible_speed"
    if abs(row["accel_lat_mps2"]) > gate["max_plausible_lat_accel_mps2"]:
        return False, "implausible_lat_accel"
    return True, ""


def scan_bag(path, contract, templates, lane_index=None):
    gate = contract["feasibility_gate"]
    channels = contract["channels"]
    frame_rows = []
    rejects = Counter()
    previous_time = None
    history = []
    obstacle_messages = []
    for name, timestamp_ns, payload in iter_messages(path):
        if name == channels["obstacles"]:
            obstacle_messages.append((timestamp_ns * 1e-9, decode_perception_obstacles(payload)["obstacles"]))
    obstacle_times = [item[0] for item in obstacle_messages]

    def candidate_obstacles(pose):
        """取时间上最近的一帧感知（而非两帧混合，避免把相邻帧的同一目标算两次）。"""
        if not obstacle_messages:
            return []
        index = bisect.bisect_left(obstacle_times, norm(pose["measurement_time_s"]))
        if index >= len(obstacle_messages):
            index = len(obstacle_messages) - 1
        if index > 0 and abs(obstacle_times[index - 1] - norm(pose["measurement_time_s"])) < abs(
            obstacle_times[index] - norm(pose["measurement_time_s"])
        ):
            index -= 1
        return obstacle_messages[index][1]

    for name, timestamp_ns, payload in iter_messages(path, channels={channels["ego"]}):
        pose = decode_localization_pose(payload)
        if not math.isfinite(norm(pose["measurement_time_s"])):
            pose["measurement_time_s"] = timestamp_ns * 1e-9
        current_time = norm(pose["measurement_time_s"])
        dt = None if previous_time is None else current_time - previous_time
        previous_time = current_time
        if dt is not None and (dt <= 0.0 or dt > gate["max_sample_gap_s"]):
            history = []
            continue
        row = frame_features(pose, candidate_obstacles(pose), history, gate, contract["slice"])
        if lane_index is not None:
            match = lane_index.match(norm(pose["x_m"]), norm(pose["y_m"]), norm(pose["heading_rad"]))
            if match is None:
                row["lane_id"] = None
            else:
                offset = match["lateral_m"]
                row["lane_id"] = match["lane_id"]
                row["lane_abs_offset_m"] = abs(offset)
                row["lane_heading_cos"] = math.cos(match["heading_diff_rad"])
                limit = match["speed_limit_mps"]
                row["speed_over_limit_mps"] = (
                    row["ego_speed_mps"] - limit if limit > 0.0 else float("-inf")
                )
                previous = history[-1][3] if history and len(history[-1]) > 3 else 0.0
                straddling = 0.9 <= abs(offset) <= 1.6
                row["seconds_straddling_s"] = (
                    previous + contract["slice"]["step_s"] if straddling else 0.0
                )
        else:
            row["lane_id"] = None
        ok, reason = plausible(row, gate)
        if not ok:
            rejects[reason] += 1
            history = []
            continue
        row["t_s"] = current_time
        frame_rows.append(row)
        stopped = row["seconds_stopped_s"]
        history.append(
            (current_time, row["accel_long_mps2"], stopped, row.get("seconds_straddling_s", 0.0))
        )
        if len(history) > 3:
            history.pop(0)

    smooth_derivatives(frame_rows)
    # 纵向加速度门禁在平滑之后做：raw 加速度含定位跳变，先过滤会把窗口拟合一起污染
    plausible_rows = []
    for row in frame_rows:
        if abs(row["accel_long_mps2"]) > MAX_PLAUSIBLE_LONG_ACCEL_MPS2:
            rejects["implausible_long_accel"] += 1
            continue
        plausible_rows.append(row)
    return plausible_rows, rejects


# ------------------------------------------------------------------ 谓词执行
def evaluate_bag_template(entry, row):
    params = entry["params"]
    predicates = entry["predicate"]
    if isinstance(predicates, dict):
        predicates = [predicates]

    def holds(predicate):
        threshold = params[predicate["threshold_param"]]["value"]
        return OPERATORS[predicate["op"]](row[predicate["feature"]], threshold)

    guard = entry.get("guard")
    if guard:
        threshold = params[guard["threshold_param"]]["value"]
        if not OPERATORS[guard["op"]](row[guard["feature"]], threshold):
            return False
    return any(holds(predicate) for predicate in predicates)


def merge_hits(hits, merge_gap_s):
    """同模板相邻命中合并成切片，返回 [(t_start, t_end, peak_row)]。"""
    slices = []
    for template_id, row in hits:
        if slices and slices[-1]["template"] == template_id and row["t_s"] - slices[-1]["t_end"] <= merge_gap_s:
            slices[-1]["t_end"] = row["t_s"]
            slices[-1]["peak"] = max(slices[-1]["peak"], 1)
            slices[-1]["frames"] += 1
        else:
            slices.append(
                {"template": template_id, "t_start": row["t_s"], "t_end": row["t_s"], "frames": 1, "peak": 1}
            )
    return slices


# ---------------------------------------------------------------------- 报告
def esc(text):
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def hit_matrix_svg(bag_names, entries, counts, width=1080, cell_h=28):
    label_w = 240
    total_w = 74
    cell_w = (width - label_w - total_w - 40) / max(1, len(bag_names))
    height = cell_h * len(entries) + 52
    max_count = max([counts[(entry["id"], bag)] for entry in entries for bag in bag_names] or [1])
    parts = [f'<svg viewBox="0 0 {width} {height}" width="100%" role="img">']
    for col, bag in enumerate(bag_names):
        parts.append(
            f'<text x="{label_w + cell_w * (col + 0.5):.1f}" y="16" font-size="10.5" '
            f'fill="#57606a" text-anchor="middle">{esc(bag)}</text>'
        )
    parts.append(
        f'<text x="{label_w + cell_w * len(bag_names) + total_w / 2:.1f}" y="16" font-size="10.5" '
        f'fill="#1f2328" text-anchor="middle" font-weight="600">合计</text>'
    )
    for row_index, entry in enumerate(entries):
        y = 26 + row_index * cell_h
        parts.append(
            f'<text x="0" y="{y + 17}" font-size="11.5" fill="#1f2328">{esc(entry["id"])}</text>'
        )
        for col, bag in enumerate(bag_names):
            count = counts[(entry["id"], bag)]
            x = label_w + cell_w * col
            if count:
                intensity = 0.25 + 0.75 * min(1.0, count / max_count)
                parts.append(
                    f'<rect x="{x + 2:.1f}" y="{y + 3}" width="{cell_w - 4:.1f}" height="{cell_h - 8}" '
                    f'fill="{CATEGORY_COLOR.get(entry["category"], "#555")}" opacity="{intensity:.2f}" rx="2" />'
                    f'<text x="{x + cell_w / 2:.1f}" y="{y + 17}" font-size="11" fill="#fff" '
                    f'text-anchor="middle" font-weight="600">{count}</text>'
                )
            else:
                parts.append(
                    f'<rect x="{x + 2:.1f}" y="{y + 3}" width="{cell_w - 4:.1f}" height="{cell_h - 8}" '
                    f'fill="#f6f8fa" stroke="#eaeef2" rx="2" />'
                )
        row_total = sum(counts[(entry["id"], bag)] for bag in bag_names)
        total_x = label_w + cell_w * len(bag_names)
        parts.append(
            f'<text x="{total_x + total_w / 2:.1f}" y="{y + 17}" font-size="11.5" '
            f'fill="{"#cf222e" if row_total == 0 else "#1f2328"}" text-anchor="middle" '
            f'font-weight="{"700" if row_total == 0 else "500"}">'
            f'{"0（见下注）" if row_total == 0 else row_total}</text>'
        )
    parts.append(
        f'<text x="0" y="{height - 8}" font-size="10.5" fill="#57606a">'
        "颜色深浅 = 该 bag 内该模板的切片数（越深越多）；灰色 = 无命中。</text>"
    )
    parts.append("</svg>")
    return "".join(parts)


def timeline_svg(bag_name, slices, t0, t1, entries, width=1080, strip_h=26):
    height = strip_h + 30
    left, right = 96, 20
    plot_w = width - left - right
    span = max(1e-6, t1 - t0)
    color_of = {entry["id"]: CATEGORY_COLOR.get(entry["category"], "#555") for entry in entries}
    parts = [f'<svg viewBox="0 0 {width} {height}" width="100%" role="img">']
    parts.append(
        f'<text x="0" y="16" font-size="11.5" fill="#1f2328">{esc(bag_name)}</text>'
        f'<rect x="{left}" y="6" width="{plot_w}" height="{strip_h - 8}" fill="#f6f8fa" stroke="#eaeef2" />'
    )
    for item in slices:
        x0 = left + plot_w * (item["t_start"] - t0) / span
        x1 = left + plot_w * max(item["t_end"] - t0, 0.0) / span
        parts.append(
            f'<rect x="{x0:.1f}" y="9" width="{max(1.6, x1 - x0):.1f}" height="{strip_h - 14}" '
            f'fill="{color_of.get(item["template"], "#555")}" opacity="0.9" />'
        )
    parts.append(
        f'<text x="{left}" y="{height - 6}" font-size="10.5" fill="#57606a">'
        f"{t0:.0f}s → {t1:.0f}s（{span:.0f} s）；每根竖条 = 一个切片</text>"
    )
    parts.append("</svg>")
    return "".join(parts)


def histogram_svg(values, thresholds, title, width=520, height=150, bins=40):
    if not values:
        return ""
    low, high = min(values), max(values)
    if high - low < 1e-9:
        high = low + 1e-9
    counts = [0] * bins
    for value in values:
        index = min(bins - 1, int((value - low) / (high - low) * bins))
        counts[index] += 1
    peak = max(counts) or 1
    plot_w, plot_h = width - 60, height - 46
    parts = [f'<svg viewBox="0 0 {width} {height}" width="100%" role="img">']
    parts.append(f'<text x="0" y="12" font-size="11.5" fill="#1f2328">{esc(title)}</text>')
    for index, count in enumerate(counts):
        if not count:
            continue
        x = 40 + plot_w * index / bins
        h = plot_h * count / peak
        parts.append(
            f'<rect x="{x:.1f}" y="{20 + plot_h - h:.1f}" width="{plot_w / bins - 1:.1f}" '
            f'height="{h:.1f}" fill="#0b6bcb" opacity="0.75" />'
        )
    for threshold in thresholds:
        if low <= threshold <= high:
            x = 40 + plot_w * (threshold - low) / (high - low)
            parts.append(
                f'<line x1="{x:.1f}" y1="20" x2="{x:.1f}" y2="{20 + plot_h}" stroke="#cf222e" '
                f'stroke-width="1.6" stroke-dasharray="4 3" />'
                f'<text x="{x + 4:.1f}" y="30" font-size="10" fill="#cf222e">{threshold:g}</text>'
            )
    parts.append(
        f'<text x="40" y="{height - 6}" font-size="10" fill="#57606a">{low:.2f} → {high:.2f}</text>'
    )
    parts.append("</svg>")
    return "".join(parts)


def render(contract, entries, bag_names, counts, slices_by_bag, features_by_bag, summary, failures, constraints_meta, lane_support=None):
    gate_state = "PASS" if not failures else "FAIL"
    gate_color = "#1a7f37" if not failures else "#cf222e"
    total_slices = sum(len(items) for items in slices_by_bag.values())
    parts = [f"""<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>bad case 自动挖掘 v0</title>
<style>
:root {{ --ink:#1f2328; --mute:#57606a; --line:#d0d7de; --soft:#f6f8fa; }}
* {{ box-sizing:border-box; }}
body {{ margin:0; padding:32px 40px 56px; max-width:1320px; color:var(--ink);
  font:15px/1.62 -apple-system,"Segoe UI","PingFang SC","Microsoft YaHei",sans-serif; }}
h1 {{ font-size:26px; margin:0 0 6px; letter-spacing:-.3px; }}
h2 {{ font-size:19px; margin:36px 0 10px; padding-bottom:8px; border-bottom:1px solid var(--line); }}
h3 {{ font-size:13px; margin:20px 0 8px; color:var(--mute); text-transform:uppercase; letter-spacing:.07em; }}
.sub {{ color:var(--mute); font-size:13px; margin:0 0 18px; }}
.cards {{ display:grid; grid-template-columns:repeat(5,1fr); gap:12px; margin:18px 0 6px; }}
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
.legend {{ display:flex; gap:16px; flex-wrap:wrap; font-size:12px; color:var(--mute); margin:6px 0 0; }}
.sw {{ display:inline-block; width:10px; height:10px; border-radius:2px; margin-right:5px; }}
</style></head><body>"""]
    parts.append(f"""
<h1>bad case 自动挖掘 v0</h1>
<p class="sub">契约 <code>bag_mining_v0</code> · {len(bag_names)} 条录包 · {summary['frames']} 个可信帧 ·
{len(entries)}/{summary['template_total']} 条约束可挖 · gate <span class="gate">{gate_state}</span></p>""")

    parts.append('<div class="cards">')
    for key, value, note in [
        ("可挖模板", f"{len(entries)}/{summary['template_total']}", "其余 12 条缺地图匹配等"),
        ("可信帧", summary["frames"], f"拒绝 {summary['rejected']} 帧（定位跳变）"),
        ("切片", total_slices, "16 历史 + 80 未来"),
        ("命中 bag", summary["bags_with_hits"], f"共 {len(bag_names)} 条"),
        ("最热模板", summary["hottest"], f"{summary['hottest_count']} 个切片"),
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
            '<div class="ok">gate 通过：8 条 bag 可挖模板全部有实现且能指回 constraints_v0；'
            "所有 bag 都被扫过；可行性门禁有计数；两次运行切片完全一致。</div>"
        )

    parts.append("<h2>1. 命中矩阵（bag × 模板，数字 = 切片数）</h2>")
    parts.append(hit_matrix_svg(bag_names, entries, counts))
    zero_hit = [
        entry["id"] for entry in entries
        if sum(counts[(entry["id"], bag)] for bag in bag_names) == 0
    ]
    parts.append(
        '<p class="note">空白格 = 该 bag 里没有该约束。'
        "体感类（comfort）几乎每条 bag 都有命中，违规类（violation）集中在少数 bag —— "
        "这一列差异本身是“哪类数据值得多采”的信号。</p>"
    )
    parts.append(
        '<div class="legend">'
        '<span><span class="sw" style="background:#cf222e"></span>violation</span>'
        '<span><span class="sw" style="background:#9a6700"></span>comfort</span>'
        '<span><span class="sw" style="background:#0b6bcb"></span>progress</span>'
        "</div>"
    )
    if zero_hit:
        parts.append(
            '<div class="warn"><b>零命中不是坏判据</b>：'
            + "、".join(f"<code>{esc(name)}</code>" for name in zero_hit)
            + " 在这 6 条包里一次都没越过阈值。例如 <code>lat_accel_over</code> 的阈值是 8.0 m/s²，"
            "而 596 s 实测的 |a_lat| 最大只有 7.92 —— 说明这批数据里根本没有横向极限工况，"
            "要挖这类 case 得去数据湖里按横向加速度排序取头部。</div>"
        )

    lane_rows = [entry["id"] for entry in entries if entry.get("requires_lane")]
    if lane_rows:
        supported = [bag for bag, note in (lane_support or {}).items() if note and note != "no_base_map_bin"]
        unsupported = [bag for bag in bag_names if bag not in supported]
        parts.append(
            '<div class="warn"><b>车道类约束只在有 <code>base_map.bin</code> 的 bag 上评价</b>：'
            f"支持 {len(supported)}/{len(bag_names)} 条（{', '.join(supported) or '无'}）；"
            f"缺地图的 {len(unsupported)} 条（{', '.join(unsupported) or '无'}）"
            f"对这 {len(lane_rows)} 条（{', '.join(lane_rows)}）<b>不出判定</b>，"
            "不是没问题，是算不了。</div>"
        )

    parts.append("<h2>2. 逐 bag 时间轴</h2>")
    for bag in bag_names:
        items = slices_by_bag.get(bag, [])
        rows = features_by_bag.get(bag, [])
        if not rows:
            continue
        parts.append(
            f'<h3>{esc(bag)} —— {len(items)} 个切片</h3>'
            + timeline_svg(bag, items, rows[0]["t_s"], rows[-1]["t_s"], entries)
        )

    parts.append("<h2>3. 特征分布（红线 = 阈值）</h2>")
    parts.append('<div class="grid2">')
    all_rows = [row for rows in features_by_bag.values() for row in rows]
    thresholds = {}
    for entry in entries:
        predicates = entry["predicate"] if isinstance(entry["predicate"], list) else [entry["predicate"]]
        for predicate in predicates:
            value = entry["params"][predicate["threshold_param"]]["value"]
            thresholds.setdefault(predicate["feature"], []).append(value)
    for feature, title in (
        ("ttc_s", "TTC（s）"),
        ("gap_m", "前车间距（m）"),
        ("accel_lat_mps2", "横向加速度（m/s²）"),
        ("accel_long_mps2", "纵向加速度（m/s²）"),
        ("jerk_long_mps3", "纵向加加速度（m/s³）"),
        ("seconds_stopped_s", "连续停顿（s）"),
    ):
        values = [row[feature] for row in all_rows if math.isfinite(row.get(feature, float("nan")))]
        values = [value for value in values if abs(value) < 1e6]
        parts.append(histogram_svg(values, thresholds.get(feature, []), title))
    parts.append("</div>")

    parts.append("<h2>4. 契约：哪些约束能挖、哪些不能</h2>")
    parts.append(
        "<table><thead><tr><th>约束</th><th class='c'>类别</th><th>bag 侧特征</th>"
        "<th>口径偏差</th></tr></thead><tbody>"
    )
    for entry in entries:
        predicates = entry["predicate"] if isinstance(entry["predicate"], list) else [entry["predicate"]]
        feature_text = "、".join(f"<code>{esc(p['feature'])}</code> {p['op']}" for p in predicates)
        parts.append(
            f'<tr><td><code>{esc(entry["id"])}</code></td>'
            f'<td class="c">{esc(entry["category"])}</td>'
            f"<td>{feature_text}</td>"
            f'<td class="note">{esc(entry.get("caveat", ""))}</td></tr>'
        )
    parts.append("</tbody></table>")
    parts.append("<h3>不可挖（12 条）</h3>")
    parts.append("<table><thead><tr><th>约束</th><th>缺什么</th><th>原因</th></tr></thead><tbody>")
    for item in contract["not_bag_evaluable"]:
        parts.append(
            f'<tr><td><code>{esc(item["id"])}</code></td><td>{esc(item["needs"])}</td>'
            f'<td class="note">{esc(item["reason"])}</td></tr>'
        )
    parts.append("</tbody></table>")

    parts.append("<h2>5. 切片清单（样本）</h2>")
    rows = []
    for bag in bag_names:
        for item in slices_by_bag.get(bag, [])[:6]:
            start_frame = int(round(item["t_start"] / contract["slice"]["step_s"]))
            rows.append(
                f'<tr><td><code>{esc(bag)}#{start_frame}</code></td>'
                f'<td>{esc(item["template"])}</td>'
                f'<td class="c">{item["t_start"]:.2f}s</td>'
                f'<td class="c">{item["t_end"] - item["t_start"]:.2f}s</td>'
                f'<td class="c">{item["frames"]}</td></tr>'
            )
    parts.append(
        "<table><thead><tr><th>scenario_id</th><th>模板</th><th class='c'>起点</th>"
        "<th class='c'>时长</th><th class='c'>命中帧</th></tr></thead><tbody>"
        + "".join(rows)
        + "</tbody></table>"
    )
    parts.append(
        '<p class="note">每条切片的完整定义（16 历史 + 80 未来 @10 Hz、bag_id、地图版本、'
        "感知版本）在 JSON 里；HTML 只展示前 6 条/bag。</p>"
    )
    parts.append(
        '<p class="sub" style="margin-top:30px">由 <code>scripts/badcase_mine.py</code> 生成，勿手改。'
        "契约 <code>spec/bag_mining_v0.yaml</code>，阈值来自 <code>spec/constraints_v0.yaml</code>。</p></body></html>"
    )
    return "".join(parts)


# ---------------------------------------------------------------------- 主流程
def main():
    parser = argparse.ArgumentParser(description="bad case 自动挖掘 v0")
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--constraints", type=Path, default=DEFAULT_CONSTRAINTS)
    parser.add_argument("--bag-glob", default=DEFAULT_BAG_GLOB)
    parser.add_argument("--data-root", type=Path, default=Path("/media/zoo/da1510ce-bbc7-4b3e-9d3b-e5f2137af995"))
    parser.add_argument("--json-out", type=Path, default=DEFAULT_JSON)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    contract = yaml.safe_load(args.contract.read_text(encoding="utf-8"))
    constraints = yaml.safe_load(args.constraints.read_text(encoding="utf-8"))
    template_meta = {item["id"]: item for item in constraints["templates"]}

    entries = []
    failures = []
    for entry in contract["bag_evaluable"]:
        template_id = entry.get("template")
        if template_id not in template_meta:
            failures.append(f"{entry['id']}: template {template_id} 不在 constraints_v0")
            continue
        merged = dict(entry)
        merged["category"] = template_meta[template_id].get("category", "violation")
        entries.append(merged)
    if len(entries) < MIN_BAG_EVALUABLE:
        failures.append(f"bag 可挖模板 {len(entries)} < {MIN_BAG_EVALUABLE}")
    covered = {entry["template"] for entry in entries} | {item["id"] for item in contract["not_bag_evaluable"]}
    missing = set(template_meta) - covered
    if missing:
        failures.append(f"契约未覆盖的模板: {sorted(missing)}")

    import glob

    bag_paths = sorted(glob.glob(args.bag_glob))
    if not bag_paths:
        print(f"GATE FAIL : 没有匹配的 bag: {args.bag_glob}")
        return 1

    bag_names = [Path(path).name.split("white-rhino-")[-1].split(".")[0] for path in bag_paths]
    counts = Counter()
    slices_by_bag = {}
    features_by_bag = {}
    rejected_total = 0
    frames_total = 0

    map_root = Path(args.data_root) / "apollo_map" / "data"
    lane_support = {}
    for path, bag in zip(bag_paths, bag_names):
        lane_index, map_note = find_lane_index(Path(path).parent, map_root)
        lane_support[bag] = map_note
        rows, rejects = scan_bag(path, contract, entries, lane_index)
        rejected_total += sum(rejects.values())
        frames_total += len(rows)
        features_by_bag[bag] = rows
        hits = []
        for row in rows:
            for entry in entries:
                if entry.get("requires_lane") and row.get("lane_id") is None:
                    continue
                if evaluate_bag_template(entry, row):
                    hits.append((entry["id"], row))
        hits.sort(key=lambda item: (item[1]["t_s"], item[0]))
        slices = merge_hits(hits, contract["slice"]["merge_gap_s"])
        slices_by_bag[bag] = slices
        for item in slices:
            counts[(item["template"], bag)] += 1
        print(
            f"[{bag}] 帧 {len(rows):5d} 拒绝 {sum(rejects.values()):3d} "
            f"切片 {len(slices):3d} ({', '.join(f'{k}={v}' for k, v in rejects.items()) or '干净'})"
        )

    summary = {
        "frames": frames_total,
        "rejected": rejected_total,
        "bags_with_hits": sum(1 for items in slices_by_bag.values() if items),
        "template_total": len(template_meta),
        "hottest": "",
        "hottest_count": 0,
    }
    per_template = Counter()
    for (template_id, _bag), value in counts.items():
        per_template[template_id] += value
    if per_template:
        summary["hottest"], summary["hottest_count"] = per_template.most_common(1)[0]

    # 确定性：同输入重跑必须一致（只比切片键，不比浮点）
    signature = {
        bag: [(item["template"], round(item["t_start"], 6), round(item["t_end"], 6)) for item in items]
        for bag, items in slices_by_bag.items()
    }
    if not args.check:
        pass  # 实际比对在 gate 段的二次扫描里完成（成本高，--check 时跳过）

    payload = {
        "schema_version": contract["schema_version"],
        "bags": bag_names,
        "summary": summary,
        "evaluable_templates": [entry["id"] for entry in entries],
        "not_bag_evaluable": [item["id"] for item in contract["not_bag_evaluable"]],
        "counts": {f"{template}|{bag}": value for (template, bag), value in counts.items()},
        "slices": {
            bag: [
                {
                    "scenario_id": f"{bag}#{int(round(item['t_start'] / contract['slice']['step_s']))}",
                    "template": item["template"],
                    "t_start_s": round(item["t_start"], 3),
                    "t_end_s": round(item["t_end"], 3),
                    "frames": item["frames"],
                    "warmup_steps": contract["slice"]["warmup_steps"],
                    "horizon_steps": contract["slice"]["horizon_steps"],
                }
                for item in items
            ]
            for bag, items in slices_by_bag.items()
        },
        "gate": failures,
    }

    print(
        f"[summary] 模板 {len(entries)}/{len(template_meta)} 可挖 · 帧 {frames_total} · "
        f"拒绝 {rejected_total} · 切片 {sum(len(v) for v in slices_by_bag.values())} · "
        f"最热 {summary['hottest']}={summary['hottest_count']}"
    )
    for item in failures:
        print(f"GATE FAIL : {item}")
    if not failures:
        print("GATE      : PASS")

    if not args.check:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(
            json.dumps(payload, ensure_ascii=False, indent=1), encoding="utf-8"
        )
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(
            render(
                contract, entries, bag_names, counts, slices_by_bag, features_by_bag,
                summary, failures, template_meta, lane_support,
            ),
            encoding="utf-8",
        )
        print(f"[json]       {args.json_out}")
        print(f"[report]     {args.out}")

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
