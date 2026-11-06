"""把一个 bad case 切片的地图拓扑串成 route，并算出 goal —— M7-1 的可视化验证。

为什么要单独跑一遍
------------------
切片只说明"车在哪、错在哪"，不说明"车想去哪"。PufferDrive 的场景 bin 里每个 agent 需要
``route``（车道索引序列）与 ``goals``；``drive.h`` 的 ``ROUTE_TARGET_DISTANCE = 1000 m``、
``MAX_ROUTE_LENGTH = 64`` 就是上限。所以场景化第一步是：把自车轨迹贴到车道上，再沿
``successor_id`` 往前走到 1000 m，得到一个可写进 bin 的 route，goal 取 route 末端。

本脚本只做一件可查验的事：给一条切片画出
  ① 附近车道 ② 自车匹配到的车道段 ③ 向前延伸的 route（含分叉选择） ④ goal 点
并报出链路统计（分支数 / 死胡同 / 转弯数 / route 总长），供 reviewer 判断口径对不对。

用法
----
``python scripts/route_preview.py``                 # 默认取 rx-3183 第一条 lane_departure
``python scripts/route_preview.py --bag 3139 --index 2``
"""

from __future__ import annotations

import argparse
import html
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from data_utils.apollo_lane_index import (  # noqa: E402
    TileLoader,
    extend_index,
    load_lane_index_for_map_dir,
)
from data_utils.apollo_record import decode_localization_pose, iter_messages  # noqa: E402

DATA_ROOT = Path("/media/zoo/da1510ce-bbc7-4b3e-9d3b-e5f2137af995")
MAP_ROOT = DATA_ROOT / "apollo_map" / "data"
MINING_JSON = REPO_ROOT / "docs" / "measured" / "badcase_mining_v0.json"
OUTPUT_HTML = REPO_ROOT / "docs" / "html" / "route_preview.html"
OUTPUT_JSON = REPO_ROOT / "docs" / "measured" / "route_preview.json"

# 与引擎常量对齐（pufferlib/ocean/drive/constants.h）
ROUTE_TARGET_DISTANCE_M = 1000.0
MAX_ROUTE_LENGTH = 64
GOAL_LANE_SNAP_MAX_DIST_M = 6.0
# 分叉选择：下一个车道起点必须接得上前一个车道终点，超过这个距离就当作不连续（标断链，不硬接）。
# 20 m 的依据：2688 的链在 307 m 处断掉，实测下一段同向车道起点距上段终点 12.7 m、航向差 1.4°
# —— 那是路口/斑马线的空洞，不是真的断头路。
ROUTE_LINK_MAX_GAP_M = 20.0
ROUTE_LINK_HEADING_MAX_RAD = math.radians(60.0)
POSE_SAMPLE_RADIUS_M = 120.0
LANE_DRAW_RADIUS_M = 160.0
GOAL_MARKER_RADIUS_M = 8.0

# Apollo map.proto 的 LaneTurn **从 1 开始**（NO_TURN=1, LEFT=2, RIGHT=3, U_TURN=4）。
# 按 0 起读会把每条车道都算成转弯 —— 实测 25 段 route 全被标成 LEFT_TURN 才发现。
TURN_NAMES = {
    1: "NO_TURN",
    2: "LEFT_TURN",
    3: "RIGHT_TURN",
    4: "U_TURN",
}
NO_TURN_VALUES = frozenset({0, 1})


def pick_slice(bag: str, index: int, template: str):
    data = json.loads(MINING_JSON.read_text(encoding="utf-8"))
    slices = data["slices"][bag]
    candidates = [item for item in slices if item["template"] == template] or slices
    return candidates[index]


def map_dir_for_bag(bag: str) -> Path:
    record_dir = DATA_ROOT / f"rx-{bag}"
    for link in record_dir.iterdir():
        if link.is_symlink() and (MAP_ROOT / link.name).is_dir():
            return MAP_ROOT / link.name
    raise FileNotFoundError(f"rx-{bag}: no map symlink")


def read_all_poses(bag: str):
    """整条 bag 的 pose（86 Hz 左右，20 s 约 1700 帧，内存无压力）。"""
    record_dir = DATA_ROOT / f"rx-{bag}"
    bag_path = next(record_dir.glob("*.bag"))
    poses = []
    for _channel, timestamp_ns, payload in iter_messages(
        str(bag_path), channels={"/apollo/localization/pose"}
    ):
        pose = decode_localization_pose(payload)
        poses.append(
            {
                "t": timestamp_ns * 1e-9,   # iter_messages 给的是纳秒
                "x": pose["x_m"],
                "y": pose["y_m"],
                "heading": pose["heading_rad"],
            }
        )
    return bag_path, poses


def select_window(poses, t_center_s: float, warmup_steps: int, horizon_steps: int):
    """按**帧索引**取窗口：挖掘器也是这么定义的（warmup 帧在前、horizon 帧在后）。

    不用时间算 dt：有些切片只有 1 帧，``t_end - t_start == 0`` 会把窗口退化成 0 秒。
    """
    anchor_idx = next(
        (i for i, pose in enumerate(poses) if pose["t"] >= t_center_s),
        len(poses) - 1,
    )
    start_idx = max(0, anchor_idx - warmup_steps)
    end_idx = min(len(poses), anchor_idx + max(1, horizon_steps))
    return poses[start_idx:end_idx], anchor_idx - start_idx


def polyline_length(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    return float(np.sum(np.hypot(*np.diff(points, axis=0).T)))


def lane_heading_at_end(lane, at_end: bool = True) -> float:
    points = lane.points
    pair = points[-2:] if at_end else points[:2]
    delta = pair[-1] - pair[0]
    return math.atan2(delta[1], delta[0])


def lane_length(lane) -> float:
    return lane.length_m if lane.length_m > 1.0 else polyline_length(lane.points)


def _continuity(lane, candidate):
    """上一个车道终点 → 候选车道起点的 (间距, 航向差)。超门限返回 None。"""
    gap = float(np.hypot(*(candidate.points[0] - lane.points[-1])))
    end_heading = lane_heading_at_end(lane, at_end=True)
    start_heading = lane_heading_at_end(candidate, at_end=False)
    heading_diff = abs(
        math.atan2(math.sin(start_heading - end_heading), math.cos(start_heading - end_heading))
    )
    if gap > ROUTE_LINK_MAX_GAP_M or heading_diff > ROUTE_LINK_HEADING_MAX_RAD:
        return None
    return gap, heading_diff


def choose_successor(lane, index, visited_ids, loader=None):
    """先走拓扑（successor_id），拓扑沉默时按几何连续性接——两条路都记在 ``why`` 里。

    Apollo 的 successor 不覆盖全部连接：2688 那条 bag 的车道链在 339 m 处后继为空，
    但空间上明显接着下一条车道。纯拓扑走法会把这 71 条切片全判成"断链"，而仿真器要的
    只是"一条能开下去的连续车道序列"。所以几何兜底必须显式存在，且必须可审计。
    """
    refs = list(lane.successor_ids)
    if refs and loader is not None:
        # 分块地图：successor 可能落在没加载的 tile 里
        extend_index(index, loader, refs)
    candidates = [index.by_id[ref] for ref in refs if ref in index.by_id]
    candidates = [candidate for candidate in candidates if candidate.lane_id not in visited_ids]
    scored = []
    for candidate in candidates:
        continuity = _continuity(lane, candidate)
        if continuity is not None:
            scored.append((continuity[0], continuity[1], candidate))
    if scored:
        scored.sort(key=lambda item: (item[0], item[1]))
        why = "successor" if len(scored) == 1 else f"branch({len(refs)} successors)"
        return scored[0][2], why
    if refs:
        return None, f"successor_unusable({len(refs)})"

    # 几何兜底：起终点接得上、航向一致、尚未访问
    fallback = []
    visited_but_continuous = False
    for candidate in index.lanes:
        continuity = _continuity(lane, candidate)
        if continuity is None:
            continue
        if candidate.lane_id in visited_ids:
            visited_but_continuous = True
            continue
        fallback.append((continuity[0], continuity[1], candidate))
    if not fallback:
        # 回环：能接上的候选都走过了 —— 2688 的链在 350 m 处绕回自己（小区环道）
        return None, "loop" if visited_but_continuous else "dead_end"
    fallback.sort(key=lambda item: (item[0], item[1]))
    return fallback[0][2], "geometric(no successor)"


def walk_route(start_lane, index, loader=None, target_distance_m=ROUTE_TARGET_DISTANCE_M):
    chain = [start_lane]
    visited_ids = {start_lane.lane_id}
    distance_m = lane_length(start_lane)
    steps = [
        {
            "lane_id": start_lane.lane_id,
            "length_m": round(lane_length(start_lane), 1),
            "turn": TURN_NAMES.get(start_lane.turn, str(start_lane.turn)),
            "speed_limit_mps": round(start_lane.speed_limit_mps, 1),
            "why": "start",
        }
    ]
    branches = 0
    stop_reason = "target_reached"
    while distance_m < target_distance_m:
        if len(chain) >= MAX_ROUTE_LENGTH:
            stop_reason = "route_length_cap"
            break
        next_lane, why = choose_successor(chain[-1], index, visited_ids, loader)
        if next_lane is None:
            stop_reason = why
            break
        if why.startswith("branch"):
            branches += 1
        chain.append(next_lane)
        visited_ids.add(next_lane.lane_id)
        length_m = lane_length(next_lane)
        distance_m += length_m
        steps.append(
            {
                "lane_id": next_lane.lane_id,
                "length_m": round(length_m, 1),
                "turn": TURN_NAMES.get(next_lane.turn, str(next_lane.turn)),
                "speed_limit_mps": round(next_lane.speed_limit_mps, 1),
                "why": why,
            }
        )
    return chain, steps, distance_m, stop_reason, branches


def match_poses(poses, index):
    matched = []
    for pose in poses:
        match = index.match(pose["x"], pose["y"], pose["heading"])
        matched.append({"pose": pose, "match": match})
    return matched


def build_geometry(matched, route_chain, matched_lanes, goal_point):
    anchors = [item["pose"] for item in matched] + [{"x": point[0], "y": point[1]} for point in
                                                  (lane.points[0] for lane in route_chain)]
    xs = [anchor["x"] for anchor in anchors]
    ys = [anchor["y"] for anchor in anchors]
    center_x = (min(xs) + max(xs)) / 2.0
    center_y = (min(ys) + max(ys)) / 2.0
    return center_x, center_y


def svg_polyline(points, scale, center_x, center_y, width, height, stroke, dash="", opacity="1"):
    coords = " ".join(
        f"{width / 2 + (x - center_x) * scale:.1f},{height / 2 - (y - center_y) * scale:.1f}"
        for x, y in points
    )
    dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
    return (
        f'<polyline points="{coords}" fill="none" stroke="{stroke}" stroke-width="2"'
        f' opacity="{opacity}"{dash_attr} stroke-linecap="round"/>'
    )


def render_svg(index, matched, route_chain, matched_lane_ids, goal_point):
    ego = [(item["pose"]["x"], item["pose"]["y"]) for item in matched]
    route_points = [point for lane in route_chain for point in lane.points]
    all_x = [point[0] for point in ego + route_points]
    all_y = [point[1] for point in ego + route_points]
    center_x = (min(all_x) + max(all_x)) / 2.0
    center_y = (min(all_y) + max(all_y)) / 2.0
    span_x = max(1.0, max(all_x) - min(all_x))
    span_y = max(1.0, max(all_y) - min(all_y))
    width, height = 900, 700
    # 等比缩放，留 8% 边距；画布要装下整条 route，不然 1000 m 的路线会跑到画面外
    scale = min(width * 0.92 / span_x, height * 0.92 / span_y)
    radius_m = max(span_x, span_y) / 2.0

    parts = [
        f'<svg viewBox="0 0 {width} {height}" width="100%" style="background:#fbfbfd;border:1px solid #e3e3e8;border-radius:8px">'
    ]
    drawn = 0
    margin_m = 0.15 * max(span_x, span_y)
    for lane in index.lanes:
        offsets = np.hypot(lane.points[:, 0] - center_x, lane.points[:, 1] - center_y)
        if float(np.min(offsets)) > radius_m + margin_m:
            continue
        drawn += 1
        parts.append(svg_polyline(lane.points, scale, center_x, center_y, width, height, "#c9ccd4"))
    for lane in route_chain:
        color = "#0969da" if lane.lane_id not in matched_lane_ids else "#bf8700"
        parts.append(svg_polyline(lane.points, scale, center_x, center_y, width, height, color))
    if goal_point is not None:
        gx = width / 2 + (goal_point[0] - center_x) * scale
        gy = height / 2 - (goal_point[1] - center_y) * scale
        parts.append(f'<circle cx="{gx:.1f}" cy="{gy:.1f}" r="{GOAL_MARKER_RADIUS_M}" fill="#cf222e"/>')
        parts.append(f'<text x="{gx + GOAL_MARKER_RADIUS_M + 4:.1f}" y="{gy:.1f}" font-size="11" fill="#cf222e">goal</text>')
    parts.append(svg_polyline(ego, scale, center_x, center_y, width, height, "#1a7f37", opacity="0.9"))
    parts.append(
        f'<text x="12" y="20" font-size="11" fill="#57606a">半径 {radius_m:.0f} m · 灰=附近车道 '
        f'{drawn} 条 · 蓝=route 延伸 · 橙=自车匹配车道 · 绿=自车轨迹 · 红=goal</text>'
    )
    parts.append("</svg>")
    return "".join(parts)


def build_html(context) -> str:
    chain_rows = "".join(
        f"<tr><td>{i}</td><td class='mono'>{html.escape(step['lane_id'])}</td>"
        f"<td class='c'>{step['length_m']:.1f}</td><td class='c'>{html.escape(step['turn'])}</td>"
        f"<td class='c'>{step['speed_limit_mps']:.1f}</td><td class='note'>{html.escape(step['why'])}</td></tr>"
        for i, step in enumerate(context["steps"])
    )
    matched_rows = "".join(
        f"<tr><td class='mono'>{html.escape(lane_id)}</td></tr>" for lane_id in context["matched_lane_ids"]
    )
    return f"""<!doctype html><html lang="zh"><head><meta charset="utf-8">
<title>route 推导预览 · {html.escape(context['scenario_id'])}</title>
<style>
body{{font-family:-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,"PingFang SC",sans-serif;
max-width:1080px;margin:0 auto;padding:32px 24px 64px;color:#1f2328;line-height:1.6}}
h1{{font-size:24px;margin:0 0 4px}} h2{{font-size:17px;margin:30px 0 10px;padding-bottom:6px;border-bottom:1px solid #e3e3e8}}
.sub{{color:#57606a;font-size:13px;margin:0 0 18px}}
.cards{{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;margin:18px 0}}
.card{{border:1px solid #e3e3e8;border-radius:8px;padding:12px 14px;background:#fbfbfd}}
.card .k{{font-size:12px;color:#57606a}} .card .v{{font-size:22px;font-weight:600;margin-top:2px}}
.card .n{{font-size:11px;color:#8c959f;margin-top:2px}}
table{{width:100%;border-collapse:collapse;font-size:13px}}
th{{text-align:left;font-size:12px;color:#57606a;border-bottom:1px solid #d0d7de;padding:6px 8px;font-weight:600}}
td{{border-bottom:1px solid #eaeef2;padding:5px 8px;vertical-align:top}}
.c{{text-align:center}} .mono{{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:12px}}
.note{{color:#57606a;font-size:12px}}
.banner{{border-left:3px solid #1a7f37;background:#f0fbf2;padding:9px 13px;border-radius:0 6px 6px 0;font-size:13px;margin:16px 0}}
.warn{{border-left-color:#bf8700;background:#fff8e6}}
</style></head><body>
<h1>route 推导预览</h1>
<p class="sub">切片 <code>{html.escape(context['scenario_id'])}</code> · 模板 {html.escape(context['template'])}
 · bag rx-{html.escape(context['bag'])} · 地图 <code>{html.escape(context['map_name'])}</code></p>
<div class="cards">
<div class="card"><div class="k">route 长度</div><div class="v">{context['route_distance_m']:.0f} m</div>
<div class="n">目标 {ROUTE_TARGET_DISTANCE_M:.0f} m · 车道 {len(context['steps'])}/{MAX_ROUTE_LENGTH} 段</div></div>
<div class="card"><div class="k">停止原因</div><div class="v">{html.escape(context['stop_reason'])}</div>
<div class="n">分支 {context['branch_count']} · 转弯 {context['turn_count']}</div></div>
<div class="card"><div class="k">自车匹配车道</div><div class="v">{len(context['matched_lane_ids'])}</div>
<div class="n">切片窗口 {len(context['matched'])} 帧</div></div>
<div class="card"><div class="k">未匹配帧</div><div class="v">{context['unmatched_frames']}</div>
<div class="n">超过半径或航向不符 → 不出判定</div></div>
</div>
{f'<div class="banner warn">route 在 {context["route_distance_m"]:.0f} m 处停下（{html.escape(context["stop_reason"])}），不足 {ROUTE_TARGET_DISTANCE_M:.0f} m：这条链在拓扑上到此结束，场景化时要么缩短目标距离，要么换 goal_source。</div>' if context['route_distance_m'] < ROUTE_TARGET_DISTANCE_M else f'<div class="banner">route 已走到 {context["route_distance_m"]:.0f} m 且拓扑连续，goal 取末端点 —— 这个口径可以直接写进 PufferDrive 场景 bin 的 route 索引。</div>'}
<h2>1. 几何</h2>
{context['svg']}
<h2>2. route 链路（successor 走法）</h2>
<table><thead><tr><th>#</th><th>lane_id</th><th class="c">长度 m</th><th class="c">turn</th>
<th class="c">限速 m/s</th><th>选择依据</th></tr></thead><tbody>{chain_rows}</tbody></table>
<h2>3. 自车实际经过的车道</h2>
<table><thead><tr><th>lane_id</th></tr></thead><tbody>{matched_rows}</tbody></table>
<p class="note">与 route 前段应逐段一致；不一致说明匹配抖动或 route 起点选错。</p>
</body></html>"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", default="3183")
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--template", default="lane_departure")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    slice_info = pick_slice(args.bag, args.index, args.template)
    all_poses_bag_path, all_poses = read_all_poses(args.bag)
    poses, anchor_offset = select_window(
        all_poses,
        slice_info["t_start_s"],
        int(slice_info["warmup_steps"]),
        int(slice_info["horizon_steps"]),
    )
    if len(poses) < 2:
        print(f"ERROR: {args.bag}: 窗口只有 {len(poses)} 帧", file=sys.stderr)
        return 1

    map_dir = map_dir_for_bag(args.bag)
    bbox = (
        min(pose["x"] for pose in poses),
        min(pose["y"] for pose in poses),
        max(pose["x"] for pose in poses),
        max(pose["y"] for pose in poses),
    )
    index, source = load_lane_index_for_map_dir(map_dir, bbox)
    matched = match_poses(poses, index)
    matched_lane_ids = []
    for item in matched:
        if item["match"] is None:
            continue
        lane_id = item["match"]["lane_id"]
        if not matched_lane_ids or matched_lane_ids[-1] != lane_id:
            matched_lane_ids.append(lane_id)
    if not matched_lane_ids:
        print(f"ERROR: {args.bag}: no matched lane in window", file=sys.stderr)
        return 1

    start_lane = index.by_id[matched_lane_ids[0]]
    loader = TileLoader(map_dir) if source.startswith("base_map.catelog") else None
    route_chain, steps, distance_m, stop_reason, branches = walk_route(start_lane, index, loader)
    goal_point = tuple(float(value) for value in route_chain[-1].points[-1])
    turn_count = sum(1 for lane in route_chain if lane.turn not in NO_TURN_VALUES)

    matched_ordered = [index.by_id[lane_id] for lane_id in matched_lane_ids if lane_id in index.by_id]
    for lane in matched_ordered[1:]:
        if all(lane.lane_id != existing.lane_id for existing in route_chain):
            route_chain.insert(route_chain.index(start_lane) + 1, lane)

    context = {
        "scenario_id": slice_info["scenario_id"],
        "template": slice_info["template"],
        "bag": args.bag,
        "map_name": map_dir.name,
        "map_source": source,
        "matched": matched,
        "unmatched_frames": sum(1 for item in matched if item["match"] is None),
        "matched_lane_ids": matched_lane_ids,
        "steps": steps,
        "route_distance_m": distance_m,
        "stop_reason": stop_reason,
        "branch_count": branches,
        "turn_count": turn_count,
        "goal": {"x": goal_point[0], "y": goal_point[1]},
    }
    context["svg"] = render_svg(index, matched, route_chain, set(matched_lane_ids), goal_point)

    OUTPUT_HTML.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_HTML.write_text(build_html(context), encoding="utf-8")
    OUTPUT_JSON.write_text(
        json.dumps({k: v for k, v in context.items() if k not in ("matched", "svg")},
                   ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(f"scenario  : {context['scenario_id']}")
    print(f"route     : {distance_m:.0f} m / {len(steps)} 段 / 停止原因 {stop_reason} / 分支 {branches}")
    print(f"匹配      : {len(matched_lane_ids)} 条车道 / 未匹配帧 {context['unmatched_frames']}/{len(matched)}")
    print(f"report    : {OUTPUT_HTML}")

    failures = []
    if context["unmatched_frames"] > 0.1 * len(matched):
        failures.append(f"未匹配帧 {context['unmatched_frames']}/{len(matched)} 超过 10%")
    if distance_m < ROUTE_TARGET_DISTANCE_M:
        failures.append(f"route {distance_m:.0f} m < {ROUTE_TARGET_DISTANCE_M:.0f} m（{stop_reason}）")
    if not matched_lane_ids:
        failures.append("没有任何匹配车道")
    for failure in failures:
        print(f"GATE FAIL : {failure}")
    print(f"GATE      : {'FAIL' if failures else 'PASS'}")
    return 1 if (args.check and failures) else 0


if __name__ == "__main__":
    raise SystemExit(main())
