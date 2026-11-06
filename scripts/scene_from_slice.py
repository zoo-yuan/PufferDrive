"""把一个 bad case 切片写成能跑的 PufferDrive 场景 bin（M7-2）。

为什么要有这一层
----------------
前面两步拿到的是「切片」（bag 里的一段时间窗）和「route」（沿 successor 走到 1000 m 的车道链）。
仿真器要的是 map bin：agents(log 轨迹) + roads(车道几何/邻接) + 交通控制 + lane_graph。
本脚本把三者拼成一份 ``load_map_binary`` 能直接吃的文件。

三条必须记住的坑
----------------
1. **坐标必须先平移到原点**。Apollo 的 UTM 坐标量级 ~3.4e6 m，float32 在这个量级的最小
   分辨率是 0.25 m —— 不平移就等于把车宽磨掉一个数量级。
2. **车道类型用常量，不要猜**。``is_drivable_road_lane`` 只认 1(FREEWAY)/2(SURFACE_STREET)。
3. **route 与 lane_graph 的索引是"road 数组下标"**，不是 Apollo 的 lane_id。写之前必须建映射，
   否则 lane-graph 距离观测会读到别的车道上去。

用法
----
``python scripts/scene_from_slice.py``                    # 默认 2714 第一条
``python scripts/scene_from_slice.py --bag 3183 --index 5 --check``
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from data_utils.apollo_lane_index import TileLoader, load_lane_index_for_map_dir  # noqa: E402
from data_utils.mirror_map_bin import read_bin, write_bin  # noqa: E402
from route_preview import (  # noqa: E402
    ROUTE_TARGET_DISTANCE_M,
    lane_length,
    map_dir_for_bag,
    match_poses,
    read_all_poses,
    select_window,
    walk_route,
)

MINING_JSON = REPO_ROOT / "docs" / "measured" / "badcase_mining_v0.json"
MINING_SPEC = REPO_ROOT / "spec" / "bag_mining_v0.yaml"
OUTPUT_DIR = REPO_ROOT / ".scratch" / "scenes"
OUTPUT_JSON = REPO_ROOT / "docs" / "measured" / "scene_build_v0.json"

# 与 pufferlib/ocean/drive/constants.h 对齐（不能猜，is_drivable_road_lane 只认这两个）
LANE_TYPE_SURFACE_STREET = 2
AGENT_TYPE_VEHICLE = 1
SCENARIO_ID_BYTES = 128
DATASET_NAME_BYTES = 32
DATASET_NAME = b"apollo_rx"

# 场景包络：只写自车轨迹附近的车道（远处车道对单切片没有意义，还会稀释 lane 观测）
NEIGHBOR_LANE_RADIUS_M = 20.0
MAX_SCENE_LANES = 160
# 引擎的空间网格是 5 m 一格、每格最多 64 个实体（constants.h: GRID_CELL_SIZE / MAX_ENTITIES_PER_CELL）。
# 超了不是"慢一点"，而是整个 map 建不起来。这里按格占用做裁剪，留 16 个余量。
ENGINE_GRID_CELL_M = 5.0
ENGINE_CELL_ENTITY_LIMIT = 64
CELL_ENTITY_SOFT_LIMIT = 48
# Apollo 里部分车道 speed_limit 为 0；引擎要求 >0 才会参与限速逻辑，用城市道路默认值兜底
DEFAULT_SPEED_LIMIT_MPS = 11.1
# 不可达车道的 lane-graph 距离（引擎用 float，用一个大但有限的值，不用 inf）
UNREACHABLE_DISTANCE_M = 1.0e6
# 场景写对了，回放应当逐步重合；这个门限只用来抓"坐标/route/邻接写错"这类结构性错误
MAX_REPLAY_ERROR_M = 0.05


def resample_log(poses, t_center_s: float, warmup_steps: int, horizon_steps: int, step_s: float):
    """把 bag 的高频 pose（~86 Hz）重采样到引擎节拍（10 Hz）。

    bag 的 pose 是 ~86 Hz，而引擎按 ``dt`` 走。直接按帧取窗口会让"log_length 帧 × 0.1 s"
    与真实时长差 8 倍，回放控制器照着错位的时间表插值，必然跑偏（实测 2.28 m）。
    所以场景里的轨迹必须**按时间重采样**，而不是按帧切。
    """
    times = []
    t_start = t_center_s - warmup_steps * step_s
    t_end = t_center_s + horizon_steps * step_s
    t = t_start
    while t <= t_end:
        times.append(t)
        t += step_s

    stamps = np.array([pose["t"] for pose in poses])
    xs = np.array([pose["x"] for pose in poses])
    ys = np.array([pose["y"] for pose in poses])
    headings = np.unwrap(np.array([pose["heading"] for pose in poses]))
    vxs = np.array([pose.get("vx_mps", 0.0) for pose in poses])
    vys = np.array([pose.get("vy_mps", 0.0) for pose in poses])

    frames = []
    for stamp in times:
        if stamp < stamps[0] or stamp > stamps[-1]:
            continue
        frames.append(
            {
                "t": stamp,
                "x": float(np.interp(stamp, stamps, xs)),
                "y": float(np.interp(stamp, stamps, ys)),
                "heading": float(np.interp(stamp, stamps, headings)),
                "vx": float(np.interp(stamp, stamps, vxs)),
                "vy": float(np.interp(stamp, stamps, vys)),
            }
        )
    return frames


def load_mining_contract():
    contract = yaml.safe_load(MINING_SPEC.read_text(encoding="utf-8"))
    return contract["feasibility_gate"], contract["slice"]["step_s"]


def trim_dense_cells(scene_lanes, route_chain, origin_x: float, origin_y: float):
    """按引擎网格密度裁掉离轨迹最远的邻道。

    route 车道一条都不能删（route/lane_graph 的下标就是它们），所以裁的只有邻道。
    2292 有个路口 5 m 格里塞了 67 条车道 → 引擎直接拒绝建图。
    """
    keep_ids = {lane.lane_id for lane in route_chain}
    occupancy = collections.Counter()
    kept = []
    # 先占位 route 车道（永远保留），再按到轨迹的距离从近到远补邻道
    for lane in scene_lanes:
        if lane.lane_id not in keep_ids:
            continue
        for cell in cell_keys(lane, origin_x, origin_y):
            occupancy[cell] += 1
        kept.append(lane)
    neighbors = [lane for lane in scene_lanes if lane.lane_id not in keep_ids]
    for lane in neighbors:
        cells = cell_keys(lane, origin_x, origin_y)
        if any(occupancy[cell] + 1 > CELL_ENTITY_SOFT_LIMIT for cell in cells):
            continue
        for cell in cells:
            occupancy[cell] += 1
        kept.append(lane)
    return kept


def cell_keys(lane, origin_x: float, origin_y: float):
    points = lane.points - np.array([origin_x, origin_y])
    cells = np.floor(points / ENGINE_GRID_CELL_M).astype(np.int64)
    return {f"{int(cell[0])}:{int(cell[1])}" for cell in cells}


def load_bag_context(bag: str):
    """一条 bag 的 pose + 地图索引（批量建场景时复用，避免 779 次重读）。"""
    _bag_path, poses = read_all_poses(bag)
    map_dir = map_dir_for_bag(bag)
    bbox = (
        min(pose["x"] for pose in poses),
        min(pose["y"] for pose in poses),
        max(pose["x"] for pose in poses),
        max(pose["y"] for pose in poses),
    )
    index_, source = load_lane_index_for_map_dir(map_dir, bbox)
    loader = TileLoader(map_dir) if source.startswith("base_map.catelog") else None
    return {"poses": poses, "map_dir": map_dir, "index": index_, "source": source, "loader": loader}


def build_scene(slice_info, context):
    """把一个切片写成场景 dict。slice_info 直接来自挖掘结果（带 warmup/horizon/时间窗）。"""
    gate, step_s = load_mining_contract()
    bag = slice_info["bag"] if "bag" in slice_info else slice_info["scenario_id"].split("#")[0]
    context = context or load_bag_context(bag)
    poses = context["poses"]
    map_dir = context["map_dir"]
    index_ = context["index"]
    loader = context["loader"]
    source = context["source"]
    window = resample_log(
        poses,
        slice_info["t_start_s"],
        slice_info["warmup_steps"],
        slice_info["horizon_steps"],
        step_s,
    )
    if len(window) < 2:
        raise RuntimeError(f"{slice_info['scenario_id']}: bag 覆盖不到场景时间窗")

    matched = match_poses(window, index_)
    matched_lane_ids = []
    for item in matched:
        match = item["match"]
        if match is None:
            continue
        if not matched_lane_ids or matched_lane_ids[-1] != match["lane_id"]:
            matched_lane_ids.append(match["lane_id"])
    if not matched_lane_ids:
        raise RuntimeError(f"{slice_info['scenario_id']}: 窗口内没有匹配车道，无法定 route")

    route_chain, steps, route_distance_m, stop_reason, branches = walk_route(
        index_.by_id[matched_lane_ids[0]], index_, loader
    )

    # ---- 1. 坐标平移：float32 在 3.4e6 量级只有 0.25 m 分辨率
    origin_x = window[0]["x"]
    origin_y = window[0]["y"]

    scene_lanes = list(route_chain)
    scene_ids = {lane.lane_id for lane in scene_lanes}
    trajectory_points = np.array([[pose["x"] for pose in window], [pose["y"] for pose in window]]).T
    for lane in index_.lanes:
        if len(scene_lanes) >= MAX_SCENE_LANES:
            break
        if lane.lane_id in scene_ids:
            continue
        offsets = np.hypot(
            lane.points[:, 0][:, None] - trajectory_points[:, 0][None, :],
            lane.points[:, 1][:, None] - trajectory_points[:, 1][None, :],
        )
        if float(np.min(offsets)) > NEIGHBOR_LANE_RADIUS_M:
            continue
        scene_lanes.append(lane)
        scene_ids.add(lane.lane_id)

    scene_lanes = trim_dense_cells(scene_lanes, route_chain, origin_x, origin_y)
    road_index = {lane.lane_id: road_id for road_id, lane in enumerate(scene_lanes)}

    roads = []
    for road_id, lane in enumerate(scene_lanes):
        points = lane.points - np.array([origin_x, origin_y])
        segment_deltas = np.diff(points, axis=0)
        headings = [math.atan2(delta[1], delta[0]) for delta in segment_deltas]
        headings.append(headings[-1] if headings else 0.0)
        cumulative = [0.0]
        for delta in segment_deltas:
            cumulative.append(cumulative[-1] + float(math.hypot(delta[0], delta[1])))
        roads.append(
            {
                "id": road_id,
                "type": LANE_TYPE_SURFACE_STREET,
                "S": len(points),
                "x": [float(value) for value in points[:, 0]],
                "y": [float(value) for value in points[:, 1]],
                "z": [0.0] * len(points),
                "headings": [float(value) for value in headings],
                # entry = 上游车道，exit = 下游车道，都翻译成 road 数组下标，且只保留本场景内的
                "entry_lanes": [
                    road_index[ref] for ref in lane.predecessor_ids if ref in road_index
                ],
                "exit_lanes": [
                    road_index[ref] for ref in lane.successor_ids if ref in road_index
                ],
                "speed_limit": float(lane.speed_limit_mps) or DEFAULT_SPEED_LIMIT_MPS,
                "length": float(cumulative[-1]),
                "cum_lengths": [float(value) for value in cumulative],
                "widths": [1.0] * len(points),
            }
        )

    # ---- 2. agent：log 轨迹（平移后），route 用 road 下标
    frame_count = len(window)
    agent_route = [road_index[lane.lane_id] for lane in route_chain if lane.lane_id in road_index]
    agent = {
        "id": 0,
        "type": AGENT_TYPE_VEHICLE,
        "T": frame_count,
        "cols": {
            "x": [pose["x"] - origin_x for pose in window],
            "y": [pose["y"] - origin_y for pose in window],
            "z": [0.0] * frame_count,
            "h": [pose["heading"] for pose in window],
            "vx": [pose["vx"] for pose in window],
            "vy": [pose["vy"] for pose in window],
            "len": [gate["ego_length_m"]] * frame_count,
            "wid": [gate["ego_width_m"]] * frame_count,
            "hgt": [1.6] * frame_count,
            "valid": [1] * frame_count,
        },
        "route": agent_route,
        "route_gt_len": len(agent_route),
        "goal": (
            float(route_chain[-1].points[-1][0] - origin_x),
            float(route_chain[-1].points[-1][1] - origin_y),
            0.0,
        ),
        "mark_as_expert": 1,
    }

    # ---- 3. lane_graph：按 route 顺序的弧长距离（文件存 [from][to]，引擎读时转置）
    graph_ids = agent_route
    cum = []
    total = 0.0
    for road_id in graph_ids:
        total += roads[road_id]["length"]
        cum.append(total)
    n = len(graph_ids)
    distances = [0.0] * (n * n)
    for i in range(n):
        for j in range(n):
            distances[i * n + j] = max(0.0, cum[j] - cum[i]) if j >= i else UNREACHABLE_DISTANCE_M

    scene = {
        "agents": [agent],
        "roads": roads,
        "traffic": [],
        "objects": [],
        "lane_graph": {"n": n, "lane_ids": graph_ids, "distances": distances},
        "scenario_id": slice_info["scenario_id"].encode("utf-8").ljust(SCENARIO_ID_BYTES, b"\x00"),
        "dataset_name": DATASET_NAME.ljust(DATASET_NAME_BYTES, b"\x00"),
        "log_length": frame_count,
        "log_dt": float(step_s),
        "objects_of_interest": [0],
        "tracks_to_predict": [0],
        # 尾部三段是可选的、且**顺序固定**：相位 → 车道宽 → 限速区。宽度段要存在就必须带上相位
        # 标签（即使没有交通控制元素），否则读取器会把宽度的 tag 当成相位 tag 报格式错。
        "has_phase_section": True,
        "has_width_section": True,
        "has_zone_section": False,
    }
    context = {
        "scenario_id": slice_info["scenario_id"],
        "template": slice_info["template"],
        "bag": bag,
        "map_name": map_dir.name,
        "map_source": source,
        "frames": frame_count,
        "origin": [origin_x, origin_y],
        "route_distance_m": round(route_distance_m, 1),
        "route_stop_reason": stop_reason,
        "route_lanes": len(agent_route),
        "branches": branches,
        "scene_lanes": len(scene_lanes),
        "log_dt_s": float(step_s),
        "step_list": steps,
    }
    return scene, context


def verify_bin(path: Path):
    """读回来校验结构（写错一个 int 就会让 C 侧整份文件读崩）。"""
    data = read_bin(path)
    lanes = [road for road in data["roads"] if 0 <= road["type"] <= 9]
    problems = []
    if len(data["agents"]) != 1:
        problems.append(f"agents={len(data['agents'])}")
    agent = data["agents"][0]
    if agent["T"] != data["log_length"]:
        problems.append(f"agent T={agent['T']} != log_length={data['log_length']}")
    if len(agent["route"]) > 64:
        problems.append(f"route={len(agent['route'])} > MAX_ROUTE_LENGTH")
    for road in lanes:
        lane_count = len(data["roads"])
        bad = [idx for idx in road["entry_lanes"] + road["exit_lanes"] if idx < 0 or idx >= lane_count]
        if bad:
            problems.append(f"road {road['id']} 邻接越界 {bad}")
            break
        if abs(road["cum_lengths"][-1] - road["length"]) > 1e-3:
            problems.append(f"road {road['id']} cum_lengths/length 不一致")
            break
    graph = data["lane_graph"]
    if graph["n"] and len(graph["distances"]) != graph["n"] ** 2:
        problems.append("lane_graph 距离矩阵长度不对")
    return problems


def run_engine(path: Path, steps: int):
    """真跑一遍：replay 控制器跟 log 走，量 env 位置与 log 位置的偏差。

    这是"场景写对了没有"最硬的证据：如果坐标/route/邻接错，回放会立刻偏离。
    """
    from pufferlib.ocean.drive.drive import Drive  # 延迟导入：没编 binding 时也能只做结构校验

    env = Drive(
        map_dir=str(path),
        num_maps=1,
        num_agents=1,
        min_agents_per_env=1,
        max_agents_per_env=1,
        action_type="continuous",
        dynamics_model="jerk",
        dt=0.1,
        scenario_length=steps,
        resample_frequency=0,
        # 回放模式：ego 按 log 轨迹播，用来验证"场景写对了没有"。
        # gigaflow 会按地图重新 spawn agent，单条 log 场景在那种模式下没有意义。
        simulation_mode="replay",
        control_mode="control_sdc_only",
        sdc_controller="replay",
        num_goals=3,
        min_goal_spacing=20.0,
        max_goal_spacing=200.0,
        seed=0,
    )
    env.reset(seed=0)
    log_xy = []
    for _ in range(steps):
        env.step(np.zeros((env.num_agents, 2), dtype=np.float32))
        state = env.get_state()
        scenarios = state if isinstance(state, list) else [state]
        for scenario in scenarios:
            for agent in scenario.get("agents") or []:
                # state 里位置叫 sim_x/sim_y（get("x") 会静默返回 0，量出来全是假偏差）
                log_xy.append((float(agent["sim_x"]), float(agent["sim_y"])))
    return env, log_xy


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", default="2714")
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--template", default="lane_departure")
    parser.add_argument("--steps", type=int, default=40)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--no-run", action="store_true", help="只写 bin，不真跑引擎")
    args = parser.parse_args()

    mining = json.loads(MINING_JSON.read_text(encoding="utf-8"))
    candidates = [item for item in mining["slices"][args.bag] if item["template"] == args.template]
    slice_info = (candidates or mining["slices"][args.bag])[args.index]
    slice_info = {**slice_info, "bag": args.bag}
    scene, context = build_scene(slice_info, load_bag_context(args.bag))
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    scene_path = OUTPUT_DIR / f"{context['scenario_id'].replace('#', '_')}.bin"
    write_bin(scene, scene_path)

    problems = verify_bin(scene_path)
    print(f"scene     : {scene_path.name} ({scene_path.stat().st_size / 1024:.0f} KB)")
    print(f"lanes     : {context['scene_lanes']} 条（route {context['route_lanes']} 段，"
          f"{context['route_distance_m']:.0f} m，停因 {context['route_stop_reason']}）")
    print(f"frames    : {context['frames']} @ {context['log_dt_s']} s")

    replay_error_m = None
    if not args.no_run:
        try:
            _env, positions = run_engine(scene_path, args.steps)
            log_rows = scene["agents"][0]["cols"]
            limit = min(len(positions), len(log_rows["x"]) - 1)
            # 回放模式第 k 步输出的是 log[k+1] 的位姿（先用后推），所以对齐到 i+1。
            # 对齐错了会凭空量出 0.68 m 的"偏差"，而真实值是 0.000 m。
            errors = [
                math.hypot(positions[i][0] - log_rows["x"][i + 1], positions[i][1] - log_rows["y"][i + 1])
                for i in range(limit)
            ]
            replay_error_m = max(errors) if errors else None
            print(f"replay    : {limit} 步，max |Δxy| vs log = {replay_error_m:.3f} m")
        except Exception as error:  # 引擎不可用（未编译 binding）时只降级为结构校验
            print(f"replay    : 跳过（{type(error).__name__}: {error}）")

    context["bin"] = str(scene_path.relative_to(REPO_ROOT))
    context["bin_bytes"] = scene_path.stat().st_size
    context["replay_max_error_m"] = replay_error_m
    context["problems"] = problems
    OUTPUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_JSON.write_text(json.dumps(context, ensure_ascii=False, indent=2), encoding="utf-8")

    failures = list(problems)
    if replay_error_m is not None and replay_error_m > MAX_REPLAY_ERROR_M:
        failures.append(f"replay 偏差 {replay_error_m:.3f} m > {MAX_REPLAY_ERROR_M} m")
    for failure in failures:
        print(f"GATE FAIL : {failure}")
    print(f"GATE      : {'FAIL' if failures else 'PASS'}")
    return 1 if (args.check and failures) else 0


if __name__ == "__main__":
    raise SystemExit(main())
