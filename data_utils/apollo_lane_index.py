"""Apollo HD map 车道索引与自车匹配（bag ↔ 地图）。

为什么需要
----------
数据湖里只有 bag，而 12 条违规型约束里有 8 条要"我在哪条车道上"：
车道偏离、车头与车道不一致、超速、跨车道漂移、闯灯、停车标志、路口汇入、斑马线让行。
仿真侧这些值来自引擎的 ``lane_dist`` / ``lane_angle``；bag 侧必须自己算。

同一个几何后面还要用来把切片转成 PufferDrive 场景（route / goal 推导），所以这一层
不是一次性的统计脚本，而是可复用的匹配层。

结构（已用真图核对）
--------------------
``Map.lane(4) → Lane.central_curve(2) → Curve.segment(1) → Segment.line_segment(1)
 → LineSegment.point(1) → Point{x(1), y(2), z(3)}``，Point 是 27 字节的三个 double。

口径
----
- ``lateral_m``：自车中心到最近车道中心线的**有符号横向距离**（左正右负），
  与引擎 ``lane_dist`` 同义但引擎带符号约定需按 ``drive.h:3468`` 核对后统一。
- ``heading_diff_rad``：自车航向与车道切线方向的夹角，``cos`` 化后与引擎
  ``lane_angle``（存 cos）可比。
- 匹配失败（最近点超过 ``match_radius_m``）**不插值、不猜**，返回 None 并计数。
"""

from __future__ import annotations

import json
import math
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from data_utils.apollo_record import _point3d, iter_fields

WIRE_LEN = 2
MATCH_RADIUS_M = 6.0
# 双向道路的对向车道中心线通常只差一个车道宽（~3.5 m）。只按"最近点"匹配会把车贴到
# 对向车道上（实测 p90 航向差 179°），所以候选点还必须与航向一致。
MATCH_MAX_HEADING_DIFF_RAD = math.radians(60.0)
# 横向超过一个车道宽说明贴到了邻道/对向/无车道区域：宁可不给判定，也不伪造车道偏离。
MATCH_MAX_LATERAL_M = 3.0
GRID_CELL_M = 25.0
MIN_POINT_SPACING_M = 0.05


@dataclass
class Lane:
    lane_id: str
    points: np.ndarray          # (M, 2) 世界坐标
    speed_limit_mps: float
    turn: int                   # Apollo LaneTurn 枚举
    lane_type: int
    length_m: float


@dataclass
class LaneIndex:
    lanes: List[Lane]
    cell_points: Dict[Tuple[int, int], np.ndarray] = field(default_factory=dict)
    cell_lane_index: Dict[Tuple[int, int], np.ndarray] = field(default_factory=dict)
    cell_point_index: Dict[Tuple[int, int], np.ndarray] = field(default_factory=dict)
    match_radius_m: float = MATCH_RADIUS_M

    @property
    def lane_count(self) -> int:
        return len(self.lanes)

    def _cells_for(self, x: float, y: float) -> List[Tuple[int, int]]:
        radius = self.match_radius_m
        cells = []
        for cell_x in range(math.floor((x - radius) / GRID_CELL_M), math.floor((x + radius) / GRID_CELL_M) + 1):
            for cell_y in range(math.floor((y - radius) / GRID_CELL_M), math.floor((y + radius) / GRID_CELL_M) + 1):
                cells.append((cell_x, cell_y))
        return cells

    def _candidates(self, x: float, y: float):
        """返回半径内所有 (lane_index, point_index, distance)，按距离升序、去重。"""
        seen = set()
        candidates = []
        for cell in self._cells_for(x, y):
            points = self.cell_points.get(cell)
            if points is None:
                continue
            offsets = np.hypot(points[:, 0] - x, points[:, 1] - y)
            for index in np.nonzero(offsets <= self.match_radius_m)[0]:
                lane_index = int(self.cell_lane_index[cell][index])
                point_index = int(self.cell_point_index[cell][index])
                key = (lane_index, point_index)
                if key in seen:
                    continue
                seen.add(key)
                candidates.append((lane_index, point_index, float(offsets[index])))
        candidates.sort(key=lambda item: item[2])
        return candidates

    def _geometry(self, lane_index: int, point_index: int, x: float, y: float, heading: float):
        lane = self.lanes[lane_index]
        points = lane.points
        low = max(0, point_index - 1)
        high = min(len(points) - 1, point_index + 1)
        tangent = points[high] - points[low]
        if float(np.hypot(tangent[0], tangent[1])) < 1e-6:
            return None
        tangent_heading = math.atan2(tangent[1], tangent[0])
        heading_diff = math.atan2(
            math.sin(heading - tangent_heading), math.cos(heading - tangent_heading)
        )
        to_ego = np.array([x, y]) - points[point_index]
        lateral = -to_ego[0] * math.sin(tangent_heading) + to_ego[1] * math.cos(tangent_heading)
        return lane, float(lateral), heading_diff, tangent_heading

    def match(self, x: float, y: float, heading: float) -> Optional[dict]:
        """最近且**航向一致**的车道点；超出半径或没有同向候选时返回 None（不猜）。"""
        candidates = self._candidates(x, y)
        if not candidates:
            return None

        chosen = None
        fallback = None
        for lane_index, point_index, distance in candidates:
            geometry = self._geometry(lane_index, point_index, x, y, heading)
            if geometry is None:
                continue
            lane, lateral, heading_diff, tangent_heading = geometry
            entry = {
                "lane_index": lane_index,
                "lane_id": lane.lane_id,
                "distance_m": distance,
                "lateral_m": lateral,
                "heading_diff_rad": heading_diff,
                "lane_heading_rad": tangent_heading,
                "speed_limit_mps": lane.speed_limit_mps,
                "turn": lane.turn,
                "lane_type": lane.lane_type,
            }
            if abs(heading_diff) > MATCH_MAX_HEADING_DIFF_RAD:
                if fallback is None:
                    fallback = entry
                continue
            if abs(lateral) > MATCH_MAX_LATERAL_M:
                continue
            chosen = entry
            break
            if fallback is None:
                fallback = entry
        # 宁可不出判定，也不贴到对向车道上
        if chosen is None:
            return None
        return chosen


def _parse_curve_points(lane_bytes: bytes) -> np.ndarray:
    """Lane.central_curve → (M,2) 点列。结构不对时抛错，不返回空数组糊过去。"""
    curve = None
    for field_number, _wire, value in iter_fields(lane_bytes):
        if field_number == 2 and isinstance(value, bytes):
            curve = value
            break
    if curve is None:
        raise ValueError("lane without central_curve")

    points: List[Tuple[float, float]] = []
    for field_number, _wire, segment in iter_fields(curve):
        if field_number != 1 or not isinstance(segment, bytes):
            continue
        for sub_number, _sw, line_segment in iter_fields(segment):
            if sub_number != 1 or not isinstance(line_segment, bytes):
                continue
            for point_number, _pw, point_bytes in iter_fields(line_segment):
                if point_number != 1 or not isinstance(point_bytes, bytes):
                    continue
                point = _point3d(point_bytes)
                points.append((point["x"], point["y"]))
    if len(points) < 2:
        raise ValueError(f"central_curve parsed to {len(points)} points")
    return np.array(points, dtype=np.float64)


def read_scalar_fields(message: bytes) -> Dict[int, float]:
    """把定长数值字段解码成数字（iter_fields 返回的是原始字节）。"""
    values: Dict[int, float] = {}
    for number, wire, value in iter_fields(message):
        if wire == 0 and isinstance(value, int):
            values[number] = value
        elif wire == 1 and isinstance(value, bytes) and len(value) == 8:
            values[number] = struct.unpack("<d", value)[0]
        elif wire == 5 and isinstance(value, bytes) and len(value) == 4:
            values[number] = struct.unpack("<f", value)[0]
    return values


def _find_text_field(lane_bytes: bytes, field_number: int) -> str:
    """读字符串字段；某些地图代际把 id 包了一层 message，需要再拆一层。"""
    for number, wire, value in iter_fields(lane_bytes):
        if number != field_number or wire != WIRE_LEN or not isinstance(value, bytes):
            continue
        text = _decode_text(value)
        if text:
            return text
        for sub_number, sub_wire, sub_value in iter_fields(value):
            if sub_number == 1 and sub_wire == WIRE_LEN and isinstance(sub_value, bytes):
                nested = _decode_text(sub_value)
                if nested:
                    return nested
    return ""


def _decode_text(raw: bytes) -> str:
    """只接受可打印 ASCII；带 protobuf 头的字节串直接判为不可用。"""
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError:
        return ""
    if text and all(32 <= ord(char) < 127 for char in text):
        return text
    return ""


CATALOG_CACHE_DIR = Path(".scratch") / "map_cache"
CATALOG_TILE_MARGIN_M = 100.0


def parse_map_catalog(catalog_path: Path) -> List[dict]:
    """解析 ``base_map.catelog``（Apollo 拼写如此）：tile 列表 + bbox。

    结构：MapCatalog.tiles(2) → MapTile{ id(1) varint, path(2) string,
    sw.x(3) sw.y(4) ne.x(5) ne.y(6) 都是 double, 其余是元素 id 列表 }。
    大图（21–34 GB 的 base_map.txt）就是靠这张表切成一万多个 500 m tile。
    """
    raw = Path(catalog_path).read_bytes()
    tiles = []
    for field_number, _wire, value in iter_fields(raw):
        if field_number != 2 or not isinstance(value, bytes):
            continue
        scalars = read_scalar_fields(value)
        path = _find_text_field(value, 2)
        if not path or 3 not in scalars:
            continue
        tiles.append(
            {
                "path": path,
                "min_x": scalars[3],
                "min_y": scalars[4],
                "max_x": scalars[5],
                "max_y": scalars[6],
            }
        )
    if not tiles:
        raise ValueError(f"{catalog_path}: no tiles parsed")
    return tiles


def load_catalog_tiles(map_dir: Path) -> List[dict]:
    """带磁盘缓存的 catalog 解析（84–115 MB 每次重解析太浪费）。"""
    map_dir = Path(map_dir)
    catalog_path = map_dir / "base_map.catelog"
    if not catalog_path.exists():
        catalog_path = map_dir / "base_map.catalog"
    if not catalog_path.exists():
        raise FileNotFoundError(f"{map_dir}: no base_map.catelog")

    CATALOG_CACHE_DIR.mkdir(parents=True, exist_ok=True)
    cache_path = CATALOG_CACHE_DIR / f"{map_dir.name}_catalog.json"
    if cache_path.exists() and cache_path.stat().st_mtime >= catalog_path.stat().st_mtime:
        return json.loads(cache_path.read_text(encoding="utf-8"))

    tiles = parse_map_catalog(catalog_path)
    cache_path.write_text(json.dumps(tiles), encoding="utf-8")
    return tiles


def select_tiles(tiles: List[dict], min_x: float, min_y: float, max_x: float, max_y: float,
                 margin_m: float = CATALOG_TILE_MARGIN_M) -> List[dict]:
    selected = [
        tile
        for tile in tiles
        if not (
            tile["max_x"] < min_x - margin_m
            or tile["min_x"] > max_x + margin_m
            or tile["max_y"] < min_y - margin_m
            or tile["min_y"] > max_y + margin_m
        )
    ]
    return selected


def load_tiled_lane_index(map_dir: Path, min_x: float, min_y: float, max_x: float, max_y: float) -> LaneIndex:
    """按轨迹 bbox 只加载相关 tile，合并成一个索引。"""
    map_dir = Path(map_dir)
    tiles = load_catalog_tiles(map_dir)
    selected = select_tiles(tiles, min_x, min_y, max_x, max_y)
    if not selected:
        raise ValueError(f"{map_dir}: no tile covers bbox ({min_x:.0f},{min_y:.0f})-({max_x:.0f},{max_y:.0f})")

    lanes: List[Lane] = []
    for tile in selected:
        tile_path = map_dir / tile["path"]
        if not tile_path.exists():
            continue
        lanes.extend(_parse_lanes(tile_path.read_bytes()))
    if not lanes:
        raise ValueError(f"{map_dir}: selected {len(selected)} tiles but parsed no lanes")
    return _build_index(lanes)


def load_lane_index_for_map_dir(map_dir: Path, bbox=None):
    """小图走 base_map.bin，分块大图走 base_map.catelog + 轨迹 bbox。返回 (索引, 来源说明)。"""
    map_dir = Path(map_dir)
    map_bin = map_dir / "base_map.bin"
    if map_bin.exists() and map_bin.stat().st_size > 0:
        return load_lane_index(map_bin), "base_map.bin"
    if (map_dir / "base_map.catelog").exists() or (map_dir / "base_map.catalog").exists():
        if bbox is None:
            raise ValueError(f"{map_dir}: tiled map requires a pose bbox")
        return load_tiled_lane_index(map_dir, *bbox), "base_map.catelog(tiles)"
    raise FileNotFoundError(f"{map_dir}: neither base_map.bin nor base_map.catelog")


def load_lane_index(map_bin: Path) -> LaneIndex:
    lanes = _parse_lanes(Path(map_bin).read_bytes())
    if not lanes:
        raise ValueError(f"{map_bin}: no lanes parsed")
    return _build_index(lanes)


def _parse_lanes(raw: bytes) -> List[Lane]:
    lanes: List[Lane] = []
    for field_number, wire, value in iter_fields(raw):
        if field_number != 4 or wire != WIRE_LEN or not isinstance(value, bytes):
            continue
        points = _parse_curve_points(value)
        fields = read_scalar_fields(value)
        lanes.append(
            Lane(
                lane_id=_find_text_field(value, 1),
                points=points,
                speed_limit_mps=float(fields.get(6) or 0.0),
                turn=int(fields.get(13) or 0),
                lane_type=int(fields.get(12) or 0),
                length_m=float(fields.get(5) or 0.0),
            )
        )
    return lanes


def _build_index(lanes: List[Lane]) -> LaneIndex:
    cell_points: Dict[Tuple[int, int], List[np.ndarray]] = {}
    cell_lane_index: Dict[Tuple[int, int], List[int]] = {}
    cell_point_index: Dict[Tuple[int, int], List[int]] = {}
    for lane_index, lane in enumerate(lanes):
        cells = np.floor(lane.points / GRID_CELL_M).astype(np.int64)
        for point_index, (point, cell) in enumerate(zip(lane.points, cells)):
            key = (int(cell[0]), int(cell[1]))
            cell_points.setdefault(key, []).append(point)
            cell_lane_index.setdefault(key, []).append(lane_index)
            cell_point_index.setdefault(key, []).append(point_index)

    return LaneIndex(
        lanes=lanes,
        cell_points={key: np.array(value) for key, value in cell_points.items()},
        cell_lane_index={key: np.array(value) for key, value in cell_lane_index.items()},
        cell_point_index={key: np.array(value) for key, value in cell_point_index.items()},
    )
