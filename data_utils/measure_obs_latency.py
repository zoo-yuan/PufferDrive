"""Measure the observation envelope of the AD stack from Apollo records.

Why
---
`docs/vehicle_profile.md` §4.1: the 12.5 m/s operational cap is an **AD system
capability** limit driven by latency and perception range, not by the chassis.
PufferDrive currently observes with 200 m range and zero delay. If the vehicle
really plans on ~0.5 s old data, a zero-delay simulator teaches the policy to be
more aggressive than the car can be - and the gap is invisible in simulation.

This script measures the three numbers needed to parameterise the simulator:

1. **perception range** - obstacle distance distribution (p50/p90/p99/max)
2. **sensor age** - ``header.timestamp_sec - header.lidar_timestamp`` per module
3. **publish lag / end-to-end age** - ``record_time - header.timestamp_sec`` and
   ``planning publish - newest perception available at that moment``

It also verifies a non-obvious fact of this fork: the planning trajectory's
time origin is the *lidar frame*, not the publish time, so ``relative_time``
starts negative (about -0.5 s) and the "now" point sits at
``relative_time ~= header.timestamp_sec - lidar_timestamp``.

Usage
-----
    python data_utils/measure_obs_latency.py \
        --record '/media/.../rx-2714/*.bag' --json-out obs_envelope.json

Every number is reported with its sample count; anything unverifiable is left as
NaN rather than filled in.
"""

from __future__ import annotations

import argparse
import bisect
import glob
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from data_utils.apollo_record import (  # noqa: E402
    decode_control,
    decode_localization_pose,
    decode_perception_obstacles,
    decode_planning,
    iter_messages,
)

POSE_CHANNEL = "/apollo/localization/pose"
PLANNING_CHANNEL = "/apollo/planning"
CONTROL_CHANNEL = "/apollo/control"
OBSTACLE_CHANNEL_CANDIDATES = (
    "/apollo/perception/obstacles",
    "/apollo/perception/safetyobstacles",
)
ALL_CHANNELS = {POSE_CHANNEL, PLANNING_CHANNEL, CONTROL_CHANNEL, *OBSTACLE_CHANNEL_CANDIDATES}

MAX_PLAUSIBLE_OBSTACLE_DISTANCE_M = 1000.0
DISTANCE_BINS_M = (0.0, 30.0, 60.0, 100.0, 150.0)
PERCENTILES = (50.0, 90.0, 99.0, 100.0)
OPERATIONAL_SPEED_MPS = 12.5


def _percentile(sorted_values: Sequence[float], percentile: float) -> float:
    if not sorted_values:
        return float("nan")
    if percentile >= 100.0:
        return float(sorted_values[-1])
    position = (len(sorted_values) - 1) * percentile / 100.0
    lower = int(math.floor(position))
    upper = min(lower + 1, len(sorted_values) - 1)
    weight = position - lower
    return float(sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight)


def _row(values: List[float]) -> Dict[str, float]:
    finite = sorted(v for v in values if math.isfinite(v))
    if not finite:
        return {f"p{int(p)}": float("nan") for p in PERCENTILES}
    return {f"p{int(p)}": round(_percentile(finite, p), 4) for p in PERCENTILES}


def _nearest(times: Sequence[float], target: float) -> Optional[int]:
    if not times:
        return None
    index = bisect.bisect_left(times, target)
    if index == 0:
        return 0
    if index >= len(times):
        return len(times) - 1
    before, after = times[index - 1], times[index]
    return index - 1 if target - before <= after - target else index


def collect(path: str, obstacle_channel: str) -> Dict[str, object]:
    poses: List[Tuple[float, float, float, float, float]] = []
    perceptions: List[Dict[str, object]] = []
    plannings: List[Dict[str, object]] = []
    controls: List[Dict[str, object]] = []

    for channel, record_time_ns, payload in iter_messages(path, channels=ALL_CHANNELS):
        record_time_s = record_time_ns * 1e-9
        if channel == POSE_CHANNEL:
            sample = decode_localization_pose(payload)
            speed = math.hypot(sample["vx_mps"], sample["vy_mps"])
            poses.append(
                (sample["measurement_time_s"], sample["x_m"], sample["y_m"], sample["heading_rad"], speed)
            )
        elif channel == obstacle_channel:
            decoded = decode_perception_obstacles(payload)
            perceptions.append(
                {
                    "header": decoded["header"],
                    "record_time_s": record_time_s,
                    "obstacles": [
                        (o["x_m"], o["y_m"], int(o["type"]))
                        for o in decoded["obstacles"]
                    ],
                }
            )
        elif channel == PLANNING_CHANNEL:
            decoded = decode_planning(payload)
            plannings.append(
                {
                    "header": decoded["header"],
                    "record_time_s": record_time_s,
                    "points": decoded["trajectory_points"],
                    "latency_stats": decoded["latency_stats"],
                }
            )
        elif channel == CONTROL_CHANNEL:
            decoded = decode_control(payload)
            controls.append(
                {"header": decoded["header"], "record_time_s": record_time_s}
            )

    poses.sort(key=lambda item: item[0])
    perceptions.sort(key=lambda item: item["record_time_s"])
    plannings.sort(key=lambda item: item["record_time_s"])
    controls.sort(key=lambda item: item["record_time_s"])
    return {
        "poses": poses,
        "perceptions": perceptions,
        "plannings": plannings,
        "controls": controls,
        "obstacle_channel": obstacle_channel,
    }


def _publish_lag(records: Sequence[Dict[str, object]], key: str = "header") -> List[float]:
    lags = []
    for record in records:
        header = record[key]
        if not header:
            continue
        lags.append(record["record_time_s"] - float(header["timestamp_sec"]))
    return lags


def _sensor_age(records: Sequence[Dict[str, object]]) -> List[float]:
    ages = []
    for record in records:
        header = record["header"]
        lidar_ns = header.get("lidar_timestamp_ns", 0)
        if not lidar_ns:
            continue
        ages.append(float(header["timestamp_sec"]) - int(lidar_ns) * 1e-9)
    return ages


def summarize(data: Dict[str, object], record_path: str) -> Dict[str, object]:
    poses = data["poses"]
    pose_times = [p[0] for p in poses]
    perceptions = data["perceptions"]
    plannings = data["plannings"]
    controls = data["controls"]

    # --- 1. perception range -------------------------------------------------
    per_message_max: List[float] = []
    all_distances: List[float] = []
    distance_bin_counts = {f"{int(lo)}-{int(hi)}m": 0 for lo, hi in zip(DISTANCE_BINS_M, DISTANCE_BINS_M[1:])}
    beyond_max_bin = 0
    type_counts: Dict[int, int] = {}
    dropped_far_outliers = 0

    for perception in perceptions:
        header_time = float(perception["header"]["timestamp_sec"])
        pose_index = _nearest(pose_times, header_time)
        if pose_index is None:
            continue
        ego_x, ego_y = poses[pose_index][1], poses[pose_index][2]
        message_max = 0.0
        for obs_x, obs_y, obs_type in perception["obstacles"]:
            if not (math.isfinite(obs_x) and math.isfinite(obs_y)):
                continue
            distance = math.hypot(obs_x - ego_x, obs_y - ego_y)
            if distance > MAX_PLAUSIBLE_OBSTACLE_DISTANCE_M:
                dropped_far_outliers += 1
                continue
            all_distances.append(distance)
            message_max = max(message_max, distance)
            type_counts[obs_type] = type_counts.get(obs_type, 0) + 1
            for (lo, hi) in zip(DISTANCE_BINS_M, DISTANCE_BINS_M[1:]):
                if lo <= distance < hi:
                    distance_bin_counts[f"{int(lo)}-{int(hi)}m"] += 1
                    break
            else:
                if distance >= DISTANCE_BINS_M[-1]:
                    beyond_max_bin += 1
        if message_max > 0.0:
            per_message_max.append(message_max)
    distance_bin_counts[f">={int(DISTANCE_BINS_M[-1])}m"] = beyond_max_bin

    # --- 2. per-module lags --------------------------------------------------
    perception_lag = _publish_lag(perceptions)
    planning_lag = _publish_lag(plannings)
    control_lag = _publish_lag(controls)
    perception_age = _sensor_age(perceptions)
    planning_age = _sensor_age(plannings)
    control_age = _sensor_age(controls)

    # --- 3. end-to-end: planning vs newest perception ------------------------
    perception_header_times = sorted(float(p["header"]["timestamp_sec"]) for p in perceptions if p["header"])
    end_to_end: List[float] = []
    for planning in plannings:
        publish_time = planning["record_time_s"]
        index = bisect.bisect_right(perception_header_times, publish_time) - 1
        if index >= 0:
            end_to_end.append(publish_time - perception_header_times[index])

    # --- 4. plan trajectory time origin -------------------------------------
    origin_residuals: List[float] = []
    now_offsets: List[float] = []
    for planning in plannings:
        header = planning["header"]
        lidar_ns = header.get("lidar_timestamp_ns", 0)
        points = planning["points"]
        if not lidar_ns or not points:
            continue
        relative_times = [p["relative_time_s"] for p in points if math.isfinite(p["relative_time_s"])]
        if not relative_times:
            continue
        origin_residuals.append(float(header["timestamp_sec"]) + min(relative_times) - int(lidar_ns) * 1e-9)
        now_offsets.append(float(header["timestamp_sec"]) - int(lidar_ns) * 1e-9)

    planning_latency = [
        float(p["latency_stats"].get("total_time_ms", float("nan")))
        for p in plannings
        if p["latency_stats"]
    ]
    perception_latency = [
        float(p["latency_stats"].get("perception_latency_ms", float("nan")))
        for p in plannings
        if p["latency_stats"]
    ]

    median_planning_age = _percentile(sorted(v for v in planning_age if math.isfinite(v)), 50.0)
    median_control_age = _percentile(sorted(v for v in control_age if math.isfinite(v)), 50.0)
    summary = {
        "record": record_path,
        "obstacle_channel": data["obstacle_channel"],
        "counts": {
            "pose": len(poses),
            "perception": len(perceptions),
            "planning": len(plannings),
            "control": len(controls),
            "obstacles": len(all_distances),
            "dropped_far_outliers": dropped_far_outliers,
        },
        "perception_range_m": {
            "per_message_max": _row(per_message_max),
            "all_obstacles": _row(all_distances),
            "distance_histogram": distance_bin_counts,
            "type_counts": type_counts,
        },
        "publish_lag_s": {
            "perception": _row(perception_lag),
            "planning": _row(planning_lag),
            "control": _row(control_lag),
        },
        "sensor_age_s": {
            "perception": _row(perception_age),
            "planning": _row(planning_age),
            "control": _row(control_age),
        },
        "end_to_end_planning_s": _row(end_to_end),
        "plan_time_origin_residual_s": _row(origin_residuals),
        "planning_latency_total_ms": _row(planning_latency),
        "planning_reported_perception_latency_ms": _row(perception_latency),
        "reaction_distance_at_operational_speed_m": {
            "from_planning": round(median_planning_age * OPERATIONAL_SPEED_MPS, 2)
            if math.isfinite(median_planning_age)
            else None,
            "from_control": round(median_control_age * OPERATIONAL_SPEED_MPS, 2)
            if math.isfinite(median_control_age)
            else None,
        },
    }
    return summary


def _print_table(title: str, summary: Dict[str, object]) -> None:
    print(f"\n### {title}")
    print("| quantity | p50 | p90 | p99 | max |")
    print("| --- | --- | --- | --- | --- |")
    for name, row in summary.items():
        if not isinstance(row, dict) or "p50" not in row:
            continue
        print(f"| {name} | " + " | ".join(f"{row[k]:.4f}" for k in ("p50", "p90", "p99", "p100")) + " |")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", action="append", required=True)
    parser.add_argument("--obstacle-channel", default=None,
                        help="override; default tries /apollo/perception/obstacles then safetyobstacles")
    parser.add_argument("--json-out", default=None)
    args = parser.parse_args()

    records: List[str] = []
    for pattern in args.record:
        matches = sorted(glob.glob(pattern))
        if not matches:
            raise SystemExit(f"no record matched: {pattern}")
        records.extend(matches)

    results = []
    for path in records:
        data = None
        if args.obstacle_channel:
            data = collect(path, args.obstacle_channel)
        else:
            for candidate in OBSTACLE_CHANNEL_CANDIDATES:
                data = collect(path, candidate)
                if data["perceptions"]:
                    break
            if not data["perceptions"]:
                raise SystemExit(f"{path}: no obstacle channel among {OBSTACLE_CHANNEL_CANDIDATES}")
        if not data["poses"] or not data["plannings"]:
            raise SystemExit(f"{path}: missing pose or planning channel")
        summary = summarize(data, path)
        results.append(summary)
        print(f"\n== {path}")
        print(f"obstacle channel: {summary['obstacle_channel']}   counts: {summary['counts']}")
        _print_table("perception range (m)", summary["perception_range_m"])
        print(f"  distance histogram: {summary['perception_range_m']['distance_histogram']}")
        print(f"  obstacle types: {summary['perception_range_m']['type_counts']}")
        _print_table("sensor age (s) = header - lidar_timestamp", summary["sensor_age_s"])
        _print_table("publish lag (s) = record_time - header", summary["publish_lag_s"])
        _print_table("end-to-end planning (s) = planning publish - newest perception",
                     {"end_to_end": summary["end_to_end_planning_s"]})
        _print_table("plan time-origin hypothesis check (s) - not reliable",
                     {"residual": summary["plan_time_origin_residual_s"]})
        _print_table("planning compute (ms)", {
            "latency_stats.total": summary["planning_latency_total_ms"],
            "latency_stats.perception": summary["planning_reported_perception_latency_ms"],
        })
        print(f"reaction distance at 12.5 m/s: {summary['reaction_distance_at_operational_speed_m']}")
        print("note: perception sensor_age is 0 by construction in this fork "
              "(header.timestamp_sec == lidar_timestamp); its real lag shows up in publish_lag.")

    if args.json_out:
        Path(args.json_out).write_text(json.dumps(results, indent=2, ensure_ascii=False))
        print(f"\nwrote {args.json_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
