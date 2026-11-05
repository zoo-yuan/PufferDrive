"""Measure the executed lateral-acceleration envelope from Apollo records.

Why
---
`docs/vehicle_profile.md` §5: Apollo `vehicle_param` has no lateral-acceleration
limit. The real constraint is steering angle + steering rate + tyres, and
PufferDrive has no friction model, so it must be given a **measured** tyre proxy.
This script produces that number from the same bags the data lake is built from.

What it computes, per ego pose sample (`/apollo/localization/pose`)
------------------------------------------------------------------
- ``v``             : ``hypot(vx, vy)``                              [m/s]
- ``yaw_rate``      : ``angular_velocity.z``                          [rad/s]
- ``a_lat``         : ``v * yaw_rate``  (exectued lateral accel)       [m/s^2]
- ``kappa``         : ``yaw_rate / v``  (path curvature)               [1/m]
- ``delta``         : ``atan(kappa * wheelbase)`` (implied road-wheel) [rad]
- ``a_lon``         : ``dv / dt``  (cross-check hard accel limits)     [m/s^2]

Usage
-----
    python data_utils/measure_lateral_accel.py \\
        --record '/media/.../white-rhino-*/*.bag' \\
        --json-out lateral_envelope.json

    python data_utils/measure_lateral_accel.py --record <one bag> --list-channels

Conventions: samples with ``v < --min-speed-mps`` are excluded from lateral
statistics (curvature is undefined at standstill) but still counted. The
suggested tyre proxy is ``p99.9`` of ``|a_lat|``; it is only meaningful once the
sample spans enough driving, which is reported and warned about explicitly.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from data_utils.apollo_record import (  # noqa: E402
    DEFAULT_POSE_CHANNELS,
    decode_localization_pose,
    iter_messages,
    list_channels,
)

VEHICLE_WHEELBASE_M = 2.20
VEHICLE_MAX_STEER_RAD = 0.60564
VEHICLE_MAX_ACCEL_MPS2 = 3.0
VEHICLE_MAX_DECEL_MPS2 = -6.0
MAX_SAMPLE_GAP_S = 0.5
MIN_USEFUL_DRIVING_SECONDS = 1800.0
MAX_EXPECTED_SPEED_MPS = 12.5
MAX_PLAUSIBLE_SPEED_MPS = 1.2 * MAX_EXPECTED_SPEED_MPS
MAX_PLAUSIBLE_CURVATURE_1PM = 0.3147
MAX_PLAUSIBLE_LATERAL_ACCEL_MPS2 = 10.0
MIN_SPEED_FOR_CURVATURE_MPS = 2.0
STEER_NEAR_LIMIT_RATIO = 0.9
ACCEL_PERCENTILES = (50.0, 90.0, 99.0, 99.9, 99.99, 100.0)


def _percentile(sorted_values: Sequence[float], percentile: float) -> float:
    if not sorted_values:
        return float("nan")
    if percentile >= 100.0:
        return float(sorted_values[-1])
    if percentile <= 0.0:
        return float(sorted_values[0])
    position = (len(sorted_values) - 1) * percentile / 100.0
    lower = int(math.floor(position))
    upper = min(lower + 1, len(sorted_values) - 1)
    weight = position - lower
    return float(sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight)


def _percentile_row(values: List[float], percentiles: Iterable[float]) -> Dict[str, float]:
    ordered = sorted(v for v in values if math.isfinite(v))
    return {f"p{p:g}": _percentile(ordered, p) for p in percentiles}


def _pick_channel(path: str, requested: Optional[str]) -> str:
    if requested:
        return requested
    available = list_channels(path)
    for candidate in DEFAULT_POSE_CHANNELS:
        if candidate in available:
            return candidate
    raise SystemExit(
        f"{path}: no pose channel found. Available: {sorted(available)}"
    )


def _expand_records(patterns: Sequence[str]) -> List[str]:
    records: List[str] = []
    for pattern in patterns:
        matches = sorted(glob.glob(pattern))
        if not matches:
            raise SystemExit(f"no record matched: {pattern}")
        records.extend(matches)
    return records


def collect_samples(records: Sequence[str], channel: Optional[str]) -> Dict[str, object]:
    speeds: List[float] = []
    lateral_accels: List[float] = []
    lateral_accels_moving: List[float] = []
    curvatures: List[float] = []
    steer_angles: List[float] = []
    longitudinal_accels_pose: List[float] = []
    longitudinal_accels_diff: List[float] = []
    moving_seconds = 0.0
    total_seconds = 0.0
    sample_count = 0
    moving_count = 0
    per_record: Dict[str, int] = {}
    per_record_speed_max: Dict[str, float] = {}
    implausible_speed = 0
    implausible_curvature = 0
    implausible_lateral = 0

    for path in records:
        resolved_channel = _pick_channel(path, channel)
        previous_time: Optional[float] = None
        previous_speed: Optional[float] = None
        record_samples = 0
        record_speed_max = 0.0

        for _name, timestamp_ns, payload in iter_messages(path, channels={resolved_channel}):
            sample = decode_localization_pose(payload)
            sample_time = sample["measurement_time_s"]
            if not math.isfinite(sample_time):
                sample_time = timestamp_ns * 1e-9

            speed = math.hypot(sample["vx_mps"], sample["vy_mps"])
            if not math.isfinite(speed):
                raise SystemExit(f"{path}: non-finite speed at t={sample_time}")

            heading = sample["heading_rad"]
            # Pose linear_acceleration is in the world (ENU) frame: project it.
            longitudinal_accels_pose.append(
                sample["ax_mps2"] * math.cos(heading) + sample["ay_mps2"] * math.sin(heading)
            )

            if previous_time is not None:
                delta_time = sample_time - previous_time
                if 0.0 < delta_time <= MAX_SAMPLE_GAP_S:
                    total_seconds += delta_time
                    if previous_speed is not None:
                        longitudinal_accels_diff.append((speed - previous_speed) / delta_time)
                    if speed >= 1.0:
                        moving_seconds += delta_time
                # Gaps mean a dropped/late pose: do not fabricate a derivative.

            record_samples += 1
            record_speed_max = max(record_speed_max, speed)
            sample_count += 1
            lateral_accel = speed * sample["yaw_rate_radps"]
            curvature = (
                sample["yaw_rate_radps"] / speed
                if speed >= MIN_SPEED_FOR_CURVATURE_MPS
                else float("nan")
            )

            # Physical plausibility gate for the 3799 profile. Raw bags contain
            # localisation jumps (RTK re-init) that produce 38 m/s and
            # kappa > 1 /m; keeping them would poison the tyre proxy. Counted,
            # never silently clamped.
            if speed > MAX_PLAUSIBLE_SPEED_MPS:
                implausible_speed += 1
            elif math.isfinite(curvature) and abs(curvature) > MAX_PLAUSIBLE_CURVATURE_1PM:
                implausible_curvature += 1
            elif abs(lateral_accel) > MAX_PLAUSIBLE_LATERAL_ACCEL_MPS2:
                implausible_lateral += 1
            else:
                speeds.append(speed)
                lateral_accels.append(lateral_accel)
                if speed >= 1.0:
                    moving_count += 1
                    lateral_accels_moving.append(lateral_accel)
                if math.isfinite(curvature):
                    curvatures.append(curvature)
                    steer_angles.append(math.atan(curvature * VEHICLE_WHEELBASE_M))

            previous_time = sample_time
            previous_speed = speed

        if record_samples == 0:
            raise SystemExit(f"{path}: channel {resolved_channel} has no messages")
        per_record[path] = record_samples
        per_record_speed_max[path] = record_speed_max

    return {
        "records": per_record,
        "record_speed_max": per_record_speed_max,
        "implausible_speed": implausible_speed,
        "implausible_curvature": implausible_curvature,
        "implausible_lateral": implausible_lateral,
        "sample_count": sample_count,
        "moving_sample_count": moving_count,
        "total_seconds": total_seconds,
        "moving_seconds": moving_seconds,
        "speeds": speeds,
        "lateral_accels": lateral_accels,
        "lateral_accels_moving": lateral_accels_moving,
        "curvatures": curvatures,
        "steer_angles": steer_angles,
        "longitudinal_accels_pose": longitudinal_accels_pose,
        "longitudinal_accels_diff": longitudinal_accels_diff,
    }


def summarize(samples: Dict[str, object]) -> Dict[str, object]:
    lateral = [abs(v) for v in samples["lateral_accels"]]
    lateral_moving = [abs(v) for v in samples["lateral_accels_moving"]]
    curvature = [abs(v) for v in samples["curvatures"]]
    steer = [abs(v) for v in samples["steer_angles"]]
    longitudinal_pose = list(samples["longitudinal_accels_pose"])
    longitudinal_diff = list(samples["longitudinal_accels_diff"])

    steering_near_limit = sum(
        1 for value in steer if value >= STEER_NEAR_LIMIT_RATIO * VEHICLE_MAX_STEER_RAD
    )
    steer_utilization = [value / VEHICLE_MAX_STEER_RAD for value in steer]
    over_accel_pose = sum(1 for value in longitudinal_pose if value > VEHICLE_MAX_ACCEL_MPS2)
    over_decel_pose = sum(1 for value in longitudinal_pose if value < VEHICLE_MAX_DECEL_MPS2)

    summary = {
        "records_parsed": len(samples["records"]),
        "record_speed_max_mps": {
            str(path): round(float(value), 3)
            for path, value in samples["record_speed_max"].items()
        },
        "sample_count": samples["sample_count"],
        "moving_sample_count": samples["moving_sample_count"],
        "implausible_speed": samples["implausible_speed"],
        "implausible_curvature": samples["implausible_curvature"],
        "implausible_lateral": samples["implausible_lateral"],
        "plausible_sample_count": len(samples["speeds"]),
        "total_seconds": round(float(samples["total_seconds"]), 3),
        "moving_seconds": round(float(samples["moving_seconds"]), 3),
        "speed_mps": _percentile_row(list(samples["speeds"]), ACCEL_PERCENTILES),
        "lateral_accel_all_mps2": _percentile_row(lateral, ACCEL_PERCENTILES),
        "lateral_accel_moving_mps2": _percentile_row(lateral_moving, ACCEL_PERCENTILES),
        "curvature_moving_1pm": _percentile_row(curvature, ACCEL_PERCENTILES),
        "steer_angle_moving_rad": _percentile_row(steer, ACCEL_PERCENTILES),
        "steer_utilization_moving": _percentile_row(steer_utilization, ACCEL_PERCENTILES),
        "lateral_accel_over_4_mps2": sum(1 for v in lateral if v > 4.0),
        "lateral_accel_over_6_mps2": sum(1 for v in lateral if v > 6.0),
        "lateral_accel_over_8_mps2": sum(1 for v in lateral if v > 8.0),
        "steer_near_limit_count": steering_near_limit,
        "longitudinal_accel_pose_mps2": _percentile_row(
            longitudinal_pose, (0.1, 1.0, 50.0, 99.0, 99.9, 100.0)
        ),
        "longitudinal_accel_diff_mps2": _percentile_row(
            longitudinal_diff, (0.1, 1.0, 50.0, 99.0, 99.9, 100.0)
        ),
        "longitudinal_pose_over_accel_limit": over_accel_pose,
        "longitudinal_pose_over_decel_limit": over_decel_pose,
        "moving_lateral_accel_p99_99_mps2": round(
            _percentile(sorted(lateral_moving), 99.99), 3
        )
        if lateral_moving
        else None,
        # A hard cap must not sit below what the vehicle demonstrably did: take the
        # observed maximum and round up to 0.5 m/s². Re-measure on a larger corpus;
        # the p99.99 is reported alongside as the robust estimate.
        "suggested_lateral_accel_cap_mps2": round(
            math.ceil(max(lateral_moving) * 2.0) / 2.0, 2
        )
        if lateral_moving
        else None,
        "tyre_proxy_is_conclusive": bool(
            float(samples["moving_seconds"]) >= MIN_USEFUL_DRIVING_SECONDS
        ),
    }
    return summary


def _format_row(label: str, row: Dict[str, float], unit: str, digits: int = 3) -> str:
    cells = " | ".join(f"{value:.{digits}f}" for value in row.values())
    return f"| {label} | {cells} | {unit} |"


def print_report(summary: Dict[str, object]) -> None:
    headers = " | ".join(summary["speed_mps"].keys())
    print(f"\nrecords: {summary['records_parsed']}   samples: {summary['sample_count']}"
          f"   moving: {summary['moving_sample_count']}"
          f"   duration: {summary['total_seconds']:.1f}s"
          f"   moving: {summary['moving_seconds']:.1f}s")
    print(f"\n| quantity | {headers} | unit |")
    print(f"| --- | {' | '.join('---' for _ in summary['speed_mps'])} | --- |")
    for key, label, unit in (
        ("speed_mps", "speed", "m/s"),
        ("lateral_accel_all_mps2", "|a_lat| (all)", "m/s²"),
        ("lateral_accel_moving_mps2", "|a_lat| (v≥1)", "m/s²"),
        ("curvature_moving_1pm", "|kappa| (v≥1)", "1/m"),
        ("steer_angle_moving_rad", "|delta| (v≥2)", "rad"),
        ("steer_utilization_moving", "delta/delta_max (v≥2)", "-"),
    ):
        print(_format_row(label, summary[key], unit))

    print("\n| check | count |")
    print("| --- | --- |")
    for key in (
        "implausible_speed",
        "implausible_curvature",
        "lateral_accel_over_4_mps2",
        "lateral_accel_over_6_mps2",
        "lateral_accel_over_8_mps2",
        "steer_near_limit_count",
        "longitudinal_pose_over_accel_limit",
        "longitudinal_pose_over_decel_limit",
    ):
        print(f"| {key} | {summary[key]} |")

    print("\n| longitudinal | p0.1 | p1 | p50 | p99 | p99.9 | p100 |")
    print("| --- | --- | --- | --- | --- | --- | --- |")
    for key, label in (
        ("longitudinal_accel_pose_mps2", "from pose projection"),
        ("longitudinal_accel_diff_mps2", "from dv/dt (noisy)"),
    ):
        row = summary[key]
        print("| " + label + " | " + " | ".join(f"{row[k]:.3f}" for k in row) + " |")

    speed_max = summary["record_speed_max_mps"]
    highest = max(speed_max.values()) if speed_max else 0.0
    if highest > MAX_EXPECTED_SPEED_MPS:
        worst = max(speed_max, key=speed_max.get)
        print(
            f"\nNOTE: observed max speed {highest:.3f} m/s ({highest * 3.6:.1f} km/h)"
            f" exceeds the documented {MAX_EXPECTED_SPEED_MPS} m/s ({MAX_EXPECTED_SPEED_MPS * 3.6:.0f} km/h)"
            f" — worst record: {worst}"
        )

    proxy = summary["suggested_lateral_accel_cap_mps2"]
    robust = summary["moving_lateral_accel_p99_99_mps2"]
    print(
        f"\nobserved max |a_lat| = {summary['lateral_accel_moving_mps2']['p100']:.3f} m/s²"
        f"   (p99.99 = {robust:.3f})\n"
        f"provisional hard cap (observed max rounded up to 0.5) = {proxy} m/s²"
    )
    if not summary["tyre_proxy_is_conclusive"]:
        print(
            "WARNING: moving time below 1800 s — not yet a usable tyre proxy."
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", action="append", required=False, default=[],
                        help="record path or glob (repeatable)")
    parser.add_argument("--channel", default=None,
                        help="pose channel; default auto-detect")
    parser.add_argument("--list-channels", action="store_true",
                        help="print channels of each record and exit")
    parser.add_argument("--json-out", default=None)
    parser.add_argument("--csv-out", default=None)
    args = parser.parse_args()

    if not args.record:
        parser.error("--record is required")

    records = _expand_records(args.record)
    if args.list_channels:
        for path in records:
            print(f"\n{path}")
            for channel, count in sorted(list_channels(path).items()):
                print(f"  {count:8d}  {channel}")
        return 0

    samples = collect_samples(records, args.channel)
    summary = summarize(samples)
    print_report(summary)

    if args.json_out:
        Path(args.json_out).write_text(json.dumps(summary, indent=2, ensure_ascii=False))
        print(f"\nwrote {args.json_out}")

    if args.csv_out:
        rows = ["a_lat_mps2,a_lon_pose_mps2,curvature_1pm,steer_rad"]
        longitudinal = samples["longitudinal_accels_pose"]
        for index, lateral in enumerate(samples["lateral_accels"]):
            speed = samples["speeds"][index]
            curvature = lateral / (speed * speed) if speed >= 1.0 else float("nan")
            steer = math.atan(curvature * VEHICLE_WHEELBASE_M) if speed >= 1.0 else float("nan")
            a_lon = longitudinal[index] if index < len(longitudinal) else float("nan")
            rows.append(f"{lateral:.6f},{a_lon:.6f},{curvature:.8f},{steer:.8f}")
        Path(args.csv_out).write_text("\n".join(rows) + "\n")
        print(f"wrote {args.csv_out}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
