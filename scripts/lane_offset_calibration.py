"""判据可行性标定：`lane_departure` 的 0.9 m 门限 vs 录制司机自己的横向残差。

为什么需要这个
--------------
`lane_departure` 谓词是 `sim_speed > 0.5 m/s 且 |lane_dist| > 0.9 m`。
但 154 个"可避"场景是这么选的：录制司机在这些切片里越了道（所以它必然违规），
而策略被要求**全程**把 `|lane_dist|` 压在 0.9 m 以内。

问题在于余量：实测录制司机 `|lane_dist|` 峰值的中位数只有 **1.13 m**，
离 0.9 m 门限只有 ~0.23 m。如果这点差距其实是**地图匹配的系统性偏移**（车道中心线
相对真实行车线偏了一米），那"修好全部 154 个"可能本来就是不可达的目标 ——
需要先把残差拆成"人的行为"与"匹配偏差"两部分。

这个脚本量的是：参考零点（录制司机）的**有符号** `lane_dist` 分布。

判读方式
--------
- `median_signed` 在整批场景里**同号且量级 ~1 m** → 系统性偏移（地图中心线偏一侧），
  门限需要按偏移修正，或者判据改用"相对自身基准线的偏移变化"。
- `median_signed` 近似 0、只有少数场景大偏移 → 是行为（司机真压线），门限合理。
- 偏移接近半个车道宽（1.5–2 m）→ 车道关联选错邻道，属地图匹配 bug。

用法
----
``python scripts/lane_offset_calibration.py``                 # 出报告
``python scripts/lane_offset_calibration.py --check``         # 残差超阈值则 FAIL
"""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path

import numpy as np

import sys

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import regression_retention as harness  # noqa: E402
from scene_feasibility import DT_S, get_metric_names, state_row  # noqa: E402

STEPS = 60
SCENE_DIR = REPO_ROOT / ".scratch" / "crossbag_scenes"
OUTPUT_JSON = REPO_ROOT / "docs" / "measured" / "lane_offset_calibration_v0.json"

# 车道关联丢失时 lane_dist 会贴住归一化上限，这些帧不参与偏移统计
LANE_DIST_SATURATION_M = 3.99
# 判据门限（与 constraints_v0 lane_departure 同源）
DEPARTURE_THRESHOLD_M = 0.9
# 单场景中位偏移超过这个量级，就不像"司机行为"而像"匹配偏差"
SYSTEMATIC_OFFSET_M = 0.5
# HALF_LANE：偏移接近半个车道宽（1.5–2.0 m）时的警告线
HALF_LANE_M = 1.5


def measure_scene(scene_path: Path, metric_names, metric_index) -> dict:
    env = harness.build_replay_env(scene_path)
    env.reset(seed=0)
    signed = []
    speeds = []
    previous = None
    for _ in range(STEPS):
        env.step(np.zeros((env.num_agents, 2), dtype=np.float32))
        state = env.get_state()
        scenario = (state if isinstance(state, list) else [state])[0]
        agent = (scenario.get("agents") or [])[0]
        row = state_row(agent, metric_names, metric_index)
        # replay 贴位姿、不积分，sim_speed 恒 0：用相邻位姿反推（与 M7-3 同一处理）
        speed = 0.0
        if previous is not None:
            speed = float(np.hypot(agent["sim_x"] - previous[0], agent["sim_y"] - previous[1])) / DT_S
        previous = (agent["sim_x"], agent["sim_y"])
        lane_dist = float(row.get("lane_dist", 0.0))
        signed.append(lane_dist)
        speeds.append(speed)

    associated = [d for d in signed if abs(d) < LANE_DIST_SATURATION_M]
    moving = [d for d, v in zip(signed, speeds) if v > 0.5 and abs(d) < LANE_DIST_SATURATION_M]
    basis = moving or associated or [0.0]
    baseline = statistics.median(basis)
    # 去偏后的剩余偏移：判据真正想问的是"相对这条路的常态偏出去多少"
    detrended = [abs(d - baseline) for d in basis]
    return {
        "steps": len(signed),
        "associated_steps": len(associated),
        "median_signed": round(baseline, 3),
        "mean_signed": round(statistics.fmean(basis), 3),
        "max_abs": round(max(abs(d) for d in signed), 3),
        "p95_abs": round(sorted(abs(d) for d in basis)[int(0.95 * (len(basis) - 1))], 3),
        "frac_steps_over_threshold": round(
            sum(1 for d in basis if abs(d) > DEPARTURE_THRESHOLD_M) / max(1, len(basis)), 3
        ),
        "max_abs_detrended": round(max(detrended), 3),
        "frac_steps_over_threshold_detrended": round(
            sum(1 for d in detrended if d > DEPARTURE_THRESHOLD_M) / max(1, len(detrended)), 3
        ),
        # 完整有符号序列留档：下游要换判据/换门限都不用重跑引擎
        "signed_lane_dist": [round(d, 3) for d in signed],
        "lane_lost": len(associated) < len(signed) // 2,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-dir", default=str(SCENE_DIR))
    parser.add_argument("--run-config", default=None,
                        help="参考侧只用 env 段（replay 不吃策略），默认 experiments/config.yaml")
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--json", default=str(OUTPUT_JSON))
    args = parser.parse_args()

    if args.run_config:
        harness.RUN_CONFIG = Path(args.run_config).resolve()

    scene_dir = Path(args.scene_dir)
    metric_names, metric_index = get_metric_names()
    scenes = [s for s in harness.scene_ids()
              if (scene_dir / f"{s['scenario_id'].replace('#', '_')}.bin").exists()]
    if not scenes:
        print(f"ERROR: {scene_dir} 里没有可评测场景", flush=True)
        return 1

    print(f"[reference] {len(scenes)} 个场景的录制司机横向残差 …", flush=True)
    rows = []
    for scene in scenes:
        path = scene_dir / f"{scene['scenario_id'].replace('#', '_')}.bin"
        row = measure_scene(path, metric_names, metric_index)
        rows.append({"scenario_id": scene["scenario_id"], "template": scene["template"],
                     "bag": scene.get("bag", ""), **row})

    medians = [r["median_signed"] for r in rows]
    abs_medians = [abs(m) for m in medians]
    same_sign_positive = sum(1 for m in medians if m > 0)
    same_sign_negative = sum(1 for m in medians if m < 0)
    summary = {
        "scenes": len(rows),
        "threshold_m": DEPARTURE_THRESHOLD_M,
        "median_of_median_signed": round(statistics.median(medians), 3),
        "median_abs_median_signed": round(statistics.median(abs_medians), 3),
        "p90_abs_median_signed": round(sorted(abs_medians)[int(0.9 * (len(abs_medians) - 1))], 3),
        "scenes_median_over_half_m": sum(1 for m in abs_medians if m > SYSTEMATIC_OFFSET_M),
        "scenes_median_over_half_lane": sum(1 for m in abs_medians if m > HALF_LANE_M),
        "sign_positive": same_sign_positive,
        "sign_negative": same_sign_negative,
        "median_max_abs": round(statistics.median([r["max_abs"] for r in rows]), 3),
        "median_frac_steps_over_threshold": round(
            statistics.median([r["frac_steps_over_threshold"] for r in rows]), 3),
        "median_max_abs_detrended": round(
            statistics.median([r["max_abs_detrended"] for r in rows]), 3),
        "p90_max_abs_detrended": round(
            sorted(r["max_abs_detrended"] for r in rows)[int(0.9 * (len(rows) - 1))], 3),
        "scenes_violating_detrended": sum(
            1 for r in rows if r["frac_steps_over_threshold_detrended"] > 0.0),
        "lane_lost_scenes": sum(1 for r in rows if r["lane_lost"]),
    }

    # 判读：偏移是系统性（同号 + 量级 > 0.5 m）还是行为
    dominant_sign = max(same_sign_positive, same_sign_negative)
    bias_ratio = dominant_sign / len(rows)
    # 判"系统性偏移"看的是**集中度**而不只是符号比例：偏移大 + 袋内离散度小
    # （p90 与中位差 < 0.2 m）说明每段路都在同一侧压着同一条偏出来的线。
    band_spread = summary["p90_abs_median_signed"] - summary["median_abs_median_signed"]
    systematic_band = summary["median_abs_median_signed"] > SYSTEMATIC_OFFSET_M and band_spread < 0.2
    if summary["scenes_median_over_half_lane"] > 0.1 * len(rows):
        verdict = "lane_association_or_selection"
    elif systematic_band:
        verdict = "systematic_offset"
    else:
        verdict = "driver_behaviour"
    summary["verdict"] = verdict
    summary["dominant_sign_ratio"] = round(bias_ratio, 3)

    print(f"  门限                       {DEPARTURE_THRESHOLD_M:.2f} m")
    print(f"  参考中位|偏移|             {summary['median_abs_median_signed']:.3f} m（p90 {summary['p90_abs_median_signed']:.3f}）")
    print(f"  最偏 10% 场景             > {summary['p90_abs_median_signed']:.3f} m")
    print(f"  同号比例                   {bias_ratio * 100:.0f}%（+{same_sign_positive}/-{same_sign_negative}）")
    print(f"  峰值中位                   {summary['median_max_abs']:.3f} m")
    print(f"  超过门限的步占比（中位）   {summary['median_frac_steps_over_threshold'] * 100:.0f}%")
    print(f"  去偏后峰值（中位 / p90）   {summary['median_max_abs_detrended']:.3f} / {summary['p90_max_abs_detrended']:.3f} m")
    print(f"  去偏后仍越道场景           {summary['scenes_violating_detrended']}/{len(rows)}")
    print(f"  车道关联丢失场景           {summary['lane_lost_scenes']}/{len(rows)}")
    print(f"  判读                       {verdict}")

    output = Path(args.json)
    if output:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps({"summary": summary, "rows": rows}, ensure_ascii=False, indent=2),
                          encoding="utf-8")
        print(f"[json] {output}")

    failures = []
    if summary["lane_lost_scenes"] > 0.1 * len(rows):
        failures.append(f"{summary['lane_lost_scenes']} 个场景超过一半步数没有车道关联（地图覆盖不足）")
    if verdict == "systematic_offset":
        failures.append(
            f"参考司机横向残差系统性同号（{bias_ratio * 100:.0f}%）、中位 "
            f"{summary['median_abs_median_signed']:.2f} m —— 0.9 m 门限量的是地图偏移，不是驾驶行为"
        )
    for failure in failures:
        print(f"GATE FAIL : {failure}")
    print(f"GATE      : {'FAIL' if failures else 'PASS'}")
    return 1 if (args.check and failures) else 0


if __name__ == "__main__":
    raise SystemExit(main())
