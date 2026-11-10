"""按部署 KPI 选 checkpoint：训练曲线不判别好坏，最好的权重在中途。

为什么需要这个
--------------
v6（900k 步）在 154 个场景上违规 **107**、中位 `lane_dist` 峰值 **0.967 m**；
同一配置跑到 2.7M（v6b）变成 **139 / 1.771 m**。两臂训练日志末尾都写
`offroad_rate 1.000`、`episode_length ~45` —— **训练聚合指标分不出好坏**。
`total_timesteps` 同时改 LR schedule，所以"多训"不是单调变好，最好的点在中途某处。

因此：训练时 `train.keep_checkpoints=true` + `train.checkpoint_interval` 留下序列，
训练后（或并发）用这个脚本按 **KPI 分母（默认 `spec/badcase_set_v1.yaml` 的 genuine 70）**
逐个评测，报分并存最优。

KPI 排序（字典序，全部越小/越大越好）
------------------------------------
1. 违规场景数（越小越好）
2. 中位 `lane_dist` 峰值（越小越好）
3. 中位车速（越大越好，仅作 tie-break；不动车的点必须被排除）
另附**车动门**：中位车速 < `--min-speed` 的 checkpoint 直接判不合格（趴窝假修复前科见 ADR-0010）。

用法
----
``python scripts/kpi_select.py --run-dir .scratch/experiments_v8``
``python scripts/kpi_select.py --run-dir ... --finalize``  # 把最优复制成 <run-dir>/best_kpi.pt
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path

import numpy as np
import torch

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import regression_retention as harness  # noqa: E402
from scene_feasibility import get_metric_names, load_template  # noqa: E402

SCENE_DIR = REPO_ROOT / ".scratch" / "crossbag_scenes"
DEFAULT_BADCASE_SET = REPO_ROOT / "spec" / "badcase_set_v1.yaml"


def load_denominator(set_path: Path) -> set[str]:
    import yaml

    document = yaml.safe_load(set_path.read_text(encoding="utf-8"))
    return {row["scenario_id"] for row in document["scenes"] if row["bucket"] == "genuine"}


def evaluate(policy, scenes, scene_dir, metric_names, metric_index, device) -> dict:
    fired = 0
    peaks = []
    speeds = []
    for scene in scenes:
        path = scene_dir / f"{scene['scenario_id'].replace('#', '_')}.bin"
        template = load_template(scene["template"])
        result = harness.evaluate_scene(policy, path, template, metric_names, metric_index, device)
        fired += int(result["fired"])
        peaks.append(result["peak"])
        speeds.append(result.get("median_speed", 0.0))
    return {
        "violations": fired,
        "scenes": len(scenes),
        "median_peak_m": round(statistics.median(peaks), 3) if peaks else 0.0,
        "median_speed_mps": round(statistics.median(speeds), 3) if speeds else 0.0,
    }


def rank_key(result: dict, min_speed: float) -> tuple:
    """字典序越小越好；不动车直接沉底。"""
    parked = result["median_speed_mps"] < min_speed
    return (parked, result["violations"], result["median_peak_m"], -result["median_speed_mps"])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--scene-dir", default=str(SCENE_DIR))
    parser.add_argument("--badcase-set", default=str(DEFAULT_BADCASE_SET))
    parser.add_argument("--checkpoints", default="models/model_*.pt",
                        help="相对 run-dir 的 glob；默认评测训练中保留的所有 checkpoint")
    parser.add_argument("--min-speed", type=float, default=1.0)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--finalize", action="store_true", help="把最优权重复制成 <run-dir>/best_kpi.pt")
    args = parser.parse_args()

    run_dir = Path(args.run_dir).resolve()
    run_config = run_dir / "config.yaml"
    if not run_config.exists():
        print(f"ERROR: 找不到 {run_config}（评测必须用本轮训练快照）", file=sys.stderr)
        return 1
    harness.RUN_CONFIG = run_config

    checkpoints = sorted(run_dir.glob(args.checkpoints))
    if not checkpoints:
        print(f"ERROR: {run_dir} 里没有匹配 {args.checkpoints} 的权重", file=sys.stderr)
        return 1

    denominator = load_denominator(Path(args.badcase_set))
    scene_dir = Path(args.scene_dir)
    scenes = [scene for scene in harness.scene_ids()
              if scene["scenario_id"] in denominator
              and (scene_dir / f"{scene['scenario_id'].replace('#', '_')}.bin").exists()]
    if args.limit:
        scenes = scenes[: args.limit]
    if not scenes:
        print(f"ERROR: KPI 分母为空（{args.badcase_set}）", file=sys.stderr)
        return 1
    print(f"[kpi] 分母 {len(scenes)} 个场景（{Path(args.badcase_set).name}, genuine）· "
          f"{len(checkpoints)} 个待评 checkpoint", flush=True)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    metric_names, metric_index = get_metric_names()
    first_path = scene_dir / f"{scenes[0]['scenario_id'].replace('#', '_')}.bin"
    policy = harness.build_policy(harness.build_env(first_path), device)

    history = []
    for checkpoint in checkpoints:
        try:
            harness.load_weights(policy, checkpoint, device)
        except RuntimeError as error:
            print(f"[skip] {checkpoint.name}: {str(error).splitlines()[0]}", flush=True)
            continue
        result = evaluate(policy, scenes, scene_dir, metric_names, metric_index, device)
        row = {"checkpoint": str(checkpoint.relative_to(run_dir)), **result}
        history.append(row)
        print(f"  {checkpoint.name:<32} 违规 {result['violations']:>3}/{result['scenes']} · "
              f"中位峰值 {result['median_peak_m']:.3f} m · 车速 {result['median_speed_mps']:.2f} m/s",
              flush=True)

    if not history:
        print("ERROR: 没有可评测的 checkpoint（全部形状不匹配？）", file=sys.stderr)
        return 1
    history.sort(key=lambda row: rank_key(row, args.min_speed))
    best = history[0]
    report = {
        "run_dir": str(run_dir),
        "run_config": str(run_config),
        "badcase_set": str(Path(args.badcase_set)),
        "denominator_scenes": len(scenes),
        "min_speed_mps": args.min_speed,
        "rank_rule": "parked_first, violations, median_peak_m, -median_speed_mps",
        "best": best,
        "history": history,
    }
    out = run_dir / "kpi_history.json"
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"[best] {best['checkpoint']} · 违规 {best['violations']}/{best['scenes']} · "
          f"中位峰值 {best['median_peak_m']:.3f} m · 车速 {best['median_speed_mps']:.2f} m/s")
    print(f"[json] {out}")

    if best["median_speed_mps"] < args.min_speed:
        print(f"GATE FAIL : 最优 checkpoint 中位车速 {best['median_speed_mps']:.2f} < {args.min_speed}"
              " —— 全都在趴窝，KPI 无意义")
        print("GATE      : FAIL")
        return 1

    if args.finalize:
        import shutil

        target = run_dir / "best_kpi.pt"
        shutil.copy(run_dir / best["checkpoint"], target)
        print(f"[finalize] {target}")
    print("GATE      : PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
