"""bad case 选集 v1：按 ADR-0012 把 154 个 lane_departure 切片分成"真越道 / 边缘 / 假 case"。

为什么需要这一层
----------------
`lane_departure` 的原始选集（154）是用**绝对**判据 `|lane_dist| > 0.9 m` 挑的，但录制司机
本身离映射车道中心线就有 **0.80 m**（p90 0.87）的系统性偏移（`docs/adr/0012`）。于是：

- 一部分切片的"越道"完全是基准线坐在门限附近造成的（参考轨迹相对自己基准线只偏 0.2–0.3 m）；
- 一部分切片里 `lane_dist` 中途变号（最近车道在平行/双向车道之间跳），读到的是地图记账。

这个脚本把 154 拆成三桶，并给 KPI 一个诚实的分母：

| 桶 | 判据（相对**本场景参考基准**的偏移） | 用途 |
|---|---|---|
| `genuine` | ≥ `threshold_m` 且该步与基准同号 | **新 KPI 分母** |
| `marginal` | `0.3 m ≤ x < threshold_m` | 单独报告，不进修复率 |
| `artefact` | < `0.3 m` | 剔除（假 bad case） |

`map_suspect`（翻转步数占比 > 25%，无 `0.3 m` 阈值不可行 —— 154 个场景里翻转步数为 0 的一个都没有，
中位 7 步）单独打标，仍留在分母里但单列。

用法
----
``python scripts/scene_badcase_set.py``           # 生成 spec/badcase_set_v1.yaml
``python scripts/scene_badcase_set.py --check``   # 断言分桶不变量
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CALIBRATION_JSON = REPO_ROOT / "docs" / "measured" / "lane_offset_calibration_v0.json"
OUTPUT_YAML = REPO_ROOT / "spec" / "badcase_set_v1.yaml"
OUTPUT_JSON = REPO_ROOT / "docs" / "measured" / "badcase_set_v1.json"

# 与 constraints_v0 / 引擎事件谓词同源
THRESHOLD_M = 0.9
# 桶边界：低于这个量级就不是"驾驶员在越道"，是基准线位置的问题
MARGINAL_MIN_M = 0.3
# 翻转步占比超过它 → lane_dist 在两条车道之间跳，读数不可信
FLIP_FRAC_MAX = 0.25
# lane_dist 归一化上限：到这里说明车道关联已经丢失
SATURATION_M = 3.99


def sign(value: float) -> int:
    return 1 if value > 0 else (-1 if value < 0 else 0)


def classify(row: dict) -> dict | None:
    series = row.get("signed_lane_dist") or []
    baseline = float(row.get("median_signed", 0.0))
    if not series or abs(baseline) < 0.1:
        return None
    base_sign = sign(baseline)
    associated = [d for d in series if abs(d) < SATURATION_M]
    if not associated:
        return None
    # 与基准异号的步 = 车道关联翻到了对面那条车道，不进判定
    consistent = [d for d in associated if sign(d) == base_sign]
    flip_frac = 1.0 - len(consistent) / len(associated)
    detrended_peak = max((abs(d - baseline) for d in consistent), default=0.0)
    if detrended_peak >= THRESHOLD_M:
        bucket = "genuine"
    elif detrended_peak >= MARGINAL_MIN_M:
        bucket = "marginal"
    else:
        bucket = "artefact"
    return {
        "scenario_id": row["scenario_id"],
        "template": row.get("template", "lane_departure"),
        "bag": row.get("bag", ""),
        "baseline_m": round(baseline, 3),
        "detrended_peak_m": round(detrended_peak, 3),
        "peak_abs_m": round(float(row.get("max_abs", 0.0)), 3),
        "flip_frac": round(flip_frac, 3),
        "map_suspect": flip_frac > FLIP_FRAC_MAX,
        "bucket": bucket,
    }


def build(calibration_path: Path) -> dict:
    calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
    rows = []
    skipped = []
    for row in calibration["rows"]:
        classified = classify(row)
        if classified is None:
            skipped.append(row["scenario_id"])
            continue
        rows.append(classified)
    if skipped:
        raise SystemExit(f"ERROR: {len(skipped)} 个场景缺少可用基准/序列（{skipped[:3]}…）")

    counts = {bucket: sum(1 for r in rows if r["bucket"] == bucket)
              for bucket in ("genuine", "marginal", "artefact")}
    map_suspect = sum(1 for r in rows if r["map_suspect"])
    map_suspect_genuine = sum(1 for r in rows if r["map_suspect"] and r["bucket"] == "genuine")
    return {
        "version": 1,
        "basis": str(calibration_path.relative_to(REPO_ROOT)),
        "rule": (
            "对每个场景取参考轨迹 lane_dist 的中位数为基准；只统计与基准同号的步"
            "（异号 = 车道关联翻转）；相对基准的最大偏移 >= threshold_m 记为 genuine"
        ),
        "threshold_m": THRESHOLD_M,
        "marginal_min_m": MARGINAL_MIN_M,
        "flip_frac_max": FLIP_FRAC_MAX,
        "counts": {
            "scenes": len(rows),
            **counts,
            "map_suspect": map_suspect,
            "map_suspect_in_genuine": map_suspect_genuine,
        },
        "kpi": {
            "denominator": "genuine",
            "reported_separately": ["marginal"],
            "excluded": ["artefact"],
            "numerator_rule": "该场景在策略 rollout 里 lane_departure 谓词不再触发",
        },
        "scenes": rows,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibration", default=str(CALIBRATION_JSON))
    parser.add_argument("--output", default=str(OUTPUT_YAML))
    parser.add_argument("--json", default=str(OUTPUT_JSON))
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    document = build(Path(args.calibration))
    counts = document["counts"]
    print(f"场景总数          {counts['scenes']}")
    print(f"  genuine（分母）  {counts['genuine']}")
    print(f"  marginal（单列） {counts['marginal']}")
    print(f"  artefact（剔除） {counts['artefact']}")
    print(f"  map_suspect      {counts['map_suspect']}（其中落在分母里 {counts['map_suspect_in_genuine']}）")
    peaks = [r["detrended_peak_m"] for r in document["scenes"] if r["bucket"] == "genuine"]
    if peaks:
        print(f"  分母内偏移中位   {statistics.median(peaks):.3f} m"
              f"（p90 {sorted(peaks)[int(0.9 * (len(peaks) - 1))]:.3f}）")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(document, allow_unicode=True, sort_keys=False), encoding="utf-8")
    print(f"[yaml] {output}")
    if args.json:
        Path(args.json).write_text(json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"[json] {args.json}")

    failures = []
    if counts["genuine"] + counts["marginal"] + counts["artefact"] != counts["scenes"]:
        failures.append("分桶之和 != 场景总数")
    if counts["genuine"] == 0:
        failures.append("genuine 桶为空 —— 判据或基准算出错了")
    for row in document["scenes"]:
        if row["bucket"] == "genuine" and row["detrended_peak_m"] < THRESHOLD_M:
            failures.append(f"{row['scenario_id']}: genuine 桶但偏移 < 门限")
    for failure in failures:
        print(f"GATE FAIL : {failure}")
    print(f"GATE      : {'FAIL' if failures else 'PASS'}")
    return 1 if (args.check and failures) else 0


if __name__ == "__main__":
    raise SystemExit(main())
