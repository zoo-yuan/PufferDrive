"""bad case 分段（ADR-0012）与 KPI 选点的契约。

背景：原来的 154 是**绝对判据** `|lane_dist| > 0.9 m` 挑出来的，但参考司机本身就偏 0.80 m，
于是 50 个"bad case"的真实偏移只有 0.2–0.3 m（假 case）、82 个场景的车道关联中途变号。
这两条都会让"修复率"的分母失真，所以分母必须是显式文件而不是脚本里的常量。

这里钉住三件事：
1. `spec/badcase_set_v1.yaml` 的分桶自洽（计数对得上、genuine 桶确实过门限）；
2. 假 case（artefact）不允许进 KPI 分母；
3. KPI 排序把"趴窝"的 checkpoint 沉底（ADR-0010 的前科：不动车会让违规数塌下来）。
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

BADCASE_SET = REPO_ROOT / "spec" / "badcase_set_v1.yaml"

pytestmark = pytest.mark.skipif(not BADCASE_SET.exists(),
                               reason="spec/badcase_set_v1.yaml 未生成（跑 scripts/scene_badcase_set.py）")


def load_set() -> dict:
    return yaml.safe_load(BADCASE_SET.read_text(encoding="utf-8"))


def test_bucket_counts_are_self_consistent():
    document = load_set()
    counts = document["counts"]
    assert counts["genuine"] + counts["marginal"] + counts["artefact"] == counts["scenes"]
    buckets = [row["bucket"] for row in document["scenes"]]
    assert len(buckets) == counts["scenes"]
    assert set(buckets) == {"genuine", "marginal", "artefact"}


def test_genuine_bucket_actually_crosses_the_threshold():
    document = load_set()
    threshold = document["threshold_m"]
    for row in document["scenes"]:
        if row["bucket"] == "genuine":
            assert row["detrended_peak_m"] >= threshold, row["scenario_id"]
        elif row["bucket"] == "marginal":
            assert document["marginal_min_m"] <= row["detrended_peak_m"] < threshold, row["scenario_id"]
        else:
            assert row["detrended_peak_m"] < document["marginal_min_m"], row["scenario_id"]


def test_every_scene_id_is_unique():
    document = load_set()
    ids = [row["scenario_id"] for row in document["scenes"]]
    assert len(ids) == len(set(ids))


def test_artefacts_are_never_the_denominator():
    document = load_set()
    assert document["kpi"]["denominator"] == "genuine"
    assert "artefact" in document["kpi"]["excluded"]
    assert "artefact" not in document["kpi"]["reported_separately"]


def test_parked_checkpoints_rank_last():
    """KPI 排序必须把不动车的点沉底，否则"违规数塌下来"的趴窝权重会当选。"""
    from kpi_select import rank_key

    parked = {"violations": 1, "median_peak_m": 0.2, "median_speed_mps": 0.05}
    driving = {"violations": 36, "median_peak_m": 0.9, "median_speed_mps": 2.0}
    assert rank_key(parked, min_speed=1.0) > rank_key(driving, min_speed=1.0)


def test_fewer_violations_then_smaller_peak_wins():
    from kpi_select import rank_key

    better = {"violations": 20, "median_peak_m": 1.2, "median_speed_mps": 2.0}
    worse = {"violations": 36, "median_peak_m": 0.8, "median_speed_mps": 2.5}
    assert rank_key(better, min_speed=1.0) < rank_key(worse, min_speed=1.0)
