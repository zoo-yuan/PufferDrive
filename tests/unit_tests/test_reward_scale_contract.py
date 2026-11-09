"""奖励尺度契约：调参前必须证明旋钮是活的、且密集项与事件项同量级。

背景（docs/STATE.md P0 / lessons 末两条）
---------------------------------------
event_reward_v1 一次性越道罚 -5.0 + 每步速度奖励，900k 步学到的是"画圈"。
原先的归因是"罚金扛不住逐步奖励流"，但把量级算出来就不成立：`reward_velocity`
整集上限 0.045，攒不出一次 -5.0。真实结构是**密集项只有 1e-3 量级、事件项 O(1)
且在 154/154 个场景都触发（常数）** → 优势函数里没有信号。

这些测试钉住四件事：
1. `reward_randomization: true` 时配置里的 α 被 REWARD_BOUNDS 抽样替换 → 旋钮是死的。
2. 只改 `reward_velocity`（旧计划①）不足以通过尺度门 —— 这条测试就是旧计划的否证。
3. 横向尺度修好之后还有第二道门：**正向通道**。没有"开起来才拿得到"的奖励，
   "趴着不动"就是构成本上的最优解（v2a/v3 实测都落在这一条）。
4. 四条门全过才算候选（v4）。
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from reward_scale_report import analyze  # noqa: E402

BASE_ENV = {
    "dt": 0.3,
    "scenario_length": 60,
    "reward_randomization": True,
    "reward_lane_center": 0.001,
    "reward_lane_align": 0.025,
    "reward_velocity": 0.0025,
    "reward_offroad": 1.5,
    "reward_lane_departure": 0.0,
}


def failed_checks(report):
    """把失败信息归约成 check 名，断言才读得懂。"""
    return {failure.split(":")[0] for failure in report["failures"]}


def test_randomization_makes_config_coefficients_inert():
    """randomization=true 时 lane_center 0.001 根本没进训练 —— 必须报 live_coefs。"""
    report = analyze({**BASE_ENV})
    assert "live_coefs" in failed_checks(report)


def test_one_shot_penalty_with_tiny_dense_signal_fails_both_gates():
    """复刻 event_reward_v1：-5.0 一次性罚 + 0.001 lane_center。"""
    report = analyze({**BASE_ENV, "reward_randomization": False, "reward_lane_departure": 5.0})
    assert failed_checks(report) == {"lane_signal", "event_dominance"}
    # 罚金是密集累积量的两个数量级以上：付一次钱之后越道免费
    assert report["event_ratio"] > 100


def test_lowering_velocity_alone_is_not_a_fix():
    """旧计划①（reward_velocity 0.0025 → 0.0005）单独改不通过尺度门。

    velocity 整集上限本来就只有 0.045，改它不改变"越道几乎免费"这个结构。
    """
    report = analyze({
        **BASE_ENV,
        "reward_randomization": False,
        "reward_velocity": 0.0005,
        "reward_lane_departure": 5.0,
    })
    assert "lane_signal" in failed_checks(report)
    assert "event_dominance" in failed_checks(report)
    assert report["velocity_total"] < 0.05


def test_v2a_fixes_lateral_scale_but_starves_the_positive_channel():
    """v2a（关 randomization、删一次性罚、lane_center 0.04，dt 0.3）——实测结果：
    违规 154/154 无改善。契约在这里只抓到 positive_channel，而 v2a 训练后
    `num_goals_reached` 恒 0、只剩 0.045 的正向，正是"没有理由开起来"的前兆。
    """
    report = analyze({
        **BASE_ENV,
        "reward_randomization": False,
        "reward_lane_center": 0.04,
        "reward_lane_departure": 0.0,
    })
    failed = failed_checks(report)
    assert failed == {"positive_channel"}
    assert report["lane_signal_ratio"] >= 0.3  # 横向尺度确实修好了


def test_v3_dt_01_creep_signature_is_predicted_by_the_gate():
    """v3（dt 0.1）实测中位车速 1.15 m/s、违规 141/154：dt 变小把密集项一起摊薄，
    正向通道只剩 0.015。这道门在训练前就把"爬行"预测出来了。
    """
    report = analyze({
        **BASE_ENV,
        "dt": 0.1,
        "reward_randomization": False,
        "reward_lane_center": 0.08,
        "reward_lane_departure": 0.0,
    })
    assert failed_checks(report) == {"positive_channel"}
    assert report["positive_total"] < 0.05


def test_v4_candidate_passes_every_gate():
    """v4：dt 0.1 + 抬高 lane_center 与 velocity（velocity_progress 带 2.5 m/s 门，
    所以奖励的是"沿车道开起来"，站着不动拿不到）。"""
    report = analyze({
        **BASE_ENV,
        "dt": 0.1,
        "reward_randomization": False,
        "reward_lane_center": 0.08,
        "reward_lane_departure": 0.0,
        "reward_velocity": 0.05,
    })
    assert report["failures"] == []
    assert report["positive_ratio"] >= 0.25


def test_lane_center_penalty_matches_engine_formula():
    """复刻 drive.h 的 Rl-center：-α·dt·(|x| - 0.05/exp(|x|-0.5))，航向对齐时。"""
    import math

    report = analyze({**BASE_ENV, "reward_randomization": False, "reward_lane_center": 0.04})
    expected = -0.04 * 0.3 * (1.5 - 0.05 / math.exp(1.5 - 0.5))
    assert report["lane_center_per_step"] == expected
