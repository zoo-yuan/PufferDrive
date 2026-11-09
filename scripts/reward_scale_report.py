"""奖励尺度契约：配置里的 shaping 旋钮是否**真的生效**，以及它们的量级能否给出梯度。

为什么需要这个脚本
------------------
event_reward_v1（900k 步）收敛到"画圈"，STATE 里记的归因是"一次性罚金扛不住
逐步速度奖励流"。但把训练日志里的量级算出来就不成立：`reward_velocity` 每步上限
2.5e-3 × dt 0.3 × velocity_progress(≤1) = 7.5e-4，60 步整集最多 0.045 —— 攒不出
一次 -5.0 的罚金。真实结构是：**每步 shaping 只有 1e-3 量级，事件项是 O(1)，
而且事件在 154/154 个场景都触发（常数）→ PPO 的优势函数里没有任何可用信号**。
奖励搜索以前没人算这一步，所以才会去调 `reward_velocity`。

同时抓第二个静默坑：`reward_randomization: true` 时 `generate_reward_coefs()`
把配置里的 α 全部替换成 `REWARD_BOUNDS` 的抽样（lane_center U(2.5e-4, 7.5e-3)、
velocity 硬编码 2.5e-3 …），**配置里的这些键在训练中完全不读**。只有独立的 env
字段（`reward_lane_departure` / `reward_safety_gap` / `reward_offroad` …）才是活的。

用法
----
``python scripts/reward_scale_report.py --config pufferlib/config/puffer_drive.yaml``
``python scripts/reward_scale_report.py --config .scratch/experiments_v2a/config.yaml --check``

三条硬门（`--check` 会 FAIL 并退出 1）
------------------------------------
1. **live_coefs**：`reward_randomization` 必须为 false，否则配置旋钮是死的。
2. **lane_signal**：一集内越道 shaping 的累积量 ≥ `SCALE_RATIO_MIN` × 终止事件代价。
3. **event_dominance**：一次性越道罚不得超过密集累积量的 `EVENT_RATIO_MAX` 倍
   —— 超了就是"付一次钱之后越道免费"，实测形态见 ADR/lessons。
"""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CONSTANTS_HEADER = REPO_ROOT / "pufferlib" / "ocean" / "drive" / "constants.h"

DT_S = 0.3
EPISODE_STEPS = 60

# 代表性越道超调：谓词门限 0.9 m 之上再走 0.6 m（真越道的典型峰值区间 1.2–2.0 m）
LANE_OVERSHOOT_M = 0.6
# 报告用的"对齐"航向（lane_center 的惩罚项乘 cos_theta > 0.5 的门）
ALIGNED_COS = 1.0
# 越道罚与密集累积量的比值上限：超过它就是"一次性买断越道权"
EVENT_RATIO_MAX = 4.0
# 越道 shaping 累积量相对终止事件代价的下限
SCALE_RATIO_MIN = 0.3
# 正向通道（沿车道前进）相对"可避免的负向累积"的下限：低于它，"趴着不动"就是
# 构成本上的最优解（v3 实测：dt=0.1 + velocity 0.0025 → 中位车速 1.15 m/s 的爬行）
POSITIVE_RATIO_MIN = 0.25
# 报告用：wrong-way 场景的车速
WRONG_WAY_SPEED_MPS = 10.0
# lane_center 的 exp 衰减项系数（drive.h: `0.05f / expf(adjusted_dist - 0.5f)`）
EXP_DECAY_SCALE = 0.05
EXP_DECAY_SHIFT = 0.5


def read_threshold_m(name: str) -> float:
    """从 constants.h 读门限，避免脚本里的常数与引擎漂移。"""
    text = CONSTANTS_HEADER.read_text(encoding="utf-8")
    match = re.search(rf"#define\s+{name}\s+([0-9.]+)f", text)
    if not match:
        raise SystemExit(f"ERROR: constants.h 里找不到 {name}")
    return float(match.group(1))


def bound_range(name: str) -> tuple[float, float] | None:
    """读 REWARD_BOUNDS 里某个 coef 的抽样区间（randomization 时配置值被它替换）。"""
    text = (REPO_ROOT / "pufferlib" / "ocean" / "drive" / "drive.h").read_text(encoding="utf-8")
    match = re.search(rf"\{{\s*(-?[0-9.e-]+)f,\s*(-?[0-9.e-]+)f,\s*[01]\s*\}},\s*//\s*{name}\b", text)
    if not match:
        return None
    return float(match.group(1)), float(match.group(2))


def lane_center_penalty_per_step(coef: float, lateral_m: float, dt: float) -> float:
    """复刻 drive.h compute_rewards 的 Rl-center 项（航向对齐时 cos_theta > 0.5 成立）。"""
    decay = EXP_DECAY_SCALE / math.exp(abs(lateral_m) - EXP_DECAY_SHIFT)
    return -coef * dt * (abs(lateral_m) - decay)


def analyze(env_config: dict) -> dict:
    dt = float(env_config.get("dt", DT_S))
    steps = int(env_config.get("scenario_length", EPISODE_STEPS))
    randomization = bool(env_config.get("reward_randomization", False))

    threshold_m = read_threshold_m("LANE_DEPARTURE_EVENT_THRESHOLD_M")
    lateral_m = threshold_m + LANE_OVERSHOOT_M

    lane_center_coef = float(env_config.get("reward_lane_center", 0.0))
    lane_align_coef = float(env_config.get("reward_lane_align", 0.0))
    velocity_coef = float(env_config.get("reward_velocity", 0.0))

    lane_center_total = lane_center_penalty_per_step(lane_center_coef, lateral_m, dt) * steps
    # velocity_progress = max(cos_theta, 0) ∈ [0, 1]，整集上限
    velocity_total = velocity_coef * dt * 1.0 * steps
    # Rl-align 的对齐分量（against 与 vel_align 项在对齐时都是 0）
    lane_align_aligned_total = lane_align_coef * dt * (0.0025 * (1.0 - 0.0)) * steps
    lane_align_wrong_way_total = lane_align_coef * dt * (-WRONG_WAY_SPEED_MPS) * steps

    offroad_event = -float(env_config.get("reward_offroad", 0.0))
    lane_departure_event = -float(env_config.get("reward_lane_departure", 0.0))

    dense_lane_total = abs(lane_center_total) + abs(lane_align_aligned_total)
    lane_signal_ratio = dense_lane_total / abs(offroad_event) if offroad_event else float("inf")
    event_ratio = abs(lane_departure_event) / dense_lane_total if dense_lane_total else float("inf")
    # 正向通道：velocity_progress 只在 sim_speed > 2.5 m/s 且有车道关联时才 >0，
    # 所以它奖励的是"沿车道开起来"，站着不动拿不到（不能靠墙速刷）。
    positive_total = velocity_total + abs(lane_align_aligned_total)
    positive_ratio = positive_total / dense_lane_total if dense_lane_total else float("inf")

    failures = []
    if randomization:
        candidates = []
        for name in ("REWARD_COEF_LANE_CENTER", "REWARD_COEF_VELOCITY", "REWARD_COEF_OFFROAD"):
            span = bound_range(name)
            if span:
                candidates.append(f"{name.replace('REWARD_COEF_', '').lower()} "
                                  f"∈ [{span[0]:g}, {span[1]:g}]")
        failures.append(
            "live_coefs: reward_randomization=true —— 配置里的 α 不生效，训练用的是 "
            "REWARD_BOUNDS 抽样（" + "，".join(candidates) + "）；"
            "reward_velocity 更是硬编码 2.5e-3。调参前必须关掉 randomization"
        )
    if lane_signal_ratio < SCALE_RATIO_MIN:
        failures.append(
            f"lane_signal: 越道 shaping 一集只累积 {dense_lane_total:.4f}，"
            f"是终止事件代价 {abs(offroad_event):.2f} 的 {lane_signal_ratio:.3f} 倍"
            f"（下限 {SCALE_RATIO_MIN}）—— 越道几乎免费"
        )
    if lane_departure_event and event_ratio > EVENT_RATIO_MAX:
        failures.append(
            f"event_dominance: 一次性越道罚 {abs(lane_departure_event):.2f} 是密集累积量 "
            f"{dense_lane_total:.4f} 的 {event_ratio:.0f} 倍（上限 {EVENT_RATIO_MAX:.0f}）"
            "—— 付一次钱之后越道免费，优势函数里被常数吃掉"
        )
    if positive_ratio < POSITIVE_RATIO_MIN:
        failures.append(
            f"positive_channel: 一集可赚的正向累积只有 {positive_total:.4f}，是越道累积量 "
            f"{dense_lane_total:.4f} 的 {positive_ratio:.3f} 倍（下限 {POSITIVE_RATIO_MIN}）"
            "—— 站着不动比开起来更划算，策略会爬行（趴窝假修复的另一种形态）"
        )

    return {
        "config_randomization": randomization,
        "dt_s": dt,
        "episode_steps": steps,
        "lane_departure_threshold_m": threshold_m,
        "probe_lateral_m": lateral_m,
        "lane_center_per_step": lane_center_penalty_per_step(lane_center_coef, lateral_m, dt),
        "lane_center_total": lane_center_total,
        "lane_align_aligned_total": lane_align_aligned_total,
        "lane_align_wrong_way_total": lane_align_wrong_way_total,
        "velocity_total": velocity_total,
        "offroad_event": offroad_event,
        "lane_departure_event": lane_departure_event,
        "dense_lane_total": dense_lane_total,
        "lane_signal_ratio": lane_signal_ratio,
        "event_ratio": event_ratio,
        "positive_total": positive_total,
        "positive_ratio": positive_ratio,
        "failures": failures,
    }


def render(report: dict) -> str:
    def row(label, value):
        return f"  {label:<34} {value}"

    lines = [
        f"配置：reward_randomization={report['config_randomization']} · "
        f"dt={report['dt_s']:.2f}s · 一集 {report['episode_steps']} 步",
        f"探针：越道 {report['probe_lateral_m']:.1f} m"
        f"（门限 {report['lane_departure_threshold_m']:.1f} m 之上 0.6 m）",
        row("lane_center / 步", f"{report['lane_center_per_step']:+.5f}"),
        row("lane_center / 集（密集）", f"{report['lane_center_total']:+.4f}"),
        row("lane_align 对齐 / 集", f"{report['lane_align_aligned_total']:+.4f}"),
        row("lane_align 逆行 / 集", f"{report['lane_align_wrong_way_total']:+.4f}"),
        row("velocity / 集（上限）", f"{report['velocity_total']:+.4f}"),
        row("offroad 终止事件", f"{report['offroad_event']:+.4f}"),
        row("lane_departure 事件", f"{report['lane_departure_event']:+.4f}"),
        row("密集越道累积 / 终止事件", f"{report['lane_signal_ratio']:.3f}"),
        f"  一次性罚 / 密集累积{'':<12} {report['event_ratio']:.1f}",
        row("正向累积 / 越道累积", f"{report['positive_ratio']:.3f}"),
    ]
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="pufferlib/config/puffer_drive.yaml",
                        help="base yaml 或训练快照（.scratch/experiments_*/config.yaml）")
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--json", default=None)
    args = parser.parse_args()

    path = Path(args.config)
    if not path.is_absolute():
        path = REPO_ROOT / path
    document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    env_config = document.get("env") or document

    report = analyze(env_config)
    report["config"] = str(path)
    print(f"[config] {path}")
    print(render(report))

    for failure in report["failures"]:
        print(f"GATE FAIL : {failure}")
    print(f"GATE      : {'FAIL' if report['failures'] else 'PASS'}")

    if args.json:
        output = Path(args.json)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"[json] {output}")
    return 1 if (args.check and report["failures"]) else 0


if __name__ == "__main__":
    raise SystemExit(main())
