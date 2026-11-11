"""把 checkpoint 的真实 rollout 渲染成 GIF（俯视 + 车道 + 轨迹尾迹 + HUD）。

为什么单独做这个脚本
--------------------
所有判断都在数字上做（违规数、中位 `lane_dist`、车速），但"策略到底在怎么开"只能看。
2026-11-06 会话③ 里两次靠肉眼轨迹推翻数字结论（"画圈刷速度"、v8 在 artefact 桶 5.4 m/s 冲出去），
所以把这条渲染路径固化成脚本，和 `kpi_select.py` 的 KPI 一起用。

三列对照（默认各取一个场景）
--------------------------
`genuine`（真越道，KPI 分母）/ `marginal`（边缘）/ `artefact`（假 bad case）——
同一份权重在这三个桶里的速度与轨迹形态完全不同，一张图就能看出来。

用法
----
``python scripts/rollout_gif.py --run-dir .scratch/experiments_v8 --checkpoint best_kpi.pt``
``python scripts/rollout_gif.py --scene-ids 3123#1790500870141#lane_departure ...``
"""

from __future__ import annotations

import argparse
import io
import math
import statistics
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as mpatches  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
import yaml  # noqa: E402
from PIL import Image  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import regression_retention as harness  # noqa: E402
from data_utils.mirror_map_bin import read_bin  # noqa: E402
from scene_feasibility import get_metric_names, state_row  # noqa: E402

SCENE_DIR = REPO_ROOT / ".scratch" / "crossbag_scenes"
BADCASE_SET = REPO_ROOT / "spec" / "badcase_set_v1.yaml"
STEPS = 60
FRAME_MS = 120
# 每列画多大范围（米）：以 ego 起点为中心
WINDOW_M = 46.0
LANE_COLOR = "#cfd4dc"
EDGE_COLOR = "#9aa3ad"
REF_COLOR = "#f59e0b"
POLICY_COLOR = "#2563eb"


def pick_scenes(badcase_set: Path, buckets: list[str]) -> list[dict]:
    document = yaml.safe_load(badcase_set.read_text(encoding="utf-8"))
    picked = []
    for bucket in buckets:
        candidates = [row for row in document["scenes"] if row["bucket"] == bucket]
        if not candidates:
            continue
        picked.append(candidates[0])
    return picked


def rollout(scene_path: Path, policy, metric_names, metric_index, device) -> dict:
    env = harness.build_env(scene_path)
    obs, _ = env.reset(seed=0)
    frames = []
    goal = None
    for step in range(STEPS):
        tensor = torch.as_tensor(np.asarray(obs), dtype=torch.float32, device=device)
        with torch.no_grad():
            logits, _value = policy.forward_eval(tensor, {})
            action = harness.select_eval_action(policy, logits).view(env.num_agents, -1)
            action = action.cpu().numpy().astype(np.float32)
        obs, _reward, terminal, truncation, _info = env.step(action)
        state = env.get_state()
        agent = (state if isinstance(state, list) else [state])[0]["agents"][0]
        row = state_row(agent, metric_names, metric_index)
        if goal is None:
            goal = (agent["current_goal_x"], agent["current_goal_y"])
        frames.append({
            "step": step,
            "x": float(agent["sim_x"]),
            "y": float(agent["sim_y"]),
            "heading": float(agent["sim_heading"]),
            "speed": float(row.get("sim_speed", 0.0)),
            "lane_dist": float(row.get("lane_dist", 0.0)),
            "length": float(agent.get("sim_length", 4.0)),
            "width": float(agent.get("sim_width", 1.8)),
            "offroad": bool(row.get("offroad", 0.0) > 0.5),
        })
        if bool(np.any(terminal)) or bool(np.any(truncation)):
            break
    return {"frames": frames, "goal": goal}


def render_panel(axis, scene_row, rollout, lanes, reference, title):
    frames = rollout["frames"]
    ego = frames[-1]  # 镜头跟随：每帧以当前自车为中心，世界在滚动
    window = WINDOW_M / 2.0
    axis.set_xlim(ego["x"] - window, ego["x"] + window)
    axis.set_ylim(ego["y"] - window, ego["y"] + window)
    axis.set_aspect("equal")
    axis.set_facecolor("#f7f8fa")
    axis.tick_params(labelbottom=False, labelleft=False, length=0)
    for spine in axis.spines.values():
        spine.set_color("#d7dbe0")

    for lane in lanes:
        axis.plot(lane[0], lane[1], color=LANE_COLOR, linewidth=1.2, solid_capstyle="round", zorder=1)
        axis.plot(lane[0], lane[1], color=EDGE_COLOR, linewidth=0.4, alpha=0.6, zorder=1)
    if reference:
        axis.plot(reference[0], reference[1], color=REF_COLOR, linewidth=1.6, linestyle="--",
                  alpha=0.85, zorder=2, label="recorded driver")
    if rollout["goal"]:
        axis.plot(*rollout["goal"], marker="*", markersize=13, color="#16a34a", zorder=4,
                  markeredgecolor="white", markeredgewidth=0.6)

    policy_x = [f["x"] for f in frames]
    policy_y = [f["y"] for f in frames]
    axis.plot(policy_x, policy_y, color=POLICY_COLOR, linewidth=1.6, alpha=0.35, zorder=3)

    last = frames[-1]
    body = mpatches.Rectangle(
        (last["x"] - last["length"] / 2.0, last["y"] - last["width"] / 2.0),
        last["length"], last["width"], angle=math.degrees(last["heading"]),
        rotation_point="center", facecolor=POLICY_COLOR if not last["offroad"] else "#dc2626",
        edgecolor="white", linewidth=1.0, zorder=6,
    )
    axis.add_patch(body)
    axis.plot([last["x"], last["x"] + 4.0 * math.cos(last["heading"])],
              [last["y"], last["y"] + 4.0 * math.sin(last["heading"])],
              color="white", linewidth=1.0, zorder=7)

    axis.set_title(title, fontsize=11, pad=6)
    peak = max(abs(f["lane_dist"]) for f in frames)
    axis.text(
        0.02, 0.02,
        f"step {last['step'] + 1:02d}/{STEPS}   v={last['speed']:.2f} m/s\n"
        f"lane_dist={last['lane_dist']:+.2f} m   peak={peak:.2f} m"
        + ("\nOFFROAD" if last["offroad"] else ""),
        transform=axis.transAxes, fontsize=8.5, family="monospace",
        va="bottom", ha="left", color="#111827",
        bbox=dict(facecolor="white", alpha=0.85, edgecolor="#e5e7eb", boxstyle="round,pad=0.35"),
    )


def collect_lanes(scene_path: Path, center: tuple[float, float], radius: float) -> list[tuple[list, list]]:
    roads = read_bin(scene_path)["roads"]
    cx, cy = center
    lanes = []
    for road in roads:
        xs = list(road["x"])
        ys = list(road["y"])
        if not xs:
            continue
        if min(math.hypot(x - cx, y - cy) for x, y in zip(xs, ys)) > radius * 1.6:
            continue
        lanes.append((xs, ys))
    return lanes


def reference_path(scene_path: Path) -> tuple[list, list]:
    agent = read_bin(scene_path)["agents"][0]
    cols = agent["cols"]
    return list(cols["x"][:STEPS]), list(cols["y"][:STEPS])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", default=".scratch/experiments_v8")
    parser.add_argument("--checkpoint", default="best_kpi.pt")
    parser.add_argument("--scene-dir", default=str(SCENE_DIR))
    parser.add_argument("--badcase-set", default=str(BADCASE_SET))
    parser.add_argument("--buckets", default="genuine,marginal,artefact")
    parser.add_argument("--scene-ids", default=None,
                        help="逗号分隔；给了就忽略 --buckets")
    parser.add_argument("--out", default="docs/html/best_model_rollout.gif")
    parser.add_argument("--dpi", type=int, default=90)
    args = parser.parse_args()

    run_dir = Path(args.run_dir).resolve()
    checkpoint = (run_dir / args.checkpoint).resolve()
    if not checkpoint.exists():
        print(f"ERROR: 找不到 {checkpoint}", file=sys.stderr)
        return 1
    harness.RUN_CONFIG = run_dir / "config.yaml"
    scene_dir = Path(args.scene_dir)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    metric_names, metric_index = get_metric_names()

    if args.scene_ids:
        buckets = None
        wanted = [s.strip() for s in args.scene_ids.split(",") if s.strip()]
        document = yaml.safe_load(Path(args.badcase_set).read_text(encoding="utf-8"))
        meta = {row["scenario_id"]: row for row in document["scenes"]}
        picks = [meta[sid] for sid in wanted]
    else:
        buckets = [b.strip() for b in args.buckets.split(",") if b.strip()]
        picks = pick_scenes(Path(args.badcase_set), buckets)

    first = scene_dir / f"{picks[0]['scenario_id'].replace('#', '_')}.bin"
    policy = harness.build_policy(harness.build_env(first), device)
    harness.load_weights(policy, checkpoint, device)
    print(f"[model] {checkpoint} · {len(picks)} 个场景", flush=True)

    panels = []
    for row in picks:
        scene_path = scene_dir / f"{row['scenario_id'].replace('#', '_')}.bin"
        rollout_result = rollout(scene_path, policy, metric_names, metric_index, device)
        xs = [f["x"] for f in rollout_result["frames"]]
        ys = [f["y"] for f in rollout_result["frames"]]
        path_center = ((min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0)
        reach = max(max(xs) - min(xs), max(ys) - min(ys)) / 2.0 + WINDOW_M
        lanes = collect_lanes(scene_path, path_center, reach)
        panels.append({
            "row": row,
            "scene_path": scene_path,
            "rollout": rollout_result,
            "lanes": lanes,
            "reference": reference_path(scene_path),
        })
        frames = rollout_result["frames"]
        speeds = [f["speed"] for f in frames]
        peak = max(abs(f["lane_dist"]) for f in frames)
        print(f"  {row['bucket']:<9} {row['scenario_id']:<34} steps {len(frames):>2} · "
              f"峰值 {peak:.2f} m · 车速中位 {statistics.median(speeds):.2f} m/s", flush=True)

    # 统一帧数：短的场景停在最后一帧
    total_frames = max(len(p["rollout"]["frames"]) for p in panels)
    figure, axes = plt.subplots(1, len(panels), figsize=(5.2 * len(panels), 5.6), dpi=args.dpi)
    if len(panels) == 1:
        axes = [axes]
    figure.patch.set_facecolor("white")
    figure.subplots_adjust(left=0.02, right=0.98, top=0.9, bottom=0.03, wspace=0.04)

    images: list[Image.Image] = []
    for frame_index in range(total_frames):
        for axis, panel in zip(axes, panels):
            axis.clear()
            frames = panel["rollout"]["frames"]
            visible = frames[: frame_index + 1]
            partial = {"frames": visible, "goal": panel["rollout"]["goal"]}
            row = panel["row"]
            title = (f"{row['bucket']} · {row['bag']} · baseline {row['baseline_m']:+.2f} m"
                     f"{' · map_suspect' if row.get('map_suspect') else ''}")
            render_panel(axis, row, partial, panel["lanes"], panel["reference"], title)
        figure.suptitle(
            f"best KPI checkpoint rollout · {checkpoint.name} · {run_dir.name}",
            fontsize=12, y=0.965, color="#111827")
        buffer = io.BytesIO()
        figure.savefig(buffer, format="png")
        buffer.seek(0)
        images.append(Image.open(buffer).convert("P", palette=Image.ADAPTIVE, colors=128))
        print(f"  frame {frame_index + 1}/{total_frames}", end="\r", flush=True)

    out = REPO_ROOT / args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    images[0].save(out, save_all=True, append_images=images[1:], duration=FRAME_MS, loop=0, optimize=True)
    print(f"\n[gif] {out}  ({out.stat().st_size / 1e6:.1f} MB · {len(images)} 帧)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
