"""回归保留率（M7-4）：训练前后，同一批 bad case 还失不失败？

两个 KPI（ADR-0007 §7）
----------------------
- **修复率** = 训练前违规、训练后不违规的占比
- **回归保留率** = 训练前不违规的场景里，训练后仍不违规的占比（老 case 是否被改坏）

评测口径和 M7-3 完全相同：同一批场景 bin、同一条约束谓词（`spec/constraints_v0.yaml`）、
同一步数、确定性动作（取策略分布的均值，不采样）。唯一变化的是策略权重。

用法
----
``python scripts/regression_retention.py``                                  # 基线 vs experiments/final_model.pt
``python scripts/regression_retention.py --checkpoint experiments/models/model_puffer_drive_000007.pt``
``python scripts/regression_retention.py --baseline-only``                  # 只看基线
"""

from __future__ import annotations

import argparse
import html
import json
import statistics
import sys
from pathlib import Path

import numpy as np
import torch
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from constraint_judge import as_predicate_list, evaluate, guard_holds  # noqa: E402
from data_utils.mirror_map_bin import read_bin  # noqa: E402
from scene_feasibility import (  # noqa: E402
    DT_S,
    WARMUP_STEPS,
    get_metric_names,
    load_template,
    state_row,
)

FEASIBILITY_JSON = REPO_ROOT / "docs" / "measured" / "scene_feasibility_v0.json"
# 训练用场景目录（mark_as_expert=0、带限速区段）；与训练时的 env.map_dir 保持一致
SCENE_DIR = REPO_ROOT / ".scratch" / "retention_scenes"
CONFIG_PATH = REPO_ROOT / "pufferlib" / "config" / "puffer_drive.yaml"
# 训练时的 env 配置快照：观测特征开关会改变网络输入维度（partner 11 vs 9、context 27 vs 9），
# 用默认参数搭 env 会让 checkpoint 加载不上。
RUN_CONFIG = REPO_ROOT / "experiments" / "config.yaml"
OUTPUT_HTML = REPO_ROOT / "docs" / "html" / "regression_retention_v0.html"
OUTPUT_JSON = REPO_ROOT / "docs" / "measured" / "regression_retention_v0.json"

STEPS = 60
RETENTION_EVAL_START_STEP = 0
SCENE_LENGTH = 60
EVAL_SEED = 0


def scene_ids():
    """跨 bag 读取：本会话为 6 条 bag 各存了一份 feasibility 结果（.scratch/feas/*.json）。
    单 bag 的旧文件仍然作为兜底。"""
    per_bag = sorted((REPO_ROOT / ".scratch" / "feas").glob("*.json"))
    rows = []
    for path in per_bag:
        if path.name == "avoidable_by_bag.json":
            continue
        data = json.loads(path.read_text(encoding="utf-8"))
        for row in data.get("rows", []):
            if row.get("verdict") == "reproduced_avoidable":
                rows.append({"scenario_id": row["scenario_id"], "template": row["template"],
                             "bag": path.stem})
    partner_path = REPO_ROOT / "docs" / "measured" / "partner_scenes_v0.json"
    if partner_path.exists():
        partner_data = json.loads(partner_path.read_text(encoding="utf-8"))
        for row in partner_data.get("rows", []):
            if row.get("fired"):
                rows.append({"scenario_id": row["scenario_id"], "template": row["template"],
                             "bag": row.get("bag", "partner"), "source": "partner"})
    if rows:
        return rows
    data = json.loads(FEASIBILITY_JSON.read_text(encoding="utf-8"))["rows"]
    return [{"scenario_id": row["scenario_id"], "template": row["template"], "bag": "single"}
            for row in data if row["verdict"] == "reproduced_avoidable"]


def training_env_config():
    """复用训练那次实验的 env 段（观测特征、奖励开关必须一致，否则网络输入维度对不上）。"""
    import inspect

    from pufferlib.ocean.drive.drive import Drive

    if not RUN_CONFIG.exists():
        return {}
    config = (yaml.safe_load(RUN_CONFIG.read_text(encoding="utf-8")) or {}).get("env") or {}
    parameters = set(inspect.signature(Drive.__init__).parameters)
    return {key: value for key, value in config.items() if key in parameters}


def build_replay_env(scene_path: Path, env_config=None):
    """参考零点：replay 控制器按 log 开。它必然违规（场景就是这么选出来的），
    用来确认"谓词 + 场景"这一侧没问题，剩下的差异才归因于策略。"""
    from pufferlib.ocean.drive.drive import Drive

    kwargs = dict(training_env_config() if env_config is None else env_config)
    kwargs.update(
        map_dir=str(scene_path),
        num_maps=1,
        num_agents=1,
        min_agents_per_env=1,
        max_agents_per_env=1,
        sdc_controller="replay",
        replay_expert_agents=0,
        dt=DT_S,
        scenario_length=SCENE_LENGTH,
        seed=EVAL_SEED,
    )
    return Drive(**kwargs)


def build_env(scene_path: Path, env_config=None):
    """策略评测 env：除场景/种子/控制者外，其余全部继承训练快照
    （experiments/config.yaml 的 env 段）。

    **dt 不许覆盖**：训练在 dt=0.3 上积分，错用 0.1 会把同一权重的车速
    低估约 8 倍（实测中位 2.44 → 0.38 m/s）。"""
    from pufferlib.ocean.drive.drive import Drive

    kwargs = dict(training_env_config() if env_config is None else env_config)
    agent_count = max(1, len(read_bin(scene_path)["agents"]))
    kwargs.update(
        map_dir=str(scene_path),
        num_maps=1,
        num_agents=agent_count,
        min_agents_per_env=1,
        max_agents_per_env=agent_count,
        sdc_controller="policy",
        replay_expert_agents=0,
        scenario_length=SCENE_LENGTH,
        seed=EVAL_SEED,
    )
    return Drive(**kwargs)


def build_policy(env, device):
    """按 config 的 policy 段构建网络；与训练用的是同一个类（pufferlib.ocean.torch.Drive）。"""
    from pufferlib import ocean

    # 权重初始化必须可复现：否则同一份配置两次跑出的"基线违规数"不同（实测 58 / 66），
    # 指标就不可审计了。
    torch.manual_seed(EVAL_SEED)
    policy_config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))["policy"]
    policy = getattr(ocean.torch, "Drive")(env, **policy_config)
    return policy.to(device).eval()


def load_weights(policy, checkpoint: Path, device):
    from pufferlib.pufferl import clean_policy_state_dict

    state = torch.load(str(checkpoint), map_location=device)
    if isinstance(state, dict) and "policy_state_dict" in state:
        state = state["policy_state_dict"]
    policy.load_state_dict(clean_policy_state_dict(state))


def evaluate_replay(scene_path: Path, template, metric_names, metric_index):
    """replay 参考跑：不给策略，直接让引擎按 log 播。"""
    env = build_replay_env(scene_path)
    predicates = as_predicate_list(template["predicate"])
    params = template.get("params", {})
    guard = template.get("guard")
    metric_name = predicates[0]["metric"]
    env.reset(seed=EVAL_SEED)
    fired = False
    peak = 0.0
    offroad_steps = 0
    speeds = []
    previous = None
    for _ in range(STEPS):
        env.step(np.zeros((env.num_agents, 2), dtype=np.float32))
        state = env.get_state()
        scenarios = state if isinstance(state, list) else [state]
        for scenario in scenarios:
            for agent in scenario.get("agents") or []:
                row = state_row(agent, metric_names, metric_index)
                # replay 贴位姿、不积分，sim_speed 恒为 0；带速度门限的谓词会因此全部失效。
                # 用相邻位姿反推速度（与 M7-3 同一处理）。
                if previous is not None:
                    moved = ((agent["sim_x"] - previous[0]) ** 2 + (agent["sim_y"] - previous[1]) ** 2) ** 0.5
                    row["sim_speed"] = moved / DT_S
                previous = (agent["sim_x"], agent["sim_y"])
                peak = max(peak, abs(row.get(metric_name, 0.0)))
                speeds.append(row.get("sim_speed", 0.0))
                if row.get("offroad", 0.0) > 0.5:
                    offroad_steps += 1
                if (guard is None or guard_holds(guard, params, row)) and evaluate(predicates, params, row):
                    fired = True
    return {
        "fired": fired,
        "peak": round(peak, 3),
        "offroad_steps": offroad_steps,
        "offroad_dominant": offroad_steps > STEPS // 2,
        # 一个不动车的策略永远不会触发带速度门限的谓词 —— 这个数用来抓那种假"修复"
        "median_speed": round(statistics.median(speeds), 3) if speeds else 0.0,
    }


def select_eval_action(policy, logits):
    """训练 eval 契约（experiments/config.yaml `eval.action_selection: mean`）：
    离散策略在连续 env 上取**物理动作表的概率加权期望**。

    不能用 argmax：驻车起步时策略分布的众数压在 no-op 格（实测 ~63%），
    argmax 自锁（停着→选 no-op→还停着），同一权重训练里 6+ m/s、
    手搓 harness 里 0.00 m/s（2026-11-06 §15 事故根因）。
    """
    if isinstance(logits, torch.Tensor):
        probs = torch.softmax(logits.float(), dim=-1)
        return policy.discrete_probs_to_continuous_mean(probs)
    return logits.loc  # 连续策略：分布均值


def evaluate_scene(policy, scene_path: Path, template, metric_names, metric_index, device):
    """跑一条场景，返回该场景代表的模板是否触发（与 M7-3 同一谓词）。"""
    env = build_env(scene_path)
    predicates = as_predicate_list(template["predicate"])
    params = template.get("params", {})
    guard = template.get("guard")
    metric_name = predicates[0]["metric"]
    obs, _ = env.reset(seed=EVAL_SEED)
    fired = False
    peak = 0.0
    offroad_steps = 0
    speeds = []
    for step_index in range(STEPS):
        obs_tensor = torch.as_tensor(np.asarray(obs), dtype=torch.float32, device=device)
        with torch.no_grad():
            logits, _value = policy.forward_eval(obs_tensor, {})
            # 策略是 jerk 表上的离散 categorical（12 模态）、环境收 2 维连续动作；
            # 动作选择必须走训练 eval 契约（mean），见 select_eval_action。
            action = select_eval_action(policy, logits)
            action = action.view(env.num_agents, -1).cpu().numpy().astype(np.float32)
        obs, _reward, terminal, truncation, _info = env.step(action)
        state = env.get_state()
        scenarios = state if isinstance(state, list) else [state]
        for scenario in scenarios:
            for agent in scenario.get("agents") or []:
                row = state_row(agent, metric_names, metric_index)
                peak = max(peak, abs(row.get(metric_name, 0.0)))
                if row.get("offroad", 0.0) > 0.5:
                    offroad_steps += 1
                speeds.append(row.get("sim_speed", 0.0))
                # 不从 warmup 之后才开始判定：切片把 ego 放在违规时刻前 1.6 s，
                # 从第 0 步就判，策略才有机会"提前避开"；从违规帧才开始判，任何策略都必失败。
                if step_index < RETENTION_EVAL_START_STEP:
                    continue
                if (guard is None or guard_holds(guard, params, row)) and evaluate(predicates, params, row):
                    fired = True
        if bool(np.any(terminal)) or bool(np.any(truncation)):
            break
    # lane_dist 撞上 4.0 的归一化上限 = 车已经不在任何映射车道附近（开出地图），
    # 那不是"越道"，必须与真越道分开计。
    return {
        "fired": fired,
        "peak": round(peak, 3),
        "offroad_steps": offroad_steps,
        "offroad_dominant": offroad_steps > STEPS // 2,
        # 一个不动车的策略永远不会触发带速度门限的谓词 —— 这个数用来抓那种假"修复"
        "median_speed": round(statistics.median(speeds), 3) if speeds else 0.0,
    }


def render_html(report) -> str:
    summary = report["summary"]
    rows = "".join(
        f"<tr><td class='mono'>{html.escape(row['scenario_id'])}</td>"
        f"<td class='c'>{html.escape(row['template'])}</td>"
        f"<td class='c'>{'违规' if row['before']['fired'] else '正常'}</td>"
        f"<td class='c'>{'违规' if row['after']['fired'] else '正常'}</td>"
        f"<td class='c'>{row['after']['peak']:.2f}</td>"
        f"<td class='note'>{html.escape(row['outcome'])}</td></tr>"
        for row in report["rows"]
    )
    return f"""<!doctype html><html lang="zh"><head><meta charset="utf-8">
<title>回归保留率 · M7-4</title>
<style>
body{{font-family:-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,"PingFang SC",sans-serif;
max-width:1000px;margin:0 auto;padding:32px 24px 64px;color:#1f2328;line-height:1.6}}
h1{{font-size:24px;margin:0 0 4px}} h2{{font-size:17px;margin:30px 0 10px;padding-bottom:6px;border-bottom:1px solid #e3e3e8}}
.sub{{color:#57606a;font-size:13px;margin:0 0 18px}}
.cards{{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;margin:18px 0}}
.card{{border:1px solid #e3e3e8;border-radius:8px;padding:12px 14px;background:#fbfbfd}}
.card .k{{font-size:12px;color:#57606a}} .card .v{{font-size:22px;font-weight:600;margin-top:2px}}
.card .n{{font-size:11px;color:#8c959f;margin-top:2px}}
table{{width:100%;border-collapse:collapse;font-size:13px}}
th{{text-align:left;font-size:12px;color:#57606a;border-bottom:1px solid #d0d7de;padding:6px 8px;font-weight:600}}
td{{border-bottom:1px solid #eaeef2;padding:5px 8px}}
.c{{text-align:center}} .mono{{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:12px}}
.note{{color:#57606a;font-size:12px}}
.banner{{border-left:3px solid #1a7f37;background:#f0fbf2;padding:9px 13px;border-radius:0 6px 6px 0;font-size:13px;margin:16px 0}}
</style></head><body>
<h1>回归保留率（M7-4）</h1>
<p class="sub">{summary['scenes']} 个"复现且可避"场景 · {STEPS} 步 · 确定性动作 · 同一谓词与阈值 ·
 基线 = 随机初始化，训练后 = <code>{html.escape(str(report['checkpoint']))}</code></p>
<div class="cards">
<div class="card"><div class="k">参考零点（replay）</div><div class="v">{summary['reference_violated']}/{summary['scenes']}</div>
<div class="n">录制司机违规的场景数（场景确实含违规）</div></div>
<div class="card"><div class="k">修复率（对录制司机）</div><div class="v">{summary['fixed_vs_reference']}/{summary['reference_violated']}</div>
<div class="n">{summary['fix_rate_vs_reference'] * 100:.0f}% 的违规被策略消除</div></div>
<div class="card"><div class="k">相对上轮 checkpoint 修复</div><div class="v">{summary['fixed']}/{summary['violated_before']}</div>
<div class="n">回归语义看这一格</div></div>
<div class="card"><div class="k">仍失败</div><div class="v">{summary['still_failing']}</div>
<div class="n">两个策略都失败（诚实记 0 进展）</div></div>
<div class="card"><div class="k">回归保留率</div><div class="v">{summary['retention'] * 100:.0f}%</div>
<div class="n">训练前正常的场景里，训练后仍正常</div></div>
<div class="card"><div class="k">被改坏</div><div class="v">{summary['regressed']}</div>
<div class="n">&gt;0 是硬阻断：老 case 转红必须拒绝这次 checkpoint</div></div>
</div>
<div class="banner">判定与 M7-3 完全同口径（同一批 bin、同一条谓词、同一步数），只有权重变了。
 低修复率不是失败信号 —— 短训练预算下更重要的是**没有把老 case 改坏**。</div>
<h2>1. 逐场景</h2>
<table><thead><tr><th>scenario</th><th class="c">bag</th><th class="c">模板</th><th class="c">训练前</th>
<th class="c">训练后</th><th class="c">训练后峰值</th><th>结果</th></tr></thead>
<tbody>{rows}</tbody></table>
</body></html>"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-dir", default=str(SCENE_DIR))
    parser.add_argument("--checkpoint", default="experiments/final_model.pt")
    parser.add_argument("--baseline-checkpoint", default=None,
                        help="基线也载入某个 checkpoint（对比两次训练才有回归语义；默认随机初始化）")
    parser.add_argument("--baseline-only", action="store_true")
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    scene_dir = Path(args.scene_dir)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    metric_names, metric_index = get_metric_names()
    scenes = scene_ids()
    if args.limit:
        scenes = scenes[: args.limit]
    scenes = [scene for scene in scenes
              if (scene_dir / f"{scene['scenario_id'].replace('#', '_')}.bin").exists()]
    if not scenes:
        print("ERROR: 没有可评测场景（先跑 scripts/scene_feasibility.py）", file=sys.stderr)
        return 1

    def scene_path(scene):
        return scene_dir / f"{scene['scenario_id'].replace('#', '_')}.bin"

    first_path = scene_path(scenes[0])
    first_template = load_template(scenes[0]["template"])
    env = build_env(first_path)
    policy = build_policy(env, device)

    print(f"[reference] replay 参考跑 {len(scenes)} 个场景（确认场景里确实含违规）…", flush=True)
    reference = []
    for scene in scenes:
        template = load_template(scene["template"])
        path = scene_path(scene)
        reference.append(evaluate_replay(path, template, metric_names, metric_index))

    if args.baseline_checkpoint:
        load_weights(policy, Path(args.baseline_checkpoint), device)
        print(f"[baseline] 基线 = {args.baseline_checkpoint}", flush=True)
    else:
        print("[baseline] 基线 = 随机初始化（只说明会动/不会动，不能当回归基线）", flush=True)
    print(f"[baseline] 评估 {len(scenes)} 个场景 …", flush=True)
    before = []
    for scene in scenes:
        template = load_template(scene["template"])
        path = scene_path(scene)
        before.append(evaluate_scene(policy, path, template, metric_names, metric_index, device))

    after = before
    if not args.baseline_only:
        checkpoint = Path(args.checkpoint)
        if not checkpoint.exists():
            print(f"ERROR: 找不到 checkpoint {checkpoint}（先训练，或传 --checkpoint）", file=sys.stderr)
            return 1
        load_weights(policy, checkpoint, device)
        print(f"[trained] 载入 {checkpoint} 后重评同一批场景 …", flush=True)
        after = []
        for scene in scenes:
            template = load_template(scene["template"])
            path = scene_path(scene)
            after.append(evaluate_scene(policy, path, template, metric_names, metric_index, device))

    rows = []
    for scene, reference_row, before_row, after_row in zip(scenes, reference, before, after):
        if before_row["fired"] and not after_row["fired"]:
            outcome = "fixed"
        elif before_row["fired"] and after_row["fired"]:
            outcome = "still_failing"
        elif not before_row["fired"] and after_row["fired"]:
            outcome = "regressed"
        else:
            outcome = "clean"
        rows.append({**scene, "reference": reference_row, "before": before_row,
                     "after": after_row, "outcome": outcome})

    violated_before = sum(1 for row in rows if row["before"]["fired"])
    clean_before = len(rows) - violated_before
    fixed = sum(1 for row in rows if row["outcome"] == "fixed")
    regressed = sum(1 for row in rows if row["outcome"] == "regressed")
    still = sum(1 for row in rows if row["outcome"] == "still_failing")
    summary = {
        "scenes": len(rows),
        "reference_violated": sum(1 for row in rows if row["reference"]["fired"]),
        "violated_before": violated_before,
        "clean_before": clean_before,
        "fixed": fixed,
        "still_failing": still,
        "regressed": regressed,
        # 基线不动的场景（速度门限挡住）不能算"正常"，否则修复率会被灌水
        "baseline_moved": sum(1 for row in rows if row["before"]["peak"] > 0.5),
        "retention": (1.0 - regressed / clean_before) if clean_before else 1.0,
        "fix_rate": (fixed / violated_before) if violated_before else 0.0,
        # 相对录制司机（场景的真实零点）的修复率：这才是"策略有没有学会"的主指标
        "fixed_vs_reference": sum(
            1 for row in rows if row["reference"]["fired"] and not row["after"]["fired"]
        ),
        "fix_rate_vs_reference": (
            sum(1 for row in rows if row["reference"]["fired"] and not row["after"]["fired"])
            / max(1, sum(1 for row in rows if row["reference"]["fired"]))
        ),
        "median_peak_after": statistics.median([row["after"]["peak"] for row in rows]),
        "median_speed_after": statistics.median([row["after"].get("median_speed", 0.0) for row in rows]),
        "moving_scenes_after": sum(1 for row in rows if row["after"].get("median_speed", 0.0) > 1.0),
    }
    report = {"summary": summary, "rows": rows, "checkpoint": str(args.checkpoint)}

    OUTPUT_HTML.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_HTML.write_text(render_html(report), encoding="utf-8")
    OUTPUT_JSON.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"[summary] 场景 {summary['scenes']} · 训练前违规 {violated_before} · 修复 {fixed} · "
          f"仍失败 {still} · 被改坏 {regressed} · 回归保留率 {summary['retention'] * 100:.0f}%")
    print(f"[report]  {OUTPUT_HTML}")

    failures = []
    if summary["moving_scenes_after"] < 0.5 * summary["scenes"]:
        failures.append(
            f"训练后的策略在 {summary['scenes'] - summary['moving_scenes_after']}/{summary['scenes']} "
            f"个场景里中位速度 ≤1 m/s（不动车 → 违规数无意义，中位 {summary['median_speed_after']:.2f} m/s）"
        )
    if regressed and args.baseline_checkpoint:
        failures.append(f"{regressed} 个原本正常的场景训练后违规（必须拒绝该 checkpoint）")
    elif regressed:
        print(f"[note] {regressed} 个场景在随机初始化下正常、训练后违规 —— 随机策略几乎不动，"
              f"这个变化没有回归语义；要比两次训练请用 --baseline-checkpoint")
    for failure in failures:
        print(f"GATE FAIL : {failure}")
    print(f"GATE      : {'FAIL' if failures else 'PASS'}")
    return 1 if (args.check and failures) else 0


if __name__ == "__main__":
    raise SystemExit(main())
