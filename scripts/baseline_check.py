"""冻结基线校验：`docs/baselines.md` 里写的数字，还能复现吗？

基线一旦对外报数，就必须有一条命令能证明"这个 checkpoint + 这批场景 + 这条谓词"
仍然给出同样的结论。否则数字会随代码漂移而变成传说。

校验三件事
----------
1. checkpoint 文件在、sha256 与清单一致；
2. 场景集数量与清单一致；
3. 重跑评测：参考零点（replay）违规数 = 清单值，基线 checkpoint 的违规数 ≤ 清单上限。

用法
----
``python scripts/baseline_check.py``          # 报告
``python scripts/baseline_check.py --check``   # 不一致则退出码非 0
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import torch

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from regression_retention import (  # noqa: E402
    build_env,
    build_policy,
    evaluate_replay,
    evaluate_scene,
    load_weights,
    scene_ids,
)
from scene_feasibility import get_metric_names, load_template  # noqa: E402

MANIFEST = REPO_ROOT / "docs" / "measured" / "baseline_lane_departure_v0.json"
SCENE_DIR = REPO_ROOT / ".scratch" / "crossbag_scenes"


def sha256_of(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    if not MANIFEST.exists():
        print(f"ERROR: 缺清单 {MANIFEST}", file=sys.stderr)
        return 1
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    checkpoint = REPO_ROOT / manifest["checkpoint"]
    problems = []

    if not checkpoint.exists():
        problems.append(f"checkpoint 不存在：{checkpoint}")
    else:
        actual = sha256_of(checkpoint)
        if actual != manifest["sha256"]:
            problems.append(f"sha256 不匹配：{actual[:12]}… ≠ {manifest['sha256'][:12]}…")

    scenes = [
        scene for scene in scene_ids()
        if (SCENE_DIR / f"{scene['scenario_id'].replace('#', '_')}.bin").exists()
    ]
    if len(scenes) != manifest["scenes"]:
        problems.append(f"场景数 {len(scenes)} ≠ 清单 {manifest['scenes']}")
    if problems:
        for problem in problems:
            print(f"GATE FAIL : {problem}")
        print("GATE      : FAIL")
        return 1 if args.check else 0

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    metric_names, metric_index = get_metric_names()

    def path_of(scene):
        return SCENE_DIR / f"{scene['scenario_id'].replace('#', '_')}.bin"

    env = build_env(path_of(scenes[0]))
    policy = build_policy(env, device)
    load_weights(policy, checkpoint, device)

    reference_fired = 0
    policy_fired = 0
    for scene in scenes:
        template = load_template(scene["template"])
        if evaluate_replay(path_of(scene), template, metric_names, metric_index)["fired"]:
            reference_fired += 1
        if evaluate_scene(policy, path_of(scene), template, metric_names, metric_index, device)["fired"]:
            policy_fired += 1

    fix_rate = 1.0 - policy_fired / max(1, reference_fired)
    print(f"reference 违规 : {reference_fired}/{len(scenes)}（清单 {manifest['reference_violations']}）")
    print(f"基线违规       : {policy_fired}/{len(scenes)}（清单上限 {manifest['baseline_violations_max']}）")
    print(f"修复率         : {fix_rate * 100:.1f}%（清单 {manifest['fix_rate'] * 100:.0f}%）")

    if reference_fired != manifest["reference_violations"]:
        problems.append(f"参考零点漂移：{reference_fired} ≠ {manifest['reference_violations']}")
    if policy_fired > manifest["baseline_violations_max"]:
        problems.append(f"基线违规 {policy_fired} > 上限 {manifest['baseline_violations_max']}")
    if fix_rate < manifest["fix_rate_min"]:
        problems.append(f"修复率 {fix_rate:.3f} < 下限 {manifest['fix_rate_min']}")

    for problem in problems:
        print(f"GATE FAIL : {problem}")
    print(f"GATE      : {'FAIL' if problems else 'PASS'}")
    return 1 if (args.check and problems) else 0


if __name__ == "__main__":
    raise SystemExit(main())
