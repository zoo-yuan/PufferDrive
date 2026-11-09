#!/usr/bin/env bash
# 用项目专属的 conda 工具链编 pufferlib._C（不改系统默认编译器，也不碰系统 g++）。
#
# 为什么需要：torch 给扩展强制传 -std=c++20，系统 g++ 9.4 只认旧拼法 -std=c++2a 且缺
# <compare>，所以训练入口 `puffer train` 起不来。做法是把 gcc 13 装在 conda 的一个
# build-only 环境里，只有跑这个脚本时它才出现在 PATH 上。
#
# 用法：
#   scripts/build_cpu_ext.sh                 # 有可用 CUDA 时编 CUDA 版（默认）
#   scripts/build_cpu_ext.sh --cpu-only      # 强制只编 C++ 版
#   CONDA_ENV_NAME=other scripts/build_cpu_ext.sh
#
# 两个必须记住的点：
#   1. PUFFER_BUILD_CPU_ONLY —— 只有 CUDA 版本不匹配（torch 的 CUDA != 本机 toolkit）时才需要。
#      2026-11-06 起 .venv 换成 torch 2.14.1+cu126，与本机 toolkit 12.6 一致，默认走 CUDA。
#   2. 链接静态 libstdc++ —— conda 的 gcc 13 依赖 GLIBCXX_3.4.3x，运行时系统里只有
#      3.4.28（Ubuntu 20.04），不静态链接会在 import 时报 version not found。

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONDA_ENV_NAME="${CONDA_ENV_NAME:-pufdrive-gcc13}"
CONDA_ENV_PATH="${CONDA_ENV_PATH:-${HOME}/miniconda3/envs/${CONDA_ENV_NAME}}"
TOOLCHAIN_CXX="${CONDA_ENV_PATH}/bin/x86_64-conda-linux-gnu-g++"

if [[ ! -x "${TOOLCHAIN_CXX}" ]]; then
  echo "[error] 找不到 ${TOOLCHAIN_CXX}" >&2
  echo "        先创建工具链环境：conda create -y -n ${CONDA_ENV_NAME} -c conda-forge gxx_linux-64=13" >&2
  exit 1
fi

cd "${REPO_ROOT}"

# 工具链与运行时共用一份导出逻辑（inductor 训练时也会调 CXX）
# shellcheck disable=SC1091
source "${REPO_ROOT}/scripts/puffer_env.sh"

if [[ ! -f .venv/bin/activate ]]; then
  echo "[error] 缺 .venv" >&2
  exit 1
fi
# shellcheck disable=SC1091
source .venv/bin/activate

if [[ "${1:-}" == "--cpu-only" ]]; then
  export PUFFER_BUILD_CPU_ONLY=1
else
  export PUFFER_BUILD_CPU_ONLY=0
fi
echo "[build] CPU_ONLY= ${PUFFER_BUILD_CPU_ONLY}"
python setup.py build_ext --inplace --force

echo "[verify] torch.ops.pufferlib (CPU + CUDA when available)"
python - <<'PY'
# 必须先 import torch：扩展链接的是 torch 的 libc10/libtorch，torch 没加载时
# import _C 会报 "libc10.so: cannot open shared object file"（不是构建失败）。
import torch
import pufferlib._C  # noqa: F401  注册 torch.ops.pufferlib

print(f"[verify] torch {torch.__version__} cuda {torch.version.cuda} "
      f"available {torch.cuda.is_available()}")
for device in (["cpu", "cuda"] if torch.cuda.is_available() else ["cpu"]):
    values = torch.randn(8, 16, device=device)
    rewards = torch.randn(8, 16, device=device)
    dones = (torch.rand(8, 16, device=device) > 0.9).float()
    importance = torch.ones(8, 16, device=device)
    advantages = torch.zeros(8, 16, device=device)
    torch.ops.pufferlib.compute_puff_advantage(
        values, rewards, dones, importance, advantages, 0.99, 0.95, 1.0, 1.0
    )
    print(f"[verify] VTrace {device}: ok ({float(advantages.mean()):.4f})")
PY
