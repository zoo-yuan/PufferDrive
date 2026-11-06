#!/usr/bin/env bash
# 把这个项目的编译/运行工具链切到 conda 里的 gcc 13（**只影响当前 shell**）。
#
#   source scripts/puffer_env.sh       # 之后 python -m pufferlib.pufferl ... 都会用 gcc13
#
# 为什么运行时也要：torch.compile（inductor）会在训练过程中现编 C++ 内核，它同样要求
# -std=c++20，系统 g++ 9.4 不支持。只编一次 _C 是不够的。

PUFFER_CONDA_ENV_NAME="${PUFFER_CONDA_ENV_NAME:-pufdrive-gcc13}"
PUFFER_CONDA_ENV_PATH="${PUFFER_CONDA_ENV_PATH:-${HOME}/miniconda3/envs/${PUFFER_CONDA_ENV_NAME}}"

if [[ ! -x "${PUFFER_CONDA_ENV_PATH}/bin/x86_64-conda-linux-gnu-g++" ]]; then
  echo "[puffer_env] 找不到 ${PUFFER_CONDA_ENV_PATH}/bin/x86_64-conda-linux-gnu-g++" >&2
  echo "[puffer_env] 先创建：conda create -y -n ${PUFFER_CONDA_ENV_NAME} -c conda-forge gxx_linux-64=13" >&2
  return 1 2>/dev/null || exit 1
fi

export CC="${PUFFER_CONDA_ENV_PATH}/bin/x86_64-conda-linux-gnu-gcc"
export CXX="${PUFFER_CONDA_ENV_PATH}/bin/x86_64-conda-linux-gnu-g++"
export PATH="${PUFFER_CONDA_ENV_PATH}/bin:${PATH}"
export LDFLAGS="${LDFLAGS:-} -static-libstdc++ -static-libgcc"

echo "[puffer_env] CXX=$("${CXX}" --version | head -1)"
