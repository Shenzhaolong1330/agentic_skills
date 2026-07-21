#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="${REPO_ROOT}/environment.gen_agent.yml"
ENV_NAME="agentic-gen-agent"

if command -v mamba >/dev/null 2>&1; then
  CONDA_CMD="mamba"
elif command -v conda >/dev/null 2>&1; then
  CONDA_CMD="conda"
else
  if command -v python3 >/dev/null 2>&1; then
    echo "Neither mamba nor conda is available; checking the existing Python instead." >&2
    python3 "${REPO_ROOT}/scripts/check_gen_agent_env.py"
    echo "Recommended command: python3 <script>"
    exit $?
  fi
  echo "Neither mamba/conda nor python3 is available; install nothing automatically." >&2
  exit 2
fi

if "${CONDA_CMD}" env list | awk '{print $1}' | grep -Fxq "${ENV_NAME}"; then
  "${CONDA_CMD}" env update --name "${ENV_NAME}" --file "${ENV_FILE}"
else
  "${CONDA_CMD}" env create --file "${ENV_FILE}"
fi

"${CONDA_CMD}" run --no-capture-output -n "${ENV_NAME}" python "${REPO_ROOT}/scripts/check_gen_agent_env.py"

echo
echo "Environment ready: ${ENV_NAME}"
echo "Recommended command: ${CONDA_CMD} run -n ${ENV_NAME} python <script>"
echo "Recommended tests: ${CONDA_CMD} run -n ${ENV_NAME} python -m pytest -q tests"
