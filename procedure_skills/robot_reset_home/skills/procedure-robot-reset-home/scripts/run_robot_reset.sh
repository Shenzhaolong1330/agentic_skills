#!/usr/bin/env bash
set -euo pipefail

CONDA_SH="${CONDA_SH:-/home/deepcybo/miniconda3/etc/profile.d/conda.sh}"
CONDA_ENV="${CONDA_ENV:-le_nero}"
REPO_ROOT="${REPO_ROOT:-/home/deepcybo/Le-nero/dual_arm_teleop}"
CONFIG=""

EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --conda-sh)
            CONDA_SH="$2"
            shift 2
            ;;
        --conda-env)
            CONDA_ENV="$2"
            shift 2
            ;;
        --repo-root)
            REPO_ROOT="$2"
            shift 2
            ;;
        --config|--config-path)
            CONFIG="$2"
            shift 2
            ;;
        *)
            EXTRA_ARGS+=("$1")
            shift
            ;;
    esac
done

if [[ ! -f "$CONDA_SH" ]]; then
    echo "ERROR: conda.sh not found: $CONDA_SH" >&2
    exit 1
fi

if [[ -n "$CONFIG" && ! -f "$CONFIG" ]]; then
    echo "ERROR: reset config not found: $CONFIG" >&2
    exit 1
fi

# shellcheck disable=SC1090
source "$CONDA_SH"
conda activate "$CONDA_ENV"

cd "$REPO_ROOT"
if [[ -n "$CONFIG" ]]; then
    echo "+ robot-reset --config $CONFIG ${EXTRA_ARGS[*]}"
    robot-reset --config "$CONFIG" "${EXTRA_ARGS[@]}"
else
    echo "+ robot-reset ${EXTRA_ARGS[*]}"
    robot-reset "${EXTRA_ARGS[@]}"
fi
