#!/usr/bin/env bash
set -euo pipefail

CONDA_SH="${CONDA_SH:-/home/deepcybo/miniconda3/etc/profile.d/conda.sh}"
CONDA_ENV="${CONDA_ENV:-le_nero}"
SKILLS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)"
WORKSPACE_ROOT="$(dirname "$SKILLS_ROOT")"
REPO_ROOT="${REPO_ROOT:-$WORKSPACE_ROOT/Le-nero/dual_arm_teleop}"
ROBOT_PYTHON="${ROBOT_PYTHON:-}"
CONFIG=""

EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --python)
            ROBOT_PYTHON="$2"
            shift 2
            ;;
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

if [[ -z "$ROBOT_PYTHON" && -x "$WORKSPACE_ROOT/envs/le_nero/bin/python" ]]; then
    ROBOT_PYTHON="$WORKSPACE_ROOT/envs/le_nero/bin/python"
fi

if [[ -n "$CONFIG" && ! -f "$CONFIG" ]]; then
    echo "ERROR: reset config not found: $CONFIG" >&2
    exit 1
fi

if [[ -n "$ROBOT_PYTHON" ]]; then
    if [[ ! -x "$ROBOT_PYTHON" ]]; then
        echo "ERROR: robot Python is not executable: $ROBOT_PYTHON" >&2
        exit 1
    fi
    RESET_COMMAND=("$ROBOT_PYTHON" -m scripts.core.reset_robot)
else
    if [[ ! -f "$CONDA_SH" ]]; then
        echo "ERROR: no workspace Python or conda.sh; use --python PATH" >&2
        exit 1
    fi
    # shellcheck disable=SC1090
    source "$CONDA_SH"
    conda activate "$CONDA_ENV"
    RESET_COMMAND=(robot-reset)
fi

cd "$REPO_ROOT"
# Console scripts start with the environment's bin directory on sys.path.
# Prefer this project's scripts package over ROS's unrelated scripts package.
if [[ ! -f "$REPO_ROOT/scripts/core/reset_robot.py" ]]; then
    echo "ERROR: reset entry point not found: $REPO_ROOT/scripts/core/reset_robot.py" >&2
    exit 1
fi
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
if [[ -n "$CONFIG" ]]; then
    echo "+ ${RESET_COMMAND[*]} --config $CONFIG ${EXTRA_ARGS[*]}"
    "${RESET_COMMAND[@]}" --config "$CONFIG" "${EXTRA_ARGS[@]}"
else
    echo "+ ${RESET_COMMAND[*]} ${EXTRA_ARGS[*]}"
    "${RESET_COMMAND[@]}" "${EXTRA_ARGS[@]}"
fi
