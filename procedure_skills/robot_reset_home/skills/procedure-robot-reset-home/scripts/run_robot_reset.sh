#!/usr/bin/env bash
set -euo pipefail

CONDA_SH="${CONDA_SH:-/home/deepcybo/miniconda3/etc/profile.d/conda.sh}"
CONDA_ENV="${CONDA_ENV:-le_nero}"
REPO_ROOT="${REPO_ROOT:-/home/deepcybo/Le-nero/dual_arm_teleop}"
CONFIG=""
MODE="live"
HARDWARE_ALLOWED=0
EXECUTE=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode)
            MODE="$2"
            shift 2
            ;;
        --hardware-allowed)
            HARDWARE_ALLOWED=1
            shift
            ;;
        --execute)
            EXECUTE=1
            shift
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
            printf 'ERROR: unknown argument: %s\n' "$1" >&2
            exit 2
            ;;
    esac
done

PREFLIGHT_CMD=(python3 "$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)/scripts/hardware_preflight.py" --operation reset --mode "$MODE")
if [[ "$HARDWARE_ALLOWED" == "1" ]]; then PREFLIGHT_CMD+=(--hardware-allowed); fi
if [[ "$EXECUTE" == "1" ]]; then PREFLIGHT_CMD+=(--execute); fi
"${PREFLIGHT_CMD[@]}"
if [[ "$MODE" != "live" ]]; then
    printf 'reset plan recorded; no reset command was executed for mode=%s\n' "$MODE"
    exit 0
fi

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
    echo "+ robot-reset --config $CONFIG"
    robot-reset --config "$CONFIG"
else
    echo "+ robot-reset"
    robot-reset
fi
