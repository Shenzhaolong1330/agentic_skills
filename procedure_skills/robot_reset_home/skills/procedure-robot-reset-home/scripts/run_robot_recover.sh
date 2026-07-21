#!/usr/bin/env bash
set -euo pipefail

CONDA_SH="${CONDA_SH:-/home/deepcybo/miniconda3/etc/profile.d/conda.sh}"
CONDA_ENV="${CONDA_ENV:-le_nero}"
REPO_ROOT="${REPO_ROOT:-/home/deepcybo/Le-nero/dual_arm_teleop}"
CLIENT_PATH="${DUAL_FRANKA_RPC_CLIENT_PATH:-$REPO_ROOT/robots/dual_franka/dual_franka_robotiq_rpc_client.py}"
SERVER_HOST="${FRANKA_RPC_HOST:-172.16.0.1}"
SERVER_PORT="${FRANKA_RPC_PORT:-4242}"
RPC_TIMEOUT_SEC="${FRANKA_RPC_TIMEOUT_SEC:-30}"
SIDE="both"
MODE="live"
HARDWARE_ALLOWED=0
EXECUTE=0

usage() {
    cat <<'EOF'
Usage: run_robot_recover.sh [options]

Request Franka controller error recovery for one or both arms. This command
does not command the arms to Home and does not open or close either gripper.

Options:
  --side SIDE             both, left_arm, or right_arm (default: both)
  --server-host HOST      RPC host (default: FRANKA_RPC_HOST or 172.16.0.1)
  --server-port PORT      RPC port (default: FRANKA_RPC_PORT or 4242)
  --rpc-timeout-sec SEC   RPC timeout (default: 30)
  --repo-root PATH        dual_arm_teleop repository root
  --conda-sh PATH         conda.sh path
  --conda-env NAME        Conda environment (default: le_nero)
  -h, --help              Show this help
EOF
}

require_value() {
    if [[ $# -lt 2 || -z "${2:-}" ]]; then
        printf 'ERROR: %s requires a value\n' "$1" >&2
        exit 2
    fi
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode) require_value "$@"; MODE="$2"; shift 2 ;;
        --hardware-allowed) HARDWARE_ALLOWED=1; shift ;;
        --execute) EXECUTE=1; shift ;;
        --side) require_value "$@"; SIDE="$2"; shift 2 ;;
        --server-host) require_value "$@"; SERVER_HOST="$2"; shift 2 ;;
        --server-port) require_value "$@"; SERVER_PORT="$2"; shift 2 ;;
        --rpc-timeout-sec|--timeout) require_value "$@"; RPC_TIMEOUT_SEC="$2"; shift 2 ;;
        --repo-root)
            require_value "$@"
            REPO_ROOT="$2"
            CLIENT_PATH="$REPO_ROOT/robots/dual_franka/dual_franka_robotiq_rpc_client.py"
            shift 2
            ;;
        --conda-sh) require_value "$@"; CONDA_SH="$2"; shift 2 ;;
        --conda-env) require_value "$@"; CONDA_ENV="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) printf 'ERROR: unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
    esac
done

REPO_ROOT_LOCAL="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)"
PREFLIGHT_CMD=(python3 "$REPO_ROOT_LOCAL/scripts/hardware_preflight.py" --operation recover --mode "$MODE")
if [[ "$HARDWARE_ALLOWED" == "1" ]]; then PREFLIGHT_CMD+=(--hardware-allowed); fi
if [[ "$EXECUTE" == "1" ]]; then PREFLIGHT_CMD+=(--execute); fi
"${PREFLIGHT_CMD[@]}"
if [[ "$MODE" != "live" ]]; then
    printf 'recovery plan recorded; no RPC recovery was executed for mode=%s\n' "$MODE"
    exit 0
fi

if [[ ! -f "$CONDA_SH" ]]; then
    printf 'ERROR: conda.sh not found: %s\n' "$CONDA_SH" >&2
    exit 1
fi
if [[ ! -f "$CLIENT_PATH" ]]; then
    printf 'ERROR: dual-Franka RPC client not found: %s\n' "$CLIENT_PATH" >&2
    exit 1
fi

# shellcheck disable=SC1090
source "$CONDA_SH"
conda activate "$CONDA_ENV"

COMMAND=(
    python3 "$CLIENT_PATH"
    --server "tcp://$SERVER_HOST:$SERVER_PORT"
    --timeout "$RPC_TIMEOUT_SEC"
    recover "$SIDE"
)
printf '+'
printf ' %q' "${COMMAND[@]}"
printf '\n'
exec "${COMMAND[@]}"
