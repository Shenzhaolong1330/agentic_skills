#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$TASK_ROOT/../../../.." && pwd)"

RUNNER="$SCRIPT_DIR/task_pick_tube_insert_rack_runner.py"
SELECT_AND_GRASP="$SCRIPT_DIR/locate_then_grasp_by_tail_side.py"
INSERT_FLOW="$SCRIPT_DIR/run_manual_grip_to_insert.py"

ANY_POSE_ROOT="${ANY_POSE_ROOT:-$REPO_ROOT/atomic_skills/object_locator}"
LOCATOR="${LOCATOR:-$ANY_POSE_ROOT/.venv/bin/object-locator}"
TUBE_CONFIG="${TUBE_CONFIG:-$ANY_POSE_ROOT/config_test_tube_cleanup_leftmost_vlm_head.yaml}"

MODE="dry_run"
EXECUTE=0
ARTIFACT_DIR=""
TUBE_RESULT_JSON=""
FULL_LOG_FILE=""
INSERT_LOG_FILE=""

LOCATOR_ARGS=()
GRASP_ARGS=()
INSERTION_ARGS=()
RUNNER_ARGS=()

usage() {
    cat <<'EOF'
Usage:
  run_full_pick_tube_insert_rack.sh [safe-mode options]
  run_full_pick_tube_insert_rack.sh --mode live --execute [options]

Modes (default: dry_run):
  --mock                  Run the task harness with mock fixtures.
  --dry-run               Generate the task command plan; no hardware access.
  --from-artifacts        Run the task harness from supplied JSON artifacts.
  --mode MODE             mock, dry_run, from_artifacts, or live.

Common options:
  --artifact-dir DIR      Trace/log directory.
  --runner-arg ARG        Pass one argument to the non-live task harness; repeat as needed.

Live-only execution gate:
  --execute

Live flow options:
  --locator PATH
  --tube-config PATH
  --tube-result-json PATH
  --full-log-file PATH
  --insert-log-file PATH
  --locator-arg ARG       Pass one argument to object-locator; repeat as needed.
  --grasp-arg ARG         Pass one argument through to grasp_right_arm_xyz.sh; repeat as needed.
  --insertion-arg ARG     Pass one argument to run_manual_grip_to_insert.py; repeat as needed.

The live sequence is:
  locate tube -> select tail-side arm -> grasp -> go home without changing grippers
  -> transition handover
  -> observe rack -> wrist locate empty hole -> insert -> release -> retract

This wrapper never performs an unconditional robot reset.
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
        --mock) MODE="mock"; shift ;;
        --dry-run) MODE="dry_run"; shift ;;
        --from-artifacts) MODE="from_artifacts"; shift ;;
        --execute) EXECUTE=1; shift ;;
        --artifact-dir) require_value "$@"; ARTIFACT_DIR="$2"; shift 2 ;;
        --locator) require_value "$@"; LOCATOR="$2"; shift 2 ;;
        --tube-config) require_value "$@"; TUBE_CONFIG="$2"; shift 2 ;;
        --tube-result-json) require_value "$@"; TUBE_RESULT_JSON="$2"; shift 2 ;;
        --full-log-file) require_value "$@"; FULL_LOG_FILE="$2"; shift 2 ;;
        --insert-log-file) require_value "$@"; INSERT_LOG_FILE="$2"; shift 2 ;;
        --locator-arg) require_value "$@"; LOCATOR_ARGS+=("$2"); shift 2 ;;
        --grasp-arg) require_value "$@"; GRASP_ARGS+=("$2"); shift 2 ;;
        --insertion-arg) require_value "$@"; INSERTION_ARGS+=("$2"); shift 2 ;;
        --runner-arg) require_value "$@"; RUNNER_ARGS+=("$2"); shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) printf 'ERROR: unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
    esac
done

case "$MODE" in
    mock|dry_run|from_artifacts|live) ;;
    *) printf 'ERROR: unsupported mode: %s\n' "$MODE" >&2; exit 2 ;;
esac

if [[ "$MODE" != "live" ]]; then
    if [[ "$EXECUTE" == "1" ]]; then
        printf 'ERROR: execution and hardware gate flags are only valid with --mode live\n' >&2
        exit 2
    fi
    CMD=(python3 "$RUNNER" --mode "$MODE")
    if [[ -n "$ARTIFACT_DIR" ]]; then
        CMD+=(--artifact-dir "$ARTIFACT_DIR")
    fi
    CMD+=("${RUNNER_ARGS[@]}")
    printf '[full-flow] safe harness command:'
    printf ' %q' "${CMD[@]}"
    printf '\n'
    exec "${CMD[@]}"
fi

if [[ "$EXECUTE" != "1" ]]; then
    printf 'ERROR: live mode requires --execute\n' >&2
    exit 2
fi
STAMP="$(date +%Y%m%d_%H%M%S)"
if [[ -z "$ARTIFACT_DIR" ]]; then
    ARTIFACT_DIR="/tmp/agentic_skills_runs/full_pick_tube_insert_rack_$STAMP"
fi
if [[ -z "$TUBE_RESULT_JSON" ]]; then
    TUBE_RESULT_JSON="$ARTIFACT_DIR/tube_detection.json"
fi
if [[ -z "$FULL_LOG_FILE" ]]; then
    FULL_LOG_FILE="$ARTIFACT_DIR/full_flow.log"
fi
if [[ -z "$INSERT_LOG_FILE" ]]; then
    INSERT_LOG_FILE="$ARTIFACT_DIR/insertion.log"
fi

for required_file in "$LOCATOR" "$TUBE_CONFIG" "$SELECT_AND_GRASP" "$INSERT_FLOW"; do
    if [[ ! -e "$required_file" ]]; then
        printf 'ERROR: required path does not exist: %s\n' "$required_file" >&2
        exit 2
    fi
done

mkdir -p "$ARTIFACT_DIR" "$(dirname "$TUBE_RESULT_JSON")" "$(dirname "$FULL_LOG_FILE")" "$(dirname "$INSERT_LOG_FILE")"
exec > >(tee -a "$FULL_LOG_FILE") 2>&1

printf '[full-flow] LIVE hardware flow\n'
printf '[full-flow] artifact_dir=%s\n' "$ARTIFACT_DIR"
printf '[full-flow] tube_result_json=%s\n' "$TUBE_RESULT_JSON"
printf '[full-flow] no unconditional reset will be run\n'

if [[ -f "$ANY_POSE_ROOT/.env" ]]; then
    set -a
    # shellcheck disable=SC1091
    source "$ANY_POSE_ROOT/.env"
    set +a
    printf '[full-flow] loaded object-locator environment\n'
fi

printf '[full-flow] stage 1/3: locate tube\n'
(
    cd "$ANY_POSE_ROOT"
    "$LOCATOR" --config "$TUBE_CONFIG" --result-json "$TUBE_RESULT_JSON" "${LOCATOR_ARGS[@]}"
)

printf '[full-flow] stage 2/3: select arm, grasp, and hand over at transition\n'
GRASP_CMD=(
    python3 "$SELECT_AND_GRASP"
    --result-json "$TUBE_RESULT_JSON"
    --execute
    --grasp-arg=--go-home-before-transition
)
for arg in "${GRASP_ARGS[@]}"; do
    GRASP_CMD+=(--grasp-arg "$arg")
done
"${GRASP_CMD[@]}"

printf '[full-flow] stage 3/3: locate rack/hole, insert, release, and retract\n'
python3 "$INSERT_FLOW" \
    --execute \
    --holder-side auto \
    --log-file "$INSERT_LOG_FILE" \
    "${INSERTION_ARGS[@]}"

printf '[full-flow] COMPLETE: all child stages returned success\n'
printf '[full-flow] log=%s\n' "$FULL_LOG_FILE"
