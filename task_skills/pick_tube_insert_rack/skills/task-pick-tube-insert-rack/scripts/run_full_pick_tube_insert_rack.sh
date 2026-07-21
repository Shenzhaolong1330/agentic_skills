#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$TASK_ROOT/../../../.." && pwd)"

RUNNER="$SCRIPT_DIR/task_pick_tube_insert_rack_runner.py"
SELECT_AND_GRASP="$SCRIPT_DIR/locate_then_grasp_by_tail_side.py"
INSERT_FLOW="$SCRIPT_DIR/run_manual_grip_to_insert.py"
MULTI_TUBE_LOCATOR="$SCRIPT_DIR/locate_all_tubes_once.py"
SAM_CACHE_SERVICE="$SCRIPT_DIR/sam_cache_service.py"
WRIST_CAMERA_SERVICE="$SCRIPT_DIR/wrist_camera_service.py"
RACK_GRID_STATE="$SCRIPT_DIR/rack_grid_state.py"
ROBOT_RESET_SCRIPT="$REPO_ROOT/procedure_skills/robot_reset_home/skills/procedure-robot-reset-home/scripts/run_robot_reset.sh"
ROBOT_RESET_CONFIG="${ROBOT_RESET_CONFIG:-/home/deepcybo/Le-nero/dual_arm_teleop/scripts/config/record_cfg.yaml}"

ANY_POSE_ROOT="${ANY_POSE_ROOT:-$REPO_ROOT/atomic_skills/object_locator}"
LOCATOR="${LOCATOR:-$ANY_POSE_ROOT/.venv/bin/object-locator}"
LOCATOR_PYTHON="${LOCATOR_PYTHON:-$ANY_POSE_ROOT/.venv/bin/python}"
TUBE_CONFIG="${TUBE_CONFIG:-$ANY_POSE_ROOT/config_test_tube_cleanup_leftmost_vlm_head.yaml}"
RACK_CONFIG="${RACK_CONFIG:-$ANY_POSE_ROOT/config_rack_center_vlm.yaml}"

MODE="dry_run"
EXECUTE=0
HARDWARE_ALLOWED=0
STOP_AFTER_INVENTORY=0
WAIT_BEFORE_MOTION=1
RESET_AFTER_TUBE_ERROR=1
ARTIFACT_DIR=""
TUBE_RESULT_JSON=""
RACK_RESULT_JSON=""
FULL_LOG_FILE=""
INSERT_LOG_FILE=""
WRIST_PERCEPTION_MODE="legacy-vlm"

LOCATOR_ARGS=()
GRASP_ARGS=()
INSERTION_ARGS=()
RUNNER_ARGS=()

usage() {
    cat <<'EOF'
Usage:
  run_full_pick_tube_insert_rack.sh [safe-mode options]
  run_full_pick_tube_insert_rack.sh --mode live --hardware-allowed --execute [options]

Modes (default: dry_run):
  --mock                  Run the task harness with mock fixtures.
  --dry-run               Generate the task command plan; no hardware access.
  --from-artifacts        Run the task harness from supplied JSON artifacts.
  --mode MODE             mock, dry_run, from_artifacts, or live.

Common options:
  --artifact-dir DIR      Trace/log directory.
  --runner-arg ARG        Pass one argument to the non-live task harness; repeat as needed.

Live-only execution gate:
  --hardware-allowed    Explicitly allow access to real cameras/RPC/hardware.
  --execute
  --stop-after-inventory  Capture/write the inventory, then stop before any robot motion.
  --no-wait-before-motion Skip the default Enter confirmation after inventory (automation only).

Live flow options:
  --locator PATH          Kept for compatibility; selects Python beside this executable.
  --locator-python PATH   Python from the object-locator environment.
  --tube-config PATH
  --rack-config PATH
  --tube-result-json PATH
  --rack-result-json PATH
  --wrist-perception-mode MODE  legacy-vlm (live default) or experimental cached-grid.
  --full-log-file PATH
  --insert-log-file PATH
  --reset-config PATH      Robot reset config.
  --no-reset-after-error   Do not reset after grasp/handover or insertion failure.
  --locator-arg ARG       Pass one argument to the one-frame multi-tube locator; repeat as needed.
  --grasp-arg ARG         Pass one argument through to grasp_right_arm_xyz.sh; repeat as needed.
  --insertion-arg ARG     Pass one argument to run_manual_grip_to_insert.py; repeat as needed.

The live sequence is:
  capture once and inventory all loose tubes plus the fixed rack
  -> sort tubes by initial image x (left to right)
  -> for each cached tube: select tail-side arm -> grasp -> transition handover
  -> move to cached rack pose -> wrist locate current empty hole -> insert -> release -> retract

This wrapper does not reset during a successful flow. By default, a failed
grasp/handover or insertion stage triggers full robot-reset before the next tube.
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
        --hardware-allowed) HARDWARE_ALLOWED=1; shift ;;
        --stop-after-inventory) STOP_AFTER_INVENTORY=1; shift ;;
        --wait-before-motion) WAIT_BEFORE_MOTION=1; shift ;;
        --no-wait-before-motion) WAIT_BEFORE_MOTION=0; shift ;;
        --artifact-dir) require_value "$@"; ARTIFACT_DIR="$2"; shift 2 ;;
        --locator)
            require_value "$@"
            LOCATOR="$2"
            LOCATOR_PYTHON="$(dirname "$LOCATOR")/python"
            shift 2
            ;;
        --locator-python) require_value "$@"; LOCATOR_PYTHON="$2"; shift 2 ;;
        --tube-config) require_value "$@"; TUBE_CONFIG="$2"; shift 2 ;;
        --rack-config) require_value "$@"; RACK_CONFIG="$2"; shift 2 ;;
        --tube-result-json) require_value "$@"; TUBE_RESULT_JSON="$2"; shift 2 ;;
        --rack-result-json) require_value "$@"; RACK_RESULT_JSON="$2"; shift 2 ;;
        --wrist-perception-mode) require_value "$@"; WRIST_PERCEPTION_MODE="$2"; shift 2 ;;
        --full-log-file) require_value "$@"; FULL_LOG_FILE="$2"; shift 2 ;;
        --insert-log-file) require_value "$@"; INSERT_LOG_FILE="$2"; shift 2 ;;
        --reset-config) require_value "$@"; ROBOT_RESET_CONFIG="$2"; shift 2 ;;
        --no-reset-after-error) RESET_AFTER_TUBE_ERROR=0; shift ;;
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
case "$WRIST_PERCEPTION_MODE" in
    cached-grid|legacy-vlm) ;;
    *) printf 'ERROR: unsupported wrist perception mode: %s\n' "$WRIST_PERCEPTION_MODE" >&2; exit 2 ;;
esac
if [[ "$MODE" != "live" ]]; then
    if [[ "$EXECUTE" == "1" || "$STOP_AFTER_INVENTORY" == "1" ]]; then
        printf 'ERROR: execution and hardware gate flags are only valid with --mode live\n' >&2
        exit 2
    fi
    CMD=(python3 "$RUNNER" --mode "$MODE")
    if [[ -n "$ARTIFACT_DIR" ]]; then
        CMD+=(--artifact-dir "$ARTIFACT_DIR")
    fi
    if [[ "$HARDWARE_ALLOWED" == "1" ]]; then CMD+=(--hardware-allowed); fi
    CMD+=("${RUNNER_ARGS[@]}")
    printf '[full-flow] safe harness command:'
    printf ' %q' "${CMD[@]}"
    printf '\n'
    exec "${CMD[@]}"
fi

PREFLIGHT_CMD=(python3 "$REPO_ROOT/scripts/hardware_preflight.py" --operation full --mode live)
if [[ "$HARDWARE_ALLOWED" == "1" ]]; then PREFLIGHT_CMD+=(--hardware-allowed); fi
if [[ "$EXECUTE" == "1" ]]; then PREFLIGHT_CMD+=(--execute); fi
"${PREFLIGHT_CMD[@]}"
STAMP="$(date +%Y%m%d_%H%M%S)"
if [[ -z "$ARTIFACT_DIR" ]]; then
    ARTIFACT_DIR="/tmp/agentic_skills_runs/full_pick_tube_insert_rack_$STAMP"
fi
if [[ -z "$TUBE_RESULT_JSON" ]]; then
    TUBE_RESULT_JSON="$ARTIFACT_DIR/tube_detection.json"
fi
if [[ -z "$RACK_RESULT_JSON" ]]; then
    RACK_RESULT_JSON="$ARTIFACT_DIR/rack_detection.json"
fi
if [[ -z "$FULL_LOG_FILE" ]]; then
    FULL_LOG_FILE="$ARTIFACT_DIR/full_flow.log"
fi
if [[ -z "$INSERT_LOG_FILE" ]]; then
    INSERT_LOG_FILE="$ARTIFACT_DIR/insertion.log"
fi

for required_file in "$LOCATOR_PYTHON" "$TUBE_CONFIG" "$RACK_CONFIG" "$MULTI_TUBE_LOCATOR" "$SAM_CACHE_SERVICE" "$WRIST_CAMERA_SERVICE" "$RACK_GRID_STATE" "$SELECT_AND_GRASP" "$INSERT_FLOW"; do
    if [[ ! -e "$required_file" ]]; then
        printf 'ERROR: required path does not exist: %s\n' "$required_file" >&2
        exit 2
    fi
done
if [[ "$RESET_AFTER_TUBE_ERROR" == "1" ]]; then
    for required_reset_file in "$ROBOT_RESET_SCRIPT" "$ROBOT_RESET_CONFIG"; do
        if [[ ! -f "$required_reset_file" ]]; then
            printf 'ERROR: required reset path does not exist: %s\n' "$required_reset_file" >&2
            exit 2
        fi
    done
fi

mkdir -p "$ARTIFACT_DIR" "$(dirname "$TUBE_RESULT_JSON")" "$(dirname "$RACK_RESULT_JSON")" "$(dirname "$FULL_LOG_FILE")" "$(dirname "$INSERT_LOG_FILE")"
exec > >(tee -a "$FULL_LOG_FILE") 2>&1

printf '[full-flow] LIVE hardware flow\n'
printf '[full-flow] artifact_dir=%s\n' "$ARTIFACT_DIR"
printf '[full-flow] tube_inventory_json=%s\n' "$TUBE_RESULT_JSON"
printf '[full-flow] cached_rack_json=%s\n' "$RACK_RESULT_JSON"
printf '[full-flow] wrist_perception_mode=%s\n' "$WRIST_PERCEPTION_MODE"
printf '[full-flow] reset policy: on grasp/handover or insertion error=%s\n' "$RESET_AFTER_TUBE_ERROR"

if [[ -f "$ANY_POSE_ROOT/.env" ]]; then
    set -a
    # shellcheck disable=SC1091
    source "$ANY_POSE_ROOT/.env"
    set +a
    printf '[full-flow] loaded object-locator environment\n'
fi

SAM_CACHE_PID=""
SAM_CACHE_SOCKET="/tmp/task_pick_tube_sam_${$}.sock"
SAM_CACHE_READY="$ARTIFACT_DIR/sam_cache_ready.json"
WRIST_CAMERA_PID=""
WRIST_CAMERA_SOCKET="/tmp/task_pick_tube_realsense_${$}.sock"
WRIST_CAMERA_READY="$ARTIFACT_DIR/realsense_cache_ready.json"
cleanup_sam_cache() {
    if [[ -n "${WRIST_CAMERA_PID:-}" ]] && kill -0 "$WRIST_CAMERA_PID" 2>/dev/null; then
        kill "$WRIST_CAMERA_PID" 2>/dev/null || true
        wait "$WRIST_CAMERA_PID" 2>/dev/null || true
    fi
    if [[ -n "$SAM_CACHE_PID" ]] && kill -0 "$SAM_CACHE_PID" 2>/dev/null; then
        kill "$SAM_CACHE_PID" 2>/dev/null || true
        wait "$SAM_CACHE_PID" 2>/dev/null || true
    fi
    rm -f "$SAM_CACHE_SOCKET"
    rm -f "${WRIST_CAMERA_SOCKET:-}"
}
trap cleanup_sam_cache EXIT
export TASK_PICK_TUBE_SAM_SOCKET="$SAM_CACHE_SOCKET"
printf '[full-flow] starting persistent SAM cache from local Hugging Face files\n'
"$LOCATOR_PYTHON" "$SAM_CACHE_SERVICE" \
    --serve \
    --socket "$SAM_CACHE_SOCKET" \
    --ready-file "$SAM_CACHE_READY" \
    --sam-model facebook/sam-vit-base \
    --device auto &
SAM_CACHE_PID=$!

export TASK_PICK_TUBE_WRIST_CAMERA_SOCKET="$WRIST_CAMERA_SOCKET"
printf '[full-flow] starting persistent RealSense cache; head/left/right are all required\n'
rm -f "$WRIST_CAMERA_READY" "$WRIST_CAMERA_SOCKET"
CAMERA_SERVICE_CMD=(
    "$LOCATOR_PYTHON" "$WRIST_CAMERA_SERVICE"
    --serve
    --socket "$WRIST_CAMERA_SOCKET"
    --ready-file "$WRIST_CAMERA_READY"
    --required-camera head
    --required-camera left
    --required-camera right
    --warmup-frames 30
    --device-discovery-timeout-sec 15
    --startup-timeout-sec 45
)
"${CAMERA_SERVICE_CMD[@]}" &
WRIST_CAMERA_PID=$!
for _ in $(seq 1 350); do
    [[ -f "$WRIST_CAMERA_READY" ]] && break
    if ! kill -0 "$WRIST_CAMERA_PID" 2>/dev/null; then
        wait "$WRIST_CAMERA_PID" || true
        printf 'ERROR: persistent RealSense service exited before ready\n' >&2
        exit 1
    fi
    sleep 0.1
done
if [[ ! -f "$WRIST_CAMERA_READY" ]]; then
    printf 'ERROR: persistent RealSense service did not become ready\n' >&2
    exit 1
fi
printf '[full-flow] persistent RealSense status: %s\n' "$(tr -d '\n' < "$WRIST_CAMERA_READY")"

for _ in $(seq 1 300); do
    [[ -f "$SAM_CACHE_READY" ]] && break
    if ! kill -0 "$SAM_CACHE_PID" 2>/dev/null; then
        wait "$SAM_CACHE_PID" || true
        printf 'ERROR: persistent SAM cache exited before model preload completed\n' >&2
        exit 1
    fi
    sleep 0.1
done
if [[ ! -f "$SAM_CACHE_READY" ]]; then
    printf 'ERROR: persistent SAM cache did not become ready within 30 seconds\n' >&2
    exit 1
fi
printf '[full-flow] persistent SAM status: %s\n' "$(tr -d '\n' < "$SAM_CACHE_READY")"

require_fresh_realsense_frames() {
    local stage="$1"
    "$LOCATOR_PYTHON" - "$WRIST_CAMERA_SOCKET" "$SCRIPT_DIR" "$stage" <<'PY'
import sys
from pathlib import Path

sys.path.insert(0, sys.argv[2])
from wrist_camera_service import WristCameraClient

stage = sys.argv[3]
required = ("head", "left", "right")
client = WristCameraClient(Path(sys.argv[1]), timeout_sec=15.0)
health = client.health()["cameras"]
missing = [side for side in required if side not in health]
if missing:
    raise RuntimeError(f"{stage}: required persistent cameras missing: {missing}")
frames = {}
for side in required:
    frame = client.frame(side, max_age_ms=500.0)
    frames[side] = {
        "generation": int(frame["generation"]),
        "age_ms": round(float(frame["age_ms"]), 1),
        "reset_count": int(frame["reset_count"]),
    }
print(f"[full-flow] RealSense fresh-frame gate stage={stage}: {frames}")
PY
}

require_fresh_realsense_frames "startup"

printf '[full-flow] stage 1: capture once and inventory all loose tubes plus rack\n'
(
    cd "$ANY_POSE_ROOT"
    INVENTORY_CMD=(
        "$LOCATOR_PYTHON" "$MULTI_TUBE_LOCATOR"
        --config "$TUBE_CONFIG" \
        --rack-config "$RACK_CONFIG" \
        --output-dir "$ARTIFACT_DIR/tubes" \
        --inventory-json "$TUBE_RESULT_JSON" \
        --rack-result-json "$RACK_RESULT_JSON" \
        --camera-service-socket "$WRIST_CAMERA_SOCKET" \
        --save-rgb "$ARTIFACT_DIR/initial_tube_inventory_rgb.jpg" \
        --save-depth "$ARTIFACT_DIR/initial_tube_inventory_depth.npy" \
        --save-vlm-response "$ARTIFACT_DIR/initial_tube_inventory_vlm_response.json" \
        --save-rack-vlm-response "$ARTIFACT_DIR/initial_rack_vlm_response.json"
    )
    if [[ "$WRIST_PERCEPTION_MODE" == "cached-grid" ]]; then
        INVENTORY_CMD+=(
            --build-rack-grid
            --rack-grid-json "$ARTIFACT_DIR/rack_grid.json"
            --rack-grid-overlay "$ARTIFACT_DIR/rack_grid_overlay.jpg"
        )
    fi
    INVENTORY_CMD+=("${LOCATOR_ARGS[@]}")
    "${INVENTORY_CMD[@]}"
)

mapfile -t TUBE_RESULTS < <(
    "$LOCATOR_PYTHON" -c '
import json, sys
payload = json.load(open(sys.argv[1], encoding="utf-8"))
for tube in payload.get("tubes", []):
    print(tube["result_json"])
' "$TUBE_RESULT_JSON"
)
if [[ "${#TUBE_RESULTS[@]}" -eq 0 ]]; then
    printf 'ERROR: inventory contains no executable tube results: %s\n' "$TUBE_RESULT_JSON" >&2
    exit 1
fi
require_fresh_realsense_frames "before_operator_confirmation"
if [[ "$STOP_AFTER_INVENTORY" == "1" ]]; then
    printf '[full-flow] STOPPED after inventory as requested; no robot motion was sent\n'
    printf '[full-flow] inspect inventory=%s rack=%s and rgb=%s\n' \
        "$TUBE_RESULT_JSON" "$RACK_RESULT_JSON" "$ARTIFACT_DIR/initial_tube_inventory_rgb.jpg"
    exit 0
fi

if [[ "$WAIT_BEFORE_MOTION" == "1" ]]; then
    printf '\n[full-flow] PRE-MOTION READY: models loaded and %d tube position(s) cached.\n' \
        "${#TUBE_RESULTS[@]}"
    printf '[full-flow] inspect rgb=%s\n' "$ARTIFACT_DIR/initial_tube_inventory_rgb.jpg"
    printf '[full-flow] inspect inventory=%s\n' "$TUBE_RESULT_JSON"
    printf '[full-flow] press Enter to start robot grasping, or Ctrl-C to abort safely: '
    if [[ ! -t 0 ]]; then
        printf '\nERROR: live pre-motion confirmation requires an interactive terminal; use --no-wait-before-motion only for intentional automation\n' >&2
        exit 2
    fi
    if ! IFS= read -r _operator_confirmation; then
        printf '\nERROR: stdin closed before pre-motion confirmation; no robot motion was sent\n' >&2
        exit 2
    fi
    printf '[full-flow] operator confirmed; starting robot motion\n'
else
    printf '[full-flow] WARNING: pre-motion Enter confirmation disabled by --no-wait-before-motion\n'
fi

printf '[full-flow] cached %d tube(s); executing left-to-right\n' "${#TUBE_RESULTS[@]}"
GRASP_FAILURE_COUNT=0
INSERT_FAILURE_COUNT=0
RESET_AFTER_ERROR_COUNT=0

reset_after_tube_error() {
    local tube_number="$1"
    local failed_stage="$2"
    local reset_log="$ARTIFACT_DIR/reset_after_tube_$(printf '%02d' "$tube_number")_${failed_stage}.log"
    if [[ "$RESET_AFTER_TUBE_ERROR" != "1" ]]; then
        printf 'WARNING: reset after error is disabled; continuing without reset\n' >&2
        return 0
    fi
    printf '[full-flow] tube %d: %s failed; running full robot reset before continuing\n' \
        "$tube_number" "$failed_stage" >&2
    printf '[full-flow] reset opens grippers and returns both arms home; log=%s\n' "$reset_log" >&2
    if "$ROBOT_RESET_SCRIPT" \
            --mode live \
            --hardware-allowed \
            --execute \
            --config "$ROBOT_RESET_CONFIG" \
            2>&1 | tee -a "$reset_log"; then
        RESET_AFTER_ERROR_COUNT=$((RESET_AFTER_ERROR_COUNT + 1))
        "$LOCATOR_PYTHON" - "$WRIST_CAMERA_SOCKET" "$SCRIPT_DIR" <<'PY'
import sys
from pathlib import Path
sys.path.insert(0, sys.argv[2])
from wrist_camera_service import WristCameraClient
client = WristCameraClient(Path(sys.argv[1]), timeout_sec=6.0)
available = sorted(client.health()["cameras"])
discarded = client.flush()["discarded_through_generation"]
for side in available:
    client.frame(side, max_age_ms=500.0, after_generation=int(discarded[side]))
print(f"[full-flow] RealSense cache flushed; fresh post-reset frames={available}")
PY
        printf '[full-flow] tube %d: reset completed; continuing with next cached tube\n' \
            "$tube_number"
        return 0
    fi
    printf 'ERROR: tube %d: robot reset failed after %s; refusing further robot motion\n' \
        "$tube_number" "$failed_stage" >&2
    return 1
}

for tube_array_index in "${!TUBE_RESULTS[@]}"; do
    tube_number=$((tube_array_index + 1))
    tube_result="${TUBE_RESULTS[$tube_array_index]}"
    require_fresh_realsense_frames "before_tube_${tube_number}_motion"
    printf '[full-flow] tube %d/%d: select arm, grasp, and hand over\n' \
        "$tube_number" "${#TUBE_RESULTS[@]}"
    GRASP_CMD=(
        python3 "$SELECT_AND_GRASP"
        --result-json "$tube_result"
        --mode live
        --hardware-allowed
        --execute
        --grasp-arg=--go-home-before-transition
    )
    for arg in "${GRASP_ARGS[@]}"; do
        GRASP_CMD+=(--grasp-arg "$arg")
    done
    if "${GRASP_CMD[@]}"; then
        printf '[full-flow] tube %d/%d: grasp and handover completed\n' \
            "$tube_number" "${#TUBE_RESULTS[@]}"
    else
        grasp_returncode=$?
        GRASP_FAILURE_COUNT=$((GRASP_FAILURE_COUNT + 1))
        printf 'WARNING: tube %d/%d grasp/handover returned %d\n' \
            "$tube_number" "${#TUBE_RESULTS[@]}" "$grasp_returncode" >&2
        if reset_after_tube_error "$tube_number" "grasp_handover"; then
            continue
        fi
        exit 1
    fi

    if [[ "$INSERT_LOG_FILE" == *.log ]]; then
        tube_insert_log="${INSERT_LOG_FILE%.log}_tube_$(printf '%02d' "$tube_number").log"
    else
        tube_insert_log="${INSERT_LOG_FILE}_tube_$(printf '%02d' "$tube_number").log"
    fi
    tube_insert_artifact_dir="$ARTIFACT_DIR/insertion_tube_$(printf '%02d' "$tube_number")"
    target_slot_id=""
    if [[ "$WRIST_PERCEPTION_MODE" == "cached-grid" ]]; then
        if ! target_slot_id=$("$LOCATOR_PYTHON" "$RACK_GRID_STATE" --grid "$ARTIFACT_DIR/rack_grid.json" --reserve-next); then
            printf 'ERROR: no confirmed empty rack slot remains for tube %d\n' "$tube_number" >&2
            exit 1
        fi
        printf '[full-flow] tube %d/%d: reserved rack slot %s\n' "$tube_number" "${#TUBE_RESULTS[@]}" "$target_slot_id"
    fi
    printf '[full-flow] tube %d/%d: use cached rack, locate current empty hole, insert, release, and retract\n' \
        "$tube_number" "${#TUBE_RESULTS[@]}"
    INSERT_CMD=(
        python3 "$INSERT_FLOW"
        --mode live
        --hardware-allowed
        --execute
        --holder-side auto
        --rack-result-json "$RACK_RESULT_JSON"
        --wrist-perception-mode "$WRIST_PERCEPTION_MODE"
        --wrist-camera-socket "$WRIST_CAMERA_SOCKET"
        --log-file "$tube_insert_log"
        --artifact-dir "$tube_insert_artifact_dir"
    )
    if [[ -n "$target_slot_id" ]]; then
        INSERT_CMD+=(
            --rack-grid-json "$ARTIFACT_DIR/rack_grid.json"
            --target-slot-id "$target_slot_id"
            --wrist-perception-report "$ARTIFACT_DIR/wrist_perception_tube_$(printf '%02d' "$tube_number").json"
        )
    fi
    INSERT_CMD+=("${INSERTION_ARGS[@]}")
    if "${INSERT_CMD[@]}"; then
        if [[ -n "$target_slot_id" ]]; then
            "$LOCATOR_PYTHON" "$RACK_GRID_STATE" --grid "$ARTIFACT_DIR/rack_grid.json" --slot "$target_slot_id" --mark occupied >/dev/null
        fi
        printf '[full-flow] tube %d/%d: insertion flow completed\n' \
            "$tube_number" "${#TUBE_RESULTS[@]}"
    else
        insert_returncode=$?
        INSERT_FAILURE_COUNT=$((INSERT_FAILURE_COUNT + 1))
        if [[ -n "$target_slot_id" ]]; then
            "$LOCATOR_PYTHON" "$RACK_GRID_STATE" --grid "$ARTIFACT_DIR/rack_grid.json" --slot "$target_slot_id" --mark unknown >/dev/null
        fi
        printf 'WARNING: tube %d/%d insertion returned %d; recorded in %s\n' \
            "$tube_number" "${#TUBE_RESULTS[@]}" "$insert_returncode" "$tube_insert_log" >&2
        if reset_after_tube_error "$tube_number" "insertion"; then
            continue
        fi
        exit 1
    fi
done

printf '[full-flow] COMPLETE: processed %d cached tube(s) left-to-right; grasp_failures=%d insertion_failures=%d resets_after_error=%d\n' \
    "${#TUBE_RESULTS[@]}" "$GRASP_FAILURE_COUNT" "$INSERT_FAILURE_COUNT" "$RESET_AFTER_ERROR_COUNT"
printf '[full-flow] log=%s\n' "$FULL_LOG_FILE"
if [[ "$GRASP_FAILURE_COUNT" -gt 0 || "$INSERT_FAILURE_COUNT" -gt 0 ]]; then
    exit 1
fi
