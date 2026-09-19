#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$TASK_ROOT/../../../.." && pwd)"
OBJECT_LOCATOR_ROOT="${OBJECT_LOCATOR_ROOT:-$REPO_ROOT/atomic_skills/object_locator}"

RACK_LOCATOR="${RACK_LOCATOR:-$OBJECT_LOCATOR_ROOT/.venv/bin/object-locator}"
LOCATOR_PYTHON="${LOCATOR_PYTHON:-$OBJECT_LOCATOR_ROOT/.venv/bin/python}"
RACK_CONFIG="${RACK_CONFIG:-$OBJECT_LOCATOR_ROOT/config_rack_center_vlm.yaml}"
CAP_CONFIG=""
PLACE_CONFIG="${PLACE_CONFIG:-$OBJECT_LOCATOR_ROOT/config_empty_table_place_vlm.yaml}"

TUBE_INSERTION="$SCRIPT_DIR/tube_insertion_skill.py"
GRASP_SCRIPT="$SCRIPT_DIR/grasp_right_arm_xyz.py"
GRIPPER_SCRIPT="$REPO_ROOT/atomic_skills/dual_franka_gripper/skills/atomic-gripper-franka-open-close/scripts/gripper_control.py"
PLACE_SCRIPT="$SCRIPT_DIR/place_held_object_on_table.py"
PREPARE_CAP_CONFIG="$SCRIPT_DIR/prepare_runtime_wrist_locator_config.py"

MODE="dry_run"
EXECUTE=0
HOLDER_SIDE="right"
PLACE_SIDE="left"
ARTIFACT_DIR=""
PLACE_XYZ=""
OBSERVE_HEIGHT_M="0.12"
OBSERVE_OFFSET="[0.0, 0.0, 0.0]"
WAIT_BEFORE_OBSERVATION=1
WAIT_BEFORE_GRASP=0
STOP_AFTER_OBSERVATION=0
STOP_AFTER_CAP=0
GRASP_LIFT_M="0.10"
RETRACT_DISTANCE_M="0.10"
RELEASE_SLEEP_SEC="0.6"
PLACE_Z_OFFSET_M="0.03"

RACK_LOCATOR_ARGS=()
CAP_LOCATOR_ARGS=()
PLACE_LOCATOR_ARGS=()
GRASP_ARGS=()
PLACE_ARGS=()

usage() {
    cat <<'EOF'
Usage:
  run_pick_yellow_cap_to_table.sh --dry-run
  run_pick_yellow_cap_to_table.sh --mode live --execute

Flow:
  head object-locator rack detection
  -> move only the selected holder arm directly to the rack observation pose in vertical posture
  -> wrist object-locator orange-cap detection with bbox-corner 3D depth
  -> open/grasp, lift 10 cm, go through swapped-side transition, hand over
  -> VLM-selected blank table area with the partner arm, release, retract

By default VLM selects the blank table area. Use --place-xyz '[x,y,z]' to
override that selection with a manually verified TCP release position.

Options:
  --holder-side left|right       Wrist camera and arm used for observation/grasp (default: right).
  --rack-config PATH              Head-camera rack locator config.
  --cap-config PATH               Wrist orange-cap locator config; selected from holder side by default.
  --place-config PATH             Head-camera blank-table VLM config.
  --artifact-dir DIR              Run artifacts and logs.
  --wait-before-observation       Pause after rack detection before the first arm motion (default).
  --no-wait-before-observation    Disable that operator gate.
  --wait-before-grasp              Pause after orange-cap detection before grasp motion.
  --stop-after-observation        Stop after the single-arm observation move.
  --stop-after-cap                Stop after cap detection; do not grasp.
  --observe-height-m M            TCP height above detected rack position (default: 0.12).
  --observe-offset '[x,y,z]'      Base-frame rack observation offset (default: [0,0,0]).
  --grasp-lift-m M                Lift after confirmed grasp (default: 0.05).
  --retract-distance-m M          Upward retract after release (default: 0.10).
  --release-sleep-sec S           Wait after opening gripper (default: 0.6).
  --place-z-offset-m M            Add this height above the VLM table surface (default: 0.10 m).
  --rack-locator-arg ARG          Extra argument for head object-locator; repeatable.
  --cap-locator-arg ARG           Extra argument for wrist object-locator; repeatable.
  --place-locator-arg ARG         Extra argument for blank-table object-locator; repeatable.
  --grasp-arg ARG                 Extra argument for grasp_right_arm_xyz.py; repeatable.
  --place-arg ARG                 Extra argument for place_held_object_on_table.py; repeatable.
  --dry-run                        Safe plan-only mode (default).
  --mode MODE                     dry_run or live; live additionally requires --execute.
  --execute                       Enable real cameras, robot motion, and grippers.
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
        --dry-run|--mock) MODE="dry_run"; shift ;;
        --execute) EXECUTE=1; shift ;;
        --holder-side) require_value "$@"; HOLDER_SIDE="$2"; shift 2 ;;
        --place-xyz) require_value "$@"; PLACE_XYZ="$2"; shift 2 ;;
        --rack-config) require_value "$@"; RACK_CONFIG="$2"; shift 2 ;;
        --cap-config) require_value "$@"; CAP_CONFIG="$2"; shift 2 ;;
        --place-config) require_value "$@"; PLACE_CONFIG="$2"; shift 2 ;;
        --artifact-dir) require_value "$@"; ARTIFACT_DIR="$2"; shift 2 ;;
        --wait-before-observation) WAIT_BEFORE_OBSERVATION=1; shift ;;
        --no-wait-before-observation) WAIT_BEFORE_OBSERVATION=0; shift ;;
        --wait-before-grasp) WAIT_BEFORE_GRASP=1; shift ;;
        --stop-after-observation) STOP_AFTER_OBSERVATION=1; shift ;;
        --stop-after-cap) STOP_AFTER_CAP=1; shift ;;
        --observe-height-m) require_value "$@"; OBSERVE_HEIGHT_M="$2"; shift 2 ;;
        --observe-offset) require_value "$@"; OBSERVE_OFFSET="$2"; shift 2 ;;
        --grasp-lift-m) require_value "$@"; GRASP_LIFT_M="$2"; shift 2 ;;
        --retract-distance-m) require_value "$@"; RETRACT_DISTANCE_M="$2"; shift 2 ;;
        --release-sleep-sec) require_value "$@"; RELEASE_SLEEP_SEC="$2"; shift 2 ;;
        --place-z-offset-m) require_value "$@"; PLACE_Z_OFFSET_M="$2"; shift 2 ;;
        --rack-locator-arg) require_value "$@"; RACK_LOCATOR_ARGS+=("$2"); shift 2 ;;
        --cap-locator-arg) require_value "$@"; CAP_LOCATOR_ARGS+=("$2"); shift 2 ;;
        --place-locator-arg) require_value "$@"; PLACE_LOCATOR_ARGS+=("$2"); shift 2 ;;
        --grasp-arg) require_value "$@"; GRASP_ARGS+=("$2"); shift 2 ;;
        --place-arg) require_value "$@"; PLACE_ARGS+=("$2"); shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) printf 'ERROR: unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
    esac
done

case "$MODE" in
    dry_run|live) ;;
    *) printf 'ERROR: unsupported --mode: %s\n' "$MODE" >&2; exit 2 ;;
esac
case "$HOLDER_SIDE" in
    left) PLACE_SIDE="right" ;;
    right) PLACE_SIDE="left" ;;
    *) printf 'ERROR: --holder-side must be left or right\n' >&2; exit 2 ;;
esac
if [[ "$MODE" == "live" && "$EXECUTE" != "1" ]]; then
    printf 'ERROR: live mode requires --execute\n' >&2
    exit 2
fi
if [[ "$MODE" != "live" && "$EXECUTE" == "1" ]]; then
    printf 'ERROR: --execute is only valid with --mode live\n' >&2
    exit 2
fi

if [[ -z "$CAP_CONFIG" ]]; then
    if [[ "$HOLDER_SIDE" == "left" ]]; then
        CAP_CONFIG="$OBJECT_LOCATOR_ROOT/config_yellow_tube_cap_left_wrist.yaml"
    else
        CAP_CONFIG="$OBJECT_LOCATOR_ROOT/config_yellow_tube_cap_right_wrist.yaml"
    fi
fi

if [[ -z "$ARTIFACT_DIR" ]]; then
    ARTIFACT_DIR="/tmp/agentic_skills_runs/pick_yellow_cap_to_table_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$ARTIFACT_DIR"
ARTIFACT_DIR="$(cd "$ARTIFACT_DIR" && pwd)"

RACK_RESULT_JSON="$ARTIFACT_DIR/rack_detection.json"
CAP_RESULT_JSON="$ARTIFACT_DIR/yellow_cap_detection.json"
CAP_RUNTIME_CONFIG="$ARTIFACT_DIR/yellow_cap_runtime_config.yaml"
PLACE_RESULT_JSON="$ARTIFACT_DIR/empty_table_place_detection.json"
LOG_FILE="$ARTIFACT_DIR/pick_yellow_cap_to_table.log"

RACK_CMD=(
    "$RACK_LOCATOR" --config "$RACK_CONFIG" --json
    --result-json "$RACK_RESULT_JSON"
    --history-dir "$ARTIFACT_DIR/rack_history"
    --output "$ARTIFACT_DIR/rack_panel.jpg"
    --output-rgb "$ARTIFACT_DIR/rack_rgb.jpg"
    --output-depth "$ARTIFACT_DIR/rack_depth.jpg"
    --save-vlm-response "$ARTIFACT_DIR/rack_vlm_response.json"
)
RACK_CMD+=("${RACK_LOCATOR_ARGS[@]}")

OBSERVE_CMD=(
    python3 "$TUBE_INSERTION" --execute
    --any-pose-dir "$OBJECT_LOCATOR_ROOT"
    --artifact-dir "$ARTIFACT_DIR/observation"
    --rack-config "$RACK_CONFIG"
    --rack-result-json "$RACK_RESULT_JSON"
    --holder-side "$HOLDER_SIDE"
    --observe-height-m "$OBSERVE_HEIGHT_M"
    --observe-offset "$OBSERVE_OFFSET"
    --rate-hz 50
    --max-translation-speed 0.18
    --max-rotation-speed 0.5
    --max-translation-step 0.01
    --max-rotation-step 0.05
    --max-correction-iters 10
    --no-yield-non-holder-arm
    --direct-observation
    --stop-after-observe
)

CAP_CMD=(
    "$RACK_LOCATOR" --config "$CAP_RUNTIME_CONFIG" --json
    --result-json "$CAP_RESULT_JSON"
    --history-dir "$ARTIFACT_DIR/cap_history"
    --output "$ARTIFACT_DIR/yellow_cap_panel.jpg"
    --output-rgb "$ARTIFACT_DIR/yellow_cap_rgb.jpg"
    --output-depth "$ARTIFACT_DIR/yellow_cap_depth.jpg"
    --save-vlm-response "$ARTIFACT_DIR/yellow_cap_vlm_response.json"
    --save-depth "$ARTIFACT_DIR/yellow_cap_depth.npy"
)
CAP_CMD+=("${CAP_LOCATOR_ARGS[@]}")

PLACE_LOCATOR_CMD=(
    "$RACK_LOCATOR" --config "$PLACE_CONFIG" --json
    --result-json "$PLACE_RESULT_JSON"
    --history-dir "$ARTIFACT_DIR/place_history"
    --output "$ARTIFACT_DIR/empty_table_place_panel.jpg"
    --output-rgb "$ARTIFACT_DIR/empty_table_place_rgb.jpg"
    --output-depth "$ARTIFACT_DIR/empty_table_place_depth.jpg"
    --save-vlm-response "$ARTIFACT_DIR/empty_table_place_vlm_response.json"
)
PLACE_LOCATOR_CMD+=("${PLACE_LOCATOR_ARGS[@]}")

OPEN_CMD=(python3 "$GRIPPER_SCRIPT" open --side "$HOLDER_SIDE" --execute)
GRASP_CMD=(
    python3 "$GRASP_SCRIPT"
    --arm "$HOLDER_SIDE"
    --result-json "$CAP_RESULT_JSON"
    --result-grasp-point bbox_center
    --result-base-frame base
    --no-result-orientation
    --rate-hz 80
    --max-translation-speed 0.12
    --max-rotation-speed 0.6
    --max-translation-step 0.003
    --max-rotation-step 0.03
    --settle-time-sec 0.7
    --approach-max-translation-speed 0.08
    --approach-max-rotation-speed 0.4
    --approach-max-translation-step 0.002
    --approach-max-rotation-step 0.02
    --approach-settle-time-sec 1.0
    --grasp-target-z-offset-m 0.01
    --lift-after-grasp-m "$GRASP_LIFT_M"
    --go-home-before-transition
    --go-home-after-transfer
    --transition-side "$PLACE_SIDE"
    --position-tolerance-m 0.005
    --rotation-tolerance-rad 0.05
    --transition-rotation-tolerance-rad 0.06
    --max-correction-iters 10
    --p2p-retries 9
    --execute
)
GRASP_CMD+=("${GRASP_ARGS[@]}")

PLACE_CMD=(
    python3 "$PLACE_SCRIPT"
    --side "$PLACE_SIDE"
    --retract-distance-m "$RETRACT_DISTANCE_M"
    --release-sleep-sec "$RELEASE_SLEEP_SEC"
    --down-pitch-rad 0.0
    --rate-hz 80
    --max-translation-speed 0.06
    --max-rotation-speed 0.4
    --max-translation-step 0.003
    --max-rotation-step 0.02
    --settle-time-sec 0.5
    --max-correction-iters 10
    --release-on-motion-failure
    --go-home-after-motion-failure
    --go-home-after-success
    --execute
)
if [[ -n "$PLACE_XYZ" ]]; then
    PLACE_CMD+=(--place-xyz "$PLACE_XYZ")
else
    PLACE_CMD+=(--place-result-json "$PLACE_RESULT_JSON" --place-z-offset-m "$PLACE_Z_OFFSET_M")
fi
PLACE_CMD+=("${PLACE_ARGS[@]}")

print_cmd() {
    printf '%q ' "$@"
    printf '\n'
}

if [[ "$MODE" == "dry_run" ]]; then
    printf '[yellow-cap-flow] plan only; no camera, motion, or gripper command will run\n'
    printf '[yellow-cap-flow] rack locator: '; print_cmd "${RACK_CMD[@]}"
    printf '[yellow-cap-flow] observation: '; print_cmd "${OBSERVE_CMD[@]}"
    printf '[yellow-cap-flow] cap locator: '; print_cmd "${CAP_CMD[@]}"
    if [[ -z "$PLACE_XYZ" ]]; then
        printf '[yellow-cap-flow] blank-table locator: '; print_cmd "${PLACE_LOCATOR_CMD[@]}"
    fi
    printf '[yellow-cap-flow] open gripper: '; print_cmd "${OPEN_CMD[@]}"
    printf '[yellow-cap-flow] grasp: '; print_cmd "${GRASP_CMD[@]}"
    printf '[yellow-cap-flow] place/release: '; print_cmd "${PLACE_CMD[@]}"
    exit 0
fi

for required_file in "$RACK_LOCATOR" "$LOCATOR_PYTHON" "$RACK_CONFIG" "$CAP_CONFIG" "$TUBE_INSERTION" "$GRASP_SCRIPT" "$GRIPPER_SCRIPT" "$PLACE_SCRIPT" "$PREPARE_CAP_CONFIG"; do
    if [[ ! -e "$required_file" ]]; then
        printf 'ERROR: required path does not exist: %s\n' "$required_file" >&2
        exit 2
    fi
done

if [[ -z "$PLACE_XYZ" ]]; then
    if [[ ! -e "$PLACE_CONFIG" ]]; then
        printf 'ERROR: required path does not exist: %s\n' "$PLACE_CONFIG" >&2
        exit 2
    fi
else
    "$LOCATOR_PYTHON" - "$PLACE_XYZ" <<'PY'
import json, math, sys
value = json.loads(sys.argv[1])
if not isinstance(value, list) or len(value) != 3 or not all(math.isfinite(float(x)) for x in value):
    raise SystemExit("--place-xyz must be a JSON list of 3 finite numbers")
PY
fi

exec > >(tee -a "$LOG_FILE") 2>&1
printf '[yellow-cap-flow] LIVE reverse rack-to-table flow\n'
if [[ -n "$PLACE_XYZ" ]]; then
    printf '[yellow-cap-flow] holder_side=%s manual_place_xyz_base=%s\n' "$HOLDER_SIDE" "$PLACE_XYZ"
else
    printf '[yellow-cap-flow] holder_side=%s placement=VLM_blank_table_area z_offset=%sm\n' "$HOLDER_SIDE" "$PLACE_Z_OFFSET_M"
fi
printf '[yellow-cap-flow] artifacts=%s\n' "$ARTIFACT_DIR"

printf '[yellow-cap-flow] stage 1: locate rack with head object-locator\n'
"${RACK_CMD[@]}"
"$LOCATOR_PYTHON" - "$RACK_RESULT_JSON" <<'PY'
import json, sys
payload = json.load(open(sys.argv[1], encoding="utf-8"))
if not payload.get("found"):
    raise SystemExit("rack locator did not find a rack")
base = payload.get("position_base")
if not isinstance(base, dict) or not base.get("available"):
    raise SystemExit("rack locator did not produce an available position_base")
print("[yellow-cap-flow] rack position_base=", [base.get(k) for k in ("x_m", "y_m", "z_m")])
PY

if [[ "$WAIT_BEFORE_OBSERVATION" == "1" ]]; then
    printf '[yellow-cap-flow] rack detected; press Enter to move only %s arm to observation, or Ctrl-C to abort: ' "$HOLDER_SIDE"
    if [[ ! -t 0 ]]; then
        printf '\nERROR: observation confirmation requires an interactive terminal; use --no-wait-before-observation for intentional automation\n' >&2
        exit 2
    fi
    IFS= read -r _confirmation
fi

printf '[yellow-cap-flow] stage 2: move only %s arm to rack observation pose\n' "$HOLDER_SIDE"
"${OBSERVE_CMD[@]}"
if [[ "$STOP_AFTER_OBSERVATION" == "1" ]]; then
    printf '[yellow-cap-flow] stopped after observation as requested\n'
    exit 0
fi

printf '[yellow-cap-flow] bind orange-cap locator to current observation flange calibration\n'
"$LOCATOR_PYTHON" "$PREPARE_CAP_CONFIG" \
    --source-config "$CAP_CONFIG" \
    --runtime-calibration-dir "$ARTIFACT_DIR/observation/object_locator_runtime" \
    --output-config "$CAP_RUNTIME_CONFIG"

printf '[yellow-cap-flow] stage 3: locate orange cap with wrist object-locator; bbox_corners 3D is configured\n'
"${CAP_CMD[@]}"
"$LOCATOR_PYTHON" - "$CAP_RESULT_JSON" <<'PY'
import json, sys
payload = json.load(open(sys.argv[1], encoding="utf-8"))
if not payload.get("found"):
    raise SystemExit("orange-cap locator did not find a cap")
base = payload.get("position_base")
if not isinstance(base, dict) or not base.get("available"):
    raise SystemExit("orange-cap locator did not produce an available position_base")
point = payload.get("points_base", {}).get("bbox_center")
if not isinstance(point, dict):
    raise SystemExit("orange-cap result has no points_base.bbox_center")
print("[yellow-cap-flow] grasp target bbox-corner mean base=", [point.get(k) for k in ("x_m", "y_m", "z_m")])
print("[yellow-cap-flow] depth strategy=", payload.get("position", {}).get("strategy"))
PY

if [[ "$STOP_AFTER_CAP" == "1" ]]; then
    printf '[yellow-cap-flow] stopped after cap detection as requested\n'
    exit 0
fi

if [[ -z "$PLACE_XYZ" ]]; then
    printf '[yellow-cap-flow] stage 3b: let VLM select an empty tabletop placement area\n'
    "${PLACE_LOCATOR_CMD[@]}"
    "$LOCATOR_PYTHON" - "$PLACE_RESULT_JSON" <<'PY'
import json, sys
payload = json.load(open(sys.argv[1], encoding="utf-8"))
if not payload.get("found"):
    raise SystemExit("blank-table locator did not find a placement area")
base = payload.get("position_base")
if not isinstance(base, dict) or not base.get("available"):
    raise SystemExit("blank-table locator did not produce an available position_base")
point = payload.get("points_base", {}).get("bbox_center")
if not isinstance(point, dict):
    raise SystemExit("blank-table result has no points_base.bbox_center")
print("[yellow-cap-flow] VLM selected table point base=", [point.get(k) for k in ("x_m", "y_m", "z_m")])
print("[yellow-cap-flow] place TCP z offset will be applied by place helper")
PY
fi

if [[ "$WAIT_BEFORE_GRASP" == "1" ]]; then
    printf '[yellow-cap-flow] inspect %s; press Enter to open/grasp, or Ctrl-C to abort: ' "$ARTIFACT_DIR/yellow_cap_panel.jpg"
    if [[ ! -t 0 ]]; then
        printf '\nERROR: grasp confirmation requires an interactive terminal; use automation flags intentionally\n' >&2
        exit 2
    fi
    IFS= read -r _grasp_confirmation
fi

printf '[yellow-cap-flow] stage 4: open %s gripper and grasp orange cap\n' "$HOLDER_SIDE"
"${OPEN_CMD[@]}"
"${GRASP_CMD[@]}"

printf '[yellow-cap-flow] stage 5: place with handover arm %s at blank table point, release, and retract\n' "$PLACE_SIDE"
"${PLACE_CMD[@]}"
printf '[yellow-cap-flow] COMPLETE; log=%s\n' "$LOG_FILE"
