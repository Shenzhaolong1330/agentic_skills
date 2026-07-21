#!/usr/bin/env bash
set -euo pipefail

ARM="${ARM:-right}"
RESULT_JSON="${RESULT_JSON:-/home/deepcybo/agentic_skills/atomic_skills/object_locator/runs/latest_grasp_result.json}"
STOP_AFTER_HANDOVER_CLOSE="${STOP_AFTER_HANDOVER_CLOSE:-0}"
STOP_BEFORE_HANDOVER_CLOSE="${STOP_BEFORE_HANDOVER_CLOSE:-0}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --arm)
            ARM="$2"
            shift 2
            ;;
        --result-json)
            RESULT_JSON="$2"
            shift 2
            ;;
        --stop-after-handover-close|--stop-after-handover-release)
            STOP_AFTER_HANDOVER_CLOSE=1
            shift
            ;;
        --stop-before-handover-close)
            STOP_BEFORE_HANDOVER_CLOSE=1
            shift
            ;;
        *)
            EXTRA_ARGS+=("$1")
            shift
            ;;
    esac
done

cd "$SCRIPT_DIR"

if [[ "$STOP_AFTER_HANDOVER_CLOSE" == "1" || "$STOP_AFTER_HANDOVER_CLOSE" == "true" ]]; then
    EXTRA_ARGS+=(--stop-after-partner-close)
fi
if [[ "$STOP_BEFORE_HANDOVER_CLOSE" == "1" || "$STOP_BEFORE_HANDOVER_CLOSE" == "true" ]]; then
    EXTRA_ARGS+=(--stop-before-partner-close)
fi

# User-approved relaxed grasp position tolerance (25 mm).
python3 -u "$SCRIPT_DIR/grasp_right_arm_xyz.py" \
    --arm "$ARM" \
    --result-json "$RESULT_JSON" \
    --result-grasp-point tail_to_head_1_5 \
    --result-base-frame base \
    --directional-orientation \
    --settle-time-sec 0.7 \
    --max-correction-iters 10 \
    --recover-stalled-controller \
    --controller-stall-translation-epsilon-m 0.0005 \
    --controller-stall-rotation-epsilon-rad 0.005 \
    --max-stalled-correction-iters 1 \
    --controller-recovery-settle-time-sec 1.0 \
    --max-translation-speed 0.12 \
    --max-rotation-speed 0.6 \
    --max-translation-step 0.003 \
    --max-rotation-step 0.03 \
    --approach-max-translation-speed 0.08 \
    --approach-max-rotation-speed 0.4 \
    --approach-max-translation-step 0.002 \
    --approach-max-rotation-step 0.02 \
    --approach-settle-time-sec 1.0 \
    --grasp-target-z-offset-m 0 \
    --grasp-arrival-observed-z-offset-m 0 \
    --rate-hz 80 \
    --position-tolerance-m 0.015 \
    --rotation-tolerance-rad 0.05 \
    --transition-rotation-tolerance-rad 0.06 \
    --compact \
    "${EXTRA_ARGS[@]}"
