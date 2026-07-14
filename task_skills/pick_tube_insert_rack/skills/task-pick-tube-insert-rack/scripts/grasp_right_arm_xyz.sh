#!/usr/bin/env bash
set -euo pipefail

ARM="${ARM:-right}"
RESULT_JSON="${RESULT_JSON:-/home/deepcybo/agentic_skills/atomic_skills/object_locator/runs/latest_grasp_result.json}"
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
        *)
            EXTRA_ARGS+=("$1")
            shift
            ;;
    esac
done

cd "$SCRIPT_DIR"

# User-approved relaxed grasp position tolerance (25 mm).
python3 "$SCRIPT_DIR/grasp_right_arm_xyz.py" \
    --arm "$ARM" \
    --result-json "$RESULT_JSON" \
    --result-grasp-point tail_to_head_1_5 \
    --result-base-frame base \
    --directional-orientation \
    --settle-time-sec 0.7 \
    --max-correction-iters 10 \
    --max-translation-speed 0.12 \
    --max-rotation-speed 0.6 \
    --max-translation-step 0.003 \
    --max-rotation-step 0.03 \
    --approach-max-translation-speed 0.08 \
    --approach-max-rotation-speed 0.4 \
    --approach-max-translation-step 0.002 \
    --approach-max-rotation-step 0.02 \
    --approach-settle-time-sec 1.0 \
    --grasp-arrival-observed-z-offset-m 0 \
    --rate-hz 80 \
    --position-tolerance-m 0.015 \
    --rotation-tolerance-rad 0.05 \
    --compact \
    "${EXTRA_ARGS[@]}"
