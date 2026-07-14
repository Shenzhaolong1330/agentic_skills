#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNNER="$SCRIPT_DIR/task_pick_tube_insert_rack_runner.py"

MODE="dry_run"
ARTIFACT_DIR=""
OUTPUT_JSON=""
AUTO_RESET=1
MAX_AUTO_RESET_ATTEMPTS=1
EXECUTE=0
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode) MODE="$2"; shift 2 ;;
        --artifact-dir) ARTIFACT_DIR="$2"; shift 2 ;;
        --output-json) OUTPUT_JSON="$2"; shift 2 ;;
        --execute) EXECUTE=1; MODE="${MODE:-live}"; shift ;;
        --auto-reset-on-abnormal) AUTO_RESET=1; shift ;;
        --disable-auto-reset) AUTO_RESET=0; shift ;;
        --max-auto-reset-attempts) MAX_AUTO_RESET_ATTEMPTS="$2"; shift 2 ;;
        --dry-run) MODE="dry_run"; shift ;;
        --mock) MODE="mock"; shift ;;
        --from-artifacts) MODE="from_artifacts"; shift ;;
        *) EXTRA_ARGS+=("$1"); shift ;;
    esac
done

CMD=(python3 "$RUNNER" --mode "$MODE" --max-auto-reset-attempts "$MAX_AUTO_RESET_ATTEMPTS")
if [[ -n "$ARTIFACT_DIR" ]]; then CMD+=(--artifact-dir "$ARTIFACT_DIR"); fi
if [[ -n "$OUTPUT_JSON" ]]; then CMD+=(--output-json "$OUTPUT_JSON"); fi
if [[ "$EXECUTE" == "1" ]]; then CMD+=(--execute); fi
if [[ "$AUTO_RESET" == "1" ]]; then CMD+=(--auto-reset-on-abnormal); else CMD+=(--disable-auto-reset); fi
CMD+=("${EXTRA_ARGS[@]}")

printf '[pick_tube_insert_rack] safe wrapper command:'
printf ' %q' "${CMD[@]}"
printf '\n'
exec "${CMD[@]}"
