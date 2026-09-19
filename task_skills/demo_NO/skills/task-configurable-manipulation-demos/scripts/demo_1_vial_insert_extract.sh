#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${DEMO_PYTHON:-python3}" "$SCRIPT_DIR/demo_common_runner.py" --config "$SCRIPT_DIR/../config/demo_1_vial_insert_extract.yaml" "$@"
