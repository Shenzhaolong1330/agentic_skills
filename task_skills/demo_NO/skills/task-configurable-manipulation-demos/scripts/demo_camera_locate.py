#!/usr/bin/env python3
"""Reuse the original capture watchdog; this adapter never invokes robot methods."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[5]
OLD = ROOT / "task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts"
LOCATOR = ROOT / "atomic_skills/object_locator"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--config", type=Path, required=True)
    args, forwarded = parser.parse_known_args(argv)
    # Capture timeout/recovery has one owner in run_object_locator; do not
    # install a different default for head vs wrist or demo vs insertion.
    sys.path.insert(0, str(OLD))
    from tube_insertion_skill import run_object_locator
    try:
        result = run_object_locator(LOCATOR, args.config.resolve(), extra_args=forwarded)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
        return 1
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
