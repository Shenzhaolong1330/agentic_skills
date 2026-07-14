#!/usr/bin/env python3
"""Atomically reserve or finalize one task-local rack-grid slot."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import tempfile

from rack_grid import reserve_next_empty, transition_slot


def save_atomic(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as file:
            json.dump(payload, file, ensure_ascii=False, indent=2)
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--grid", type=Path, required=True)
    operation = parser.add_mutually_exclusive_group(required=True)
    operation.add_argument("--reserve-next", action="store_true")
    operation.add_argument("--mark", choices=("occupied", "unknown"))
    parser.add_argument("--slot")
    args = parser.parse_args()
    grid = json.loads(args.grid.read_text(encoding="utf-8"))
    if args.reserve_next:
        identifier = reserve_next_empty(grid)
    else:
        if not args.slot:
            parser.error("--mark requires --slot")
        transition_slot(grid, args.slot, args.mark)
        identifier = args.slot
    save_atomic(args.grid, grid)
    print(identifier)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
