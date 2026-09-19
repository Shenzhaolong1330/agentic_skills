#!/usr/bin/env python3
"""Live-only inventory adapter; the gated demo runner owns invocation."""
from pathlib import Path
import sys


def main():
    # Import camera/model dependencies only when the live adapter is invoked.
    from dotenv import load_dotenv
    root = Path(__file__).resolve().parents[5]
    load_dotenv(root / "atomic_skills/object_locator/.env")
    sys.path.insert(0, str(root / "task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts"))
    from locate_all_tubes_once import main as inventory_main
    return inventory_main()


if __name__ == "__main__":
    raise SystemExit(main())
