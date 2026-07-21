from __future__ import annotations

from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]


def test_capability_index_is_deterministic_and_current():
    command = [sys.executable, "scripts/generate_capability_index.py", "--check"]
    first = subprocess.run(command, cwd=ROOT, text=True, capture_output=True, check=False)
    second = subprocess.run(command, cwd=ROOT, text=True, capture_output=True, check=False)
    assert first.returncode == 0
    assert second.returncode == 0
    assert first.stdout == second.stdout
    index = (ROOT / "CAPABILITY_INDEX.md").read_text(encoding="utf-8")
    assert "motion.move_to_pose" in index
    assert "internal.task." not in index
    assert "/home/" not in index
    assert "shell" + "=True" not in index
    assert "physical verification limited: `true`" in index
