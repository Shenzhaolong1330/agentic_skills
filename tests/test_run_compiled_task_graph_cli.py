from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]


def test_cli_exposes_only_offline_modes():
    command = [sys.executable, str(ROOT / "scripts/run_compiled_task_graph.py"), "--help"]
    output = subprocess.run(command, cwd=ROOT, text=True, capture_output=True, check=False)
    assert output.returncode == 0
    assert "--mode {mock,dry_run,from_artifacts}" in output.stdout
    assert "--execute" not in output.stdout
    assert "--hardware-allowed" not in output.stdout


def test_cli_rejects_live_mode_before_execution(tmp_path):
    compiled = Path("/tmp/agentic_skills_gen_agent/S07/precondition_s45_s6/compiled_examples/observe_object/compiled_task_graph.json")
    command = [sys.executable, str(ROOT / "scripts/run_compiled_task_graph.py"), "--compiled-graph", str(compiled), "--manifest", "skill_manifest.json", "--artifact-dir", str(tmp_path), "--mode", "live"]
    output = subprocess.run(command, cwd=ROOT, text=True, capture_output=True, check=False)
    assert output.returncode != 0
