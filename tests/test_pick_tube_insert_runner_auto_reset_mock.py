import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/task_pick_tube_insert_rack_runner.py"


class PickTubeInsertRunnerAutoResetMockTests(unittest.TestCase):
    def test_mock_abnormal_preflight_auto_reset_then_complete(self):
        with tempfile.TemporaryDirectory(prefix="agentic_mock_reset_") as tmp:
            out = Path(tmp) / "task_result.json"
            completed = subprocess.run([
                sys.executable, str(RUNNER),
                "--mode", "mock",
                "--mock-robot-health", "abnormal",
                "--auto-reset-on-abnormal",
                "--artifact-dir", tmp,
                "--output-json", str(out),
            ], text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            data = json.loads(out.read_text())
            self.assertTrue(data["completion_flag"])
            self.assertFalse(data["physical_verified"])
            resets = data["reset_recovery"]
            self.assertEqual(resets[0]["outcome"], "MOCK_RESET_OK")
            self.assertTrue((Path(tmp) / "reset_recovery_1.json").exists())

    def test_dry_run_abnormal_plans_reset(self):
        with tempfile.TemporaryDirectory(prefix="agentic_dry_reset_") as tmp:
            out = Path(tmp) / "task_result.json"
            completed = subprocess.run([
                sys.executable, str(RUNNER),
                "--mode", "dry_run",
                "--mock-robot-health", "abnormal",
                "--auto-reset-on-abnormal",
                "--artifact-dir", tmp,
                "--output-json", str(out),
            ], text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            data = json.loads(out.read_text())
            self.assertFalse(data["physical_verified"])
            self.assertEqual(data["reset_recovery"][0]["outcome"], "PLANNED_ONLY")
            plans = json.loads((Path(tmp) / "command_plan.json").read_text())
            self.assertTrue(plans)


if __name__ == "__main__":
    unittest.main()
