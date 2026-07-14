import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/task_pick_tube_insert_rack_runner.py"


class PickTubeInsertRunnerMockTests(unittest.TestCase):
    def test_mock_runner_completes_without_hardware(self):
        with tempfile.TemporaryDirectory(prefix="agentic_mock_") as tmp:
            out = Path(tmp) / "task_result.json"
            completed = subprocess.run([sys.executable, str(RUNNER), "--mode", "mock", "--artifact-dir", tmp, "--output-json", str(out)], text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            data = json.loads(out.read_text())
            self.assertTrue(data["completion_flag"])
            self.assertFalse(data["physical_verified"])
            self.assertTrue((Path(tmp) / "command_plan.json").exists())


if __name__ == "__main__":
    unittest.main()
