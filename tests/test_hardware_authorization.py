import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts"


class HardwareAuthorizationTests(unittest.TestCase):
    def run_wrapper(self, name: str, *args: str):
        with tempfile.TemporaryDirectory(prefix="gen_agent_wrapper_", dir="/tmp") as tmp:
            result = subprocess.run(
                [str(SCRIPTS / name), *args, "--artifact-dir", tmp],
                cwd=ROOT,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            payload_path = Path(tmp) / "task_result.json"
            payload = json.loads(payload_path.read_text(encoding="utf-8")) if payload_path.exists() else None
            return result, payload

    def test_live_execute_without_hardware_allowed_fails_before_service_start(self):
        for name in ("run_single_pick_tube_insert_rack.sh", "run_full_pick_tube_insert_rack.sh"):
            with self.subTest(wrapper=name):
                result, _ = self.run_wrapper(name, "--mode", "live", "--execute")
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("hardware_allowed_required", result.stdout + result.stderr)
                self.assertNotIn("starting persistent", result.stdout + result.stderr)

    def test_live_hardware_allowed_without_execute_fails_before_service_start(self):
        for name in ("run_single_pick_tube_insert_rack.sh", "run_full_pick_tube_insert_rack.sh"):
            with self.subTest(wrapper=name):
                result, _ = self.run_wrapper(name, "--mode", "live", "--hardware-allowed")
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("execute_required_for_side_effects", result.stdout + result.stderr)
                self.assertNotIn("starting persistent", result.stdout + result.stderr)

    def test_mock_and_dry_run_never_start_hardware_services(self):
        for name in ("run_single_pick_tube_insert_rack.sh", "run_full_pick_tube_insert_rack.sh"):
            for mode in ("mock", "dry_run"):
                with self.subTest(wrapper=name, mode=mode):
                    result, payload = self.run_wrapper(name, "--mode", mode)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertNotIn("starting persistent", result.stdout + result.stderr)
                    self.assertIsNotNone(payload)
                    self.assertFalse(payload["context"]["hardware_allowed"])

    def test_preflight_helper_is_side_effect_free(self):
        command = [
            sys.executable,
            str(ROOT / "scripts/hardware_preflight.py"),
            "--operation",
            "task",
            "--mode",
            "live",
            "--hardware-allowed",
            "--execute",
        ]
        result = subprocess.run(command, cwd=ROOT, text=True, capture_output=True, check=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        payload = json.loads(result.stdout)
        self.assertTrue(payload["decision"]["allowed"])
        self.assertEqual(payload["decision"]["reason"], "hardware_gate_passed")

    def test_internal_grasp_and_insert_operations_are_gated(self):
        for operation, args, reason in (
            ("grasp", ("--execute",), "hardware_allowed_required"),
            ("insert", ("--hardware-allowed",), "execute_required_for_side_effects"),
        ):
            with self.subTest(operation=operation):
                result = subprocess.run(
                    [
                        sys.executable,
                        str(ROOT / "scripts/hardware_preflight.py"),
                        "--operation",
                        operation,
                        "--mode",
                        "live",
                        *args,
                    ],
                    cwd=ROOT,
                    text=True,
                    capture_output=True,
                    check=False,
                )
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(reason, result.stdout)


if __name__ == "__main__":
    unittest.main()
