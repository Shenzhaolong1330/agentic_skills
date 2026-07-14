import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch

from agentic_skills_harness.command_runner import CommandPlan, CommandResult


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT / "task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts"
RUNNER_PATH = SCRIPT_DIR / "task_pick_tube_insert_rack_runner.py"
FIXTURE = SCRIPT_DIR.parent / "mock/mock_tube_detection.json"


def load_runner():
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    spec = importlib.util.spec_from_file_location("task_pick_tube_insert_rack_runner_live_test", RUNNER_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RUNNER = load_runner()


class PickTubeInsertRunnerLiveGatedTests(unittest.TestCase):
    def test_live_missing_env_token_aborts_without_subprocess(self):
        with tempfile.TemporaryDirectory(prefix="agentic_live_gate_denied_") as tmp:
            args = RUNNER.build_parser().parse_args([
                "--mode", "live",
                "--execute",
                "--hardware-allowed",
                "--operator-token", "must-not-be-written",
                "--artifact-dir", tmp,
            ])
            with patch.dict(os.environ, {}, clear=False):
                os.environ.pop("AGENTIC_SKILLS_HARDWARE_TOKEN", None)
                with patch("agentic_skills_harness.command_runner.subprocess.run", side_effect=AssertionError("must not execute")):
                    result = RUNNER.run(args)
            self.assertFalse(result.ok)
            self.assertEqual(result.stopped_reason, "hardware_gate_denied")
            task = json.loads((Path(tmp) / "task_result.json").read_text())
            context = json.loads((Path(tmp) / "context.json").read_text())
            self.assertIsNone(task["context"]["operator_token"])
            self.assertIsNone(context["operator_token"])

    def test_mocked_live_commands_complete_through_command_runner(self):
        with tempfile.TemporaryDirectory(prefix="agentic_live_mocked_commands_") as tmp:
            args = RUNNER.build_parser().parse_args([
                "--mode", "live",
                "--execute",
                "--hardware-allowed",
                "--operator-token", "test-token",
                "--artifact-dir", tmp,
            ])

            def fake_run(_self, command, *, context, entrypoint_ref, entrypoint, recovery=False, **_kwargs):
                plan = CommandPlan(
                    argv=list(command),
                    entrypoint_ref=entrypoint_ref,
                    requires_hardware=True,
                    would_execute=True,
                    executed=True,
                    recovery=recovery,
                    reason="hardware gate passed",
                )
                if entrypoint_ref == "PREFLIGHT_ROBOT_HEALTH_CHECK":
                    raw = {"classification": "healthy", "needs_reset": False, "errors": [], "warnings": []}
                elif entrypoint_ref == "LOCATE_TUBE":
                    target = Path(command[command.index("--result-json") + 1])
                    shutil.copyfile(FIXTURE, target)
                    raw = json.loads(FIXTURE.read_text())
                elif entrypoint_ref == "LOCATE_RACK_HOLE_INSERT_RELEASE_RETRACT":
                    raw = {
                        "stages": [
                            {"name": "observe_rack", "returncode": 0},
                            {"name": "move_above_hole_from_wrist", "returncode": 0},
                            {"name": "insert_release_retract", "returncode": 0},
                        ]
                    }
                else:
                    raw = {"ok": True}
                return CommandResult(
                    plan=plan,
                    returncode=0,
                    stdout=json.dumps(raw),
                    executed=True,
                    raw_json=raw,
                    gate_decision={"allowed": True, "reason": "hardware gate passed"},
                )

            with patch.dict(os.environ, {"AGENTIC_SKILLS_HARDWARE_TOKEN": "test-token"}, clear=False):
                with patch.object(RUNNER.CommandRunner, "run", new=fake_run):
                    result = RUNNER.run(args)
            self.assertTrue(result.ok)
            self.assertEqual(result.task_state.value, "COMPLETE")
            self.assertTrue(result.completion_flag)
            self.assertTrue(result.physical_verified)
            task = json.loads((Path(tmp) / "task_result.json").read_text())
            plans = json.loads((Path(tmp) / "command_plan.json").read_text())
            self.assertIsNone(task["context"]["operator_token"])
            self.assertEqual([item["entrypoint_ref"] for item in plans], [
                "PREFLIGHT_ROBOT_HEALTH_CHECK",
                "LOCATE_TUBE",
                "GRASP_AND_HANDOVER",
                "LOCATE_RACK_HOLE_INSERT_RELEASE_RETRACT",
            ])
            self.assertTrue(all(item["executed"] for item in plans))


if __name__ == "__main__":
    unittest.main()
