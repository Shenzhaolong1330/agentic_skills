"""Regression tests for post-handover behavior and noisy insertion stdout; no hardware."""
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
SKILL = ROOT / "task_skills/demo_NO/skills/task-configurable-manipulation-demos"
OLD = ROOT / "task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts"


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


demo = load("demo_insert_result_runner", SKILL / "scripts/demo_common_runner.py")
adapter = load("demo_insert_result_adapter", SKILL / "scripts/demo_1_insert.py")
insertion = load("demo_insert_result_skill", OLD / "tube_insertion_skill.py")


class InsertResultTests(unittest.TestCase):
    def test_adapter_writes_report_despite_stdout_progress_and_restores_hooks(self):
        report = {"completion_flag": True, "completion_status": "completed_with_insert_tolerance_warning",
                  "insert_tolerance_warning": "depth tolerance warning", "achieved_insert_depth_m": .05453}
        old_dispatch = lambda command: None
        def original_print(value, *, compact):
            print(json.dumps(value))
        flow = SimpleNamespace(run_stage=old_dispatch, print_report=original_print)
        def run(argv):
            self.assertEqual(argv, ["--execute", "--holder-side", "right"])
            print('到达识别孔上方error: {"final_error": {}}')
            print('向下插入error: {"guarded_insert": {"ok": false}}')
            flow.print_report(report, compact=False)
            return 0
        flow.main = run
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "insert/result.json"
            with patch.dict(sys.modules, {"run_manual_grip_to_insert": flow}), redirect_stdout(io.StringIO()):
                self.assertEqual(adapter.main(["--output-json", str(output), "--execute", "--holder-side", "right"]), 0)
                self.assertEqual(json.loads(output.read_text()), report)
                self.assertIs(flow.run_stage, old_dispatch)
                self.assertIs(flow.print_report, original_print)
                with self.assertRaises(FileExistsError):
                    adapter.main(["--output-json", str(output), "--execute"])

    def test_runner_uses_file_and_never_repeats_insertion_when_result_is_bad(self):
        good = {"completion_flag": True, "completion_status": "completed_with_insert_tolerance_warning",
                "insert_tolerance_warning": "preserve warning"}
        for payload in (good, {"completion_flag": False}, {"completion_flag": "true"}, "invalid", None):
            with self.subTest(payload=payload), tempfile.TemporaryDirectory() as temp:
                config_path = SKILL / "config/demo_1_vial_insert_extract.yaml"
                task = demo.Demo(yaml.safe_load(config_path.read_text()), config_path, temp, "live")
                def command(name, argv):
                    if payload is not None:
                        target = Path(argv[argv.index("--output-json") + 1])
                        target.parent.mkdir(parents=True, exist_ok=True)
                        target.write_text(payload if isinstance(payload, str) else json.dumps(payload))
                    return '到达识别孔上方error: {}\n向下插入error: {}\n{"completion_flag":true}'
                with patch.object(task, "locate", return_value=Path(temp) / "rack.json"), \
                     patch.object(task, "command", side_effect=command) as invoked, redirect_stdout(io.StringIO()):
                    if payload is good:
                        task.insert(1, "right", [1, 1])
                        self.assertEqual(task.events[-1]["insert_tolerance_warning"], "preserve warning")
                    else:
                        with self.assertRaisesRegex(RuntimeError, "do not replay"):
                            task.insert(1, "right", [1, 1])
                    self.assertEqual(invoked.call_count, 1)

    def test_home_reanchors_targets_without_reset_or_gripper_commands(self):
        calls = []
        class Client:
            def go_home(self, side, duration, rate):
                calls.append(("home", side))
                return {"ok": True}
            def step(self, action):
                calls.append(("step", action))
                return {"observation": {side: {"end_pose": [0.] * 6} for side in ("left_arm", "right_arm")}}
            def reset(self):
                raise AssertionError("reset must not be used while the partner holds an object")
        report = insertion.move_side_to_home(client=Client(), side="left", duration_sec=4.,
                                             rate_hz=50., execute=True, stage_name="test_home")
        self.assertEqual(calls, [("home", "left"), ("step", None)])
        self.assertTrue(report["result"]["ok"])
        self.assertEqual(report["cartesian_target_reanchor"]["method"], "rpc_step_none_target_to_current")


if __name__ == "__main__":
    unittest.main()
