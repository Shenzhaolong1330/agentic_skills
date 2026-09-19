"""Offline orchestration and real leaf-parser contracts; no hardware connections."""
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch

import yaml
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
SKILL = ROOT / "task_skills/demo_NO/skills/task-configurable-manipulation-demos"
SCRIPTS = SKILL / "scripts"
spec = importlib.util.spec_from_file_location("demo_NO", SCRIPTS / "demo_common_runner.py")
demo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(demo)


def config(number=1):
    return yaml.safe_load((SKILL / "config" / f"{demo.DEMO_NAMES[number]}.yaml").read_text())


def detection():
    return {"found": True, "points_base": {"available": True,
        "head": {"base": dict(x_m=0.5, y_m=-0.2, z_m=0.1)},
        "tail": {"base": dict(x_m=0.5, y_m=0.2, z_m=0.1)}}}


class DemoTests(unittest.TestCase):
    def test_demo4_generated_locator_configs_use_configured_reasoning(self):
        with tempfile.TemporaryDirectory() as temp:
            task = self.run_mock(config(4), temp)
            generated = list((Path(temp) / 'configs').glob('cap_*_right.yaml'))
            self.assertEqual(len(generated), 4)
            for path in generated:
                self.assertEqual(yaml.safe_load(path.read_text())['openrouter']['reasoning_effort'], 'low')

    def test_inventory_only_returns_before_prompt_or_robot_commands(self):
        for mode in ("mock", "dry_run"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as temp:
                task = demo.Demo(config(), SKILL / "config/demo_1_vial_insert_extract.yaml", temp, mode,
                                 stop_after_inventory=True)
                with patch.object(demo.subprocess, "run", side_effect=AssertionError("offline must not execute")), \
                     patch.object(task, "pause", side_effect=AssertionError("must stop before motion prompt")), \
                     patch.object(task, "grasp", side_effect=AssertionError("must not grasp")), \
                     redirect_stdout(io.StringIO()):
                    task.run()
                self.assertFalse(any(e["stage"].startswith(("grasp_", "open_", "insert_", "extract_")) for e in task.events))
        with tempfile.TemporaryDirectory() as temp:
            self.assertEqual(demo.main(["--config", str(SKILL / "config/demo_1_vial_insert_extract.yaml"),
                                       "--mock", "--stop-after-inventory", "--artifact-dir", temp]), 0)
            report = demo.read_json(Path(temp) / "task_result.json")
            self.assertTrue(report["inventory_complete"])
            self.assertFalse(report["completion_flag"])

    def run_mock(self, c, folder):
        task = demo.Demo(c, SKILL / "config" / f"{demo.DEMO_NAMES[c['demo']]}.yaml", folder, "mock")
        with patch.object(demo.subprocess, "run", side_effect=AssertionError("offline must not spawn children")), redirect_stdout(io.StringIO()):
            task.run()
        return task

    def test_grasp_both_fifths_and_endpoint_side(self):
        tail, axis, arm = demo.grasp_geometry(detection(), 1, "auto")
        head, front_axis, front_arm = demo.grasp_geometry(detection(), 2, "auto")
        self.assertAlmostEqual(tail[1], 0.12)
        self.assertAlmostEqual(head[1], -0.12)
        self.assertEqual((arm, front_arm), ("left", "right"))
        self.assertEqual(axis, front_axis)
        self.assertEqual(demo.grasp_geometry(detection(), 2, "left")[2], "left")

    def test_invalid_perception_never_falls_back_to_center(self):
        for mutate in (lambda d: d.update(found=False),
                       lambda d: d["points_base"].pop("head"),
                       lambda d: d["points_base"]["tail"]["base"].update(x_m=float("nan")),
                       lambda d: d["points_base"].update(tail=d["points_base"]["head"])):
            data = detection()
            mutate(data)
            with self.assertRaises(ValueError):
                demo.grasp_geometry(data, 1, "left")

    def test_validate_slots_and_occupancy(self):
        demo.validate(config())
        for bad in ([0, 1], [1, 3], [True, 1], [1.5, 1], "centre"):
            with self.assertRaises(ValueError):
                demo.slot(bad)
        c = config()
        c["cycles"][1]["insert"] = [1, 1]
        with self.assertRaisesRegex(ValueError, "已占用"):
            demo.validate(c)
        c = config()
        c["cycles"][0]["extract"] = [2, 1]
        with self.assertRaisesRegex(ValueError, "空孔"):
            demo.validate(c)
        c["initial_occupied"] = [[2, 1]]
        demo.validate(c)

    def test_hold_requires_single_object(self):
        c = config(2)
        c["after_handover"] = "hold"
        with self.assertRaises(ValueError):
            demo.validate(c)
        c["count"] = 1
        with tempfile.TemporaryDirectory() as temp:
            task = self.run_mock(c, temp)
            self.assertFalse(any(e["stage"].startswith("drop") for e in task.events))

    def test_all_insertions_before_extraction_same_arm_no_second_handover(self):
        c = config()
        c["cycles"][1]["grasp_position"] = 2
        with tempfile.TemporaryDirectory() as temp:
            task = self.run_mock(c, temp)
        stages = [e["stage"] for e in task.events]
        self.assertLess(stages.index("insert_4"), stages.index("extract_observe_1"))
        for i in range(1, 5):
            geometry = next(e for e in task.events if e["stage"] == f"grasp_geometry_{i}")
            side = geometry["receiver"]
            extract = next(e for e in task.events if e["stage"] == f"extract_{i}_{side}")
            self.assertIn("--no-return-transition", extract["argv"])
            self.assertIn(f"drop_{i}_{side}", stages)
            self.assertLess(stages.index(f"bind_wrist_calibration_{i}"), stages.index(f"extract_cap_{i}"))
        self.assertEqual(sum(s.startswith("grasp_handover_") for s in stages), 4)

    def test_failure_stops_before_next_object_and_extraction(self):
        with tempfile.TemporaryDirectory() as temp:
            task = demo.Demo(config(), SKILL / "config/demo_1_vial_insert_extract.yaml", temp, "mock")
            original = task.command
            def fail(name, argv):
                if name == "grasp_handover_2":
                    raise RuntimeError("simulated receiver-close failure")
                return original(name, argv)
            with patch.object(task, "command", side_effect=fail), redirect_stdout(io.StringIO()), self.assertRaises(RuntimeError):
                task.run()
            self.assertFalse(any(e["stage"] in ("insert_2", "grasp_handover_3", "extract_observe_1") for e in task.events))

    def test_dry_run_has_no_fake_xyz_and_no_child_process(self):
        with tempfile.TemporaryDirectory() as temp:
            task = demo.Demo(config(), SKILL / "config/demo_1_vial_insert_extract.yaml", temp, "dry_run")
            with patch.object(demo.subprocess, "run", side_effect=AssertionError()), redirect_stdout(io.StringIO()):
                task.run()
            self.assertFalse(any("xyz" in e for e in task.events if e["stage"].startswith("grasp")))
            c = yaml.safe_load((Path(temp) / "configs/hole_1_left.yaml").read_text())
            self.assertIn("exactly row 1, column 1", c["target"]["description"])
            self.assertIn("found=false", c["target"]["description"])
            self.assertFalse(c["detector"]["fallback_to_color_on_vlm_mismatch"])
            self.assertTrue(Path(c["calibration"]["file"]).is_absolute())

    def test_view_rotation_supports_sideways_rack(self):
        c = config()
        c["wrist_view"]["left"] = dict(row1_at="left", col1_at="bottom")
        demo.validate(c)
        text = demo.target_description("左下", c["wrist_view"]["left"], True)
        self.assertIn("row 3, column 1", text)
        self.assertIn("row 1 is at the left", text)
        c["wrist_view"]["left"]["col1_at"] = "right"
        with self.assertRaises(ValueError):
            demo.validate(c)

    def test_leaf_command_arguments_match_existing_parsers(self):
        # Importing these parsers does not instantiate RPC clients or cameras.
        sys.path.insert(0, str(demo.OLD))
        modules = {}
        with tempfile.TemporaryDirectory() as temp:
            task = self.run_mock(config(), str(Path(temp) / "demo1"))
            extraction = self.run_mock(config(4), str(Path(temp) / "demo4"))
            for event in task.events + extraction.events:
                argv = event.get("argv", [])
                if len(argv) < 2:
                    continue
                path = Path(argv[1])
                leaf_args = argv[2:]
                if path.name == "demo_1_insert.py":
                    adapter_spec = importlib.util.spec_from_file_location("contract_demo_insert", path)
                    adapter = importlib.util.module_from_spec(adapter_spec)
                    adapter_spec.loader.exec_module(adapter)
                    adapter_args, leaf_args = adapter.parse_adapter_args(leaf_args)
                    self.assertIsNotNone(adapter_args.output_json)
                    path = demo.OLD / "run_manual_grip_to_insert.py"
                if path.parent != demo.OLD:
                    continue
                if path.name not in modules:
                    name = "contract_" + path.stem
                    spec = importlib.util.spec_from_file_location(name, path)
                    module = importlib.util.module_from_spec(spec)
                    sys.modules[name] = module
                    spec.loader.exec_module(module)
                    modules[path.name] = module
                parser = modules[path.name].build_parser()
                parser.parse_args(leaf_args)

    def test_insert_adapter_binds_latest_observation_before_wrist(self):
        from collections import namedtuple
        spec = importlib.util.spec_from_file_location("demo_insert", SCRIPTS / "demo_1_insert.py")
        adapter = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(adapter)
        Stage = namedtuple("Stage", "name argv")
        executed = []
        dispatch = adapter.calibrated_dispatch(lambda command: executed.append(command))
        wrist = Stage("move_above_hole_from_wrist", ["python", "wrist.py", "--execute", "--hole-config", "/old.yaml", "--artifact-dir", "/run/wrist"])
        with self.assertRaises(RuntimeError):
            dispatch(wrist)
        dispatch(Stage("observe_rack", ["python", "observe.py", "--execute", "--artifact-dir", "/run/observation"]))
        self.assertIn("--no-yield-non-holder-arm", executed[-1].argv)
        with patch.object(adapter.subprocess, "run") as bind:
            dispatch(wrist)
            self.assertIn("/run/observation/object_locator_runtime", bind.call_args.args[0])
            self.assertTrue(bind.call_args.kwargs["check"])
        self.assertIn("/run/wrist/current_wrist_hole.yaml", executed[-1].argv)
        self.assertIn("/old.yaml", wrist.argv)  # original command remains immutable

    def test_live_gate_fails_before_invoking_any_command(self):
        for args in (["--mode", "live"], ["--mock", "--execute"], ["--execute"]):
            result = subprocess.run([sys.executable, str(SCRIPTS / "demo_common_runner.py"),
                                     "--config", str(SKILL / "config/demo_1_vial_insert_extract.yaml"), *args],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)

    def test_openrouter_402_is_reported_and_stderr_preserved(self):
        error = 'object_locator.openrouter_vlm.OpenRouterError: OpenRouter returned HTTP 402: {"error":{"message":"Insufficient credits"}}'
        with tempfile.TemporaryDirectory() as temp:
            task = demo.Demo(config(), SKILL / "config/demo_1_vial_insert_extract.yaml", temp, "live")
            fake_result = subprocess.CompletedProcess(["fake"], 1, "", error)
            with patch.object(demo, "run_logged_command", return_value=fake_result), redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(RuntimeError, "OpenRouter HTTP 402.*余额不足"):
                    task.command("inventory", ["fake_inventory"])
            self.assertEqual((Path(temp) / "inventory_stderr.txt").read_text(), error)
            self.assertIn(error, (Path(temp) / "demo.log").read_text())
        self.assertEqual(demo.failure_reason("RuntimeError: camera unavailable", ""), "RuntimeError: camera unavailable")
        reason = demo.failure_reason("OpenRouter request failed: SSLEOFError; initial HTTP 400", "")
        self.assertIn("TLS", reason)
        self.assertIn("HTTP 400", reason)

    def test_renamed_entries_and_copied_full_flow_work_outside_repo(self):
        for entry in ("demo_1_vial_insert_extract", "demo_2_elongated_handover", "demo_3_all_vials_to_rack"):
            with self.subTest(entry=entry), tempfile.TemporaryDirectory() as temp:
                result = subprocess.run(["bash", str(SCRIPTS / f"{entry}.sh"), "--dry-run", "--artifact-dir", str(Path(temp) / "run")],
                                        cwd=temp, capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                if entry.startswith("demo_3"):
                    self.assertIn(str(demo.OLD / "task_pick_tube_insert_rack_runner.py"), result.stdout)

    def test_p2p_speeds_are_forwarded_and_can_be_overridden(self):
        c = config(2)
        c["count"] = 1
        c["after_handover"] = "hold"
        c["p2p"]["max_translation_speed"] = 0.10
        with tempfile.TemporaryDirectory() as temp:
            task = self.run_mock(c, temp)
        argv = next(e["argv"] for e in task.events if e["stage"] == "grasp_handover_1")
        for flag, value in (("--max-translation-speed", 0.10), ("--max-rotation-speed", 0.6),
                            ("--rate-hz", 80), ("--max-translation-step", 0.003),
                            ("--approach-max-translation-speed", 0.08)):
            self.assertEqual(float(argv[argv.index(flag) + 1]), value)
        # Motion speed changes do not relax grasp arrival tolerances.
        self.assertEqual(argv[argv.index("--position-tolerance-m") + 1], "0.005")

    def test_invalid_p2p_config_rejected_before_motion(self):
        for value in (-0.1, 0, float("nan"), True, "fast"):
            c = config()
            c["p2p"]["max_translation_speed"] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                demo.validate(c)
        c = config()
        c["p2p"]["max_translaton_speed"] = 0.12
        with self.assertRaises(ValueError):
            demo.validate(c)

    def test_handover_wait_configuration_reaches_grasp_helper(self):
        c = config()
        c["handover_release_delay_sec"] = 3.0
        with tempfile.TemporaryDirectory() as temp:
            task = self.run_mock(c, temp)
        command = next(e["argv"] for e in task.events if e["stage"] == "grasp_handover_1")
        self.assertNotIn("--go-home-after-transfer", command)
        self.assertEqual(command[command.index("--retreat-after-transfer-m") + 1], "0.2")
        for flag, expected in (("--after-partner-close-sleep-sec", "3.0"),
                               ("--partner-gripper-stable-sec", "0.5"),
                               ("--partner-gripper-close-timeout-sec", "5.0")):
            self.assertEqual(command[command.index(flag) + 1], expected)
        for value in (-1, float("nan"), True):
            c["handover_release_delay_sec"] = value
            with self.assertRaises(ValueError):
                demo.validate(c)

    def test_retreat_config_and_both_demo_commands(self):
        for number in (1, 2):
            c = config(number)
            c["handover_retreat_distance_m"] = .15
            with tempfile.TemporaryDirectory() as temp:
                task = self.run_mock(c, temp)
            for event in task.events:
                if event["stage"].startswith("grasp_handover_"):
                    command = event["argv"]
                    self.assertNotIn("--go-home-after-transfer", command)
                    self.assertEqual(command[command.index("--retreat-after-transfer-m") + 1], "0.15")
        for value in (-.2, 0, float("nan"), True, "0.2"):
            c = config()
            c["handover_retreat_distance_m"] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                demo.validate(c)

    def test_rpc_failure_uses_stderr_and_never_replays_motion(self):
        stderr = "Traceback (most recent call last):\nzerorpc.exceptions.LostRemote: Lost remote after 10s heartbeat\n"
        stdout = "=== stage2_align_gripper_to_orientation_left ===\np2p attempt: 1/up-to-2\n"
        reason = demo.failure_reason(stderr, stdout)
        self.assertIn("Lost remote after 10s heartbeat", reason)
        self.assertIn("stage2_align_gripper", reason)
        self.assertNotIn("p2p attempt", reason)
        diagnostic = 'p2p_rpc_failure: {"waypoint": 17, "last_confirmed_step_count": 1780}'
        self.assertIn(diagnostic, demo.failure_reason(diagnostic + "\n" + stderr, stdout))
        with tempfile.TemporaryDirectory() as temp:
            task = demo.Demo(config(), SKILL / "config/demo_1_vial_insert_extract.yaml", temp, "live")
            fake_result = subprocess.CompletedProcess(["fake"], 1, stdout, stderr)
            with patch.object(demo, "run_logged_command", return_value=fake_result) as run, redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(RuntimeError, "Lost remote"):
                    task.command("grasp_handover_1", ["fake_grasp"])
                self.assertEqual(run.call_count, 1)
        self.assertEqual(demo.failure_reason("ValueError: actual failure", "p2p attempt: 1/up-to-2"), "ValueError: actual failure")

    def test_extract_observation_corrections_preserve_tolerances(self):
        c = config()
        with tempfile.TemporaryDirectory() as temp:
            task = self.run_mock(c, temp)
        commands = [e["argv"] for e in task.events if e["stage"].startswith("extract_observe_")]
        self.assertEqual(len(commands), 4)
        for command in commands:
            for flag, value in (("--max-correction-iters", "10"),
                                ("--position-tolerance-m", "0.003"),
                                ("--rotation-tolerance-rad", "0.03")):
                self.assertEqual(command[command.index(flag) + 1], value)
            self.assertEqual(command.count("--no-yield-non-holder-arm"), 1)
        for value in (0, -1, True, 1.5, "10"):
            c["extract_observe_max_correction_iters"] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                demo.validate(c)

    def test_camera_reset_exhaustion_is_not_hidden_by_last_uvc_error(self):
        stderr = ('[tube-insertion] RealSense reset attempt 3/3 failed\n'
                  'ERROR: RealSense did not recover after 3 forced reset attempt(s) for config=head.yaml\n'
                  'stdout:\nERROR: set_xu(...). xioctl(UVCIOC_CTRL_QUERY) failed on control 1 Last Error: Protocol error\n')
        reason = demo.failure_reason(stderr, '')
        self.assertIn('3 次相机复位仍未恢复', reason)
        self.assertIn('Protocol error', reason)
        self.assertIn('USB', reason)

    def test_compensation_limit_failure_explains_why_more_iterations_will_not_help(self):
        stdout = json.dumps({"stopped_reason": "move_holder_to_wrist_observation_pose_failed", "stages": [
            {"result": {"ok": False, "stalled": {"reason": "tracking_compensation_limit",
                "right_bias": {"translation_m": .025, "rotation_rad": .20,
                               "max_translation_m": .025, "max_rotation_rad": .20}}}}]})
        reason = demo.failure_reason('', stdout)
        self.assertIn('tracking_compensation_limit', reason)
        self.assertIn('25.0000/25.0000 mm', reason)
        self.assertIn('0.2000/0.2000 rad', reason)

    def test_extract_observation_failure_reports_residual_and_stops_before_grasp(self):
        stdout = json.dumps({"stopped_reason": "align_holder_tcp_vertical_failed", "stages": [
            {"stage": "yield_non_holder_arm", "skipped": True},
            {"stage": "align_holder_tcp_vertical", "result": {"ok": False, "final_error": {
                "max_translation_error_m": .01202918, "max_rotation_error_rad": .0534234,
                "correction_index": 1}}}]})
        stderr = "[tube-insertion] using cached initial rack detection: /tmp/rack.json\n"
        reason = demo.failure_reason(stderr, stdout)
        self.assertIn("align_holder_tcp_vertical_failed", reason)
        self.assertIn("12.0292 mm", reason)
        self.assertIn("0.0534 rad", reason)
        self.assertIn("correction_index=1", reason)
        self.assertNotIn("cached", reason)
        self.assertEqual(demo.failure_reason("RuntimeError: RPC failed", stdout), "RuntimeError: RPC failed")
        with tempfile.TemporaryDirectory() as temp:
            task = demo.Demo(config(), SKILL / "config/demo_1_vial_insert_extract.yaml", temp, "live")
            result = subprocess.CompletedProcess(["fake"], 3, stdout, stderr)
            with patch.object(task, "locate", return_value=Path(temp) / "rack.json"), \
                 patch.object(demo, "run_logged_command", return_value=result) as run, redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(RuntimeError, "align_holder_tcp_vertical_failed"):
                    task.extract(1, "right", [1, 1])
                self.assertEqual(run.call_count, 1)
                self.assertEqual([e['stage'] for e in task.events], ["extract_observe_1"])

    def test_extract_only_orientation_is_base_axis_not_rack_axis(self):
        for choice in (1, 2):
            for axis in ("x", "y"):
                rotation = Rotation.from_rotvec(demo.extract_tcp_rotvec(choice, axis))
                opening = rotation.apply([1, 0, 0] if axis == "x" else [0, 1, 0])
                self.assertAlmostEqual(abs(opening[0]), 0 if choice == 1 else 1)
                self.assertAlmostEqual(opening[2], 0)
                np.testing.assert_allclose(rotation.apply([0, 0, 1]), [0, 0, -1], atol=1e-12)

    def test_extract_only_respects_order_side_and_orientation_without_insert_handover(self):
        for side in ("left", "right"):
            for choice in (1, 2):
                c = config(4)
                c.update(extract_arm=side, gripper_opening_direction=choice,
                         extract_order=["右下", "左上", "右上", "左下"])
                with tempfile.TemporaryDirectory() as temp:
                    task = self.run_mock(c, temp)
                selections = [e for e in task.events if e['stage'].startswith('extract_selection_')]
                self.assertEqual([e['position'] for e in selections], c['extract_order'])
                self.assertEqual([e['side'] for e in selections], [side] * 4)
                self.assertFalse(any(e['stage'].startswith(('inventory', 'insert_', 'grasp_handover')) for e in task.events))
                for i in range(1, 5):
                    observe = next(e['argv'] for e in task.events if e['stage'] == f'extract_observe_{i}')
                    self.assertEqual(observe[observe.index('--max-correction-iters') + 1], '20')
                    self.assertNotIn('--jaw-perpendicular-to-rack', observe)
                    np.testing.assert_allclose(json.loads(observe[observe.index('--aligned-tcp-rotvec') + 1]),
                                               demo.extract_tcp_rotvec(choice, c['jaw_opening_axis']))
                    extract = next(e['argv'] for e in task.events if e['stage'] == f'extract_{i}_{side}')
                    self.assertIn('--keep-current-rotation', extract)
                    self.assertIn('--no-return-transition', extract)
                    drop = next(e['argv'] for e in task.events if e['stage'] == f'drop_{i}_{side}')
                    self.assertNotIn('--go-home-after-success', drop)

    def test_extract_only_rejects_invalid_order_and_orientation_before_motion(self):
        for updates in ({'extract_order': ['左上'] * 4}, {'extract_order': ['左上', '左下', '右上']},
                        {'extract_order': ['左上', '左下', '右上', '中间']},
                        {'gripper_opening_direction': True}, {'gripper_opening_direction': 3},
                        {'extract_arm': 'auto'}):
            c = {**config(4), **updates}
            with self.subTest(updates=updates), self.assertRaises(ValueError):
                demo.validate(c)

    def test_extract_rack_occluded_depth_rejected_before_observation(self):
        with tempfile.TemporaryDirectory() as temp:
            task = demo.Demo(config(4), SKILL / 'config/demo_4_vial_extract.yaml', temp, 'mock')
            original = task.locate
            def occluded(name, conf, **kwargs):
                path = original(name, conf, **kwargs)
                data = demo.read_json(path)
                # Regression: gripper depth during the 15:29 run yielded +8.9 cm rack Z.
                data['position_base']['z_m'] = .0893439066
                data['points_base']['bbox_center']['base']['z_m'] = .0893439066
                demo.write_json(path, data)
                return path
            with patch.object(task, 'locate', side_effect=occluded), redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(ValueError, '红架高度异常'):
                    task.run()
            self.assertFalse(any(e['stage'].startswith(('extract_observe', 'open_for_extract')) for e in task.events))

    def test_extract_reuses_reference_and_rejects_cap_outside_rack_before_open(self):
        with tempfile.TemporaryDirectory() as temp:
            task = self.run_mock(config(4), temp)
            rack_commands = [e for e in task.events if e['stage'].startswith('rack_before_extract') and 'argv' in e]
            self.assertEqual(len(rack_commands), 1)
            reused = [e for e in task.events if e['stage'].startswith('reuse_rack_reference_')]
            self.assertEqual(len(reused), 3)
            self.assertTrue(all('argv' not in e for e in reused))
            observes = [e['argv'] for e in task.events if e['stage'].startswith('extract_observe')]
            self.assertEqual(len({a[a.index('--rack-result-json') + 1] for a in observes}), 1)
        with tempfile.TemporaryDirectory() as temp:
            task = demo.Demo(config(4), SKILL / 'config/demo_4_vial_extract.yaml', temp, 'mock')
            original = task.locate
            def bad_cap(name, conf, **kwargs):
                path = original(name, conf, **kwargs)
                if name == 'extract_cap_1':
                    data = demo.read_json(path)
                    data['points_base']['bbox_center']['base']['z_m'] = .2
                    demo.write_json(path, data)
                return path
            with patch.object(task, 'locate', side_effect=bad_cap), redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(ValueError, '拔管目标偏离'):
                    task.run()
            self.assertFalse(any(e['stage'].startswith('open_for_extract') for e in task.events))

    def test_drop_must_be_right_and_cannot_fall_back_to_front(self):
        c = config(4)
        rack = [.4956, .0011, -.2024]
        demo.validate_right_drop([.49, -.20, -.23], rack, c, surface=True)
        for xyz in ([.3321, .0101, -.2221], [.49, .20, -.23], [.49, -.5, -.23],
                    [.1, -.20, -.23], [.49, -.20, .1], [.49, float('nan'), -.23]):
            with self.subTest(xyz=xyz), self.assertRaises(ValueError):
                demo.validate_right_drop(xyz, rack, c, surface=True)

    def test_rack_misidentified_as_table_is_rejected_in_image_space(self):
        rack = {'intrinsics': {'width': 640, 'height': 480},
                'detection': {'bbox': dict(x_min=353., y_min=148., x_max=452., y_max=261.)}}
        wrong = {'intrinsics': rack['intrinsics'],
                 'detection': {'bbox': dict(x_min=371.2, y_min=168., x_max=403.2, y_max=216.)}}
        with self.assertRaisesRegex(ValueError, '红架图像右侧'):
            demo.validate_drop_image(wrong, rack)
        good = {'intrinsics': rack['intrinsics'],
                'detection': {'bbox': dict(x_min=467.2, y_min=168., x_max=531.2, y_max=216.)}}
        demo.validate_drop_image(good, rack)
        with self.assertRaises(ValueError):
            demo.validate_drop_image({**good, 'intrinsics': {'width': 1280, 'height': 720}}, rack)

    def test_rejected_drop_relocalizes_before_exactly_one_drop_command(self):
        with tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()):
            task = demo.Demo(config(4), SKILL / 'config/demo_4_vial_extract.yaml', temp, 'mock')
            task.extract_rack_reference = task.locate('rack_before_extract_1', task.rack_config())
            task.extract_rack_xyz = demo.validate_extract_rack(demo.read_json(task.extract_rack_reference), task.c)
            original = task.locate
            def bad_first(name, conf, **kwargs):
                path = original(name, conf, **kwargs)
                if name == 'drop_area_2':
                    data = demo.read_json(path)
                    data['detection']['bbox'].update(x_min=300, x_max=330)
                    demo.write_json(path, data)
                return path
            with patch.object(task, 'locate', side_effect=bad_first) as locate:
                task.drop(2, 'right')
            self.assertEqual(locate.call_count, 2)
            drops = [e for e in task.events if e['stage'] == 'drop_2_right']
            self.assertEqual(len(drops), 1)
            self.assertIn(str(Path(temp) / 'drop_area_2_retry_1.json'), drops[0]['argv'])
            self.assertFalse(any(e['stage'].startswith(('extract_2', 'open_')) for e in task.events))
            retry_config = yaml.safe_load((Path(temp) / 'configs/drop_2_retry_1.yaml').read_text())
            self.assertIn('Previous candidate was REJECTED', retry_config['target']['description'])

    def test_repeated_invalid_drop_stops_before_motion(self):
        c = config(4)
        with tempfile.TemporaryDirectory() as temp:
            task = demo.Demo(c, SKILL / 'config/demo_4_vial_extract.yaml', temp, 'mock')
            original = task.locate
            def front_drop(name, conf, **kwargs):
                path = original(name, conf, **kwargs)
                if name.startswith('drop_area'):
                    data = demo.read_json(path)
                    data['points_base']['bbox_center']['base']['y_m'] = 0
                    demo.write_json(path, data)
                return path
            with patch.object(task, 'locate', side_effect=front_drop), redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(ValueError, '落料点不在红架右侧'):
                    task.run()
            self.assertFalse(any(e['stage'] in ('drop_1_right', 'extract_selection_2') for e in task.events))
        with tempfile.TemporaryDirectory() as temp:
            task = self.run_mock(config(4), temp)
        for event in task.events:
            if event['stage'].startswith('drop_') and event['stage'].endswith('_right'):
                argv = event['argv']
                self.assertIn('--keep-current-rotation', argv)
                self.assertEqual(argv[argv.index('--retract-distance-m') + 1], '0')
                self.assertEqual(argv[argv.index('--max-release-rise-m') + 1], '0.005')

    def test_observation_tolerance_is_separate_from_extraction_precision(self):
        c = config(4)
        with tempfile.TemporaryDirectory() as temp:
            task = self.run_mock(c, temp)
        spec = importlib.util.spec_from_file_location('extraction_tolerance_grasp', demo.OLD / 'grasp_right_arm_xyz.py')
        grasp = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(grasp)
        for i in range(1, 5):
            observe = next(e['argv'] for e in task.events if e['stage'] == f'extract_observe_{i}')
            self.assertEqual(float(observe[observe.index('--position-tolerance-m') + 1]), .005)
            self.assertEqual(float(observe[observe.index('--rotation-tolerance-rad') + 1]), .06)
            self.assertEqual(float(observe[observe.index('--settle-time-sec') + 1]), .5)
            extract = next(e['argv'] for e in task.events if e['stage'] == f'extract_{i}_right')
            parsed = grasp.build_parser().parse_args(extract[2:])
            self.assertEqual(parsed.position_tolerance_m, .003)
            self.assertEqual(parsed.rotation_tolerance_rad, .03)
            self.assertEqual(parsed.lift_position_tolerance_m, .005)
            self.assertEqual(parsed.lift_rotation_tolerance_rad, .06)
            self.assertEqual(parsed.pregrasp_xy_position_tolerance_m, .005)
            self.assertEqual(parsed.pregrasp_xy_rotation_tolerance_rad, .06)
            self.assertEqual(parsed.pregrasp_xy_settle_time_sec, .3)
            self.assertEqual(parsed.approach_settle_time_sec, .6)
            self.assertEqual(parsed.settle_time_sec, .4)
            drop = next(e['argv'] for e in task.events if e['stage'] == f'drop_{i}_right')
            self.assertEqual(float(drop[drop.index('--position-tolerance-m') + 1]), .010)
            self.assertEqual(float(drop[drop.index('--rotation-tolerance-rad') + 1]), .10)
            self.assertEqual(float(drop[drop.index('--settle-time-sec') + 1]), .3)
        for key in ('extract_observe_position_tolerance_m', 'extract_observe_rotation_tolerance_rad',
                    'extract_position_tolerance_m', 'extract_rotation_tolerance_rad',
                    'extract_lift_position_tolerance_m', 'extract_lift_rotation_tolerance_rad',
                    'drop_position_tolerance_m', 'drop_rotation_tolerance_rad',
                    'extract_observe_settle_time_sec', 'extract_pregrasp_xy_settle_time_sec',
                    'extract_approach_settle_time_sec', 'extract_lift_settle_time_sec', 'drop_settle_time_sec'):
            for value in (0, -.1, True, float('nan'), float('inf'), '0.005'):
                invalid = {**c, key: value}
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    demo.validate(invalid)

    def test_demo4_speed_settings_reach_observe_extract_and_drop(self):
        c = config(4)
        c['p2p']['max_translation_speed'] = .09
        c['p2p']['max_rotation_speed'] = .45
        with tempfile.TemporaryDirectory() as temp:
            task = self.run_mock(c, temp)
        for index in range(1, 5):
            for stage in (f'extract_observe_{index}', f'extract_{index}_right', f'drop_{index}_right'):
                argv = next(e['argv'] for e in task.events if e['stage'] == stage)
                self.assertEqual(float(argv[argv.index('--max-translation-speed') + 1]), .09)
                self.assertEqual(float(argv[argv.index('--max-rotation-speed') + 1]), .45)
                self.assertEqual(float(argv[argv.index('--rate-hz') + 1]), 80)
                if stage == f'extract_{index}_right':
                    self.assertEqual(float(argv[argv.index('--approach-max-translation-speed') + 1]), .04)
                else:
                    self.assertNotIn('--approach-max-translation-speed', argv)

    def test_extract_only_dry_run_and_shell_work_without_hardware(self):
        with tempfile.TemporaryDirectory() as temp:
            entry = SCRIPTS / 'demo_4_vial_extract.sh'
            result = subprocess.run(['bash', str(entry), '--dry-run', '--artifact-dir', temp],
                                    cwd='/tmp', capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            report = demo.read_json(Path(temp) / 'task_result.json')
            self.assertFalse(report['completion_flag'])
            plan = demo.read_json(Path(temp) / 'command_plan.json')
            self.assertEqual([e['position'] for e in plan if e['stage'].startswith('extract_') and 'position' in e],
                             config(4)['extract_order'])

    def test_extract_only_pauses_after_perception_and_stops_on_observation_failure(self):
        with tempfile.TemporaryDirectory() as temp:
            task = demo.Demo(config(4), SKILL / 'config/demo_4_vial_extract.yaml', temp, 'mock')
            original = task.command
            def command(name, argv):
                if name == 'extract_observe_1':
                    raise RuntimeError('observed motion failed')
                return original(name, argv)
            def pause():
                self.assertEqual(task.events[-1]['stage'], 'rack_before_extract_1')
            with patch.object(task, 'command', side_effect=command), patch.object(task, 'pause', side_effect=pause) as paused, \
                 redirect_stdout(io.StringIO()), self.assertRaisesRegex(RuntimeError, 'observed motion failed'):
                task.run()
            paused.assert_called_once()
            self.assertFalse(any(e['stage'].startswith(('open_for_extract', 'extract_selection_2', 'drop_')) for e in task.events))


if __name__ == "__main__":
    unittest.main()
