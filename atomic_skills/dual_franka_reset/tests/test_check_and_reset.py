from __future__ import annotations

import contextlib
import importlib.util
import io
import json
from pathlib import Path
import unittest
from unittest.mock import patch


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "skills"
    / "atomic-state-dual-franka-reset"
    / "scripts"
    / "check_and_reset.py"
)
SPEC = importlib.util.spec_from_file_location("check_and_reset_under_test", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
RESET = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RESET)


def make_state() -> dict:
    def make_side(x: float) -> dict:
        return {
            "robot_state": {
                "joint_positions": [x, 0.1, 0.2, -1.0, 0.3, 1.2, -0.4],
                "joint_velocities": [0.0] * 7,
                "eef_pose": {
                    "position": [0.5, x, 0.1],
                    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                },
            },
            "gripper": {"enabled": True},
        }

    return {"left_arm": make_side(0.2), "right_arm": make_side(-0.2)}


class EvaluateStateTest(unittest.TestCase):
    def test_robot_mode_enum_strings_are_normalized(self) -> None:
        self.assertEqual(RESET._normalize_robot_mode("RobotMode.MOVE"), "move")
        self.assertEqual(RESET._normalize_robot_mode("RobotMode::kReflex"), "reflex")

    def test_missing_native_diagnostics_is_unknown_by_default(self) -> None:
        status = RESET.evaluate_state(make_state())
        self.assertEqual(status["classification"], "unknown")
        self.assertIsNone(status["needs_reset"])
        self.assertEqual(status["diagnostic_coverage"], "telemetry_only")

    def test_telemetry_only_requires_explicit_opt_in(self) -> None:
        status = RESET.evaluate_state(make_state(), allow_telemetry_only=True)
        self.assertEqual(status["classification"], "healthy")
        self.assertFalse(status["needs_reset"])

    def test_current_errors_and_reflex_require_reset(self) -> None:
        state = make_state()
        for side in RESET.SIDES:
            state[side]["robot_state"]["current_errors"] = {"joint_reflex": False}
            state[side]["robot_state"]["robot_mode"] = 1
        self.assertEqual(RESET.evaluate_state(state)["classification"], "healthy")

        state["left_arm"]["robot_state"]["current_errors"]["joint_reflex"] = True
        self.assertEqual(RESET.evaluate_state(state)["classification"], "needs_reset")

        state["left_arm"]["robot_state"]["current_errors"]["joint_reflex"] = False
        state["left_arm"]["robot_state"]["robot_mode"] = 4
        self.assertEqual(RESET.evaluate_state(state)["classification"], "needs_reset")

    def test_string_false_current_errors_do_not_trigger_reset(self) -> None:
        state = make_state()
        for side in RESET.SIDES:
            state[side]["robot_state"]["current_errors"] = {
                "zero": "0",
                "no": "no",
                "off": "off",
                "inactive": "inactive",
            }
        self.assertEqual(RESET.evaluate_state(state)["classification"], "healthy")

    def test_opaque_current_errors_are_unknown_not_active(self) -> None:
        state = make_state()
        for side in RESET.SIDES:
            state[side]["robot_state"]["current_errors"] = "opaque ROS message"
            state[side]["robot_state"]["robot_mode"] = 1
        status = RESET.evaluate_state(state)
        self.assertEqual(status["classification"], "unknown")
        self.assertFalse(status["auto_reset_allowed"])
        self.assertTrue(status["unresolved_reasons"])

    def test_historical_errors_do_not_trigger_reset(self) -> None:
        state = make_state()
        state["left_arm"]["robot_state"]["last_motion_errors"] = {"joint_reflex": True}
        status = RESET.evaluate_state(state)
        self.assertEqual(status["classification"], "unknown")
        self.assertEqual(status["diagnostic_coverage"], "historical_only")

    def test_manual_and_recovering_modes_block_reset(self) -> None:
        state = make_state()
        state["left_arm"]["robot_state"]["current_errors"] = {"joint_reflex": True}
        state["left_arm"]["robot_state"]["robot_mode"] = 5
        state["right_arm"]["robot_state"]["robot_mode"] = 6
        manual = RESET.evaluate_state(state)
        self.assertEqual(manual["classification"], "manual_intervention")
        self.assertTrue(manual["needs_reset"])
        self.assertFalse(manual["auto_reset_allowed"])
        self.assertTrue(any("automatic_error_recovery" in reason for reason in manual["reasons"]))

        del state["right_arm"]["robot_state"]["robot_mode"]
        state["left_arm"]["robot_state"]["robot_mode"] = 6
        recovering = RESET.evaluate_state(state)
        self.assertEqual(recovering["classification"], "recovering")
        self.assertTrue(recovering["needs_reset"])
        self.assertFalse(recovering["auto_reset_allowed"])

    def test_gripper_health_does_not_stand_in_for_arm_health(self) -> None:
        state = make_state()
        for side in RESET.SIDES:
            state[side]["gripper"]["has_error"] = False
        self.assertEqual(RESET.evaluate_state(state)["classification"], "unknown")

    def test_arbitrary_nested_flags_are_not_arm_diagnostics(self) -> None:
        state = make_state()
        state["left_arm"]["metadata"] = {"has_error": True}
        state["debug"] = {"needs_reset": True}
        self.assertEqual(RESET.evaluate_state(state)["classification"], "unknown")

    def test_unknown_mode_prevents_telemetry_only_health(self) -> None:
        state = make_state()
        state["left_arm"]["robot_state"]["robot_mode"] = "UNRECOGNIZED"
        status = RESET.evaluate_state(state, allow_telemetry_only=True)
        self.assertEqual(status["classification"], "unknown")

    def test_fault_with_incomplete_telemetry_is_not_safe_to_reset(self) -> None:
        state = make_state()
        state["left_arm"]["robot_state"]["current_errors"] = {"joint_reflex": True}
        del state["right_arm"]["robot_state"]["joint_positions"]
        status = RESET.evaluate_state(state)
        self.assertEqual(status["classification"], "needs_reset")
        self.assertFalse(status["auto_reset_allowed"])

    def test_fault_without_both_arm_diagnostics_is_not_safe_to_reset(self) -> None:
        state = make_state()
        state["left_arm"]["robot_state"]["current_errors"] = {"joint_reflex": True}
        status = RESET.evaluate_state(state)
        self.assertEqual(status["classification"], "needs_reset")
        self.assertFalse(status["diagnostics_complete"])
        self.assertFalse(status["auto_reset_allowed"])

    def test_fault_with_unresolved_other_arm_mode_is_not_safe_to_reset(self) -> None:
        state = make_state()
        state["left_arm"]["robot_state"]["robot_mode"] = 4
        state["right_arm"]["robot_state"]["robot_mode"] = 0
        status = RESET.evaluate_state(state)
        self.assertEqual(status["classification"], "needs_reset")
        self.assertFalse(status["auto_reset_allowed"])
        self.assertTrue(status["unresolved_reasons"])


class MainSafetyTest(unittest.TestCase):
    def test_ensure_without_execute_only_prints_plan(self) -> None:
        status = {
            "classification": "needs_reset",
            "needs_reset": True,
            "auto_reset_allowed": True,
            "reasons": ["test fault"],
            "arms": {},
        }
        output = io.StringIO()
        with (
            patch.object(RESET, "_read_status", return_value=(status, {"ok": True}, {})),
            patch.object(RESET, "_build_reset_command", return_value=["robot-reset"]),
            patch.object(RESET, "_run_reset", side_effect=AssertionError("must not execute")),
            contextlib.redirect_stdout(output),
        ):
            exit_code = RESET.main(["ensure", "--compact"])
        report = json.loads(output.getvalue())
        self.assertEqual(exit_code, 2)
        self.assertTrue(report["reset"]["planned"])
        self.assertFalse(report["reset"]["attempted"])

    def test_healthy_status_skips_reset(self) -> None:
        status = {
            "classification": "healthy",
            "needs_reset": False,
            "reasons": [],
            "arms": {},
        }
        output = io.StringIO()
        with (
            patch.object(RESET, "_read_status", return_value=(status, {"ok": True}, {})),
            patch.object(RESET, "_run_reset", side_effect=AssertionError("must not execute")),
            contextlib.redirect_stdout(output),
        ):
            exit_code = RESET.main(["ensure", "--compact"])
        report = json.loads(output.getvalue())
        self.assertEqual(exit_code, 0)
        self.assertEqual(report["reset"]["skipped"], "reset not needed")

    def test_incomplete_telemetry_blocks_reset_even_with_fault(self) -> None:
        status = {
            "classification": "needs_reset",
            "needs_reset": True,
            "auto_reset_allowed": False,
            "reasons": ["test fault"],
            "arms": {},
        }
        output = io.StringIO()
        with (
            patch.object(RESET, "_read_status", return_value=(status, {"ok": True}, {})),
            patch.object(RESET, "_run_reset", side_effect=AssertionError("must not execute")),
            contextlib.redirect_stdout(output),
        ):
            exit_code = RESET.main(["ensure", "--execute", "--compact"])
        report = json.loads(output.getvalue())
        self.assertEqual(exit_code, 3)
        self.assertFalse(report["reset"]["attempted"])

    def test_reset_target_must_match_inspected_endpoint(self) -> None:
        args = RESET.build_parser().parse_args(["ensure"])
        with (
            patch.object(RESET, "_reset_target_from_config", return_value=("10.0.0.99", 4242)),
            self.assertRaisesRegex(RuntimeError, "status/reset target mismatch"),
        ):
            RESET._build_reset_command(args)

    def test_empty_inline_robot_config_uses_dual_franka_detail_config(self) -> None:
        reset_config = Path("/tmp/reset.yaml")
        repo_root = Path("/tmp/repo")
        detail_path = repo_root / "scripts" / "config" / "robots" / "franka_config.yaml"

        def fake_load(path: Path):
            if path == reset_config:
                return {"record": {"robot_type": "franka_dual_arm", "robot": {}}}
            self.assertEqual(path, detail_path)
            return {"robot": {"robot_ip": "172.16.0.1", "robot_port": 4242}}

        with patch.object(RESET, "_load_yaml_mapping", side_effect=fake_load):
            self.assertEqual(
                RESET._reset_target_from_config(reset_config, repo_root),
                ("172.16.0.1", 4242),
            )

    def test_single_arm_franka_config_is_rejected(self) -> None:
        with (
            patch.object(
                RESET,
                "_load_yaml_mapping",
                return_value={"record": {"robot_type": "franka"}},
            ),
            self.assertRaisesRegex(ValueError, "not dual Franka"),
        ):
            RESET._reset_target_from_config(Path("reset.yaml"), Path("repo"))

    def test_nonfinite_reset_timeout_never_starts_process(self) -> None:
        with patch.object(RESET.subprocess, "Popen", side_effect=AssertionError("must not start")):
            for timeout in (float("nan"), float("inf"), 0.0, -1.0):
                result = RESET._run_reset(["robot-reset"], timeout)
                self.assertFalse(result["attempted"])
                self.assertFalse(result["ok"])


if __name__ == "__main__":
    unittest.main()
