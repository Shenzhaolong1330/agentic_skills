"""Offline grasp-to-atomic integration; no live client or motion is constructed."""

import copy
from contextlib import redirect_stderr, redirect_stdout
import io
import json
import unittest

import numpy as np

from test_grasp_stall_warning import FakeMotionClient, grasp, pose


def error_report(position=0.009, rotation=0.13, right_position=0.001,
                 right_rotation=0.002, index=0):
    return {"left_translation_error_m": position, "left_rotation_error_rad": rotation,
            "right_translation_error_m": right_position, "right_rotation_error_rad": right_rotation,
            "correction_index": index}


RECORDED_FIRST = error_report(0.009120984753754613, 0.13145914007029108,
                              0.0016226265662288171, 0.0023209018606169408)
RECORDED_SECOND = error_report(0.009308781283858996, 0.13146681146448727,
                               0.0022528775417438823, 0.0031347972923773024, 1)
ARRIVED = error_report(0, 0, 0, 0)


def update_measured_poses(client, left_target, right_target, report):
    # Test targets have zero rotation. Norms are realized as +x translation and
    # +z rotation so final measured poses agree with the callback report.
    client.left, client.right = list(left_target), list(right_target)
    client.left[0] += report["left_translation_error_m"]
    client.left[5] = report["left_rotation_error_rad"]
    client.right[0] += report["right_translation_error_m"]
    client.right[5] = report["right_rotation_error_rad"]


class AtomicCallbackClient(FakeMotionClient):
    """Exercise callbacks passed by move_stage; do not emulate robot dynamics."""

    def __init__(self, reports, *, native=True, feedback_patch=None, returned=None, final_patch=None):
        super().__init__([], native=native)
        self.reports = copy.deepcopy(reports)
        self.feedback_patch, self.final_patch = feedback_patch, final_patch
        self.returned = returned
        self.feedback_calls = 0
        self.accepted_reports = []
        self.atomic_returned = False

    def get_observation(self):
        observation = super().get_observation()
        if self.atomic_returned and self.final_patch:
            side, fields = self.final_patch
            observation[side]["native_robot_state"].update(fields)
        return observation

    def dual_robot_move_to_ee_pose(self, left, right, **kwargs):
        self.calls.append((copy.deepcopy(left), copy.deepcopy(right), dict(kwargs)))
        if len(self.calls) > 1:
            raise AssertionError("Continuous mode must not resend the whole target")
        if len(self.reports) > kwargs["max_correction_iters"] + 1:
            raise AssertionError("Test reports exceed the negotiated atomic correction budget")
        for report in self.reports:
            update_measured_poses(self, left, right, report)
            observation = self.get_observation()
            if self.feedback_patch and report["correction_index"] == self.feedback_patch[0]:
                _, side, fields = self.feedback_patch
                observation[side]["native_robot_state"].update(copy.deepcopy(fields))
            self.feedback_calls += 1
            kwargs["feedback_guard"](observation)
            kwargs["correction_guard"](copy.deepcopy(report))
            self.accepted_reports.append(copy.deepcopy(report))
        self.atomic_returned = True
        if self.returned is not None:
            return copy.deepcopy(self.returned)
        return {"ok": False, "stalled": {"detected": True},
                "final_error": self.reports[-1], "corrections": self.accepted_reports}


class LegacyPoseClient(FakeMotionClient):
    def __init__(self, reports):
        super().__init__([])
        self.reports = copy.deepcopy(reports)

    def dual_robot_move_to_ee_pose(self, left, right, **kwargs):
        self.calls.append((copy.deepcopy(left), copy.deepcopy(right), dict(kwargs)))
        if len(self.calls) > len(self.reports):
            raise AssertionError("Unexpected extra legacy whole-target call")
        report = self.reports[len(self.calls) - 1]
        update_measured_poses(self, left, right, report)
        return {"ok": False, "final_error": report}


class GraspContinuousCorrectionTests(unittest.TestCase):
    def setUp(self):
        self.stdout, self.stderr = io.StringIO(), io.StringIO()

    def move(self, client, *extra, continuous=True, **stage_options):
        argv = ["--xyz", "[0.1,0,0]", "--arm", "left", "--p2p-retries", "9",
                "--position-tolerance-m", ".003", "--rotation-tolerance-rad", ".03",
                "--settle-time-sec", "0", "--warn-and-retry-stall"]
        if continuous:
            argv.append("--p2p-continuous-corrections")
        args = grasp.build_parser().parse_args(argv + list(extra))
        with redirect_stdout(self.stdout), redirect_stderr(self.stderr):
            return grasp.move_stage(client, "continuous_test", np.asarray(pose(.1)), np.asarray(pose()),
                                    args, recover_stalled_side="left_arm", **stage_options)

    def progress_events(self):
        return [json.loads(line.split(": ", 1)[1]) for line in self.stdout.getvalue().splitlines()
                if line.startswith("p2p_correction_progress: ")]

    def test_single_atomic_call_receives_ten_round_budget_and_both_guards(self):
        client = AtomicCallbackClient([error_report(), {**ARRIVED, "correction_index": 1}])
        left, right, result = self.move(client)
        self.assertEqual(len(client.calls), 1)
        options = client.calls[0][2]
        self.assertEqual(options["max_correction_iters"], 9)
        self.assertEqual(options["stall_policy"], "warn")
        self.assertEqual(options["feedback_mode"], "read_after_step")
        self.assertTrue(callable(options["feedback_guard"]))
        self.assertTrue(callable(options["correction_guard"]))
        self.assertEqual(options["position_tolerance_m"], 0.003)
        self.assertEqual(options["rotation_tolerance_rad"], 0.03)
        self.assertEqual(options["max_tracking_translation_bias_m"], 0.025)
        self.assertEqual(options["max_tracking_rotation_bias_rad"], 0.20)
        np.testing.assert_allclose(left, pose(.1))
        np.testing.assert_allclose(right, pose())
        self.assertFalse(result["ok"])  # Native measured arrival takes precedence.
        self.assertEqual([event["status"] for event in self.progress_events()],
                         ["baseline", "within_tolerance"])
        self.assertTrue(all(event["max_rounds"] == 10 for event in self.progress_events()))

    def test_ten_exhausted_rounds_never_resend_or_allow_next_stage(self):
        client = AtomicCallbackClient([error_report(index=index) for index in range(10)])
        next_stage = []
        with self.assertRaisesRegex(RuntimeError, "10/10 continuous P2P rounds") as raised:
            self.move(client, "--continue-on-tolerance-miss")
            next_stage.append("gripper")
        self.assertEqual(next_stage, [])
        self.assertEqual(len(client.calls), 1)
        self.assertEqual(len(client.accepted_reports), 10)
        self.assertIn("no whole-target restart or subsequent gripper commands", str(raised.exception))

    def test_zero_corrections_still_cannot_bypass_arrival_with_legacy_flag(self):
        client = AtomicCallbackClient([error_report()])
        with self.assertRaisesRegex(RuntimeError, "1/1 continuous P2P rounds"):
            self.move(client, "--p2p-retries", "0", "--continue-on-tolerance-miss")
        self.assertEqual(len(client.calls), 1)
        self.assertEqual(client.calls[0][2]["max_correction_iters"], 0)

    def test_missing_native_health_is_rejected_before_any_atomic_call(self):
        client = AtomicCallbackClient([ARRIVED], native=False)
        with self.assertRaisesRegex(RuntimeError, "Missing .* native diagnostics"):
            self.move(client)
        self.assertEqual(client.calls, [])

    def test_feedback_callback_rejects_fault_staleness_or_collision_before_more_corrections(self):
        invalid = ({"current_errors": {"joint_reflex": True}}, {"current_errors": {}},
                   {"sensor_clock_progressing": False}, {"age_ns": 250_000_001},
                   {"robot_mode": 4}, {"cartesian_collision": [True]})
        for side in ("left_arm", "right_arm"):
            for fields in invalid:
                with self.subTest(side=side, fields=fields):
                    client = AtomicCallbackClient([error_report(), {**ARRIVED, "correction_index": 1}],
                                                  feedback_patch=(1, side, fields))
                    with self.assertRaises(RuntimeError):
                        self.move(client)
                    self.assertEqual(len(client.calls), 1)
                    self.assertEqual(len(client.accepted_reports), 1)

    def test_native_health_is_rechecked_after_atomic_reports_success(self):
        client = AtomicCallbackClient([ARRIVED], returned={"ok": True},
                                      final_patch=("right_arm", {"sensor_clock_progressing": False}))
        with self.assertRaisesRegex(RuntimeError, "Unverified right_arm native health"):
            self.move(client)
        self.assertEqual(len(client.calls), 1)

    def test_continuous_recorded_noise_is_plateau_then_actual_arrival_returns(self):
        client = AtomicCallbackClient([RECORDED_FIRST, RECORDED_SECOND,
                                      {**ARRIVED, "correction_index": 2}])
        self.move(client)
        self.assertEqual(len(client.calls), 1)
        self.assertEqual([event["status"] for event in self.progress_events()],
                         ["baseline", "plateau", "within_tolerance"])

    def test_non_dominant_growth_in_continuous_callback_is_rejected(self):
        client = AtomicCallbackClient([error_report(position=.004, rotation=.20),
                                      error_report(position=.0046, rotation=.10, index=1),
                                      {**ARRIVED, "correction_index": 2}])
        with self.assertRaisesRegex(RuntimeError, "p2p_error_increased.*left_translation_error_m"):
            self.move(client)
        self.assertEqual(len(client.calls), 1)
        self.assertEqual(len(client.accepted_reports), 1)

    def test_continuous_controller_recovery_is_rejected_before_dispatch(self):
        client = AtomicCallbackClient([ARRIVED])
        with self.assertRaisesRegex(ValueError, "does not restart a controller"):
            self.move(client, "--recover-stalled-controller")
        self.assertEqual(client.calls, [])

    def test_more_than_ten_total_rounds_is_rejected_before_dispatch(self):
        client = AtomicCallbackClient([ARRIVED])
        with self.assertRaisesRegex(ValueError, "at most 9 corrections"):
            self.move(client, "--p2p-retries", "10")
        self.assertEqual(client.calls, [])

    def test_synthetic_observed_z_offset_is_rejected_before_dispatch(self):
        for offset in (-0.001, 0.001):
            with self.subTest(offset=offset):
                client = AtomicCallbackClient([ARRIVED])
                with self.assertRaisesRegex(ValueError, "observed-z offsets are unsupported"):
                    self.move(client, arrival_observed_z_offset_side="left_arm",
                              arrival_observed_z_offset_m=offset)
                self.assertEqual(client.calls, [])

    def test_legacy_recorded_noise_can_retry_but_large_growth_still_rejects(self):
        client = LegacyPoseClient([RECORDED_FIRST, RECORDED_SECOND, ARRIVED])
        self.move(client, continuous=False)
        self.assertEqual(len(client.calls), 3)
        self.assertNotIn("correction_guard", client.calls[0][2])
        self.assertIn("insufficient_progress", self.stderr.getvalue())
        client = LegacyPoseClient([RECORDED_FIRST, error_report(rotation=.15), ARRIVED])
        with self.assertRaisesRegex(RuntimeError, "p2p_error_increased"):
            self.move(client, continuous=False)
        self.assertEqual(len(client.calls), 2)


if __name__ == "__main__":
    unittest.main()
