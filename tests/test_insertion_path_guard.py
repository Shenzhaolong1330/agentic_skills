"""Insertion corridor regressions; fake clients only, no hardware."""
import contextlib
import io
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
    "task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts"))
import insert_down_from_current_pose as module


class InsertionPathGuardTests(unittest.TestCase):
    def run_feedback(self, start, poses, force):
        client = Mock()
        client.step.return_value = {"ok": True}
        observation = {"right_arm": {"robot_state": {"wrench": {"force": force}}}}
        args = module.build_parser().parse_args([
            "--position-tolerance-m", ".003", "--rotation-tolerance-rad", ".10",
            "--settle-time-sec", "0", "--seated-min-depth-m", ".047"])
        with patch.object(module.skill, "read_ee_poses",
                          side_effect=[(observation, {"right": np.array(p)}) for p in poses]), \
                patch.object(module.time, "sleep"):
            result = module.guarded_vertical_insert(
                client=client, side="right", start_pose=np.array(start),
                base_force=np.zeros(3), depth_m=.055, args=args)
        return client, result

    def test_low_force_lateral_drift_stops_before_another_command(self):
        start = [.43, -.04, -.09, 0, 0, 0]
        client, result = self.run_feedback(start,
            [[.4329, -.04, -.0915, 0, 0, 0],
             [.4331, -.04, -.0916, 0, 0, 0]], [0, 0, .5])
        self.assertFalse(result["ok"])
        self.assertEqual(result["stopped_reason"], "insertion_path_deviation")
        self.assertEqual(result["force_classification"], "continue")
        self.assertEqual(client.step.call_count, 2)
        self.assertEqual(result["lateral_path_limit_m"], .003)
        self.assertAlmostEqual(result["lateral_path_error_m"], .0031)

    def test_off_axis_seated_contact_cannot_become_success(self):
        client, result = self.run_feedback([.43, -.04, -.09, 0, 0, 0],
            [[.444, -.04, -.14, 0, 0, 0]], [0, 0, 7])
        self.assertFalse(result["ok"])
        self.assertTrue(result["path_guard_failure"])
        self.assertEqual(result["force_classification"], "seated_axial_contact")
        self.assertEqual(client.step.call_count, 1)

    def test_rotational_drift_stops_without_lateral_drift(self):
        _, result = self.run_feedback([.43, -.04, -.09, 0, 0, 0],
            [[.43, -.04, -.091, .101, 0, 0]], [0, 0, 0])
        self.assertTrue(result["path_guard_failure"])
        self.assertAlmostEqual(result["rotation_path_error_rad"], .101)

    def test_valid_axial_contact_retains_existing_success_rule(self):
        _, result = self.run_feedback([.43, -.04, -.09, 0, 0, 0],
            [[.43, -.04, -.14, 0, 0, 0]], [0, 0, 7])
        self.assertTrue(result["ok"])
        self.assertEqual(result["insert_success_kind"], "seated_axial_contact")

    def test_path_stop_forbids_release_retract_and_home_even_with_continue(self):
        client = Mock()
        client.ping.return_value = {"ok": True}
        observation = {"right_arm": {"robot_state": {"wrench": {"force": [0, 0, 0]}},
            "gripper": {"position": .8, "open_position": 0., "closed_position": 1.}}}
        start = np.array([.43, -.04, -.09, 0, 0, 0])
        end = start + np.array([.014, 0, -.055, 0, 0, 0])
        stopped = {"ok": False, "path_guard_failure": True, "force_guard_failure": False,
            "stopped_reason": "insertion_path_deviation", "correction_index": 3}
        output = io.StringIO()
        with patch.object(module.skill, "DualFrankaRobotiqRpcClient", return_value=client), \
                patch.object(module.skill, "read_ee_poses", side_effect=[
                    (observation, {"right": start}), (observation, {"right": end})]), \
                patch.object(module, "guarded_vertical_insert", return_value=stopped), \
                patch.object(module, "_move_side") as move, contextlib.redirect_stdout(output):
            code = module.main(["--execute", "--side", "right", "--insert-depth-m", ".055",
                "--monitor-force-during-insert", "--release-after-insert",
                "--continue-on-p2p-failure", "--retract-after-release-m", ".06",
                "--home-after-release"])
        report = json.loads(output.getvalue())
        self.assertEqual(code, 3)
        self.assertFalse(report["insert_success_confirmed"])
        self.assertFalse(report["release_after_insert_executed"])
        client.open_gripper.assert_not_called()
        move.assert_not_called()
        client.home.assert_not_called()


if __name__ == "__main__":
    unittest.main()
