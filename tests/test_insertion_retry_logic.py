import contextlib
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT / "task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts"
sys.path.insert(0, str(SCRIPT_DIR))

import insert_down_from_current_pose as insert_down
import run_manual_grip_to_insert as insertion_flow


class InsertionRetryLogicTests(unittest.TestCase):
    def test_force_classifier_treats_observed_axial_contact_as_seated(self):
        state = insert_down.classify_insert_force(
            force_delta=np.array([1.5108, 1.2558, 9.1460]),
            achieved_depth_m=0.00507,
            max_total_delta_n=12.0,
            max_lateral_delta_n=4.0,
            seated_axial_delta_n=6.0,
            seated_min_depth_m=0.003,
        )
        self.assertEqual(state["classification"], "seated_axial_contact")

    def test_force_classifier_retries_lateral_or_hard_collision(self):
        common = {
            "achieved_depth_m": 0.005,
            "max_total_delta_n": 12.0,
            "max_lateral_delta_n": 4.0,
            "seated_axial_delta_n": 6.0,
            "seated_min_depth_m": 0.003,
        }
        lateral = insert_down.classify_insert_force(
            force_delta=np.array([5.0, 0.0, 7.0]), **common
        )
        hard = insert_down.classify_insert_force(
            force_delta=np.array([0.0, 0.0, 13.0]), **common
        )
        early = insert_down.classify_insert_force(
            force_delta=np.array([0.0, 0.0, 7.0]),
            **{**common, "achieved_depth_m": 0.001},
        )
        self.assertEqual(lateral["classification"], "lateral_collision")
        self.assertEqual(hard["classification"], "hard_collision")
        self.assertEqual(early["classification"], "early_axial_collision")

    def test_guarded_descent_reads_force_and_stops_on_seated_contact(self):
        class FakeClient:
            def __init__(self):
                self.actions = []

            def step(self, action):
                self.actions.append(action)
                return {"ok": True}

        client = FakeClient()
        observation = {
            "left_arm": {
                "robot_state": {"wrench": {"force": [1.5, 1.25, 9.15]}}
            }
        }
        poses = {"left": np.array([0.3, 0.0, 0.095, 0.0, 0.0, 0.0])}
        args = insert_down.build_parser().parse_args(
            ["--monitor-force-during-insert", "--rate-hz", "1000000"]
        )
        with mock.patch.object(
            insert_down.skill,
            "read_ee_poses",
            return_value=(observation, poses),
        ):
            result = insert_down.guarded_vertical_insert(
                client=client,
                side="left",
                start_pose=np.array([0.3, 0.0, 0.1, 0.0, 0.0, 0.0]),
                base_force=np.zeros(3),
                depth_m=0.055,
                args=args,
            )
        self.assertTrue(result["ok"])
        self.assertEqual(result["insert_success_kind"], "seated_axial_contact")
        self.assertEqual(len(client.actions), 1)

    def test_guarded_step_respects_speed_rate_and_step_caps(self):
        args = insert_down.build_parser().parse_args(
            [
                "--rate-hz",
                "50",
                "--max-translation-speed",
                "0.01",
                "--max-translation-step",
                "0.001",
                "--insert-force-step-m",
                "0.0005",
            ]
        )
        translation_step, rotation_step = insert_down.guarded_motion_step_limits(args)
        self.assertAlmostEqual(translation_step, 0.0002)
        self.assertAlmostEqual(rotation_step, 0.0012)

    def test_guarded_descent_settles_and_runs_three_corrections(self):
        class LaggingClient:
            def __init__(self):
                self.pose = np.array([0.3, 0.0, 0.1, 0.0, 0.0, 0.0])
                self.actions = []

            def step(self, action):
                self.actions.append(action)
                motion = action["left_arm"]["motion"]
                self.pose[:3] += 0.5 * np.asarray(motion["translation"], dtype=float)
                rotation = np.asarray(motion["rotation_rotvec"], dtype=float)
                self.pose[3:] = (
                    insert_down.R.from_rotvec(0.5 * rotation)
                    * insert_down.R.from_rotvec(self.pose[3:])
                ).as_rotvec()
                return {"ok": True}

        client = LaggingClient()

        def read_pose(_client):
            observation = {
                "left_arm": {"robot_state": {"wrench": {"force": [0.0, 0.0, 0.0]}}}
            }
            return observation, {"left": client.pose.copy()}

        args = insert_down.build_parser().parse_args(
            [
                "--monitor-force-during-insert",
                "--rate-hz",
                "50",
                "--max-translation-speed",
                "0.01",
                "--insert-force-step-m",
                "0.0005",
                "--position-tolerance-m",
                "0.0001",
                "--max-correction-iters",
                "3",
                "--settle-time-sec",
                "0.1",
            ]
        )
        with (
            mock.patch.object(insert_down.skill, "read_ee_poses", side_effect=read_pose),
            mock.patch.object(insert_down.time, "sleep"),
        ):
            result = insert_down.guarded_vertical_insert(
                client=client,
                side="left",
                start_pose=client.pose.copy(),
                base_force=np.zeros(3),
                depth_m=0.001,
                args=args,
            )
        self.assertTrue(result["ok"])
        self.assertTrue(result["pose_tolerance_reached"])
        self.assertEqual(result["max_correction_iters"], 3)
        self.assertEqual(
            [item["correction_index"] for item in result["correction_reports"]],
            [0, 1, 2, 3],
        )
        self.assertTrue(all(item["settle_time_sec"] == 0.1 for item in result["correction_reports"]))

    def test_depth_gate_uses_position_tolerance(self):
        self.assertTrue(
            insert_down.insertion_depth_reached(
                achieved_depth_m=0.049,
                requested_depth_m=0.055,
                position_tolerance_m=0.006,
            )
        )
        self.assertFalse(
            insert_down.insertion_depth_reached(
                achieved_depth_m=0.0489,
                requested_depth_m=0.055,
                position_tolerance_m=0.006,
            )
        )

    def test_near_depth_gate_accepts_safe_near_complete_descent(self):
        self.assertFalse(
            insert_down.insertion_depth_reached(
                achieved_depth_m=0.0482,
                requested_depth_m=0.055,
                position_tolerance_m=0.006,
            )
        )
        self.assertTrue(
            insert_down.insertion_depth_near_reached(
                achieved_depth_m=0.0482,
                requested_depth_m=0.055,
                position_tolerance_m=0.006,
                near_depth_tolerance_m=0.008,
            )
        )
        self.assertFalse(
            insert_down.insertion_depth_near_reached(
                achieved_depth_m=0.040,
                requested_depth_m=0.055,
                position_tolerance_m=0.006,
                near_depth_tolerance_m=0.008,
            )
        )

    def test_post_release_lift_is_not_an_insert_failure_gate(self):
        self.assertTrue(
            insert_down.insertion_warning_is_completed(
                achieved_depth_m=0.0482,
                requested_depth_m=0.055,
                position_tolerance_m=0.006,
                near_depth_tolerance_m=0.008,
                release_after_insert=True,
                release_confirmed_open=True,
            )
        )

    def test_success_release_retract_options_are_forwarded_to_insert_stage(self):
        args = insertion_flow.build_parser().parse_args(
            [
                "--holder-side",
                "left",
                "--retract-after-release-m",
                "0.06",
            ]
        )
        commands = insertion_flow.build_stage_commands(args, holder_side="left")
        insert_command = next(
            command for command in commands if command.name == "insert_release_retract"
        )
        self.assertIn("--release-after-insert", insert_command.argv)
        self.assertIn("--monitor-force-during-insert", insert_command.argv)
        correction_index = insert_command.argv.index("--max-correction-iters")
        self.assertEqual(insert_command.argv[correction_index + 1], "3")
        settle_index = insert_command.argv.index("--settle-time-sec")
        self.assertEqual(insert_command.argv[settle_index + 1], "0.8")
        retract_index = insert_command.argv.index("--retract-after-release-m")
        self.assertEqual(insert_command.argv[retract_index + 1], "0.06")

    def test_failed_depth_does_not_run_success_lift_cleanup(self):
        class FakeClient:
            def __init__(self):
                self.open_calls = []

            def ping(self):
                return {"ok": True}

            def open_gripper(self, side):
                self.open_calls.append(side)
                return {"ok": True}

            def close(self):
                return None

        fake_client = FakeClient()

        def observation(closed_fraction=0.8):
            return {
                "left_arm": {
                    "gripper": {
                        "position": closed_fraction,
                        "open_position": 0.0,
                        "closed_position": 1.0,
                    },
                    "robot_state": {"wrench": {"force": [0.0, 0.0, 1.0]}},
                }
            }

        poses = [
            {"left": np.array([0.3, 0.0, 0.10, 0.0, 0.0, 0.0])},
            {"left": np.array([0.3, 0.0, 0.08, 0.0, 0.0, 0.0])},
        ]
        pose_reads = iter((observation(), pose) for pose in poses)
        failed_insert = {"stage": "insert_left", "result": {"ok": False}}
        stdout = io.StringIO()
        with (
            mock.patch.object(insert_down.skill, "DualFrankaRobotiqRpcClient", return_value=fake_client),
            mock.patch.object(insert_down.skill, "read_ee_poses", side_effect=lambda _client: next(pose_reads)),
            mock.patch.object(insert_down, "_move_side", return_value=failed_insert),
            contextlib.redirect_stdout(stdout),
        ):
            returncode = insert_down.main(
                [
                    "--execute",
                    "--side",
                    "left",
                    "--insert-depth-m",
                    "0.055",
                    "--position-tolerance-m",
                    "0.006",
                    "--release-after-insert",
                ]
            )
        report = json.loads(stdout.getvalue())
        self.assertEqual(returncode, 3)
        self.assertEqual(fake_client.open_calls, [])
        self.assertEqual([stage["stage"] for stage in report["stages"]], ["insert_left"])

    def test_flow_does_not_retry_insert_by_lifting(self):
        sequence = [
            {"name": "observe_rack", "returncode": 0, "report": {}},
            {"name": "move_above_hole_from_wrist", "returncode": 0, "report": {}},
            {
                "name": "insert_release_retract",
                "returncode": 3,
                "report": {"completion_status": "insert_failed", "completion_flag": False},
            },
        ]
        stage_names = []

        def fake_run_stage(command):
            stage_names.append(command.name)
            return sequence[len(stage_names) - 1]

        with tempfile.TemporaryDirectory() as tmpdir:
            with (
                mock.patch.object(
                    insertion_flow,
                    "read_gripper_fractions",
                    return_value={"left": 0.8, "right": 0.0},
                ),
                mock.patch.object(insertion_flow, "run_stage", side_effect=fake_run_stage),
                mock.patch.object(insertion_flow, "print_report"),
            ):
                returncode = insertion_flow.main(
                    [
                        "--execute",
                        "--holder-side",
                        "left",
                        "--log-file",
                        str(Path(tmpdir) / "retry.log"),
                    ]
                )
        self.assertEqual(returncode, 3)
        self.assertEqual(
            stage_names,
            [
                "observe_rack",
                "move_above_hole_from_wrist",
                "insert_release_retract",
            ],
        )


if __name__ == "__main__":
    unittest.main()
