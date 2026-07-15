from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
GRASP_PATH = (
    ROOT
    / "task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/grasp_right_arm_xyz.py"
)
CLIENT_PATH = (
    ROOT
    / "atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/dual_franka_robotiq_rpc_client.py"
)


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


grasp = load_module("grasp_controller_recovery_test", GRASP_PATH)
rpc_client = load_module("rpc_client_recovery_test", CLIENT_PATH)


class FakeMotionClient:
    def __init__(self, *, first_attempt_rotation_rad: float = 0.0, recovery_ok: bool = True):
        self.left = np.zeros(6, dtype=float)
        self.right = np.zeros(6, dtype=float)
        self.first_attempt_rotation_rad = float(first_attempt_rotation_rad)
        self.recovery_ok = bool(recovery_ok)
        self.move_calls = 0
        self.move_kwargs: list[dict] = []
        self.recovery_calls: list[str] = []
        self.reanchor_calls = 0

    def get_observation(self):
        return {
            "left_arm": {"end_pose": self.left.tolist()},
            "right_arm": {"end_pose": self.right.tolist()},
        }

    def step(self, action=None):
        if action is None:
            self.reanchor_calls += 1
        return {"observation": self.get_observation()}

    def dual_robot_move_to_ee_pose(self, left_target, right_target, **_kwargs):
        self.move_calls += 1
        self.move_kwargs.append(dict(_kwargs))
        if self.move_calls == 1:
            self.left[5] = self.first_attempt_rotation_rad
            return {"ok": False, "final_error": {"correction_index": 10}}
        self.left = np.asarray(left_target, dtype=float)
        self.right = np.asarray(right_target, dtype=float)
        return {"ok": True, "final_error": {"correction_index": 0}}

    def recover_robot(self, side: str):
        self.recovery_calls.append(side)
        return {"ok": self.recovery_ok, "sides": [side]}


def motion_args():
    return grasp.build_parser().parse_args(
        [
            "--xyz",
            "[0, 0, 0]",
            "--recover-stalled-controller",
            "--controller-recovery-settle-time-sec",
            "0",
            "--rotation-tolerance-rad",
            "0.05",
            "--position-tolerance-m",
            "0.01",
        ]
    )


class ControllerRecoveryTests(unittest.TestCase):
    def test_rpc_client_exposes_server_recovery_method(self):
        calls = []

        class RpcProxy:
            def recover_robot(self, side):
                calls.append(side)
                return {"ok": True}

        client = object.__new__(rpc_client.DualFrankaRobotiqRpcClient)
        client._client = RpcProxy()

        self.assertEqual(client.recover_robot("left"), {"ok": True})
        self.assertEqual(calls, ["left_arm"])

    def test_smooth_p2p_returns_early_when_active_arm_observation_is_stalled(self):
        class StaticObservationClient(rpc_client.DualFrankaRobotiqRpcClient):
            def __init__(self):
                self.step_calls = 0

            def get_observation(self):
                return {
                    "left_arm": {"end_pose": [0.0] * 6},
                    "right_arm": {"end_pose": [0.0] * 6},
                }

            def step(self, action=None):
                self.step_calls += 1
                return {"observation": self.get_observation(), "action": action}

        client = StaticObservationClient()
        result = client.dual_robot_move_to_ee_pose(
            [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0] * 6,
            delta=False,
            smooth=True,
            sleep=False,
            settle_time_sec=0.0,
            position_tolerance_m=0.001,
            rotation_tolerance_rad=0.01,
            max_correction_iters=10,
            stall_detection_side="left_arm",
            stall_translation_epsilon_m=0.0005,
            stall_rotation_epsilon_rad=0.005,
            max_stalled_correction_iters=1,
        )

        self.assertFalse(result["ok"])
        self.assertEqual(result["final_error"]["correction_index"], 0)
        self.assertEqual(len(result["corrections"]), 1)
        self.assertTrue(result["stalled"]["detected"])
        self.assertEqual(result["stalled"]["side"], "left_arm")

    def test_stalled_pregrasp_attempt_recovers_then_retries_target(self):
        client = FakeMotionClient(first_attempt_rotation_rad=0.0)
        args = motion_args()
        left_target = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.2])
        right_target = np.zeros(6, dtype=float)

        left, _, result = grasp.move_stage(
            client,
            "stage2_test",
            left_target,
            right_target,
            args,
            recover_stalled_side="left_arm",
        )

        np.testing.assert_allclose(left, left_target)
        self.assertTrue(result["ok"])
        self.assertEqual(client.recovery_calls, ["left_arm"])
        self.assertEqual(client.move_calls, 2)

    def test_failed_but_moving_attempt_retries_without_controller_recovery(self):
        client = FakeMotionClient(first_attempt_rotation_rad=0.1)
        args = motion_args()
        left_target = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.2])

        grasp.move_stage(
            client,
            "stage2_test",
            left_target,
            np.zeros(6, dtype=float),
            args,
            recover_stalled_side="left_arm",
        )

        self.assertEqual(client.recovery_calls, [])
        self.assertEqual(client.move_calls, 2)

    def test_stall_on_last_regular_attempt_gets_recovery_retry(self):
        class PartialThenStalledClient(FakeMotionClient):
            def dual_robot_move_to_ee_pose(self, left_target, right_target, **_kwargs):
                self.move_calls += 1
                self.move_kwargs.append(dict(_kwargs))
                if self.move_calls == 1:
                    self.left[5] = 0.1
                    return {"ok": False, "final_error": {"correction_index": 10}}
                if self.move_calls == 2:
                    return {"ok": False, "final_error": {"correction_index": 10}}
                self.left = np.asarray(left_target, dtype=float)
                self.right = np.asarray(right_target, dtype=float)
                return {"ok": True, "final_error": {"correction_index": 0}}

        client = PartialThenStalledClient()
        args = motion_args()
        left_target = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.2])

        left, _, result = grasp.move_stage(
            client,
            "stage2_test",
            left_target,
            np.zeros(6, dtype=float),
            args,
            recover_stalled_side="left_arm",
        )

        np.testing.assert_allclose(left, left_target)
        self.assertTrue(result["ok"])
        self.assertEqual(client.recovery_calls, ["left_arm"])
        self.assertEqual(client.move_calls, 3)
        self.assertTrue(all(call["stall_detection_side"] == "left_arm" for call in client.move_kwargs))
        self.assertTrue(all(call["max_stalled_correction_iters"] == 1 for call in client.move_kwargs))


if __name__ == "__main__":
    unittest.main()
