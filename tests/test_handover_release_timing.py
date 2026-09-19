"""Offline handover timing with simulated gripper feedback and a virtual clock."""
import importlib.util
import io
from pathlib import Path
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch

import numpy as np

PATH = Path(__file__).resolve().parents[1] / "task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/grasp_right_arm_xyz.py"
spec = importlib.util.spec_from_file_location("handover_timing_test", PATH)
grasp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(grasp)


class Clock:
    now = 0.0

    def sleep(self, seconds):
        self.now += seconds

    def monotonic(self):
        return self.now


class Grippers:
    def __init__(self, clock, donor, mode):
        self.clock, self.donor, self.mode = clock, donor, mode
        self.receiver = "right_arm" if donor == "left_arm" else "left_arm"
        self.receiver_closed_at = None
        self.opened = []
        self.homed = []
        self.reads = 0

    def ping(self):
        return {"ok": True}

    def close(self):
        pass

    def close_gripper(self, side):
        if side == self.receiver:
            self.receiver_closed_at = self.clock.now
        return {"ok": True}

    def open_gripper(self, side):
        self.opened.append((side, self.clock.now))
        return {"ok": True}

    def go_home(self, side, duration, rate):
        self.homed.append((side, self.clock.now))
        return {"ok": self.mode != "home_failed"}

    def get_observation(self):
        self.reads += 1
        elapsed = 0 if self.receiver_closed_at is None else self.clock.now - self.receiver_closed_at
        # Closure crosses 35% well before the receiver stops moving at 1.6s.
        position = min(.8, elapsed * .5)
        if self.mode == "moving":
            position = .4 + .03 * elapsed
        elif self.mode in ("stale", "missing"):
            position = .8
        state = {"position": position, "open_position": 0., "closed_position": 1.,
                 "stamp": {"sec": 1, "nanosec": 0 if self.mode == "stale" else self.reads}}
        if self.mode == "missing":
            state.pop("stamp")
        return {self.donor: {"gripper": {"position": .8, "closed_position": 1.}},
                self.receiver: {"gripper": state}}


class HandoverTimingTests(unittest.TestCase):
    def run_handover(self, side, mode, *, retreat=False):
        clock = Clock()
        client = Grippers(clock, side, mode)
        poses = (np.array([.57, .02, .02, 1.7, -1.6, 1.0]),
                 np.array([.58, -.02, .03, -1.8, -1.6, -1.0]))
        client.initial_poses = poses
        client.retreats = []
        client.target_events = []

        def anchor(_client, enabled):
            client.target_events.append(("anchor", enabled))

        def move(_client, name, left, right, args, **kwargs):
            if name.startswith("post_transfer_retreat_"):
                client.target_events.append(("retreat", True))
                client.retreats.append((name, left.copy(), right.copy(), kwargs, clock.now))
                if mode == "retreat_failed":
                    raise RuntimeError("retreat motion outcome unconfirmed")
            return (*poses, {})

        with (
            patch.object(grasp, "DualFrankaRobotiqRpcClient", return_value=client),
            patch.object(grasp, "read_ee_poses", return_value=poses),
            patch.object(grasp, "move_stage", side_effect=move),
            patch.object(grasp, "reanchor", side_effect=anchor),
            patch.object(grasp, "load_transition_targets", return_value=(*poses, "mock")),
            patch.object(grasp.time, "sleep", side_effect=clock.sleep),
            patch.object(grasp.time, "monotonic", side_effect=clock.monotonic),
            redirect_stdout(io.StringIO()),
        ):
            argv = ["--xyz", "[0,0,0]", "--arm", side, "--keep-current-rotation",
                    "--after-close-sleep-sec", "0", "--gripper-close-retries", "0",
                    "--execute"]
            argv += ["--retreat-after-transfer-m", "0.20"] if retreat else ["--go-home-after-transfer"]
            if mode == "normal":
                self.assertEqual(grasp.main(argv), 0)
            elif mode == "home_failed":
                with self.assertRaisesRegex(RuntimeError, "after_transfer failed"):
                    grasp.main(argv)
            elif mode == "retreat_failed":
                with self.assertRaisesRegex(RuntimeError, "retreat motion outcome unconfirmed"):
                    grasp.main(argv)
            else:
                with self.assertRaisesRegex(RuntimeError, "retaining the tube"):
                    grasp.main(argv)
        return client

    def test_both_directions_wait_for_stopped_receiver_then_release_delay(self):
        for donor in ("left_arm", "right_arm"):
            with self.subTest(donor=donor):
                client = self.run_handover(donor, "normal")
                self.assertEqual(len(client.opened), 1)
                side, released_at = client.opened[0]
                self.assertEqual(side, donor)
                self.assertGreaterEqual(released_at - client.receiver_closed_at, 1.6 + .5 + 2. - 1e-6)
                self.assertEqual(len(client.homed), 1)
                home_side, home_at = client.homed[0]
                self.assertEqual(home_side, donor)
                self.assertGreaterEqual(home_at - released_at, .5 - 1e-6)

    def test_unsettled_or_stale_feedback_keeps_donor_closed(self):
        for mode in ("moving", "stale", "missing"):
            with self.subTest(mode=mode):
                client = self.run_handover("left_arm", mode)
                self.assertEqual(client.opened, [])
                self.assertEqual(client.homed, [])

    def test_home_failure_is_reported_after_donor_release(self):
        for donor in ("left_arm", "right_arm"):
            with self.subTest(donor=donor):
                client = self.run_handover(donor, "home_failed")
                self.assertEqual([side for side, _ in client.homed], [donor])
                self.assertEqual([side for side, _ in client.opened], [donor])

    def test_retreat_moves_only_donor_outward_after_release_without_home(self):
        for donor, index, sign in (("left_arm", 0, 1), ("right_arm", 1, -1)):
            with self.subTest(donor=donor):
                client = self.run_handover(donor, "normal", retreat=True)
                self.assertEqual(client.homed, [])
                self.assertEqual(len(client.retreats), 1)
                self.assertEqual(client.target_events[-2:], [("anchor", True), ("retreat", True)])
                _, left, right, kwargs, started_at = client.retreats[0]
                expected = client.initial_poses[index].copy()
                expected[1] += sign * .20
                np.testing.assert_allclose((left, right)[index], expected)
                np.testing.assert_array_equal((left, right)[1-index], client.initial_poses[1-index])
                self.assertGreaterEqual(started_at - client.opened[0][1], .5 - 1e-6)
                self.assertLessEqual(kwargs['max_translation_speed'], .04)
                self.assertLessEqual(kwargs['max_translation_step'], .001)

    def test_retreat_failure_does_not_fall_back_to_home_or_replay(self):
        client = self.run_handover("left_arm", "retreat_failed", retreat=True)
        self.assertEqual(len(client.retreats), 1)
        self.assertEqual(client.homed, [])

    def test_unconfirmed_receiver_never_retreats(self):
        client = self.run_handover("left_arm", "stale", retreat=True)
        self.assertEqual(client.opened, [])
        self.assertEqual(client.homed, [])
        self.assertEqual(client.retreats, [])

    def test_invalid_retreat_is_rejected_before_rpc(self):
        with patch.object(grasp, "DualFrankaRobotiqRpcClient", side_effect=AssertionError("must not connect")):
            for value in ("nan", "inf", "-0.2"):
                with self.subTest(value=value), self.assertRaises(ValueError):
                    grasp.main(["--xyz", "[0,0,0]", "--retreat-after-transfer-m", value, "--execute"])


if __name__ == "__main__":
    unittest.main()
