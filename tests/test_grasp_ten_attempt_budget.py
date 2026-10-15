"""Offline wrapper and measured-arrival checks for the ten-call P2P ceiling."""
from __future__ import annotations

import copy
from contextlib import redirect_stderr, redirect_stdout
import importlib.util
import io
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

import numpy as np


SCRIPTS = (Path(__file__).resolve().parents[1] /
           'task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts')
spec = importlib.util.spec_from_file_location('grasp_ten_attempt_test', SCRIPTS / 'grasp_right_arm_xyz.py')
grasp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(grasp)


def pose(x=0.0, rotation=0.0):
    return [x, 0.0, 0.0, 0.0, 0.0, rotation]


def outcome(left=None, right=None, report=None):
    return (pose(.1) if left is None else left, pose() if right is None else right,
            {'ok': True} if report is None else report)


class FakeMotionClient:
    def __init__(self, outcomes):
        self.outcomes = list(outcomes)
        self.left, self.right = pose(), pose()
        self.calls = []

    def get_observation(self):
        return {'left_arm': {'end_pose': list(self.left)},
                'right_arm': {'end_pose': list(self.right)}}

    def dual_robot_move_to_ee_pose(self, left, right, **kwargs):
        self.calls.append((copy.deepcopy(left), copy.deepcopy(right), copy.deepcopy(kwargs)))
        if len(self.calls) > len(self.outcomes):
            raise AssertionError('Unexpected extra P2P call')
        self.left, self.right, report = copy.deepcopy(self.outcomes[len(self.calls) - 1])
        return report


class TenAttemptBudgetTests(unittest.TestCase):
    def move(self, client):
        args = grasp.build_parser().parse_args([
            '--xyz', '[0.1,0,0]', '--arm', 'left', '--p2p-retries', '9',
            '--position-tolerance-m', '.01', '--rotation-tolerance-rad', '.05',
            '--settle-time-sec', '0'])
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            return grasp.move_stage(client, 'ten_attempt_test', np.asarray(pose(.1)),
                                    np.asarray(pose()), args, recover_stalled_side='left_arm')

    def test_wrapper_uses_nine_retries_and_python_default_remains_two(self):
        # Replace only the wrapper's Python executable with an argument recorder.
        # No skill process or hardware client can run during this check.
        with tempfile.TemporaryDirectory() as folder:
            recorder = Path(folder) / 'python3'
            recorder.write_text('#!/bin/sh\nprintf \'%s\\0\' "$@"\n')
            recorder.chmod(0o700)
            command = ['bash', str(SCRIPTS / 'grasp_right_arm_xyz.sh')]
            env = {**os.environ, 'PATH': folder + os.pathsep + os.environ.get('PATH', '')}
            for extra, expected in (([], 9), (['--p2p-retries', '0'], 0)):
                output = subprocess.run(command + extra, env=env, check=True, capture_output=True).stdout
                argv = output.decode().rstrip('\0').split('\0')[2:]
                self.assertEqual(grasp.build_parser().parse_args(argv).p2p_retries, expected)
                self.assertTrue(grasp.build_parser().parse_args(argv).warn_and_retry_stall)
        self.assertEqual(grasp.build_parser().parse_args(['--xyz', '[0.1,0,0]']).p2p_retries, 2)

    def test_tenth_call_can_succeed_without_changing_targets_or_limits(self):
        client = FakeMotionClient([outcome(left=pose(.005 * i)) for i in range(1, 10)] + [outcome()])
        left, right, _ = self.move(client)
        self.assertEqual(len(client.calls), 10)
        np.testing.assert_allclose(left, pose(.1))
        np.testing.assert_allclose(right, pose())
        self.assertTrue(all(call == client.calls[0] for call in client.calls))

    def test_tenth_failed_call_exhausts_budget_without_an_eleventh(self):
        client = FakeMotionClient([outcome(left=pose(.005 * i)) for i in range(1, 11)] + [outcome()])
        with self.assertRaisesRegex(RuntimeError, 'after 10 P2P attempt'):
            self.move(client)
        self.assertEqual(len(client.calls), 10)

    def test_measured_arrival_returns_early_even_if_rpc_ok_is_false(self):
        client = FakeMotionClient([outcome(left=pose(.05)), outcome(report={'ok': False}), outcome()])
        self.move(client)
        self.assertEqual(len(client.calls), 2)

    def test_each_of_four_pose_errors_must_pass_before_returning(self):
        misses = [outcome(left=pose(.08)), outcome(left=pose(.1, rotation=.1)),
                  outcome(right=pose(.02)), outcome(right=pose(rotation=.1))]
        for index, miss in enumerate(misses):
            with self.subTest(residual=index):
                client = FakeMotionClient([miss, outcome()])
                self.move(client)
                self.assertEqual(len(client.calls), 2)

    def test_stall_still_stops_on_first_call_with_budget_remaining(self):
        client = FakeMotionClient([outcome(left=pose(.05), report={
            'ok': False, 'stalled': {'detected': True, 'side': 'left_arm'},
            'final_error': {'stall_detection': {'detected': True}}}), outcome()])
        with self.assertRaisesRegex(RuntimeError, 'persistent_motion_stall'):
            self.move(client)
        self.assertEqual(len(client.calls), 1)


if __name__ == '__main__':
    unittest.main()
