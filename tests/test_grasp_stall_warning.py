"""Explicit bounded stall warnings never bypass arrival or native health checks."""
from __future__ import annotations

import copy
from contextlib import redirect_stderr, redirect_stdout
import importlib.util
import io
from pathlib import Path
import unittest

import numpy as np


SCRIPT = (Path(__file__).resolve().parents[1] /
          'task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/grasp_right_arm_xyz.py')
spec = importlib.util.spec_from_file_location('grasp_stall_warning_test', SCRIPT)
grasp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(grasp)


def pose(x=0.0):
    return [x, 0.0, 0.0, 0.0, 0.0, 0.0]


STALL = {'ok': False, 'stalled': {'detected': True, 'side': 'left_arm'},
         'final_error': {'stall_detection': {'detected': True}}}
HEALTH = {'age_ns': 1_000_000, 'sensor_clock_progressing': True, 'robot_mode': 2,
          'current_errors': {'joint_reflex': False, 'cartesian_reflex': False}}


class FakeMotionClient:
    def __init__(self, outcomes, *, native=True, health_patch=None, changed_before_retry=None):
        self.outcomes = list(outcomes)
        self.native, self.health_patch = native, health_patch
        self.changed_before_retry = changed_before_retry
        self.left, self.right = pose(), pose()
        self.calls, self.observations = [], 0

    def get_observation(self):
        self.observations += 1
        result = {'left_arm': {'end_pose': list(self.left)}, 'right_arm': {'end_pose': list(self.right)}}
        if self.native:
            for side in result:
                result[side]['native_robot_state'] = copy.deepcopy(HEALTH)
            if self.calls and self.health_patch:
                side, fields = self.health_patch
                result[side]['native_robot_state'].update(copy.deepcopy(fields))
            if self.observations >= 3 and self.changed_before_retry:
                side, fields = self.changed_before_retry
                result[side]['native_robot_state'].update(copy.deepcopy(fields))
        return result

    def dual_robot_move_to_ee_pose(self, left, right, **kwargs):
        self.calls.append((copy.deepcopy(left), copy.deepcopy(right), copy.deepcopy(kwargs)))
        if len(self.calls) > len(self.outcomes):
            raise AssertionError('Unexpected extra P2P call')
        item = self.outcomes[len(self.calls) - 1]
        if isinstance(item, BaseException):
            raise item
        x, report = item
        self.left = pose(x)
        return copy.deepcopy(report)

    def recover_robot(self, side):
        raise AssertionError('Warning retry must never recover the controller')


class StallWarningTests(unittest.TestCase):
    def setUp(self):
        self.log = io.StringIO()

    def move(self, client, *extra, warning=True):
        argv = ['--xyz', '[0.1,0,0]', '--arm', 'left', '--p2p-retries', '9',
                '--position-tolerance-m', '.01', '--rotation-tolerance-rad', '.05',
                '--settle-time-sec', '0']
        if warning:
            argv.append('--warn-and-retry-stall')
        args = grasp.build_parser().parse_args(argv + list(extra))
        with redirect_stdout(io.StringIO()), redirect_stderr(self.log):
            return grasp.move_stage(client, 'warning_test', np.asarray(pose(.1)), np.asarray(pose()),
                                    args, recover_stalled_side='left_arm')

    def test_default_remains_disabled_and_stall_stops(self):
        self.assertFalse(grasp.build_parser().parse_args(['--xyz', '[0.1,0,0]']).warn_and_retry_stall)
        client = FakeMotionClient([(.05, STALL), (.1, {'ok': True})])
        with self.assertRaisesRegex(RuntimeError, 'persistent_motion_stall'):
            self.move(client, warning=False)
        self.assertEqual(len(client.calls), 1)

    def test_healthy_stall_warns_and_retries_same_target_then_returns_at_arrival(self):
        client = FakeMotionClient([(0, STALL), (.05, STALL), (.1, STALL), (.1, {'ok': True})])
        self.move(client, '--recover-stalled-controller')
        self.assertEqual(len(client.calls), 3)
        self.assertEqual(self.log.getvalue().count('WARNING: p2p_stall_retry:'), 2)
        self.assertTrue(all(call == client.calls[0] for call in client.calls))

    def test_healthy_arrival_with_stall_returns_on_first_call(self):
        client = FakeMotionClient([(.1, STALL), (.1, {'ok': True})])
        self.move(client)
        self.assertEqual(len(client.calls), 1)
        self.assertEqual(self.log.getvalue(), '')

    def test_insufficient_progress_can_spend_remaining_budget_but_never_eleven_calls(self):
        client = FakeMotionClient([(0, {'ok': False})] * 10 + [(.1, {'ok': True})])
        with self.assertRaisesRegex(RuntimeError, 'after 10 P2P attempt'):
            self.move(client)
        self.assertEqual(len(client.calls), 10)
        self.assertEqual(self.log.getvalue().count('WARNING: p2p_stall_retry:'), 9)
        self.assertIn('insufficient_progress', self.log.getvalue())

    def test_subthreshold_progress_warns_but_increasing_error_stops(self):
        client = FakeMotionClient([(.001, {'ok': False}), (.1, {'ok': True})])
        self.move(client)
        self.assertEqual(len(client.calls), 2)
        self.assertIn('insufficient_progress', self.log.getvalue())
        for report in (STALL, {'ok': False}):
            with self.subTest(report=report):
                client = FakeMotionClient([(-.01, report), (.1, {'ok': True})])
                with self.assertRaisesRegex(RuntimeError, 'p2p_error_increased'):
                    self.move(client)
                self.assertEqual(len(client.calls), 1)

    def test_pose_only_observation_cannot_authorize_warning_retry_or_arrival(self):
        for x in (0, .1):
            with self.subTest(x=x):
                client = FakeMotionClient([(x, STALL), (.1, {'ok': True})], native=False)
                with self.assertRaisesRegex(RuntimeError, 'Missing .* native diagnostics'):
                    self.move(client)
                self.assertEqual(len(client.calls), 1)

    def test_fault_stale_missing_diagnostics_or_collision_stops_even_at_arrival(self):
        invalid = [{'current_errors': {}}, {'current_errors': None}, {'sensor_clock_progressing': None},
                   {'sensor_clock_progressing': False}, {'robot_mode': None}, {'age_ns': 250_000_001},
                   {'current_errors': {'joint_reflex': True}}, {'robot_mode': 4},
                   {'joint_collision': [True]}, {'cartesian_collision': [True]}]
        for side in ('left_arm', 'right_arm'):
            for fields in invalid:
                with self.subTest(side=side, fields=fields):
                    client = FakeMotionClient([(.1, STALL)], health_patch=(side, fields))
                    with self.assertRaises(RuntimeError):
                        self.move(client)
                    self.assertEqual(len(client.calls), 1)

    def test_health_is_rechecked_before_dispatching_retry(self):
        client = FakeMotionClient([(0, STALL), (.1, {'ok': True})],
                                  changed_before_retry=('right_arm', {'sensor_clock_progressing': False}))
        with self.assertRaises(RuntimeError):
            self.move(client)
        self.assertEqual(len(client.calls), 1)

    def test_rpc_error_or_exception_never_retries(self):
        for first in (RuntimeError('RPC lost'), (0, {'ok': False, 'error': 'fault'}),
                      (0, {'ok': False, 'info': {'error': 'fault'}})):
            with self.subTest(first=first):
                client = FakeMotionClient([first, (.1, {'ok': True})])
                with self.assertRaises(RuntimeError):
                    self.move(client)
                self.assertEqual(len(client.calls), 1)

    def test_warning_mode_never_uses_legacy_tolerance_continuation(self):
        report = {'ok': False, 'stalled': {'reason': 'tracking_compensation_limit'}}
        client = FakeMotionClient([(.05, report)])
        with self.assertRaisesRegex(RuntimeError, 'after 1 P2P attempt'):
            self.move(client, '--continue-on-tolerance-miss', '--p2p-retries', '0')
        self.assertEqual(len(client.calls), 1)


if __name__ == '__main__':
    unittest.main()
