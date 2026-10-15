"""Offline same-target retry contracts; no robot or RPC connection is created."""
from __future__ import annotations

import copy
import importlib.util
import io
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
GRASP_PATH = ROOT / 'task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/grasp_right_arm_xyz.py'
spec = importlib.util.spec_from_file_location('grasp_bounded_p2p_retry_test', GRASP_PATH)
if spec is None or spec.loader is None:
    raise RuntimeError(f'Cannot import {GRASP_PATH}')
grasp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(grasp)


def pose(x=0.0, rotation=0.0):
    return [x, 0.0, 0.0, 0.0, 0.0, rotation]


def outcome(x, *, right=None, report=None, native_patch=None):
    return {
        'left': pose(x), 'right': pose() if right is None else right,
        'report': report if report is not None else {
            'ok': False, 'stalled': {'reason': 'tracking_compensation_limit'},
            'final_error': {'correction_index': 10}},
        'native_patch': native_patch,
    }


class FakeMotionClient:
    def __init__(self, outcomes, *, native=False):
        self.outcomes = list(outcomes)
        self.left, self.right = pose(), pose()
        self.native = native
        self.native_patch = None
        self.move_calls = []
        self.step_calls = []
        self.recovery_calls = []
        self.closed = False

    def get_observation(self):
        observation = {
            'left_arm': {'end_pose': list(self.left)},
            'right_arm': {'end_pose': list(self.right)},
        }
        if self.native:
            for side in ('left_arm', 'right_arm'):
                observation[side]['native_robot_state'] = {
                    'age_ns': 1_000_000, 'robot_mode': 2, 'robot_mode_name': 'move',
                    'current_errors': {'joint_reflex': False},
                }
            if self.native_patch:
                side, fields = self.native_patch
                observation[side]['native_robot_state'].update(copy.deepcopy(fields))
        return observation

    def dual_robot_move_to_ee_pose(self, left_target, right_target, **kwargs):
        self.move_calls.append((copy.deepcopy(left_target), copy.deepcopy(right_target), copy.deepcopy(kwargs)))
        index = len(self.move_calls) - 1
        if index >= len(self.outcomes):
            raise AssertionError('Unexpected extra P2P attempt')
        item = self.outcomes[index]
        if isinstance(item, BaseException):
            raise item
        self.left, self.right = list(item['left']), list(item['right'])
        self.native_patch = item['native_patch']
        return copy.deepcopy(item['report'])

    def step(self, action=None):
        self.step_calls.append(copy.deepcopy(action))
        return {'observation': self.get_observation()}

    def recover_robot(self, side):
        self.recovery_calls.append(side)
        return {'ok': True}

    def ping(self):
        return {'ok': True}

    def close(self):
        self.closed = True


def motion_args(*extra):
    return grasp.build_parser().parse_args([
        '--xyz', '[0.1,0,0]', '--arm', 'left', '--position-tolerance-m', '0.01',
        '--rotation-tolerance-rad', '0.05', '--settle-time-sec', '0',
        '--max-translation-speed', '0.04', '--max-rotation-speed', '0.2',
        '--max-translation-step', '0.001', '--max-rotation-step', '0.012',
        '--max-tracking-translation-bias-m', '0.025', '--max-tracking-rotation-bias-rad', '0.20',
        *extra,
    ])


class BoundedP2PRetryTests(unittest.TestCase):
    def setUp(self):
        self.left_target = np.asarray(pose(.1))
        self.right_target = np.asarray(pose())

    def move(self, client, args=None):
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            return grasp.move_stage(client, 'stage_test', self.left_target, self.right_target,
                                    args or motion_args(), recover_stalled_side='left_arm')

    def assert_no_recovery_or_outer_reanchor(self, client):
        self.assertEqual(client.recovery_calls, [])
        self.assertEqual(client.step_calls, [])

    def test_parser_defaults_enable_two_bounded_retries_with_two_percent_progress(self):
        args = motion_args()
        self.assertEqual(args.p2p_retries, 2)
        self.assertAlmostEqual(args.p2p_retry_min_progress_ratio, .02)
        self.assertFalse(args.recover_stalled_controller)

    def test_compensation_limit_can_converge_on_second_or_third_same_target_attempt(self):
        for xs in ((.05, .1), (.05, .075, .1)):
            with self.subTest(attempts=len(xs)):
                client = FakeMotionClient([outcome(x) for x in xs])
                left, right, _ = self.move(client)
                np.testing.assert_allclose(left, self.left_target)
                np.testing.assert_allclose(right, self.right_target)
                self.assertEqual(len(client.move_calls), len(xs))
                self.assert_no_recovery_or_outer_reanchor(client)

    def test_attempt_budget_is_exact_even_while_error_keeps_decreasing(self):
        for retries, xs in ((0, [.05]), (1, [.05, .07]), (2, [.05, .07, .08])):
            with self.subTest(retries=retries):
                client = FakeMotionClient([outcome(x) for x in xs])
                with self.assertRaises(RuntimeError):
                    self.move(client, motion_args('--p2p-retries', str(retries)))
                self.assertEqual(len(client.move_calls), 1 + retries)
                self.assert_no_recovery_or_outer_reanchor(client)

    def test_no_progress_worsening_oscillation_and_subthreshold_progress_stop(self):
        for xs, expected in (([0, .1], 1), ([-.01, .1], 1),
                             ([.05, .03, .1], 2), ([.001, .1], 1)):
            with self.subTest(positions=xs):
                client = FakeMotionClient([outcome(x) for x in xs])
                with self.assertRaises(RuntimeError):
                    self.move(client)
                self.assertEqual(len(client.move_calls), expected)
                self.assert_no_recovery_or_outer_reanchor(client)

    def test_convergence_uses_other_arm_rotation_not_only_active_arm_position(self):
        client = FakeMotionClient([
            outcome(.06, right=pose(rotation=.3)),
            outcome(.095, right=pose(rotation=.4)),
            outcome(.1),
        ])
        with self.assertRaises(RuntimeError):
            self.move(client)
        self.assertEqual(len(client.move_calls), 2)

    def test_persistent_stall_stops_despite_position_progress(self):
        client = FakeMotionClient([outcome(.05, report={
            'ok': False, 'final_error': {'correction_index': 1, 'stall_detection': {'detected': True}},
            'stalled': {'detected': True, 'side': 'left_arm'},
        }), outcome(.1)])
        with self.assertRaises(RuntimeError):
            self.move(client)
        self.assertEqual(len(client.move_calls), 1)
        self.assert_no_recovery_or_outer_reanchor(client)

    def test_explicit_controller_recovery_consumes_the_same_three_call_budget(self):
        client = FakeMotionClient([outcome(0, report={
            'ok': False, 'final_error': {'stall_detection': {'detected': True}},
            'stalled': {'detected': True, 'side': 'left_arm'},
        }), outcome(.05), outcome(.075), outcome(.1)])
        args = motion_args('--recover-stalled-controller', '--controller-recovery-settle-time-sec', '0')
        with self.assertRaisesRegex(RuntimeError, 'after 3 P2P attempt'):
            self.move(client, args)
        self.assertEqual(len(client.move_calls), 3)
        self.assertEqual(client.recovery_calls, ['left_arm'])
        self.assertEqual(client.step_calls, [])

    def test_rpc_exception_is_not_replayed(self):
        error = RuntimeError('RPC reply lost; command outcome unknown')
        client = FakeMotionClient([error, outcome(.1)])
        with self.assertRaises(RuntimeError) as caught:
            self.move(client)
        self.assertIs(caught.exception, error)
        self.assertEqual(len(client.move_calls), 1)
        self.assert_no_recovery_or_outer_reanchor(client)

    def test_rpc_ok_cannot_override_failed_measured_pose(self):
        client = FakeMotionClient([outcome(.05, report={'ok': True}),
                                   outcome(.05, report={'ok': True}), outcome(.1)])
        with self.assertRaises(RuntimeError):
            self.move(client)
        self.assertEqual(len(client.move_calls), 2)

    def test_retry_preserves_both_targets_and_all_motion_limits(self):
        args = motion_args()
        client = FakeMotionClient([outcome(.05), outcome(.075), outcome(.1)])
        left_before, right_before = self.left_target.copy(), self.right_target.copy()
        self.move(client, args)
        for left, right, kwargs in client.move_calls:
            np.testing.assert_array_equal(left, left_before)
            np.testing.assert_array_equal(right, right_before)
            self.assertFalse(kwargs['delta'])
            self.assertTrue(kwargs['smooth'])
            for key in ('position_tolerance_m', 'rotation_tolerance_rad', 'max_translation_speed',
                        'max_rotation_speed', 'max_translation_step', 'max_rotation_step',
                        'max_tracking_translation_bias_m', 'max_tracking_rotation_bias_rad'):
                self.assertEqual(kwargs[key], getattr(args, key))
        np.testing.assert_array_equal(self.left_target, left_before)
        np.testing.assert_array_equal(self.right_target, right_before)
        self.assert_no_recovery_or_outer_reanchor(client)

    def test_legacy_continue_does_not_skip_retries_or_arrival_tolerance(self):
        client = FakeMotionClient([outcome(.05), outcome(.1)])
        left, _, _ = self.move(client, motion_args('--continue-on-tolerance-miss'))
        np.testing.assert_allclose(left, self.left_target)
        self.assertEqual(len(client.move_calls), 2)

    def test_explicit_legacy_continue_with_zero_retries_is_preserved(self):
        client = FakeMotionClient([outcome(.05)])
        left, _, result = self.move(client, motion_args('--continue-on-tolerance-miss', '--p2p-retries', '0'))
        self.assertAlmostEqual(left[0], .05)
        self.assertFalse(result['ok'])
        self.assertEqual(len(client.move_calls), 1)

    def test_native_fault_or_stale_feedback_rejects_even_arrived_pose(self):
        for side in ('left_arm', 'right_arm'):
            for fields in ({'age_ns': 250_000_001}, {'current_errors': {'joint_reflex': True}},
                           {'robot_mode': 4, 'robot_mode_name': 'reflex'}):
                with self.subTest(side=side, fields=fields):
                    client = FakeMotionClient([outcome(.1, report={'ok': True}, native_patch=(side, fields))], native=True)
                    with self.assertRaises((RuntimeError, ValueError)):
                        self.move(client)
                    self.assertEqual(len(client.move_calls), 1)
                    self.assert_no_recovery_or_outer_reanchor(client)

    def test_existing_native_fault_stops_before_first_command(self):
        client = FakeMotionClient([outcome(.1)], native=True)
        client.native_patch = ('left_arm', {'current_errors': {'joint_reflex': True}})
        with self.assertRaises((RuntimeError, ValueError)):
            self.move(client)
        self.assertEqual(client.move_calls, [])

    def test_main_cannot_reach_close_or_later_stages_after_retry_failure(self):
        client = FakeMotionClient([outcome(.05), outcome(.05)])
        with patch.object(grasp, 'DualFrankaRobotiqRpcClient', return_value=client), \
             patch.object(grasp, 'reanchor'), \
             patch.object(grasp, 'close_gripper_and_confirm') as close_gripper, \
             patch.object(grasp, 'move_stage', wraps=grasp.move_stage) as move_stage, \
             redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            with self.assertRaises(RuntimeError):
                grasp.main(['--xyz', '[0.1,0,0]', '--arm', 'left', '--execute',
                            '--keep-current-rotation', '--no-return-transition',
                            '--position-tolerance-m', '.01', '--rotation-tolerance-rad', '.05',
                            '--p2p-retries', '2'])
        self.assertEqual(move_stage.call_count, 1)
        self.assertIn('stage1_left_move_xy_keep_z', move_stage.call_args.args[1])
        self.assertEqual(len(client.move_calls), 2)
        close_gripper.assert_not_called()
        self.assertTrue(client.closed)


if __name__ == '__main__':
    unittest.main()
