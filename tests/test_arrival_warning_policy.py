"""Explicit two-band endpoint acceptance; synthetic feedback, no robot RPC."""
from contextlib import redirect_stderr
import io
import json
import os
import unittest
from unittest.mock import patch

from test_continuous_p2p import adapter, Clock, FakeClient
import test_grasp_continuous_correction as grasp_tests
from test_grasp_continuous_correction import AtomicCallbackClient, error_report
from test_p2p_continuous_correction import SavedTargetPlant

POLICY = {'position_tolerance_m': .005, 'rotation_tolerance_rad': .2, 'warning_multiplier': 2}


class ArrivalWarningPolicyTests(unittest.TestCase):
    def setUp(self):
        self.environment = patch.dict(os.environ, {adapter.legacy.ARRIVAL_POLICY_ENV: json.dumps(POLICY)})
        self.environment.start()
        self.addCleanup(self.environment.stop)

    def check(self, position, rotation):
        return adapter.legacy.arrival_check(error_report(position, rotation, 0, 0), POLICY)

    def test_exact_boundaries_and_independent_components(self):
        self.assertEqual(self.check(.005, .2)['status'], 'within_tolerance')
        for p, r in ((.005001, 0), (0, .200001), (.01, .4)):
            self.assertEqual(self.check(p, r)['status'], 'warning')
        for p, r in ((.010001, 0), (0, .400001), (.011, .41)):
            self.assertFalse(self.check(p, r)['accepted'])
        with self.assertRaises(ValueError):
            self.check(float('nan'), 0)

    def test_continuous_warning_requires_stationary_fresh_hold(self):
        clock = Clock()
        client = FakeClient(clock, left_saved=(0,) * 6, right_saved=(0,) * 6)
        with patch.object(adapter, 'time', clock), redirect_stderr(io.StringIO()) as log:
            result = adapter.run(client, [.007, 0, 0, 0, 0, .3], [0] * 6)
        self.assertTrue(result['ok'])
        self.assertEqual(result['arrival_check']['status'], 'warning')
        self.assertEqual(len(result['hold_confirmation']), 3)
        self.assertEqual(result['tolerances']['position_tolerance_m'], .01)
        self.assertIn('p2p_arrival_warning', log.getvalue())
        self.assertFalse(result['physical_stop_ack'])

    def test_continuous_outside_band_and_native_fault_never_succeed(self):
        for fault in (False, True):
            with self.subTest(fault=fault):
                clock = Clock()
                client = FakeClient(clock, left_saved=(0,) * 6, right_saved=(0,) * 6)
                client.frozen = True
                if fault:
                    client.mutate = lambda obs, _: obs['left_arm']['native_robot_state']['current_errors'].update(joint_reflex=True)
                with patch.object(adapter, 'time', clock), redirect_stderr(io.StringIO()):
                    with self.assertRaises(adapter.ContinuousP2PError):
                        adapter.run(client, [.011, 0, 0, 0, 0, .41], [0] * 6)
                if fault:
                    self.assertFalse(client.actions)

    def test_segmented_returns_warning_after_bounded_correction(self):
        client = SavedTargetPlant(bias=[.007, 0, 0, 0, 0, .3])
        with redirect_stderr(io.StringIO()):
            result = client.dual_robot_move_to_ee_pose([0, .35, 0, 0, 0, 0], [0, -.38, 0, 0, 0, 0],
                controller='segmented', smooth=True, sleep=False, max_correction_iters=0)
        self.assertTrue(result['ok'])
        self.assertEqual(result['arrival_check']['status'], 'warning')

    def test_hold_drift_beyond_warning_band_stops_without_reacquisition(self):
        clock = Clock()
        client = FakeClient(clock, left_saved=(0,) * 6, right_saved=(0,) * 6)
        def drift(observation, source):
            if source.sequence >= 4:
                observation['left_arm']['native_robot_state']['torso_eef_pose']['position'][0] = .03
        client.mutate = drift
        with patch.object(adapter, 'time', clock), redirect_stderr(io.StringIO()):
            with self.assertRaisesRegex(adapter.ContinuousP2PError, 'arrival tolerance|explicit stop tolerance'):
                adapter.run(client, [.007, 0, 0, 0, 0, .3], [0] * 6)
        self.assertFalse(client.actions)

    def test_grasp_rechecks_new_measured_pose_and_stops_outside_band(self):
        fixture = grasp_tests.GraspContinuousCorrectionTests()
        fixture.setUp()
        client = AtomicCallbackClient([error_report(.007, .3, 0, 0)])
        _, _, result = fixture.move(client)
        self.assertEqual(result['independent_arrival_check']['status'], 'warning')
        client = AtomicCallbackClient([error_report(.0101, .1, 0, 0)])
        with self.assertRaisesRegex(RuntimeError, 'no whole-target restart'):
            fixture.move(client)
        self.assertEqual(len(client.calls), 1)

    def test_invalid_policy_is_rejected_before_dispatch(self):
        for policy in ({**POLICY, 'warning_multiplier': 3}, {**POLICY, 'position_tolerance_m': float('inf')}, {},
                       {**POLICY, 'precision_position_tolerance_m': .006},
                       {**POLICY, 'precision_position_tolerance_m': float('nan')},
                       {**POLICY, 'precision_position_tolerance_m': True}):
            with patch.dict(os.environ, {adapter.legacy.ARRIVAL_POLICY_ENV: json.dumps(policy)}):
                with self.assertRaises(ValueError):
                    adapter.legacy.arrival_policy(.003, .1)

    def test_precision_goal_does_not_change_outer_band_or_accept_imprecise_result(self):
        policy = {**POLICY, 'precision_position_tolerance_m': .002}
        checked = adapter.legacy.arrival_check(error_report(.00233, .01, 0, 0), policy)
        self.assertFalse(checked['accepted'])
        self.assertEqual(checked['status'], 'precision_not_reached')
        self.assertEqual(checked['stop_position_error_m'], .01)
        self.assertEqual(checked['stop_rotation_error_rad'], .4)
        self.assertTrue(adapter.legacy.arrival_check(error_report(.002, .3, 0, 0), policy)['accepted'])

    def test_force_protection_records_exact_component_despite_warning_policy(self):
        clock = Clock()
        client = FakeClient(clock, left_saved=(0,) * 6, right_saved=(0,) * 6)
        def loaded(observation, source):
            observation['left_arm']['native_robot_state']['wrench']['force_n'] = [0, 0, 10.1]
        client.mutate = loaded
        with patch.object(adapter, 'time', clock), redirect_stderr(io.StringIO()):
            with self.assertRaisesRegex(adapter.ContinuousP2PError, 'force_norm_n.*10.1'):
                adapter.run(client, [.007, 0, 0, 0, 0, .3], [0] * 6)
        self.assertFalse(client.actions)


if __name__ == '__main__':
    unittest.main()
