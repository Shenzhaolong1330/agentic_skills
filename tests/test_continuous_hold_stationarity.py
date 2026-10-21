"""Read-only settling regressions using synthetic feedback, never robot motion."""
import io
import json
import os
import unittest
from contextlib import redirect_stderr
from unittest.mock import patch

from test_continuous_p2p import adapter, Clock, FakeClient


class HoldStationarityTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.client = FakeClient(self.clock)
        self.log = io.StringIO()
        self.hold_reads = 0
        self.hold_commands = []
        self.hold_started = None
        self.mutate = lambda observation: None
        snapshot = adapter._Run.snapshot

        def sample(runner, **kwargs):
            if runner.phase == 'hold':
                self.hold_reads += 1
                self.hold_commands.append(len(self.client.actions))
                if self.hold_started is None:
                    self.hold_started = self.clock.now
                self.client.mutate = lambda observation, client: self.mutate(observation)
            return snapshot(runner, **kwargs)

        for active in (patch.object(adapter, 'time', self.clock),
                       patch.object(adapter._Run, 'snapshot', sample),
                       patch('socket.socket', side_effect=AssertionError('network forbidden'))):
            active.start()
            self.addCleanup(active.stop)

    def run_servo(self):
        with redirect_stderr(self.log):
            return adapter.run(self.client, [.005, 0., 0., 0., 0., 0.], [0.] * 6,
                               position_tolerance_m=.003, rotation_tolerance_rad=.1)

    def events(self):
        return [json.loads(line) for line in self.log.getvalue().splitlines()
                if json.loads(line)['event'] == 'p2p_servo_hold_check']

    def assert_read_only_hold(self):
        self.assertTrue(self.hold_commands)
        self.assertEqual(len(set(self.hold_commands)), 1)
        self.assertEqual(len(self.client.actions), self.hold_commands[0])

    def test_single_final_velocity_excursion_waits_for_complete_new_window(self):
        def mutate(observation):
            if self.hold_reads == 4:
                observation['left_arm']['native_robot_state']['joint_velocities'][0] = .02
        self.mutate = mutate
        result = self.run_servo()
        self.assertTrue(result['ok'])
        self.assertEqual(result['settle_reacquisitions'], 0)
        self.assertGreaterEqual(self.hold_reads, 8)
        self.assert_read_only_hold()
        failures = [e for e in self.events() if e['stage'] == 'independent' and not e['stationary']]
        self.assertEqual(len(failures), 1)
        self.assertEqual(failures[0]['metrics']['left_arm']['max_joint_velocity_rad_s'], .02)
        self.assertGreaterEqual(result['hold_confirmation'][0]['monotonic_s'],
                                self.hold_started + .5)

    def test_final_small_pose_shift_within_arrival_waits_without_extra_motion(self):
        def mutate(observation):
            if self.hold_reads >= 4:
                observation['left_arm']['native_robot_state']['torso_eef_pose']['position'][0] += .0002
        self.mutate = mutate
        result = self.run_servo()
        self.assertTrue(result['ok'])
        self.assertEqual(result['settle_reacquisitions'], 0)
        self.assert_read_only_hold()
        final_checks = [e for e in self.events() if e['stage'] == 'independent']
        self.assertFalse(final_checks[0]['metrics']['left_arm']['drift_ok'])
        self.assertTrue(final_checks[-1]['stationary'])
        self.assertLessEqual(result['final_error']['max_translation_error_m'], .003)

    def test_persistent_motion_times_out_under_original_hold_deadline(self):
        def mutate(observation):
            if self.hold_reads >= 4:
                observation['left_arm']['native_robot_state']['joint_velocities'][0] = .02
        self.mutate = mutate
        with self.assertRaisesRegex(adapter.ContinuousP2PError, 'within 6 seconds'):
            self.run_servo()
        self.assertAlmostEqual(self.clock.now - self.hold_started, 6., places=6)
        self.assert_read_only_hold()

    def test_repeated_final_excursions_cannot_renew_deadline(self):
        # Each new window can pass, but every independent read moves by 0.2 mm.
        # Inject on the independent read, identified by its shorter read interval.
        previous = None
        shift = 0.
        def oscillate(observation):
            nonlocal previous, shift
            now = self.clock.now
            if previous is not None and now - previous < .1:
                shift = .0002 - shift
            previous = now
            observation['left_arm']['native_robot_state']['torso_eef_pose']['position'][0] += shift
        self.mutate = oscillate
        with self.assertRaisesRegex(adapter.ContinuousP2PError, 'within 6 seconds'):
            self.run_servo()
        self.assertLessEqual(self.clock.now - self.hold_started, 6.001)
        self.assertGreater(len([e for e in self.events() if e['stage'] == 'independent']), 2)
        self.assert_read_only_hold()

    def test_fault_after_unsettled_final_read_still_fails_immediately(self):
        def mutate(observation):
            native = observation['left_arm']['native_robot_state']
            if self.hold_reads == 4:
                native['joint_velocities'][0] = .02
            if self.hold_reads == 5:
                native['current_errors']['joint_reflex'] = True
        self.mutate = mutate
        with self.assertRaisesRegex(adapter.ContinuousP2PError, 'fault/mode change'):
            self.run_servo()
        self.assertEqual(self.hold_reads, 5)
        self.assert_read_only_hold()

    def test_above_old_speed_cap_resets_hold_window_until_actually_stationary(self):
        def mutate(observation):
            if self.hold_reads == 4:
                observation['left_arm']['native_robot_state']['joint_velocities'][0] = .16
        self.mutate = mutate
        result = self.run_servo()
        self.assertTrue(result['ok'])
        self.assertGreaterEqual(self.hold_reads, 8)
        self.assert_read_only_hold()

    def test_explicit_arrival_uses_bounded_millimetre_stationarity(self):
        def oscillate(observation):
            observation['left_arm']['native_robot_state']['torso_eef_pose']['position'][0] += .0004 * (self.hold_reads % 2)
        self.mutate = oscillate
        policy = {'position_tolerance_m': .005, 'rotation_tolerance_rad': .2, 'warning_multiplier': 2}
        with patch.dict(os.environ, {adapter.legacy.ARRIVAL_POLICY_ENV: json.dumps(policy)}):
            result = self.run_servo()
        self.assertTrue(result['ok'])
        self.assert_read_only_hold()
        self.assertTrue(all(e['limits']['translation_drift_m'] == .001 for e in self.events()))
        self.assertLess(self.clock.now - self.hold_started, 2.)

    def test_explicit_arrival_does_not_accept_persistent_large_drift(self):
        def oscillate(observation):
            observation['left_arm']['native_robot_state']['torso_eef_pose']['position'][1] += .0012 * (self.hold_reads % 2)
        self.mutate = oscillate
        policy = {'position_tolerance_m': .005, 'rotation_tolerance_rad': .2, 'warning_multiplier': 2}
        with patch.dict(os.environ, {adapter.legacy.ARRIVAL_POLICY_ENV: json.dumps(policy)}):
            with self.assertRaisesRegex(adapter.ContinuousP2PError, 'within 6 seconds'):
                self.run_servo()
        self.assert_read_only_hold()


if __name__ == '__main__':
    unittest.main()
