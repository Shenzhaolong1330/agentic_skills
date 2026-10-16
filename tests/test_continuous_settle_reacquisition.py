"""Arrival drift regressions; fake transport and time, never robot motion."""
import io
import json
import math
from pathlib import Path
import unittest
from contextlib import redirect_stderr
from unittest.mock import patch

from test_continuous_p2p import adapter, Clock, FakeClient


class SettleReacquisitionTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.client = FakeClient(self.clock)
        for active in (patch.object(adapter, 'time', self.clock),
                       patch('socket.socket', side_effect=AssertionError('network forbidden'))):
            active.start()
            self.addCleanup(active.stop)
        self.goal = [.005, 0., 0., 0., 0., 0.]
        self.log = io.StringIO()

    def run_servo(self):
        with redirect_stderr(self.log):
            return adapter.run(self.client, self.goal, [0.] * 6,
                               position_tolerance_m=.003, rotation_tolerance_rad=.1)

    def drift(self, size=.002):
        self.client.bias['left_arm'][0] += size
        self.client.measured['left_arm'][0] -= size

    def test_small_hold_drift_resumes_same_goal_and_compensation_then_stabilizes(self):
        hold = adapter._Run.hold
        calls, saved = [], []
        def disturbed(runner, **kw):
            calls.append(len(self.client.actions))
            saved.append(list(self.client.saved['left_arm']))
            if len(calls) == 1:
                self.drift()
            return hold(runner, **kw)
        with patch.object(adapter._Run, 'hold', disturbed):
            result = self.run_servo()
        self.assertTrue(result['ok'])
        self.assertEqual(result['settle_reacquisitions'], 1)
        self.assertEqual(len(calls), 2)
        self.assertGreater(saved[1][0], saved[0][0])
        self.assertLessEqual(result['final_error']['left_translation_error_m'], .003)
        self.assertEqual(self.client.count - 40, len(self.client.actions))
        self.assertIn('p2p_servo_reacquire', self.log.getvalue())
        self.assertGreaterEqual(result['hold_confirmation'][-1]['monotonic_s'] -
                                result['hold_confirmation'][0]['monotonic_s'], .5)

    def test_repeated_drift_stops_after_three_reacquisitions(self):
        hold = adapter._Run.hold
        attempts = []
        def disturbed(runner, **kw):
            attempts.append(len(self.client.actions))
            self.drift(.001)
            return hold(runner, **kw)
        with patch.object(adapter._Run, 'hold', disturbed):
            with self.assertRaisesRegex(adapter.ContinuousP2PError, '3 settle reacquisitions'):
                self.run_servo()
        self.assertEqual(len(attempts), 4)
        self.assertEqual(len(self.client.actions), attempts[-1])

    def test_drift_during_braking_waits_for_zero_command_then_reacquires(self):
        arrival, injected = None, False
        def perturb(o, c):
            nonlocal arrival, injected
            position = o['left_arm']['native_robot_state']['torso_eef_pose']['position']
            if abs(position[0] - .005) <= .003 and arrival is None:
                arrival = c.sequence
            if arrival is not None and c.sequence > arrival and not injected:
                self.drift(.001)
                position[0] -= .001
                injected = True
        self.client.mutate = perturb
        result = self.run_servo()
        self.assertTrue(result['ok'])
        self.assertEqual(result['settle_reacquisitions'], 1)
        events = [json.loads(line) for line in self.log.getvalue().splitlines()]
        self.assertEqual([e['phase'] for e in events if e['event'] == 'p2p_servo_reacquire'], ['servo'])

    def test_drift_at_independent_final_read_restarts_stability_window(self):
        snapshot = adapter._Run.snapshot
        hold_reads = 0
        def perturb(runner, **kw):
            nonlocal hold_reads
            if runner.phase == 'hold':
                hold_reads += 1
                if hold_reads == 4:
                    self.drift(.001)
            return snapshot(runner, **kw)
        with patch.object(adapter._Run, 'snapshot', perturb):
            result = self.run_servo()
        self.assertTrue(result['ok'])
        self.assertEqual(result['settle_reacquisitions'], 1)
        self.assertGreaterEqual(hold_reads, 8)

    def test_left_reacquisition_preserves_other_arms_pending_command(self):
        injected, arrival = False, None
        def perturb(o, c):
            nonlocal injected, arrival
            position = o['left_arm']['native_robot_state']['torso_eef_pose']['position']
            if abs(position[0] - .005) <= .003 and arrival is None:
                arrival = c.sequence
            if arrival is not None and c.sequence > arrival and not injected:
                self.drift(.001)
                position[0] -= .001
                injected = True
        # Remove angular bias to focus on simultaneous two-arm translation.
        self.client = FakeClient(self.clock, right_saved=(0., .02, 0., 0., 0., 0.))
        self.client.mutate = perturb
        with redirect_stderr(self.log):
            result = adapter.run(self.client, self.goal, [0., .015, 0., 0., 0., 0.],
                                 position_tolerance_m=.003, rotation_tolerance_rad=.1)
        self.assertTrue(result['ok'])
        self.assertEqual(result['settle_reacquisitions'], 1)
        self.assertLessEqual(result['final_error']['max_translation_error_m'], .003)
        self.assertEqual(self.client.count - 40, len(self.client.actions))

    def test_original_servo_deadline_is_not_renewed_after_hold_drift(self):
        commands = []
        def late_drift(runner, **kw):
            commands.append(len(self.client.actions))
            self.drift()
            self.clock.now += 31.
            return None
        with patch.object(adapter._Run, 'hold', late_drift):
            with self.assertRaisesRegex(adapter.ContinuousP2PError, '30-second servo budget'):
                self.run_servo()
        self.assertEqual(len(self.client.actions), commands[-1])

    def test_fault_or_foreign_command_during_drift_still_aborts_without_motion(self):
        hold = adapter._Run.hold
        for kind in ('fault', 'count', 'saved'):
            with self.subTest(kind=kind):
                self.client = FakeClient(self.clock)
                commands = []
                def disturbed(runner, **kw):
                    commands.append(len(self.client.actions))
                    self.drift()
                    def mutate(o, c):
                        if kind == 'fault':
                            o['left_arm']['native_robot_state']['current_errors']['joint_reflex'] = True
                        elif kind == 'count':
                            o['command_target_state']['step_count'] += 1
                        else:
                            o['command_target_state']['arms']['left_arm']['position_m'][0] += .001
                    self.client.mutate = mutate
                    return hold(runner, **kw)
                with patch.object(adapter._Run, 'hold', disturbed):
                    with self.assertRaises(adapter.ContinuousP2PError):
                        self.run_servo()
                self.assertEqual(len(self.client.actions), commands[-1])

    def test_math_rearm_requires_confirmed_zero_velocity_and_keeps_state(self):
        math = adapter.geometry
        goal = adapter._pose6([0.] * 6)
        saved = adapter._pose6([.01, 0., 0., 0., 0., 0.])
        servo = math.ContinuousPoseServo(goal, saved)
        with self.assertRaisesRegex(RuntimeError, 'before a latched'):
            servo.resume_after_settle_drift()
        servo.update(goal, .05)
        with self.assertRaisesRegex(RuntimeError, 'unconfirmed'):
            servo.resume_after_settle_drift()
        servo.confirm_command(saved)
        servo.resume_after_settle_drift()
        self.assertEqual(servo.confirmed_command, saved)
        self.assertEqual(servo.goal, goal)
        proposal = servo.update(adapter._pose6([-.004, 0., 0., 0., 0., 0.]), .05)
        self.assertFalse(proposal.arrival_latched)
        self.assertGreater(proposal.proposed_delta[0], 0)

    def test_math_cannot_rearm_while_still_braking(self):
        goal = adapter._pose6([0.] * 6)
        servo = adapter.geometry.ContinuousPoseServo(goal, goal)
        for _ in range(10):
            proposal = servo.update(adapter._pose6([-.01, 0., 0., 0., 0., 0.]), .05)
            servo.confirm_command(proposal.command)
        proposal = servo.update(goal, .05)
        servo.confirm_command(proposal.command)
        self.assertTrue(proposal.arrival_latched)
        with self.assertRaisesRegex(RuntimeError, 'braking completes'):
            servo.resume_after_settle_drift()

    def test_recorded_terminal_drift_can_be_reacquired_in_static_bias_model(self):
        record = json.loads((Path(__file__).parent / 'fixtures/p2p_settle_drift_20261016.json').read_text())
        sample = record['terminal_samples'][0]
        goals = record['goals']
        # Recorded post-failure state initializes a constant-bias fake plant.
        # A synthetic extra transient at first hold exercises reacquisition;
        # unrecorded per-cycle physical feedback is not reconstructed here.
        for side in adapter.SIDES:
            actual, saved = sample['actual'][side], sample['saved'][side]
            self.client.measured[side] = adapter._six(adapter.Pose(actual['position'], actual['orientation_xyzw']))
            self.client.saved[side] = adapter._six(adapter.Pose(saved['position_m'], saved['quaternion_xyzw']))
            self.client.bias[side] = adapter.legacy._absolute_target_to_delta(
                self.client.measured[side], self.client.saved[side])
        initial = math.dist(self.client.measured['left_arm'][:3], goals['left_arm'][:3])
        self.assertGreater(initial, .003)
        self.assertLess(initial, .0032)
        hold = adapter._Run.hold
        calls = []
        def disturbed(runner, **kw):
            calls.append(1)
            if len(calls) == 1:
                current = self.client.measured['left_arm']
                residual = [current[i] - goals['left_arm'][i] for i in range(3)]
                magnitude = math.hypot(*residual)
                for i in range(3):
                    increment = .0004 * residual[i] / magnitude
                    current[i] += increment
                    self.client.bias['left_arm'][i] -= increment
            return hold(runner, **kw)
        with patch.object(adapter._Run, 'hold', disturbed), redirect_stderr(self.log):
            result = adapter.run(self.client, goals['left_arm'], goals['right_arm'],
                position_tolerance_m=.003, rotation_tolerance_rad=.1,
                max_tracking_translation_bias_m=None)
        self.assertTrue(result['ok'])
        self.assertEqual(result['settle_reacquisitions'], 1)
        self.assertLessEqual(result['final_error']['max_translation_error_m'], .003)
        self.assertLessEqual(result['final_error']['max_rotation_error_rad'], .1)


if __name__ == '__main__':
    unittest.main()
