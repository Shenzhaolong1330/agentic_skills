"""Full-path feedback tests: synthetic plant/time, no robot or socket access."""
import importlib.util
import io
import os
import unittest
from contextlib import redirect_stderr
from unittest.mock import patch

from test_continuous_p2p import Clock, FakeClient, PATH

spec = importlib.util.spec_from_file_location('test_external_tracking_adapter', PATH.with_name('external_tracking_p2p.py'))
adapter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter)
g = adapter.tracking.g


class CoupledPlant(FakeClient):
    """Synthetic quasi-static cross coupling, not a hardware or recorded replay."""
    def step(self, action):
        result = super().step(action)
        # Horizontal displacement changes an unmodelled orientation disturbance.
        self.measured['left_arm'][5] = self.saved['left_arm'][5] - 1.5 * self.measured['left_arm'][0]
        return result


class ExternalTrackingTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.client = FakeClient(self.clock)
        self.log = io.StringIO()
        for active in (patch.object(adapter.base, 'time', self.clock),
                       patch.dict(os.environ),
                       patch('socket.socket', side_effect=AssertionError('network forbidden')),
                       patch('subprocess.Popen', side_effect=AssertionError('process forbidden'))):
            active.start()
            self.addCleanup(active.stop)
        os.environ.pop('FRANKA_P2P_TRANSLATION_BIAS_LIMIT_M', None)
        os.environ.pop('FRANKA_P2P_ARRIVAL_POLICY', None)

    def run_servo(self, goal=(.06, 0., 0., 0., 0., 0.), **kwargs):
        options = {'position_tolerance_m': .003, 'rotation_tolerance_rad': .1,
                   'max_tracking_translation_bias_m': None}
        options.update(kwargs)
        with redirect_stderr(self.log):
            return adapter.run(self.client, goal, [0.] * 6, **options)

    def test_entire_path_uses_feedback_and_preserves_initial_compensation(self):
        saved = list(self.client.saved['left_arm'])
        result = self.run_servo()
        self.assertTrue(result['ok'])
        self.assertEqual(result['controller'], 'external_tracking')
        self.assertEqual({t['phase'] for t in result['timings']}, {'servo'})
        first = self.client.actions[0]['left_arm']['motion']['translation'][0]
        self.assertGreater(first, 0.)
        self.assertLess(first, .001)
        self.assertGreater(self.client.saved['left_arm'][0], .06 + saved[0] - .003)
        self.assertLessEqual(result['final_error']['max_translation_error_m'], .003)
        self.assertGreaterEqual(result['hold_confirmation'][-1]['monotonic_s'] -
                                result['hold_confirmation'][0]['monotonic_s'], .5)
        self.assertTrue(all(b-a >= .05-1e-12 for a,b in zip(self.client.send_times,self.client.send_times[1:])))

    def test_rotation_is_corrected_before_horizontal_approach_finishes(self):
        self.client = CoupledPlant(self.clock)
        result = self.run_servo(goal=(.15, 0., 0., 0., 0., 0.))
        self.assertTrue(result['ok'])
        rows = [p['tracking']['left_arm'] for p in result['continuous_progress']]
        self.assertTrue(any(.1 < r['path_fraction'] < .9 and
                            abs(r['reference']['position_m'][0]) > .01 for r in rows))
        early_rotation = [a['left_arm']['motion']['rotation_rotvec'] for a in self.client.actions[:100]]
        self.assertTrue(any(g._norm(r) > 1e-6 for r in early_rotation))
        self.assertTrue(any(r['requested_scale'] < .9 for r in rows))
        self.assertLessEqual(result['final_error']['max_rotation_error_rad'], .1)

    def test_governor_freezes_reference_then_resumes_without_reset(self):
        start = g.Pose((0.,0.,0.), (0.,0.,0.,1.))
        goal = g.Pose((.15,0.,0.), start.quaternion_xyzw)
        config = adapter.tracking.command_config(g.ServoConfig(max_translation_bias_m=None))
        servo = adapter.tracking.TrackingServo(start, goal, start, config, 18.)
        for _ in range(30):
            proposal = servo.update(servo.reference, .05)
            servo.confirm_command(proposal.command)
        for _ in range(20):
            delayed = g.Pose(tuple(x-.015 if i==0 else x for i,x in enumerate(servo.reference.position_m)),
                             servo.reference.quaternion_xyzw)
            proposal = servo.update(delayed, .05)
            servo.confirm_command(proposal.command)
        fraction = servo.virtual_time
        self.assertAlmostEqual(servo.governor_scale, 0.)
        proposal = servo.update(delayed, .05)
        servo.confirm_command(proposal.command)
        self.assertEqual(servo.virtual_time, fraction)
        for _ in range(20):
            proposal = servo.update(servo.reference, .05)
            servo.confirm_command(proposal.command)
        self.assertGreater(servo.virtual_time, fraction)

    def test_pending_reference_never_advances_without_ack(self):
        start = g.Pose((0.,0.,0.), (0.,0.,0.,1.))
        servo = adapter.tracking.TrackingServo(start, g.Pose((.02,0.,0.),start.quaternion_xyzw),
            start, adapter.tracking.command_config(g.ServoConfig()), 3.)
        update = servo.update(start, .05)
        self.assertEqual(servo.virtual_time, 0.)
        self.assertEqual(servo.confirmed_command, start)
        with self.assertRaisesRegex(RuntimeError, 'pending'):
            servo.update(start, .05)
        servo.confirm_command(update.command)
        self.assertGreater(servo.virtual_time, 0.)

    def test_frozen_plant_exhausts_one_budget_without_success(self):
        self.client.frozen = True
        with self.assertRaisesRegex(adapter.base.ContinuousP2PError, 'budget exhausted'):
            self.run_servo()
        self.assertLess(self.clock.now, 40.)
        self.assertNotIn('p2p_servo_arrived', self.log.getvalue())

    def test_lost_ack_is_not_replayed(self):
        self.client.step_error = TimeoutError('synthetic lost ACK')
        with self.assertRaisesRegex(adapter.base.ContinuousP2PError, 'lost ACK'):
            self.run_servo()
        self.assertEqual(len(self.client.actions), 1)

    def test_slow_rpc_does_not_send_a_catch_up_command(self):
        self.client.latency = .11
        with self.assertRaisesRegex(adapter.base.ContinuousP2PError, 'exceeded cycle limit'):
            self.run_servo()
        self.assertEqual(len(self.client.actions), 1)

    def test_feedback_fault_stops_during_approach_not_only_at_endpoint(self):
        def mutate(observation, client):
            if len(client.actions) == 3:
                observation['left_arm']['native_robot_state']['current_errors']['joint_reflex'] = True
        self.client.mutate = mutate
        with self.assertRaisesRegex(adapter.base.ContinuousP2PError, 'fault/mode change'):
            self.run_servo()
        self.assertEqual(len(self.client.actions), 3)

    def test_initial_excess_bias_is_not_hidden_by_reanchoring(self):
        self.client.saved['left_arm'][5] = .21
        with self.assertRaisesRegex(adapter.base.ContinuousP2PError, 'bias cap'):
            self.run_servo()
        self.assertFalse(self.client.actions)

    def test_already_arrived_is_read_only(self):
        result = self.run_servo(goal=(0.,)*6)
        self.assertTrue(result['ok'])
        self.assertFalse(self.client.actions)

    def test_holding_arm_targets_remain_exactly_unchanged(self):
        saved = self.client.saved['right_arm'][:]
        self.run_servo()
        self.assertEqual(saved, self.client.saved['right_arm'])
        self.assertTrue(all(a['right_arm']['motion']['translation'] == [0.,0.,0.]
                            and a['right_arm']['motion']['rotation_rotvec'] == [0.,0.,0.]
                            for a in self.client.actions))

    def test_combined_commands_respect_speed_step_and_acceleration(self):
        result = self.run_servo(goal=(.04,.02,.01,0.,0.,.08),
                                max_translation_speed=.01, max_rotation_speed=.04)
        previous = {'translation': (0.,)*3, 'rotation_rotvec': (0.,)*3}
        for action, timing in zip(self.client.actions, result['timings']):
            dt = timing['control_dt_s']
            for key, speed, acceleration, step in [('translation',.01,.04,.001),
                                                     ('rotation_rotvec',.04,.16,.004)]:
                increment = action['left_arm']['motion'][key]
                velocity = g._scale(increment,1./dt)
                self.assertLessEqual(g._norm(increment), step+1e-9)
                self.assertLessEqual(g._norm(velocity), speed+1e-9)
                self.assertLessEqual(g._norm(g._sub(velocity,previous[key])), acceleration*dt+1e-9)
                previous[key] = velocity

    def test_stale_feedback_and_foreign_command_stop_without_extra_motion(self):
        for kind in ('age','count','saved','speed'):
            with self.subTest(kind=kind):
                self.client = FakeClient(self.clock)
                def mutate(observation, client):
                    if len(client.actions) != 2:
                        return
                    if kind == 'age':
                        observation['left_arm']['native_robot_state']['age_ns'] = 999_000_000
                    elif kind == 'count':
                        observation['command_target_state']['step_count'] += 1
                    elif kind == 'saved':
                        observation['command_target_state']['arms']['left_arm']['position_m'][0] += .001
                    else:
                        observation['left_arm']['native_robot_state']['joint_velocities'][0] = .16
                self.client.mutate = mutate
                with self.assertRaises(adapter.base.ContinuousP2PError):
                    self.run_servo()
                self.assertEqual(len(self.client.actions), 2)

    def test_hold_reacquires_small_drift_under_the_same_deadline(self):
        hold = adapter.TrackingRun.hold
        attempts = []
        def disturbed(runner, **kwargs):
            attempts.append(len(self.client.actions))
            if len(attempts) == 1:
                self.client.bias['left_arm'][0] += .004
                self.client.measured['left_arm'][0] -= .004
            return hold(runner, **kwargs)
        with patch.object(adapter.TrackingRun, 'hold', disturbed):
            result = self.run_servo()
        self.assertTrue(result['ok'])
        self.assertEqual(result['settle_reacquisitions'], 1)
        self.assertEqual(len(attempts), 2)

    def test_old_and_new_modes_are_explicit_and_raw_step_is_untouched(self):
        client = adapter.base.legacy
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(client.p2p_controller_mode(), 'segmented')
        with patch.dict(os.environ, {'FRANKA_P2P_CONTROLLER':'external_tracking'}):
            self.assertEqual(client.p2p_controller_mode(), 'external_tracking')
            self.assertEqual(client.p2p_controller_mode('segmented'), 'segmented')
        self.assertIs(client._continuous_p2p_module('external_tracking').TrackingRun.__base__, adapter.base._Run)

    def test_both_arms_share_governed_path_progress(self):
        self.client = CoupledPlant(self.clock)
        self.client.saved['right_arm'] = [0., .02, 0., 0., 0., 0.]
        self.client.bias['right_arm'] = [0., .02, 0., 0., 0., 0.]
        with redirect_stderr(self.log):
            result = adapter.run(self.client, [.15,0.,0.,0.,0.,0.], [0.,.04,0.,0.,0.,0.],
                position_tolerance_m=.003, rotation_tolerance_rad=.1, max_tracking_translation_bias_m=None)
        self.assertTrue(result['ok'])
        for row in result['continuous_progress']:
            self.assertAlmostEqual(row['tracking']['left_arm']['path_fraction'],
                                   row['tracking']['right_arm']['path_fraction'])

    def test_rotation_and_vertical_moves_use_the_same_feedback_path(self):
        for goal in ((0.,0.,0.,0.,0.,.4), (0.,0.,.1,0.,0.,0.)):
            with self.subTest(goal=goal):
                self.client = FakeClient(self.clock)
                result = self.run_servo(goal=goal)
                self.assertTrue(result['ok'])
                self.assertLessEqual(result['final_error']['max_translation_error_m'], .003)
                self.assertLessEqual(result['final_error']['max_rotation_error_rad'], .1)


if __name__ == '__main__':
    unittest.main()
