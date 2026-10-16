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
        os.environ.pop('FRANKA_P2P_TRACKING_PROFILE', None)

    def run_servo(self, goal=(.06, 0., 0., 0., 0., 0.), **kwargs):
        options = {'position_tolerance_m': .003, 'rotation_tolerance_rad': .1,
                   'max_tracking_translation_bias_m': None}
        options.update(kwargs)
        with redirect_stderr(self.log):
            return adapter.run(self.client, goal, [0.] * 6, **options)

    def test_fast_profile_ideal_150mm_includes_hold_under_five_seconds(self):
        os.environ['FRANKA_P2P_TRACKING_PROFILE'] = 'fast_free_space'
        start = self.clock.now
        result = self.run_servo(goal=(.15, 0., 0., 0., 0., 0.),
                                max_translation_speed=.1, max_translation_step=.005)
        self.assertLess(self.clock.now - start, 5.)
        self.assertLessEqual(result['final_error']['max_translation_error_m'], .003)
        self.assertGreaterEqual(result['hold_confirmation'][-1]['monotonic_s'] -
                                result['hold_confirmation'][0]['monotonic_s'], .5)
        previous = (0.,) * 3
        for action, timing in zip(self.client.actions, result['timings']):
            dt = timing['control_dt_s']
            delta = action['left_arm']['motion']['translation']
            velocity = g._scale(delta, 1. / dt)
            self.assertLessEqual(g._norm(delta), .005 + 1e-9)
            self.assertLessEqual(g._norm(velocity), .1 + 1e-9)
            self.assertLessEqual(g._norm(g._sub(velocity, previous)), .2 * dt + 1e-9)
            previous = velocity

    def test_fast_profile_preserves_satisfied_goal_compensation_without_larger_cap(self):
        goal = g.Pose((.15, 0., 0.), (0., 0., 0., 1.))
        initial = g.Pose((0., 0., 0.), g._quat_from_rotvec((0., 0., -.08)))
        saved = g.Pose(initial.position_m, g._quat_from_rotvec((0., 0., .19)))
        config = adapter.tracking.command_config(g.ServoConfig(), profile='fast_free_space')
        servo = adapter.tracking.TrackingServo(initial, goal, saved, config, 4., profile='fast_free_space')
        self.assertEqual(servo.confirmed_command, saved)
        self.assertEqual(servo.reference.quaternion_xyzw, goal.quaternion_xyzw)
        self.assertEqual(config.max_rotation_bias_rad, .2)
        outside = g.Pose(saved.position_m, g._quat_from_rotvec((0., 0., .201)))
        with self.assertRaisesRegex(ValueError, 'bias cap'):
            adapter.tracking.TrackingServo(initial, goal, outside, config, 4., profile='fast_free_space')
        unsatisfied = g.Pose(initial.position_m, g._quat_from_rotvec((0., 0., -.11)))
        with self.assertRaisesRegex(ValueError, 'bias cap'):
            adapter.tracking.TrackingServo(unsatisfied, goal, saved, config, 4., profile='fast_free_space')

    def test_fast_profile_respects_caller_speed_and_default_stays_slow(self):
        for name, speed, step in [('conservative', .02, .001), ('responsive', .02, .001),
                                  ('fast_free_space', .1, .005)]:
            config = adapter.tracking.command_config(g.ServoConfig(), .2, translation_step=.01, profile=name)
            self.assertEqual(config.max_translation_speed_m_s, speed)
            self.assertEqual(config.max_translation_step_m, step)
        config = adapter.tracking.command_config(g.ServoConfig(), .04, translation_step=.002,
                                                 profile='fast_free_space')
        self.assertEqual(config.max_translation_speed_m_s, .04)
        self.assertEqual(config.max_translation_step_m, .002)

    def test_cap_roundtrip_can_wait_without_false_failure_or_drift(self):
        initial = g.Pose((0., 0., 0.), (0., 0., 0., 1.))
        saved = g.Pose(initial.position_m, g._quat_from_rotvec((0., 0., .2 + 1e-12)))
        measured = g.Pose(initial.position_m, g._quat_from_rotvec((0., 0., -.07)))
        goal = g.Pose((.15, 0., 0.), initial.quaternion_xyzw)
        config = adapter.tracking.command_config(g.ServoConfig(max_translation_bias_m=None),
                                                 profile='fast_free_space')
        servo = adapter.tracking.TrackingServo(measured, goal, saved, config, 4., profile='fast_free_space')
        for _ in range(100):
            update = servo.update(measured, .05)
            # Exercise the actual command transport's quaternion / rotation-vector roundtrip.
            six = adapter.base._six(update.command)
            reconstructed = adapter.base._pose6(six)
            servo.confirm_command(reconstructed)
            self.assertLessEqual(g._error(initial, reconstructed)['rotation_rad'], .2 + 4e-12)
        outside = g.Pose(initial.position_m, g._quat_from_rotvec((0., 0., .2 + 1e-10)))
        with self.assertRaisesRegex(ValueError, 'bias cap'):
            adapter.tracking.TrackingServo(initial, goal, outside, config, 4., profile='fast_free_space')

    def test_fast_profile_still_stops_at_original_joint_speed_limit(self):
        os.environ['FRANKA_P2P_TRACKING_PROFILE'] = 'fast_free_space'
        def mutate(observation, client):
            if len(client.actions) == 2:
                observation['left_arm']['native_robot_state']['joint_velocities'][0] = .151
        self.client.mutate = mutate
        with self.assertRaisesRegex(adapter.base.ContinuousP2PError, 'joint speed'):
            self.run_servo(goal=(.15, 0., 0., 0., 0., 0.),
                           max_translation_speed=.1, max_translation_step=.005)
        self.assertEqual(len(self.client.actions), 2)

    def test_cap_roundoff_can_brake_in_place_then_move_inward(self):
        center=(0., 0., 0., 1.)
        current=g._quat_from_rotvec((.2 + 2e-12, 0., 0.))
        options=dict(dt=.05, speed=.04, step=.002, acceleration=.16, cap=.2, angular=True)
        stopped=g._bounded_velocity(current, center, (1e-8, 0., 0.), (.04, 0., 0.), **options)
        self.assertEqual(stopped, (0., 0., 0.))
        inward=g._bounded_velocity(current, center, stopped, (-.01, 0., 0.), **options)
        self.assertLess(inward[0], 0.)
        outside=g._quat_from_rotvec((.2 + 1e-9, 0., 0.))
        with self.assertRaises(g.NoFeasibleStep):
            g._bounded_velocity(outside, center, (1e-8, 0., 0.), (.04, 0., 0.), **options)

    def test_fast_governor_slows_before_existing_physical_limits(self):
        profile=adapter.tracking.feedback_profile('fast_free_space')
        for speed, expected in ((.09,1.),(.1175,.5),(.135,0.),(.151,0.)):
            self.assertAlmostEqual(adapter.tracking.joint_tracking_scale([[speed]*7],profile),expected)
        origin=g.Pose((0.,0.,0.),(0.,0.,0.,1.))
        goal=g.Pose((.15,0.,0.),origin.quaternion_xyzw)
        for tolerance in (.1,.01):
            config=adapter.tracking.command_config(g.ServoConfig(rotation_tolerance_rad=tolerance),
                                                   profile='fast_free_space')
            servo=adapter.tracking.TrackingServo(origin,goal,origin,config,4.,profile='fast_free_space')
            for fraction, expected in ((.7,1.),(.8,.5),(.9,0.)):
                measured=g.Pose(origin.position_m,g._quat_from_rotvec((0.,0.,fraction*tolerance)))
                self.assertAlmostEqual(servo.tracking_scale(measured),expected)

    def test_endpoint_only_orientation_does_not_slow_xyz_but_still_blocks_arrival(self):
        origin=g.Pose((0.,0.,0.),(0.,0.,0.,1.))
        goal=g.Pose((.15,0.,0.),origin.quaternion_xyzw)
        config=adapter.tracking.command_config(g.ServoConfig(), profile='fast_endpoint')
        servo=adapter.tracking.TrackingServo(origin,goal,origin,config,4.,profile='fast_endpoint')
        rotated=g.Pose(origin.position_m,g._quat_from_rotvec((0.,0.,.4)))
        self.assertEqual(servo.tracking_scale(rotated),1.)
        lagged=g.Pose((-.016,0.,0.),rotated.quaternion_xyzw)
        self.assertEqual(servo.tracking_scale(lagged),0.)
        at_xyz=g.Pose(goal.position_m,g._quat_from_rotvec((0.,0.,.11)))
        update=servo.update(at_xyz,.05)
        self.assertFalse(update.within_tolerance)
        self.assertFalse(update.arrival_latched)

    def test_endpoint_profile_accepts_transient_attitude_then_requires_final_hold(self):
        os.environ['FRANKA_P2P_TRACKING_PROFILE']='fast_endpoint'
        def mutate(observation,client):
            if 10 <= len(client.actions) <= 20:
                observation['left_arm']['native_robot_state']['torso_eef_pose']['orientation_xyzw']=list(
                    g._quat_from_rotvec((0.,0.,.5)))
        self.client.mutate=mutate
        result=self.run_servo(goal=(.15,0.,0.,0.,0.,0.),max_translation_speed=.08,max_translation_step=.004)
        self.assertTrue(result['ok'])
        self.assertLessEqual(result['final_error']['max_rotation_error_rad'],.1)
        self.assertGreaterEqual(result['hold_confirmation'][-1]['monotonic_s']-
                                result['hold_confirmation'][0]['monotonic_s'],.5)

    def test_endpoint_feedback_stops_advancing_when_joint_governor_pauses(self):
        initial=g.Pose((0.,0.,0.),(0.,0.,0.,1.))
        goal=g.Pose((.15,0.,0.),initial.quaternion_xyzw)
        config=adapter.tracking.command_config(g.ServoConfig(), profile='fast_endpoint')
        servo=adapter.tracking.TrackingServo(initial,goal,initial,config,4.,profile='fast_endpoint')
        measured=g.Pose((-.005,0.,0.),g._quat_from_rotvec((0.,0.,.15)))
        update=servo.update(measured,.05,reference_scale=0.,feedback_scale=0.)
        self.assertEqual(update.proposed_delta,(0.,)*6)
        self.assertFalse(update.within_tolerance)

    def test_endpoint_combined_commands_respect_linear_and_angular_limits(self):
        os.environ['FRANKA_P2P_TRACKING_PROFILE']='fast_endpoint'
        result=self.run_servo(goal=(.15,0.,0.,0.,0.,.2),max_translation_speed=.1,
            max_translation_step=.005,max_rotation_speed=.12,max_rotation_step=.006)
        previous={'translation':(0.,)*3,'rotation_rotvec':(0.,)*3}
        for action,timing in zip(self.client.actions,result['timings']):
            dt=timing['control_dt_s']
            for key,speed,step,acceleration in [('translation',.1,.005,.2),('rotation_rotvec',.12,.006,.4)]:
                delta=action['left_arm']['motion'][key];velocity=g._scale(delta,1./dt)
                self.assertLessEqual(g._norm(delta),step+1e-9)
                self.assertLessEqual(g._norm(velocity),speed+1e-9)
                self.assertLessEqual(g._norm(g._sub(velocity,previous[key])),acceleration*dt+1e-9)
                previous[key]=velocity
        self.assertLessEqual(result['final_error']['max_rotation_error_rad'],.1)

    def test_endpoint_capture_tightens_braking_without_changing_acceptance(self):
        origin=g.Pose((0.,0.,0.),(0.,0.,0.,1.))
        config=adapter.tracking.command_config(g.ServoConfig(),profile='fast_endpoint')
        servo=adapter.tracking.TrackingServo(origin,origin,origin,config,.5,profile='fast_endpoint')
        servo.virtual_time=.5
        measured=g.Pose((-.002,0.,0.),origin.quaternion_xyzw)
        update=servo.update(measured,.05)
        self.assertTrue(update.within_tolerance)
        self.assertFalse(update.arrival_latched)
        self.assertGreater(update.proposed_delta[0],0.)
        servo.confirm_command(update.command)
        self.assertEqual(servo.capture_components(g.Pose((-.0008,0.,0.),g._quat_from_rotvec((0.,0.,.09)))),(True,False))
        self.assertEqual(servo.capture_components(g.Pose((-.0008,0.,0.),g._quat_from_rotvec((0.,0.,.07)))),(True,True))
        self.assertEqual(config.position_tolerance_m,.003)
        self.assertEqual(config.rotation_tolerance_rad,.1)

    def test_endpoint_capture_waits_for_measured_rest_even_after_path_finishes(self):
        origin=g.Pose((0.,0.,0.),(0.,0.,0.,1.))
        config=adapter.tracking.command_config(g.ServoConfig(),profile='fast_endpoint')
        servo=adapter.tracking.TrackingServo(origin,origin,origin,config,.5,profile='fast_endpoint')
        servo.virtual_time=.5
        update=servo.update(origin,.05,allow_capture=False)
        self.assertTrue(update.within_tolerance)
        self.assertFalse(update.arrival_latched)
        servo.confirm_command(update.command)
        update=servo.update(g.Pose((-.002,0.,0.),origin.quaternion_xyzw),.05,allow_capture=False)
        self.assertGreater(update.proposed_delta[0],0.)
        self.assertFalse(update.arrival_latched)
        servo.confirm_command(update.command)
        update=servo.update(origin,.05,allow_capture=True)
        self.assertTrue(update.arrival_latched)

    def test_responsive_profile_completes_with_existing_bounds_and_hold(self):
        os.environ['FRANKA_P2P_TRACKING_PROFILE'] = 'responsive'
        result = self.run_servo()
        self.assertTrue(result['ok'])
        self.assertEqual(result['feedback_profile']['translation_gain_per_s'], .6)
        self.assertEqual(result['feedback_profile']['rotation_speed_rad_s'], .01)
        self.assertLessEqual(result['final_error']['max_translation_error_m'], .003)
        self.assertGreaterEqual(result['hold_confirmation'][-1]['monotonic_s'] -
                                result['hold_confirmation'][0]['monotonic_s'], .5)

    def test_profile_changes_translation_feedback_only_and_respects_caller_speed(self):
        origin = g.Pose((0., 0., 0.), (0., 0., 0., 1.))
        measured = g.Pose((-.006, 0., 0.), origin.quaternion_xyzw)
        speeds = {}
        for name in ('conservative', 'responsive'):
            config = adapter.tracking.command_config(g.ServoConfig(), .008)
            servo = adapter.tracking.TrackingServo(origin, origin, origin, config, 1., profile=name)
            for _ in range(3):
                update = servo.update(measured, .05)
                servo.confirm_command(update.command)
            speeds[name] = update.command_velocity['translation_m_s'][0]
        self.assertAlmostEqual(speeds['conservative'], .0018)
        self.assertAlmostEqual(speeds['responsive'], .0036)
        limited = adapter.tracking.command_config(g.ServoConfig(), .001)
        servo = adapter.tracking.TrackingServo(origin, origin, origin, limited, 1., profile='responsive')
        update = servo.update(measured, .05)
        self.assertLessEqual(g._norm(update.command_velocity['translation_m_s']), .001)

    def test_invalid_profile_rejected_before_any_observation_or_command(self):
        os.environ['FRANKA_P2P_TRACKING_PROFILE'] = 'typo'
        before = self.clock.now
        with self.assertRaisesRegex(adapter.base.ContinuousP2PError, 'profile'):
            self.run_servo()
        self.assertEqual(self.client.actions, [])
        self.assertEqual(self.clock.now, before)

    def test_responsive_translation_preserves_satisfied_orientation_and_recovers_drift(self):
        origin = g.Pose((0., 0., 0.), (0., 0., 0., 1.))
        goal = g.Pose((.1, 0., 0.), origin.quaternion_xyzw)
        saved = g.Pose(origin.position_m, g._integrate(origin.quaternion_xyzw, (0., .09, 0.), True))
        config = adapter.tracking.command_config(g.ServoConfig(rotation_tolerance_rad=.1))
        servo = adapter.tracking.TrackingServo(origin, goal, saved, config, 10., profile='responsive')
        measured = g.Pose(origin.position_m, g._integrate(origin.quaternion_xyzw, (0., .01, 0.), True))
        for _ in range(5):
            update = servo.update(measured, .05)
            self.assertEqual(tuple(update.proposed_delta[3:]), (0., 0., 0.))
            servo.confirm_command(update.command)
        self.assertLess(g._error(saved,servo.confirmed_command)['rotation_rad'], 1e-12)
        drifted = g.Pose(origin.position_m, g._integrate(origin.quaternion_xyzw, (0., .06, 0.), True))
        update = servo.update(drifted, .05)
        self.assertGreater(g._norm(update.proposed_delta[3:]), 0.)
        self.assertLessEqual(g._norm(update.command_velocity['rotation_rad_s']), .01)

    def test_deadband_does_not_suppress_requested_rotation_or_tighter_tolerance(self):
        origin = g.Pose((0., 0., 0.), (0., 0., 0., 1.))
        goal = g.Pose((.1, 0., 0.), g._integrate(origin.quaternion_xyzw, (0., .2, 0.), True))
        config = adapter.tracking.command_config(g.ServoConfig(rotation_tolerance_rad=.01))
        servo = adapter.tracking.TrackingServo(origin, goal, origin, config, 10., profile='responsive')
        self.assertEqual(servo.orientation_deadband_rad, 0.)
        same = g.Pose(goal.position_m, origin.quaternion_xyzw)
        servo = adapter.tracking.TrackingServo(origin, same, origin, config, 10., profile='responsive')
        self.assertEqual(servo.orientation_deadband_rad, .005)

    def test_actual_goal_arrival_brakes_even_when_reference_clock_is_incomplete(self):
        origin = g.Pose((0., 0., 0.), (0., 0., 0., 1.))
        goal = g.Pose((.1, 0., 0.), origin.quaternion_xyzw)
        config = adapter.tracking.command_config(g.ServoConfig(max_translation_bias_m=None,
                                                               rotation_tolerance_rad=.1))
        servo = adapter.tracking.TrackingServo(origin, goal, origin, config, 20.)
        for _ in range(10):
            update = servo.update(origin, .05)
            servo.confirm_command(update.command)
        reference, virtual = servo.reference, servo.virtual_time
        measured = g.Pose((.0994, 0., 0.), g._integrate(origin.quaternion_xyzw, (0., .082, 0.), True))
        for _ in range(20):
            update = servo.update(measured, .05)
            self.assertTrue(update.arrival_latched)
            self.assertTrue(update.within_tolerance)
            servo.confirm_command(update.command)
            self.assertEqual(servo.reference, reference)
            self.assertEqual(servo.virtual_time, virtual)
        self.assertTrue(update.command_stationary)
        self.assertLess(servo.virtual_time, servo.duration)

    def test_incomplete_reference_does_not_accept_out_of_tolerance_goal(self):
        origin = g.Pose((0., 0., 0.), (0., 0., 0., 1.))
        goal = g.Pose((.1, 0., 0.), origin.quaternion_xyzw)
        config = adapter.tracking.command_config(g.ServoConfig(max_translation_bias_m=None,
                                                               rotation_tolerance_rad=.1))
        for measured in (g.Pose((.096, 0., 0.), origin.quaternion_xyzw),
                         g.Pose((.1, 0., 0.), g._integrate(origin.quaternion_xyzw, (0., .11, 0.), True))):
            servo = adapter.tracking.TrackingServo(origin, goal, origin, config, 20.)
            update = servo.update(measured, .05)
            self.assertFalse(update.arrival_latched)

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
                # Set an absolute 4 mm final-goal residual, inside the 6 mm
                # recovery envelope even if arrival latched near its boundary.
                shift = self.client.measured['left_arm'][0] - (.06 - .004)
                self.client.bias['left_arm'][0] += shift
                self.client.measured['left_arm'][0] -= shift
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
