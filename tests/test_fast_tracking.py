"""Synthetic free-space regression tests; never evidence of hardware success."""
import json
import copy
import os
import unittest
import tempfile
from pathlib import Path
from unittest.mock import patch

import test_external_tracking as existing

adapter, g = existing.adapter, existing.g


class FastTrackingTests(unittest.TestCase):
    def setUp(self):
        existing.ExternalTrackingTests.setUp(self)
        os.environ.pop(adapter.continuity.ENV, None)
        os.environ.pop(adapter.base.ROTATION_LIMIT_ENV, None)
    run_servo = existing.ExternalTrackingTests.run_servo

    def servo(self, *, profile='fast_tracking', tolerance=.4, nominal=.2):
        start = g.Pose((0., 0., 0.), (0., 0., 0., 1.))
        config = adapter.tracking.command_config(g.ServoConfig(rotation_tolerance_rad=tolerance),
            .1, translation_step=.005, profile=profile)
        return adapter.tracking.TrackingServo(start, g.Pose((.15, 0., 0.), start.quaternion_xyzw),
            start, config, 4., profile=profile, path_rotation_tolerance_rad=nominal)

    def test_transient_interstage_velocity_settles_readonly_before_dispatch(self):
        started = self.clock.now
        def transient(o, c):
            if not c.actions and self.clock.now - started < .4:
                o['right_arm']['native_robot_state']['joint_velocities'][0] = .012
        self.client.mutate = transient
        result = self.run_servo(goal=(.01,0.,0.,0.,0.,0.))
        self.assertTrue(result['ok'])
        self.assertIn('p2p_preflight_settled', self.log.getvalue())
        self.assertTrue(self.client.actions)

    def test_persistent_start_velocity_never_dispatches(self):
        self.client.mutate = lambda o,c: o['right_arm']['native_robot_state']['joint_velocities'].__setitem__(0,.02)
        with self.assertRaisesRegex(adapter.base.ContinuousP2PError, 'did not settle'):
            self.run_servo()
        self.assertFalse(self.client.actions)

    def test_external_command_during_passive_start_wait_aborts(self):
        def changed(o, c):
            o['right_arm']['native_robot_state']['joint_velocities'][0] = .02
            if self.clock.now > 1.1:
                o['command_target_state']['step_count'] += 1
        self.client.mutate = changed
        with self.assertRaises(adapter.base.ContinuousP2PError):
            self.run_servo()
        self.assertFalse(self.client.actions)

    def test_recorded_angular_lag_does_not_stall_translation_inside_nominal_tolerance(self):
        measured = g.Pose((0., 0., 0.), g._quat_from_rotvec((0., 0., .07731986)))
        old, new = self.servo(profile='conservative'), self.servo()
        self.assertLess(old.tracking_scale(measured), .06)
        self.assertEqual(new.tracking_scale(measured), 1.)
        for actual, expected in zip(new.rotation_governor_limits(), (.2, .4)):
            self.assertAlmostEqual(actual, expected)
        # The user's warning band must not disable intermediate attitude control.
        bad = g.Pose((0., 0., 0.), g._quat_from_rotvec((0., 0., .401)))
        self.assertEqual(new.tracking_scale(bad), 0.)
        self.assertFalse(new.feedback.endpoint_orientation_only)

    def test_strict_caller_tolerance_keeps_strict_path_governor(self):
        servo = self.servo(tolerance=.1, nominal=.03)
        self.assertEqual(servo.rotation_governor_limits(), (.03, .06))
        self.assertEqual(servo.config.max_rotation_bias_rad, .2)

    def test_recorded_saturated_attitude_within_nominal_allows_translation_to_finish(self):
        origin = g.Pose((0., 0., 0.), (0., 0., 0., 1.))
        measured = g.Pose(origin.position_m, g._quat_from_rotvec((0., 0., -.18200717)))
        saved = g.Pose(origin.position_m, g._quat_from_rotvec((0., 0., .3)))
        config = adapter.tracking.command_config(g.ServoConfig(
            rotation_tolerance_rad=.4, max_rotation_bias_rad=.3), profile='fast_tracking')
        servo = adapter.tracking.TrackingServo(measured,
            g.Pose((.15, 0., 0.), origin.quaternion_xyzw), saved, config, 8.,
            profile='fast_tracking', path_rotation_tolerance_rad=.2,
            confirmed_orientation_reference=origin.quaternion_xyzw)
        for _ in range(400):
            update = servo.update(measured, .05)
            servo.confirm_command(update.command)
            # Synthetic plant: positional tracking, persistent measured angular
            # lag despite a saturated angular command. Not a physical trial.
            measured = g.Pose(update.command.position_m, measured.quaternion_xyzw)
            self.assertLessEqual(update.bias['rotation_rad'], .3 + 4e-12)
            if update.arrival_latched and update.command_stationary:
                break
        else:
            self.fail('Nominally acceptable attitude must not deadlock XYZ')
        self.assertLessEqual(g._error(measured, servo.final_goal)['translation_m'], .003)
        warning = g.Pose(servo.reference.position_m, g._quat_from_rotvec((0., 0., -.3)))
        self.assertGreater(servo.tracking_scale(warning), 0.)
        self.assertLess(servo.tracking_scale(warning), 1.)

    def test_no_warning_policy_does_not_enlarge_strict_governor(self):
        servo = self.servo(tolerance=.03, nominal=.03)
        self.assertEqual(servo.rotation_governor_limits(), (.018, .03))

    def test_warning_only_orientation_is_corrected_instead_of_skipped(self):
        os.environ['FRANKA_P2P_TRACKING_PROFILE'] = 'fast_tracking'
        os.environ['FRANKA_P2P_ROTATION_BIAS_LIMIT_RAD'] = '.3'
        os.environ['FRANKA_P2P_ARRIVAL_POLICY'] = json.dumps({
            'position_tolerance_m': .005, 'rotation_tolerance_rad': .2, 'warning_multiplier': 2})
        result = self.run_servo(goal=(0., 0., 0., 0., 0., .272))
        self.assertNotIn('left_arm', result['held_sides'])
        self.assertGreater(result['commands'], 0)
        self.assertLessEqual(result['final_error']['left_rotation_error_rad'], .2)

    def warning_band(self):
        os.environ['FRANKA_P2P_TRACKING_PROFILE'] = 'fast_tracking'
        os.environ['FRANKA_P2P_ROTATION_BIAS_LIMIT_RAD'] = '.3'
        os.environ['FRANKA_P2P_ARRIVAL_POLICY'] = json.dumps({
            'position_tolerance_m':.005, 'rotation_tolerance_rad':.2, 'warning_multiplier':2})

    def test_stationary_endpoint_at_cap_accepts_explicit_warning_after_correction(self):
        self.warning_band()
        self.client.frozen = True
        result = self.run_servo(goal=(0., 0., 0., 0., 0., .211))
        self.assertTrue(result['ok'])
        self.assertGreater(result['commands'], 0)
        self.assertAlmostEqual(result['final_error']['left_rotation_error_rad'], .211)
        warning = result['endpoint_warning_captures']
        self.assertTrue(warning)
        self.assertEqual(warning[0]['path_fraction'], 1.)
        self.assertGreaterEqual(warning[0]['stationary_dwell_s'], .75)
        self.assertGreaterEqual(result['hold_confirmation'][-1]['monotonic_s']-
                                result['hold_confirmation'][0]['monotonic_s'], .5)
        # A following call must retain the acknowledged reference and the same
        # compensation cap even when the accepted endpoint was a warning.
        self.assertEqual(adapter.continuity.read(self.client)['accepted_rotation_tolerance_rad'], .4)
        again = self.run_servo(goal=(0., 0., 0., 0., 0., .211))
        self.assertTrue(again['ok'])
        self.assertEqual(again['servo_config']['max_rotation_bias_rad'], .3)

    def test_endpoint_warning_cannot_accept_excess_error_or_moving_robot(self):
        self.warning_band()
        self.client.frozen = True
        with self.assertRaises(adapter.base.ContinuousP2PError):
            self.run_servo(goal=(0., 0., 0., 0., 0., .401))

    def test_endpoint_warning_wait_does_not_accept_nonstationary_native_feedback(self):
        self.warning_band()
        self.client.frozen = True
        def moving(observation, client):
            if client.actions:
                observation['left_arm']['native_robot_state']['joint_velocities'][0] = .02
        self.client.mutate = moving
        with self.assertRaises(adapter.base.ContinuousP2PError):
            self.run_servo(goal=(0., 0., 0., 0., 0., .211))

    def test_interruption_reconciliation_requires_exact_final_ack_and_fresh_reference(self):
        self.finish_with_compensation()
        previous = adapter.continuity.read(self.client)
        saved = {s: adapter.base._pose6(self.client.saved[s]) for s in adapter.SIDES}
        measured = {s: adapter.base._pose6(self.client.measured[s]) for s in adapter.SIDES}
        record = previous['arms']['left_arm']
        tracking = {'left_arm': {'reference': record['reference']}}
        progress = {'event': 'p2p_servo_progress', 'commands': 3,
            'step_count': previous['step_count'] + 3, 'tracking': tracking,
            'servo': {'left_arm': {'command': record['saved']}}}
        failure = {'event': 'p2p_servo_failed', 'phase': 'servo',
            'message': 'External tracking budget exhausted; synthetic',
            'confirmed_commands': 3, 'dispatch_attempts': 3, 'tracking': tracking}
        args = dict(session=self.client.session, count=progress['step_count'], saved=saved,
            measured=measured, config=g.ServoConfig(), nominal_rotation=.1, geometry=g)
        receipt = adapter.continuity.verified_interruption(previous, progress, failure, **args)
        self.assertFalse(receipt['task_completed'])
        self.assertFalse(receipt['automatic_retry'])
        for change in ({'dispatch_attempts': 4}, {'confirmed_commands': 2},
                       {'message': 'acknowledgement lost'}, {'phase': 'hold'}):
            with self.assertRaises(ValueError):
                adapter.continuity.verified_interruption(previous, progress, {**failure, **change}, **args)
        for change in ({'session': 'foreign'}, {'count': args['count'] + 1},
            {'saved': {**saved, 'left_arm': g.Pose((1., 0., 0.), saved['left_arm'].quaternion_xyzw)}},
            {'measured': {**measured, 'left_arm': g.Pose((0., 0., 0.), g._quat_from_rotvec((0., 0., 1.)))}}):
            with self.assertRaises(ValueError):
                adapter.continuity.verified_interruption(previous, progress, failure, **{**args, **change})
        mismatched = copy.deepcopy(failure)
        mismatched['tracking']['left_arm']['reference']['position_m'] = (1., 0., 0.)
        with self.assertRaises(ValueError):
            adapter.continuity.verified_interruption(previous, progress, mismatched, **args)

    def test_step_rate_limit_is_in_the_nominal_duration(self):
        start = g.Pose((0., 0., 0.), (0., 0., 0., 1.))
        goal = g.Pose((.15, 0., 0.), start.quaternion_xyzw)
        for period, expected in ((.05, .02), (.1, .01)):
            cfg = adapter.tracking.command_config(g.ServoConfig(), .1, translation_step=.001,
                period_s=period, profile='fast_tracking')
            self.assertAlmostEqual(cfg.max_translation_speed_m_s, expected)
            self.assertGreaterEqual(adapter.tracking.path_duration(start, goal, cfg),
                                    1.875 * .15 / (expected * .8) - 1e-10)

    def test_ideal_100mm_completes_with_final_hold_under_five_seconds(self):
        os.environ['FRANKA_P2P_TRACKING_PROFILE'] = 'fast_tracking'
        result = self.run_servo(goal=(.10, 0., 0., 0., 0., 0.),
                                max_translation_speed=.1, max_translation_step=.005)
        self.assertLess(result['performance']['actual_call_s'], 5.)
        self.assertTrue(result['performance']['met_target'])
        self.assertEqual(result['orientation_constraint'], 'whole_path')
        self.assertLessEqual(result['final_error']['max_translation_error_m'], .003)
        self.assertGreaterEqual(result['hold_confirmation'][-1]['monotonic_s'] -
                                result['hold_confirmation'][0]['monotonic_s'], .5)

    def test_coupled_orientation_is_corrected_during_approach(self):
        self.client = existing.CoupledPlant(self.clock)
        os.environ['FRANKA_P2P_TRACKING_PROFILE'] = 'fast_tracking'
        result = self.run_servo(goal=(.10, 0., 0., 0., 0., 0.),
                                max_translation_speed=.1, max_translation_step=.005)
        self.assertTrue(result['ok'])
        self.assertLessEqual(result['final_error']['max_rotation_error_rad'], .1)
        angular_during_transit = [a['left_arm']['motion']['rotation_rotvec']
            for a in self.client.actions if abs(a['left_arm']['motion']['translation'][0]) > 1e-6]
        self.assertTrue(any(g._norm(v) > 1e-6 for v in angular_during_transit))

    def test_joint_governor_stops_correction_as_well_as_reference(self):
        servo = self.servo()
        measured = g.Pose((-.005, 0., 0.), g._quat_from_rotvec((0., 0., .1)))
        update = servo.update(measured, .05, reference_scale=0., feedback_scale=0.)
        self.assertEqual(update.proposed_delta, (0.,) * 6)

    def test_slowdown_is_immediate_and_recovery_is_ramped(self):
        servo = self.servo()
        servo.governor_scale = 1.
        update = servo.update(servo.initial, .05, reference_scale=.1)
        self.assertAlmostEqual(servo.telemetry['governor_scale'], .1)
        servo.confirm_command(update.command)
        update = servo.update(servo.initial, .05, reference_scale=1.)
        self.assertLessEqual(servo.telemetry['governor_scale'], .2)

    def test_joint_pause_unwinds_new_integral_without_clearing_inherited_compensation(self):
        origin = g.Pose((0., 0., 0.), (0., 0., 0., 1.))
        saved = g.Pose((.01, 0., 0.), g._quat_from_rotvec((0., 0., .1)))
        config = adapter.tracking.command_config(g.ServoConfig(), .1, translation_step=.005,
                                                profile='fast_tracking')
        servo = adapter.tracking.TrackingServo(origin, g.Pose((.15, 0., 0.), origin.quaternion_xyzw),
                                               saved, config, 4., profile='fast_tracking')
        measured = g.Pose((-.005, 0., 0.), g._quat_from_rotvec((0., 0., -.04)))
        for _ in range(12):
            update = servo.update(measured, .05, reference_scale=0.)
            servo.confirm_command(update.command)
        peak = g._error(servo.reference, servo.confirmed_command)
        self.assertGreater(peak['translation_m'], .01)
        previous = update.command_velocity
        for _ in range(30):
            update = servo.update(measured, .05, reference_scale=0., feedback_scale=0.)
            for key, acceleration in (('translation_m_s', config.max_translation_acceleration_m_s2),
                                      ('rotation_rad_s', config.max_rotation_acceleration_rad_s2)):
                self.assertLessEqual(g._norm(g._sub(update.command_velocity[key], previous[key])),
                                     acceleration * .05 + 1e-9)
            previous = update.command_velocity
            servo.confirm_command(update.command)
        bias = g._error(servo.reference, servo.confirmed_command)
        self.assertLess(bias['translation_m'], peak['translation_m'])
        self.assertLess(bias['rotation_rad'], peak['rotation_rad'])
        self.assertGreaterEqual(bias['translation_m'], .01 - 1e-9)
        self.assertGreaterEqual(bias['rotation_rad'], .1 - 1e-9)
        self.assertEqual(servo.initial_bias['translation_m'], .01)
        self.assertTrue(servo.telemetry['anti_windup_active'])

    def test_driver_reports_removed_joint_speed_limit_and_records_actual_speed(self):
        os.environ['FRANKA_P2P_TRACKING_PROFILE'] = 'fast_tracking'
        def mutate(observation, client):
            if 1 <= len(client.actions) <= 10:
                observation['left_arm']['native_robot_state']['joint_velocities'][0] = .16452360153198242
        self.client.mutate = mutate
        result = self.run_servo(goal=(.1, 0., 0., 0., 0., 0.),
                                max_translation_speed=.1, max_translation_step=.005)
        config = next(json.loads(l) for l in self.log.getvalue().splitlines()
                      if json.loads(l)['event'] == 'p2p_tracking_config')
        self.assertIsNone(config['governor']['joint_speed_stop_limit_rad_s'])
        self.assertFalse(config['governor']['joint_speed_governor_enabled'])
        speeds = [arm['measured_joint_speed_rad_s'] for item in result['continuous_progress']
                  for arm in item['tracking'].values()]
        self.assertGreater(max(speeds), .15)

    def test_anti_windup_never_advances_a_lagging_command_during_joint_pause(self):
        servo = self.servo()
        servo.virtual_time = 1.
        servo.reference = servo._reference_at(1.)
        servo._goal = servo.reference
        update = servo.update(servo.initial, .05, reference_scale=0., feedback_scale=0.)
        self.assertEqual(update.proposed_delta, (0.,) * 6)

    def test_fault_or_uncertain_ack_still_prevents_next_command(self):
        os.environ['FRANKA_P2P_TRACKING_PROFILE'] = 'fast_tracking'
        self.client.raw_mutate = lambda response, client: response.update(step_count=-1)
        with self.assertRaisesRegex(adapter.base.ContinuousP2PError, 'acknowledgement'):
            self.run_servo()
        self.assertEqual(len(self.client.actions), 1)

    def test_nominal_tolerance_not_warning_band_is_forwarded_by_driver(self):
        os.environ['FRANKA_P2P_TRACKING_PROFILE'] = 'fast_tracking'
        os.environ['FRANKA_P2P_ARRIVAL_POLICY'] = json.dumps({
            'position_tolerance_m': .005, 'rotation_tolerance_rad': .2, 'warning_multiplier': 2})
        self.run_servo()
        config = next(json.loads(l) for l in self.log.getvalue().splitlines()
                      if json.loads(l)['event'] == 'p2p_tracking_config')
        self.assertEqual(config['governor']['rotation_slow_rad'], .2)
        self.assertAlmostEqual(config['governor']['rotation_pause_rad'], .4)
        self.assertEqual(config['command_config']['rotation_tolerance_rad'], .4)

    def test_contact_precision_reacquires_recorded_braking_drift_under_original_outer_band(self):
        os.environ['FRANKA_P2P_TRACKING_PROFILE'] = 'fast_tracking'
        os.environ['FRANKA_P2P_ARRIVAL_POLICY'] = json.dumps({
            'position_tolerance_m': .005, 'rotation_tolerance_rad': .2, 'warning_multiplier': 2,
            'precision_position_tolerance_m': .002})
        hold = adapter.TrackingRun.hold
        attempts = []
        def disturbed(runner, **kwargs):
            attempts.append(len(self.client.actions))
            if len(attempts) == 1:
                # The measured 2026-10-17 residual that previously stopped a
                # pre-close descent. Change the plant, not its ACK or command.
                shift = self.client.measured['left_arm'][0] - (.06 - .0023299166)
                self.client.bias['left_arm'][0] += shift
                self.client.measured['left_arm'][0] -= shift
            return hold(runner, **kwargs)
        with patch.object(adapter.TrackingRun, 'hold', disturbed):
            result = self.run_servo()
        self.assertTrue(result['ok'])
        self.assertEqual(result['settle_reacquisitions'], 1)
        self.assertEqual(len(attempts), 2)
        self.assertGreater(attempts[-1], attempts[0])
        self.assertLessEqual(result['final_error']['max_translation_error_m'], .002)
        self.assertTrue(result['arrival_check']['precision_reached'])
        self.assertEqual(result['arrival_check']['stop_position_error_m'], .01)
        self.assertEqual(result['arrival_check']['stop_rotation_error_rad'], .4)

    def test_contact_precision_never_accepts_stationary_warning_position(self):
        os.environ['FRANKA_P2P_TRACKING_PROFILE'] = 'fast_tracking'
        os.environ['FRANKA_P2P_ARRIVAL_POLICY'] = json.dumps({
            'position_tolerance_m': .005, 'rotation_tolerance_rad': .2, 'warning_multiplier': 2,
            'precision_position_tolerance_m': .002})
        self.client.frozen = True
        with self.assertRaises(adapter.base.ContinuousP2PError):
            self.run_servo(goal=(.003,0.,0.,0.,0.,0.))
        self.assertTrue(self.client.actions)  # Must attempt the 2 mm goal, not skip it.
        self.assertNotIn('p2p_servo_arrived', self.log.getvalue())

    def test_contact_precision_outer_band_violation_still_stops_without_next_command(self):
        os.environ['FRANKA_P2P_TRACKING_PROFILE'] = 'fast_tracking'
        os.environ['FRANKA_P2P_ARRIVAL_POLICY'] = json.dumps({
            'position_tolerance_m': .005, 'rotation_tolerance_rad': .2, 'warning_multiplier': 2,
            'precision_position_tolerance_m': .002})
        hold = adapter.TrackingRun.hold
        commands = []
        def disturbed(runner, **kwargs):
            commands.append(len(self.client.actions))
            shift = self.client.measured['left_arm'][0] - (.06 - .0101)
            self.client.bias['left_arm'][0] += shift
            self.client.measured['left_arm'][0] -= shift
            return hold(runner, **kwargs)
        with patch.object(adapter.TrackingRun, 'hold', disturbed):
            with self.assertRaisesRegex(adapter.base.ContinuousP2PError, 'explicit stop tolerance'):
                self.run_servo()
        self.assertEqual(commands, [len(self.client.actions)])

    def test_new_profile_keeps_measured_orientation_corridor(self):
        os.environ['FRANKA_P2P_TRACKING_PROFILE'] = 'fast_tracking'
        def drift(observation, client):
            if len(client.actions) == 2:
                observation['left_arm']['native_robot_state']['torso_eef_pose']['orientation_xyzw'] = list(
                    g._quat_from_rotvec((0., 0., .5)))
        self.client.mutate = drift
        with self.assertRaisesRegex(adapter.base.ContinuousP2PError, 'orientation corridor'):
            self.run_servo()
        self.assertEqual(len(self.client.actions), 2)

    def test_capture_uses_nominal_not_warning_tolerance(self):
        servo = self.servo()
        # Nominal defaults to the configured .003 m in this direct construction.
        at_goal = g.Pose((.15 - .0017, 0., 0.), servo.final_goal.quaternion_xyzw)
        self.assertTrue(servo.capture_components(at_goal)[0])
        outside = g.Pose((.15 - .0031, 0., 0.), servo.final_goal.quaternion_xyzw)
        self.assertFalse(servo.capture_components(outside)[0])

    def test_recorded_endpoint_within_user_tolerance_is_not_blocked_by_hidden_capture_cap(self):
        origin = g.Pose((0., 0., 0.), (0., 0., 0., 1.))
        config = adapter.tracking.command_config(g.ServoConfig(
            position_tolerance_m=.010, rotation_tolerance_rad=.4), profile='fast_tracking')
        servo = adapter.tracking.TrackingServo(origin, origin, origin, config, 1.,
            profile='fast_tracking', nominal_position_tolerance_m=.005,
            path_rotation_tolerance_rad=.2)
        # Recorded console endpoint: stationary, 2.267 mm / 0.080461 rad.
        measured = g.Pose((.002267, 0., 0.), g._quat_from_rotvec((0., 0., .080461)))
        self.assertEqual(servo.capture_components(measured), (True, True))
        update = servo.update(measured, .05, allow_capture=True)
        self.assertTrue(update.arrival_latched)
        self.assertTrue(update.command_stationary)
        # The two-times warning band does not silently become the capture goal.
        warning_only = g.Pose((.006, 0., 0.), g._quat_from_rotvec((0., 0., .21)))
        self.assertEqual(servo.capture_components(warning_only), (False, False))

    def finish_with_compensation(self):
        self.client.saved['left_arm'][5] = .19
        self.client.bias['left_arm'][5] = .23
        os.environ['FRANKA_P2P_TRACKING_PROFILE'] = 'fast_tracking'
        first = self.run_servo(goal=(.06, 0., 0., 0., 0., 0.), max_translation_speed=.1,
                               max_translation_step=.005)
        self.assertTrue(first['ok'])
        self.assertGreater(abs(self.client.saved['left_arm'][5] - self.client.measured['left_arm'][5]), .2)
        return first

    def test_recorded_saturated_cross_stage_projects_inward_and_completes(self):
        # Start/goal/saved/reference recorded from console 2026-10-16 stage 2.
        # Following samples are synthetic; this is not a hardware success.
        case = json.loads((Path(__file__).parent / 'fixtures' /
                           'tracking_saturated_cross_stage.json').read_text())
        for alpha, retain_bias in ((1., False), (1., True), (.2, True)):
            with self.subTest(alpha=alpha, retain_bias=retain_bias):
                measured = g.Pose(**case['measured'])
                saved = g.Pose(**case['saved'])
                config = g.ServoConfig(**case['config'])
                servo = adapter.tracking.TrackingServo(measured, g.Pose(**case['goal']),
                    saved, config, case['duration'], profile='fast_tracking',
                    path_rotation_tolerance_rad=.2, nominal_position_tolerance_m=.005,
                    confirmed_orientation_reference=case['reference']['quaternion_xyzw'])
                bias = g._error(saved, measured)
                projections = pauses = 0
                previous = {'translation_m_s': g._ZERO, 'rotation_rad_s': g._ZERO}
                for index in range(1600):
                    before_time = servo.virtual_time
                    update = servo.update(measured, .05)
                    projections += bool(servo.telemetry['moving_reference_inward_projection'])
                    paused = 'reference_paused_for_projection' in servo.telemetry
                    pauses += paused
                    # Exercise actual transport's rotation-vector roundtrip and
                    # independent ACK speed, acceleration and braking checks.
                    actual = adapter.base._pose6(adapter.base._six(update.command))
                    servo.confirm_command(actual)
                    if paused:
                        self.assertEqual(servo.virtual_time, before_time)
                    self.assertLessEqual(update.bias['rotation_rad'], .2 + 4e-12)
                    for key, acceleration in (('translation_m_s', .1), ('rotation_rad_s', .2)):
                        self.assertLessEqual(g._norm(g._sub(update.command_velocity[key], previous[key])),
                                             acceleration * .05 + 1e-9)
                    previous = update.command_velocity
                    target = actual if not retain_bias else g.Pose(
                        g._add(actual.position_m, bias['translation_vector_m']),
                        g._integrate(actual.quaternion_xyzw, bias['rotation_vector_rad'], True))
                    error = g._error(measured, target)
                    measured = g.Pose(
                        g._add(measured.position_m, g._scale(error['translation_vector_m'], alpha)),
                        g._integrate(measured.quaternion_xyzw,
                                     g._scale(error['rotation_vector_rad'], alpha), True))
                    if update.arrival_latched and update.command_stationary:
                        break
                self.assertLess(index, 1599)
                self.assertGreater(projections, 0)
                if retain_bias:
                    self.assertGreater(pauses, 0)
                self.assertEqual(servo.capture_components(measured), (True, True))
                self.assertEqual(config.max_rotation_bias_rad, .2)

    def test_two_calls_preserve_verified_reference_without_enlarging_cap(self):
        self.finish_with_compensation()
        second = self.run_servo(goal=(.06, 0., 0., 0., 0., .5))
        self.assertTrue(second['ok'])
        self.assertEqual(second['servo_config']['max_rotation_bias_rad'], .2)
        references = [p['tracking']['left_arm']['orientation_reference']
                      for p in second['continuous_progress'] if 'left_arm' in p['tracking']]
        self.assertIn('confirmed_previous_reference', references)

    def test_explicit_cap_allows_strict_correction_with_existing_offset(self):
        self.client.saved['left_arm'][5] = .19
        self.client.measured['left_arm'][5] = -.0355
        self.client.bias['left_arm'][5] = .2255
        os.environ['FRANKA_P2P_TRACKING_PROFILE'] = 'fast_tracking'
        # The original .20 default still rejects this inherited .2255 offset.
        with self.assertRaisesRegex(adapter.base.ContinuousP2PError, 'bias cap'):
            self.run_servo(goal=(0.,0.,0.,0.,0.,0.), rotation_tolerance_rad=.025)
        self.assertEqual(self.client.actions, [])
        os.environ[adapter.base.ROTATION_LIMIT_ENV] = '.3'
        result = self.run_servo(goal=(0.,0.,0.,0.,0.,0.), rotation_tolerance_rad=.025)
        self.assertTrue(result['ok'])
        self.assertEqual(result['servo_config']['max_rotation_bias_rad'], .3)
        self.assertLessEqual(result['final_error']['max_rotation_error_rad'], .025)
        self.assertGreater(result['commands'], 0)
        # No stored compensation was discarded to obtain the strict arrival.
        self.assertGreater(abs(self.client.saved['left_arm'][5] - self.client.measured['left_arm'][5]), .2)

    def test_invalid_explicit_cap_fails_before_dispatch(self):
        for value in ('0', '-1', 'nan', 'inf', 'none', '2'):
            with self.subTest(value=value), patch.dict(os.environ, {adapter.base.ROTATION_LIMIT_ENV:value}):
                with self.assertRaises((ValueError, adapter.base.ContinuousP2PError)):
                    self.run_servo()
        self.assertEqual(self.client.actions, [])

    def test_receipt_survives_new_client_process_but_not_session_or_count_change(self):
        with tempfile.TemporaryDirectory() as directory:
            os.environ[adapter.continuity.ENV] = str(Path(directory) / 'reference.json')
            self.finish_with_compensation()
            delattr(self.client, adapter.continuity.ATTRIBUTE)
            self.assertTrue(adapter.continuity.read(self.client))
            # A foreign command invalidates the receipt and the original bias
            # check rejects this start before any new command is sent.
            actions = len(self.client.actions)
            self.client.count += 1
            with self.assertRaisesRegex(adapter.base.ContinuousP2PError, 'bias cap'):
                self.run_servo(goal=(.06, 0., 0., 0., 0., .5))
            self.assertEqual(len(self.client.actions), actions)

    def test_unrelated_session_or_changed_saved_pose_cannot_import_reference(self):
        self.finish_with_compensation()
        receipt = adapter.continuity.read(self.client)
        state = {s: adapter.base._pose6(self.client.saved[s]) for s in adapter.SIDES}
        measured = {s: adapter.base._pose6(self.client.measured[s]) for s in adapter.SIDES}
        args = dict(count=self.client.count, saved=state, measured=measured,
                    config=g.ServoConfig(), nominal_rotation=.1, geometry=g)
        self.assertEqual(adapter.continuity.matching_references(receipt, session='other', **args), {})
        altered = dict(state);altered['right_arm'] = g.Pose((.1, 0., 0.), state['right_arm'].quaternion_xyzw)
        self.assertEqual(adapter.continuity.matching_references(receipt, session=self.client.session,
            **{**args, 'saved': altered}), {})

    def test_confirmed_gripper_only_command_preserves_reference_for_next_motion(self):
        self.finish_with_compensation()
        receipt = adapter.continuity.read(self.client)
        self.client.count += 1  # Exactly one gripper-only server command.
        adapter.continuity.after_gripper(self.client, receipt, {'step_count': self.client.count}, g)
        self.assertEqual(adapter.continuity.read(self.client)['step_count'], self.client.count)
        self.assertTrue(self.run_servo(goal=(.06, 0., 0., 0., 0., .5))['ok'])

    def test_uncertain_gripper_ack_or_changed_pose_cannot_advance_receipt(self):
        self.finish_with_compensation()
        receipt = adapter.continuity.read(self.client)
        self.client.count += 1
        adapter.continuity.after_gripper(self.client, receipt, {'error': 'lost reply'}, g)
        self.assertEqual(adapter.continuity.read(self.client), receipt)
        self.client.saved['right_arm'][0] += .01
        adapter.continuity.after_gripper(self.client, receipt, {'step_count': self.client.count}, g)
        self.assertEqual(adapter.continuity.read(self.client), receipt)


if __name__ == '__main__':
    unittest.main()
