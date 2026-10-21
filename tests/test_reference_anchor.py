"""Warning-band reference selection: synthetic poses/plant, no hardware."""
import io
import json
import os
import unittest
from contextlib import redirect_stderr
from unittest.mock import patch
from test_external_tracking import adapter, FakeClient, Clock

g = adapter.tracking.g


class ReferenceAnchorTests(unittest.TestCase):
    def servo(self, angle, saved_angle=0.):
        initial = g.Pose((0., 0., 0.), (0., 0., 0., 1.))
        goal = g.Pose((0., 0., 0.), g._quat_from_rotvec((0., 0., angle)))
        saved = g.Pose(initial.position_m, g._quat_from_rotvec((0., 0., saved_angle)))
        config = adapter.tracking.command_config(g.ServoConfig(
            rotation_tolerance_rad=.4, max_rotation_bias_rad=.3), profile='fast_tracking')
        return adapter.tracking.TrackingServo(initial, goal, saved, config, 5.,
            profile='fast_tracking', path_rotation_tolerance_rad=.2)

    def test_recorded_warning_band_rotation_keeps_measured_start(self):
        servo = self.servo(.30746310483436007)
        self.assertEqual(servo.orientation_reference, 'measured_start')
        self.assertLess(g._error(servo.reference, servo.confirmed_command)['rotation_rad'], 1e-12)
        self.assertEqual(servo.config.max_rotation_bias_rad, .3)

    def test_satisfied_goal_anchor_must_fit_unchanged_saved_bias(self):
        servo = self.servo(.1, saved_angle=-.25)
        self.assertEqual(servo.orientation_reference, 'measured_start')
        self.assertAlmostEqual(servo.initial_bias['rotation_rad'], .25)

    def test_invalid_saved_bias_still_rejects(self):
        with self.assertRaisesRegex(ValueError, 'bias cap'):
            self.servo(.1, saved_angle=-.31)

    def test_warning_band_rotation_runs_bounded_path_instead_of_preflight_stop(self):
        clock = Clock()
        client = FakeClient(clock, left_saved=(0.,)*6, right_saved=(0.,)*6)
        environment = {'FRANKA_P2P_TRACKING_PROFILE':'fast_tracking',
            'FRANKA_P2P_ROTATION_BIAS_LIMIT_RAD':'.3',
            'FRANKA_P2P_TRANSLATION_BIAS_LIMIT_M':'none',
            'FRANKA_P2P_ARRIVAL_POLICY':json.dumps({'position_tolerance_m':.005,
                'rotation_tolerance_rad':.2,'warning_multiplier':2})}
        with patch.object(adapter.base, 'time', clock), patch.dict(os.environ, environment), \
             patch('socket.socket', side_effect=AssertionError('network forbidden')), \
             patch('subprocess.Popen', side_effect=AssertionError('process forbidden')), \
             redirect_stderr(io.StringIO()):
            result = adapter.run(client, (0.,0.,0.,0.,0.,.30746310483436007), (0.,)*6,
                position_tolerance_m=.005, rotation_tolerance_rad=.2,
                max_tracking_translation_bias_m=None, max_tracking_rotation_bias_rad=.3)
        self.assertTrue(result['ok'])
        self.assertTrue(client.actions)
        self.assertLessEqual(result['final_error']['max_rotation_error_rad'], .2)
        self.assertEqual(result['servo_config']['max_rotation_bias_rad'], .3)
