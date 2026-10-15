"""Recorded posture-transition regression; a fake plant is not physical validation."""
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import sys
import unittest
from contextlib import redirect_stderr
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('recorded_transition_fakes', HERE / 'test_continuous_p2p.py')
fixtures = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = fixtures
spec.loader.exec_module(fixtures)
adapter = fixtures.adapter


class RecordedTransitionTests(unittest.TestCase):
    def setUp(self):
        self.record = json.loads((HERE / 'fixtures/p2p_posture_transition_20261015.json').read_text())
        self.clock = fixtures.Clock()
        self.client = fixtures.FakeClient(self.clock)
        self.goals = self.record['goals']
        first = self.record['first_nominal']
        for side in adapter.SIDES:
            bias = first['bias'][side]
            self.client.saved[side] = adapter.legacy._pose_from_delta(
                self.goals[side], bias['translation_vector_m'] + bias['rotation_vector_rad'])
            error = first['final_error'][side.removesuffix('_arm') + '_error_delta']
            self.client.measured[side] = adapter.legacy._pose_from_delta(self.goals[side], [-v for v in error])
            self.client.bias[side] = adapter.legacy._absolute_target_to_delta(
                self.client.measured[side], self.client.saved[side])
        self.client.count = first['step_count']

    def test_record_confirms_position_was_arrived_and_its_compensation_legal(self):
        first, last = self.record['first_nominal'], self.record['last_progress']
        self.assertLess(first['final_error']['left_translation_error_m'], .003)
        self.assertLess(first['bias']['left_arm']['translation_m'], .025)
        self.assertGreater(first['bias']['left_arm']['rotation_rad'], .2)
        self.assertGreater(last['final_error']['left_translation_error_m'], .003)
        self.assertLess(last['final_error']['left_rotation_error_rad'], .1)
        self.assertAlmostEqual(last['bias']['left_arm']['translation_m'], .025, places=10)

    def test_nominal_rotation_preserves_recorded_position_compensation(self):
        before_saved = {side: list(pose) for side, pose in self.client.saved.items()}
        before_measured = list(self.client.measured['left_arm'])
        runner = adapter._Run(self.client, self.goals['left_arm'], self.goals['right_arm'], False,
            None, None, {'position_tolerance_m': .003, 'rotation_tolerance_rad': .1,
                         'max_translation_speed': .04, 'max_rotation_speed': .2})
        with patch.object(adapter, 'time', self.clock), \
                patch('socket.socket', side_effect=AssertionError('network forbidden')), \
                redirect_stderr(io.StringIO()):
            runner.started = self.clock.monotonic()
            runner.prepare()
            metadata = runner.nominal()
        self.assertGreater(metadata['steps'], 0)
        for action in self.client.actions:
            self.assertLess(math.hypot(*action['left_arm']['motion']['translation']), 1e-12)
            self.assertEqual(action['right_arm']['motion']['translation'], [0.] * 3)
            self.assertEqual(action['right_arm']['motion']['rotation_rotvec'], [0.] * 3)
        self.assertLess(math.dist(self.client.saved['left_arm'][:3], before_saved['left_arm'][:3]), 1e-12)
        self.assertLess(math.dist(self.client.measured['left_arm'][:3], before_measured[:3]), 1e-12)
        self.assertLess(math.dist(self.client.measured['left_arm'][:3], self.goals['left_arm'][:3]), .003)
        held_error = adapter.legacy._absolute_target_to_delta(
            self.client.saved['right_arm'], before_saved['right_arm'])
        self.assertLess(math.hypot(*held_error), 1e-12)

    def test_uncapped_controller_reaches_tolerance_in_recorded_static_bias_model(self):
        # Initialize from the later read-only snapshot, then use the fake plant's
        # constant bias. This verifies removal of the software limit, not that
        # this approximate model will predict the real robot's next motion.
        observation = self.record['later_readonly']
        for side in adapter.SIDES:
            actual, saved = observation['measured'][side], observation['saved'][side]
            self.client.measured[side] = adapter._six(adapter.Pose(
                actual['position'], actual['orientation_xyzw']))
            self.client.saved[side] = adapter._six(adapter.Pose(
                saved['position_m'], saved['quaternion_xyzw']))
            self.client.bias[side] = adapter.legacy._absolute_target_to_delta(
                self.client.measured[side], self.client.saved[side])
        self.client.count = observation['step_count']
        with patch.object(adapter, 'time', self.clock), \
                patch.dict(os.environ, FRANKA_P2P_TRANSLATION_BIAS_LIMIT_M='none'), \
                patch('socket.socket', side_effect=AssertionError('network forbidden')), \
                redirect_stderr(io.StringIO()):
            result = adapter.run(self.client, self.goals['left_arm'], self.goals['right_arm'],
                position_tolerance_m=.003, rotation_tolerance_rad=.1,
                max_tracking_translation_bias_m=.025)
        self.assertTrue(result['ok'])
        self.assertIsNone(result['servo_config']['max_translation_bias_m'])
        self.assertLessEqual(result['final_error']['left_translation_error_m'], .003)
        self.assertGreater(math.dist(self.client.saved['left_arm'][:3], self.goals['left_arm'][:3]), .025)


if __name__ == '__main__':
    unittest.main()
