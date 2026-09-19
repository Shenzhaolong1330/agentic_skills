"""Model the deployed server: action deltas accumulate into saved targets."""
import copy
import importlib.util
import io
from pathlib import Path
from contextlib import redirect_stderr
import unittest

PATH = Path(__file__).resolve().parents[1] / 'atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/dual_franka_robotiq_rpc_client.py'
spec = importlib.util.spec_from_file_location('target_reference_rpc', PATH)
rpc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rpc)


class TargetAccumulator(rpc.DualFrankaRobotiqRpcClient):
    def __init__(self, bias=0, sync_error=None, bias_delta=None):
        self.actual = {'left_arm': [0., .35, 0., 0., 0., 0.],
                       'right_arm': [0., -.38, 0., 0., 0., 0.]}
        self.target = {'left_arm': [.5, -.2, .1, 0., 0., .8],
                       'right_arm': [.6, .05, .3, 0., 0., -.4]}
        self.actions, self.published = [], []
        self.bias, self.sync_error = bias, sync_error
        self.bias_delta = bias_delta or [-bias, 0., 0., 0., 0., 0.]

    def get_observation(self):
        return {side: {'end_pose': pose.copy()} for side, pose in self.actual.items()}

    def step(self, action=None):
        self.actions.append(copy.deepcopy(action))
        if action is None:
            if self.sync_error:
                raise self.sync_error
            self.target = copy.deepcopy(self.actual)
        else:
            for side in self.target:
                motion = action[side]['motion']
                delta = motion['translation'] + motion['rotation_rotvec']
                self.target[side] = rpc._pose_from_delta(self.target[side], delta)
            self.published.append(copy.deepcopy(self.target))
            self.actual = copy.deepcopy(self.target)
            self.actual['right_arm'] = rpc._pose_from_delta(self.target['right_arm'], self.bias_delta)
        return {'observation': self.get_observation(), 'step_count': len(self.actions)}


class TargetReferenceTests(unittest.TestCase):
    def move(self, client, right, corrections=0):
        with redirect_stderr(io.StringIO()):
            return client.dual_robot_move_to_ee_pose(
                client.actual['left_arm'], right, delta=False, smooth=True,
                sleep=False, rate_hz=10, max_correction_iters=corrections,
                max_translation_step=.003, max_rotation_step=.025,
                position_tolerance_m=.001, rotation_tolerance_rad=.01)

    def test_rotation_only_cannot_republish_old_xyz_of_either_arm(self):
        client = TargetAccumulator()
        original = copy.deepcopy(client.actual)
        target = client.actual['right_arm'].copy()
        target[5] = .2
        result = self.move(client, target)
        self.assertTrue(result['ok'])
        self.assertIsNone(client.actions[0])
        for published in client.published:
            for side in original:
                self.assertEqual(published[side][:3], original[side][:3])
        self.assertAlmostEqual(client.target['right_arm'][5], .2)

    def test_tracking_bias_converges_without_reanchoring_away_compensation(self):
        client = TargetAccumulator(bias=.006)
        right = [.1, -.38, 0., 0., 0., 0.]
        result = self.move(client, right, corrections=3)
        self.assertTrue(result['ok'])
        self.assertEqual(sum(a is None for a in client.actions), 1)
        self.assertLessEqual(max(p['right_arm'][0] for p in client.published), .106000001)
        self.assertLessEqual(result['final_error']['right_translation_error_m'], .001)

    def test_incident_sized_position_and_rotation_bias_converges_with_original_tolerances(self):
        # Approximate the first measured residual from the 23:12 incident; this
        # is a biased plant simulation, not a claim about physical root cause.
        client = TargetAccumulator(bias_delta=[-.0114, -.0092, -.00137, .1354, -.0396, -.0586])
        right = [.574492, -.388898, -.127010, 2.221441469, 2.221441469, 0.]
        client.actual['right_arm'] = [.574494, -.388899, -.127009, -2.437896, -1.696670, -.316208]
        with redirect_stderr(io.StringIO()):
            result = client.dual_robot_move_to_ee_pose(
                client.actual['left_arm'], right, delta=False, smooth=True, sleep=False,
                rate_hz=80, max_translation_speed=.1, max_rotation_speed=.5,
                max_correction_iters=10, position_tolerance_m=.005, rotation_tolerance_rad=.03)
        self.assertTrue(result['ok'])
        self.assertLessEqual(result['final_error']['right_translation_error_m'], .005)
        self.assertLessEqual(result['final_error']['right_rotation_error_rad'], .03)
        self.assertEqual(sum(a is None for a in client.actions), 1)

    def test_unresponsive_robot_hits_compensation_limit_without_unbounded_target_growth(self):
        class Stuck(TargetAccumulator):
            def step(self, action=None):
                actual = copy.deepcopy(self.actual)
                super().step(action)
                self.actual = actual
                return {'observation': self.get_observation()}
        client = Stuck()
        right = [.01, -.38, 0., 0., 0., 0.]
        result = self.move(client, right, corrections=20)
        self.assertFalse(result['ok'])
        self.assertEqual(result['stalled']['reason'], 'tracking_compensation_limit')
        self.assertLessEqual(max(p['right_arm'][0] for p in client.published), .035000001)
        self.assertEqual(sum(a is None for a in client.actions), 1)

    def test_correction_step_and_total_rotation_limits(self):
        goal = [0.] * 6
        measured = [-.015, 0., 0., 0., 0., -.137]
        candidate, bias = rpc._bounded_tracking_target(goal, measured, goal, .005, .03)
        self.assertAlmostEqual(candidate[0], .003)
        self.assertAlmostEqual(candidate[5], .03)
        candidate, bias = rpc._bounded_tracking_target([0.,0.,0.,0.,0.,.195], measured, goal, .005, .03)
        self.assertAlmostEqual(candidate[0], .003)
        self.assertAlmostEqual(candidate[5], .20)
        self.assertLessEqual(bias['rotation_rad'], .20)
        self.assertEqual(bias['capped_components'], ['rotation'])

    def test_saturated_rotation_still_allows_translation_then_stops_when_no_room(self):
        goal = [0.] * 6
        measured = [-.015, 0., 0., 0., 0., -.137]
        candidate, bias = rpc._bounded_tracking_target([.024,0.,0.,0.,0.,.20], measured, goal, .005, .03)
        self.assertAlmostEqual(candidate[0], .025)
        self.assertAlmostEqual(candidate[5], .20)
        self.assertEqual(bias['capped_components'], ['translation', 'rotation'])
        candidate, bias = rpc._bounded_tracking_target(candidate, measured, goal, .005, .03)
        self.assertIsNone(candidate)
        # An inactive arm must not stop correction of the other arm.
        candidate, _ = rpc._bounded_tracking_target(goal, goal, goal, .005, .03)
        self.assertEqual(candidate, goal)

    def test_non_collinear_rotation_and_translation_clipping_stay_within_budgets(self):
        goal = [.3, -.1, -.05, 2.221441469, 2.221441469, 0.]
        command = rpc._pose_from_delta(goal, [.024, 0., 0., .198, 0., 0.])
        measured = rpc._pose_from_delta(goal, [-.015, -.01, 0., -.1, -.1, 0.])
        candidate, bias = rpc._bounded_tracking_target(command, measured, goal, .005, .03)
        self.assertIsNotNone(candidate)
        self.assertLessEqual(bias['translation_m'], .025)
        self.assertLessEqual(bias['rotation_rad'], .20)
        step = rpc._motion_norms(rpc._absolute_target_to_delta(command, candidate))
        self.assertLessEqual(step[0], .003 + 1e-9)
        self.assertLessEqual(step[1], .03 + 1e-9)

    def test_already_out_of_budget_command_is_rejected(self):
        goal = [0.] * 6
        candidate, _ = rpc._bounded_tracking_target([.026,0.,0.,0.,0.,0.], goal, goal, .005, .03)
        self.assertIsNone(candidate)

    def test_234428_observation_can_take_partial_seventh_correction(self):
        goal = [.3334726484965278, -.12511820133371374, -.04905326067595178,
                2.221441469079183, 2.221441469079183, 0.]
        command = [.3291517316661404, -.10923414746403709, -.044142490799232284,
                   2.173057245814768, 2.059696856019013, -.13660626289099248]
        residual = [.002281466359992157, .008964241194854644, -.0005020878987996164,
                    -.04783450973123488, .0015716191268847263, -.012563287179956141]
        measured = rpc._pose_from_delta(goal, [-v for v in residual])
        candidate, bias = rpc._bounded_tracking_target(command, measured, goal, .005, .03)
        self.assertIsNotNone(candidate)
        self.assertAlmostEqual(bias['translation_m'], .01970612623072805)
        self.assertAlmostEqual(bias['rotation_rad'], .20)
        self.assertLessEqual(bias['rotation_rad'], .20)
        self.assertEqual(bias['capped_components'], ['rotation'])

    def test_lost_sync_reply_aborts_without_motion_or_retry(self):
        error = RuntimeError('lost synchronization reply')
        client = TargetAccumulator(sync_error=error)
        with self.assertRaises(RuntimeError) as caught:
            self.move(client, [.1, -.38, 0., 0., 0., 0.])
        self.assertIs(caught.exception, error)
        self.assertEqual(client.actions, [None])
        self.assertEqual(client.published, [])

    def test_missing_sync_observation_aborts_before_stream(self):
        class MissingSnapshot(TargetAccumulator):
            def step(self, action=None):
                self.actions.append(action)
                return {'ok': True}
        client = MissingSnapshot()
        with self.assertRaisesRegex(RuntimeError, 'no observation'):
            self.move(client, [.1, -.38, 0., 0., 0., 0.])
        self.assertEqual(client.actions, [None])


if __name__ == '__main__':
    unittest.main()
