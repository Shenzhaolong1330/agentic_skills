"""Offline selection/native-feedback/grasp integration; no transport is created."""
import copy
from contextlib import redirect_stderr, redirect_stdout
import importlib.util
import io
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from test_grasp_stall_warning import FakeMotionClient, grasp, pose


CLIENT = Path(__file__).resolve().parents[1] / 'atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/dual_franka_robotiq_rpc_client.py'
spec = importlib.util.spec_from_file_location('p2p_controller_dispatch_rpc', CLIENT)
rpc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rpc)
ENVIRONMENT_KEY = 'FRANKA_P2P_CONTROLLER'


def native_observation():
    return {'left_arm': {'end_pose': [.9, .8, .7, 0., 0., 0.], 'native_robot_state': {
        'sequence': 7,
        'torso_eef_pose': {'schema': 'aoe.franka-source-bound-pose.v1', 'frame': 'torso',
                           'source_sequence': 7, 'position': [.1, .2, .3],
                           'orientation_xyzw': list(rpc._rotvec_to_quat_xyzw([0., 0., .2]))},
    }}}


class OldPoseOnlyClient(rpc.DualFrankaRobotiqRpcClient):
    def __init__(self):
        self.actual = {'left_arm': [0.] * 6, 'right_arm': [0.] * 6}
        self.actions = []

    def get_observation(self):
        return {side: {'end_pose': values.copy()} for side, values in self.actual.items()}

    def step(self, action=None):
        self.actions.append(copy.deepcopy(action))
        if action is not None:
            for side in self.actual:
                motion = action[side]['motion']
                self.actual[side] = rpc._pose_from_delta(
                    self.actual[side], motion['translation'] + motion['rotation_rotvec'])
        return {'observation': self.get_observation(), 'step_count': len(self.actions)}


class SourceBoundGraspClient(FakeMotionClient):
    def get_observation(self):
        observation = super().get_observation()
        for side in ('left_arm', 'right_arm'):
            values = observation[side]['end_pose']
            observation[side]['native_robot_state'].update({
                'sequence': self.observations,
                'torso_eef_pose': {'schema': 'aoe.franka-source-bound-pose.v1', 'frame': 'torso',
                                   'source_sequence': self.observations, 'position': values[:3],
                                   'orientation_xyzw': list(rpc._rotvec_to_quat_xyzw(values[3:]))},
            })
        return observation


class IsolatedEnvironmentTests(unittest.TestCase):
    def setUp(self):
        environment = patch.dict(os.environ)
        environment.start()
        self.addCleanup(environment.stop)
        os.environ.pop(ENVIRONMENT_KEY, None)


class ControllerDispatchTests(IsolatedEnvironmentTests):
    def client(self):
        client = object.__new__(rpc.DualFrankaRobotiqRpcClient)
        client._smooth_dual_robot_move_to_ee_pose = Mock(return_value={'legacy': True})
        client._call = Mock(return_value={'legacy_rpc': True})
        return client

    def test_default_segmented_never_loads_continuous_adapter(self):
        self.assertEqual(rpc.p2p_controller_mode(), 'segmented')
        client = self.client()
        with patch.object(rpc, '_continuous_p2p_module', side_effect=AssertionError('new adapter imported')):
            result = client.dual_robot_move_to_ee_pose([0.] * 6, [.01, 0., 0., 0., 0., 0.], delta=False)
        self.assertEqual(result, {'legacy': True})
        client._smooth_dual_robot_move_to_ee_pose.assert_called_once()
        client._call.assert_not_called()

    def test_explicit_selection_has_precedence_over_environment(self):
        for environment in ('segmented', 'continuous_servo', 'invalid'):
            os.environ[ENVIRONMENT_KEY] = environment
            self.assertEqual(rpc.p2p_controller_mode('segmented'), 'segmented')
            self.assertEqual(rpc.p2p_controller_mode('continuous_servo'), 'continuous_servo')
        for invalid in ('', 'typo', True):
            with self.assertRaises(ValueError):
                rpc.p2p_controller_mode(invalid)

    def test_continuous_dispatch_forwards_existing_motion_contract_to_neighbor_adapter(self):
        for explicit in (False, True):
            with self.subTest(explicit=explicit):
                os.environ[ENVIRONMENT_KEY] = 'segmented' if explicit else 'continuous_servo'
                client = self.client()
                adapter = SimpleNamespace(run=Mock(return_value={'controller': 'continuous_servo', 'ok': True}))
                guard, correction = Mock(), Mock()
                with patch.object(rpc, '_continuous_p2p_module', return_value=adapter):
                    result = client.dual_robot_move_to_ee_pose(
                        [0.] * 6, [.01, 0., 0., 0., 0., 0.], delta=False, wait=True,
                        smooth=True, rate_hz=20, max_translation_speed=.002, max_rotation_speed=.01,
                        position_tolerance_m=.003, rotation_tolerance_rad=.1,
                        max_tracking_translation_bias_m=.025, max_tracking_rotation_bias_rad=.2,
                        feedback_guard=guard, correction_guard=correction,
                        controller='continuous_servo' if explicit else None)
                self.assertEqual(result, {'controller': 'continuous_servo', 'ok': True})
                args, options = adapter.run.call_args
                self.assertIs(args[0], client)
                self.assertEqual(args[1:], ([0.] * 6, [.01, 0., 0., 0., 0., 0.]))
                for key, value in {'delta': False, 'wait': True, 'rate_hz': 20,
                                   'max_translation_speed': .002, 'max_rotation_speed': .01,
                                   'position_tolerance_m': .003, 'rotation_tolerance_rad': .1,
                                   'max_tracking_translation_bias_m': .025,
                                   'max_tracking_rotation_bias_rad': .2}.items():
                    self.assertEqual(options[key], value)
                self.assertIs(options['feedback_guard'], guard)
                self.assertIs(options['correction_guard'], correction)
                client._smooth_dual_robot_move_to_ee_pose.assert_not_called()
                client._call.assert_not_called()

    def test_continuous_smooth_false_and_invalid_environment_reject_before_loading_or_rpc(self):
        for mode, options in (('continuous_servo', {'smooth': False}), ('typo', {}), ('', {})):
            with self.subTest(mode=mode):
                os.environ[ENVIRONMENT_KEY] = mode
                client = self.client()
                with patch.object(rpc, '_continuous_p2p_module') as loader, self.assertRaises(ValueError):
                    client.dual_robot_move_to_ee_pose([0.] * 6, [0.] * 6, delta=False, **options)
                loader.assert_not_called()
                client._call.assert_not_called()
                client._smooth_dual_robot_move_to_ee_pose.assert_not_called()

    def test_explicit_segmented_preserves_pose_only_client_under_continuous_environment(self):
        for environment in ('continuous_servo', 'typo'):
            with self.subTest(environment=environment):
                os.environ[ENVIRONMENT_KEY] = environment
                client = OldPoseOnlyClient()
                with patch.object(rpc, '_continuous_p2p_module', side_effect=AssertionError('adapter imported')):
                    with redirect_stderr(io.StringIO()):
                        result = client.dual_robot_move_to_ee_pose(
                            [0.] * 6, [.002, 0., 0., 0., 0., 0.], delta=False,
                            smooth=True, sleep=False, controller='segmented', max_correction_iters=0)
                self.assertTrue(result['ok'])
                self.assertEqual(sum(action is None for action in client.actions), 1)

    def test_adapter_loader_uses_neighbor_file_and_caches_that_exact_module(self):
        name = '_franka_skill_continuous_p2p'
        neighbor = CLIENT.with_name('continuous_p2p.py')
        fake_module = SimpleNamespace(__file__=str(neighbor))
        fake_spec = SimpleNamespace(loader=SimpleNamespace(exec_module=Mock()))
        with patch.dict(sys.modules):
            sys.modules.pop(name, None)
            with patch.object(rpc.importlib.util, 'spec_from_file_location', return_value=fake_spec) as spec_loader:
                with patch.object(rpc.importlib.util, 'module_from_spec', return_value=fake_module):
                    first = rpc._continuous_p2p_module()
                    self.assertIs(first, fake_module)
                    self.assertIs(rpc._continuous_p2p_module(), first)
            spec_loader.assert_called_once_with(name, neighbor)
            fake_spec.loader.exec_module.assert_called_once_with(fake_module)


class NativePoseSelectionTests(IsolatedEnvironmentTests):
    def test_continuous_chooses_source_bound_native_pose_over_conflicting_legacy_pose(self):
        os.environ[ENVIRONMENT_KEY] = 'continuous_servo'
        result = rpc._pose_from_side_observation(native_observation(), 'left_arm')
        np.testing.assert_allclose(result, [.1, .2, .3, 0., 0., .2], atol=1e-12)

    def test_segmented_pose_selection_remains_legacy_compatible(self):
        observation = native_observation()
        observation['left_arm']['native_robot_state']['torso_eef_pose']['frame'] = 'invalid'
        self.assertEqual(rpc._pose_from_side_observation(observation, 'left_arm'), [.9, .8, .7, 0., 0., 0.])

    def test_native_schema_frame_and_both_sequence_types_must_be_valid(self):
        os.environ[ENVIRONMENT_KEY] = 'continuous_servo'
        mutations = [('native', 'sequence', 0), ('native', 'sequence', True),
                     ('native', 'sequence', 7.), ('pose', 'source_sequence', 8),
                     ('pose', 'source_sequence', 7.), ('pose', 'frame', 'left_base'),
                     ('pose', 'schema', 'legacy')]
        for location, key, value in mutations:
            with self.subTest(location=location, key=key, value=value):
                observation = native_observation()
                native = observation['left_arm']['native_robot_state']
                (native if location == 'native' else native['torso_eef_pose'])[key] = value
                with self.assertRaises((RuntimeError, ValueError)):
                    rpc._pose_from_side_observation(observation, 'left_arm')
        observation = native_observation()
        observation['left_arm']['native_robot_state']['sequence'] = 1
        observation['left_arm']['native_robot_state']['torso_eef_pose']['source_sequence'] = True
        with self.assertRaises((RuntimeError, ValueError)):
            rpc._pose_from_side_observation(observation, 'left_arm')

    def test_invalid_native_quaternion_or_missing_bound_pose_never_falls_back(self):
        os.environ[ENVIRONMENT_KEY] = 'continuous_servo'
        for quaternion in ([0., 0., 0., 0.], [0., 0., 0., .9], [0., 0., 0.],
                           [0., 0., float('nan'), 1.]):
            with self.subTest(quaternion=quaternion):
                observation = native_observation()
                observation['left_arm']['native_robot_state']['torso_eef_pose']['orientation_xyzw'] = quaternion
                with self.assertRaises((RuntimeError, ValueError)):
                    rpc._pose_from_side_observation(observation, 'left_arm')
        for missing in ('native_robot_state', 'torso_eef_pose'):
            observation = native_observation()
            parent = observation['left_arm'] if missing == 'native_robot_state' else observation['left_arm']['native_robot_state']
            parent.pop(missing)
            with self.assertRaises((RuntimeError, ValueError)):
                rpc._pose_from_side_observation(observation, 'left_arm')


class GraspControllerDispatchTests(IsolatedEnvironmentTests):
    def move(self, client, *extra):
        args = grasp.build_parser().parse_args([
            '--xyz', '[0.1,0,0]', '--arm', 'left', '--p2p-retries', '9',
            '--position-tolerance-m', '.003', '--rotation-tolerance-rad', '.1',
            '--settle-time-sec', '0', *extra])
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            return grasp.move_stage(client, 'controller_dispatch', np.asarray(pose(.1)), np.asarray(pose()),
                                    args, recover_stalled_side='left_arm')

    def test_continuous_reanchor_never_synchronizes_saved_commands(self):
        os.environ[ENVIRONMENT_KEY] = 'continuous_servo'
        client = SimpleNamespace(step=Mock())
        with patch.object(grasp.time, 'sleep') as sleep:
            grasp.reanchor(client, True)
        client.step.assert_not_called()
        sleep.assert_not_called()

    def test_segmented_reanchor_remains_unchanged(self):
        client = SimpleNamespace(step=Mock())
        with patch.object(grasp.time, 'sleep') as sleep:
            grasp.reanchor(client, True)
        client.step.assert_called_once_with(None)
        sleep.assert_called_once_with(.2)

    def test_invalid_environment_rejects_before_reanchor_motion(self):
        os.environ[ENVIRONMENT_KEY] = 'typo'
        client = SimpleNamespace(step=Mock())
        with patch.object(grasp.time, 'sleep'), self.assertRaises(ValueError):
            grasp.reanchor(client, True)
        client.step.assert_not_called()

    def test_continuous_arrival_requires_success_from_the_selected_controller(self):
        os.environ[ENVIRONMENT_KEY] = 'continuous_servo'
        invalid = ({'ok': False, 'controller': 'continuous_servo'}, {'ok': True},
                   {'ok': True, 'controller': 'segmented'})
        for report in invalid:
            with self.subTest(report=report):
                client = SourceBoundGraspClient([(.1, report), (.1, {'ok': True, 'controller': 'continuous_servo'})])
                with self.assertRaisesRegex(RuntimeError, 'continuous servo did not verify stable arrival'):
                    self.move(client)
                self.assertEqual(len(client.calls), 1)
        client = SourceBoundGraspClient([(.1, {'ok': True, 'controller': 'continuous_servo'})])
        self.move(client)
        self.assertEqual(len(client.calls), 1)

    def test_continuous_miss_or_uncertain_rpc_never_retries_the_whole_target(self):
        os.environ[ENVIRONMENT_KEY] = 'continuous_servo'
        for first in ((.05, {'ok': True, 'controller': 'continuous_servo'}), TimeoutError('lost reply')):
            with self.subTest(first=first):
                client = SourceBoundGraspClient([first, (.1, {'ok': True, 'controller': 'continuous_servo'})])
                with self.assertRaises((RuntimeError, TimeoutError)):
                    self.move(client)
                self.assertEqual(len(client.calls), 1)

    def test_continuous_cannot_enable_out_of_tolerance_progression(self):
        os.environ[ENVIRONMENT_KEY] = 'continuous_servo'
        client = SourceBoundGraspClient([(.1, {'ok': True, 'controller': 'continuous_servo'})])
        with self.assertRaisesRegex(ValueError, 'cannot allow out-of-tolerance progression'):
            self.move(client, '--continue-on-tolerance-miss')
        self.assertEqual(client.calls, [])

    def test_segmented_arrival_acceptance_and_outer_retry_remain_compatible(self):
        client = SourceBoundGraspClient([(.05, {'ok': False}), (.1, {'ok': False})])
        _, _, result = self.move(client)
        self.assertFalse(result['ok'])
        self.assertEqual(len(client.calls), 2)


if __name__ == '__main__':
    unittest.main()
