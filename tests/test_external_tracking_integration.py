"""Skill boundaries for the external loop; raw policy RPC remains a passthrough."""
import io
import importlib.util
import os
from pathlib import Path
from contextlib import redirect_stdout
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from test_external_tracking import adapter
import test_grasp_continuous_correction as grasp_fixture
from test_continuous_placement import place

grasp = grasp_fixture.grasp


class BoundPoseClient(grasp_fixture.AtomicCallbackClient):
    def get_observation(self):
        observation = super().get_observation()
        for side, pose in (('left_arm', self.left), ('right_arm', self.right)):
            native = observation[side]['native_robot_state']
            native['sequence'] = 1
            native['torso_eef_pose'] = {'schema':'aoe.franka-source-bound-pose.v1',
                'frame':'torso', 'source_sequence':1, 'position':pose[:3],
                'orientation_xyzw':adapter.tracking.g._quat_from_rotvec(pose[3:])}
        return observation


class ExternalTrackingIntegrationTests(unittest.TestCase):
    def setUp(self):
        active = patch.dict(os.environ, FRANKA_P2P_CONTROLLER='external_tracking')
        active.start()
        self.addCleanup(active.stop)
        os.environ.pop('FRANKA_P2P_ARRIVAL_POLICY', None)

    def test_raw_policy_step_does_not_load_or_run_the_outer_loop(self):
        cls = adapter.base.legacy.DualFrankaRobotiqRpcClient
        client = cls.__new__(cls)
        action = {'left_arm': {'motion': {'translation': [.001,0.,0.]}}}
        client._call = Mock(return_value={'step_count': 17})
        with patch.object(adapter.base.legacy, '_continuous_p2p_module', side_effect=AssertionError('policy must bypass outer loop')):
            result = client.step(action)
        self.assertEqual(result, {'step_count':17})
        client._call.assert_called_once_with('step', action)

    def test_grasp_never_reanchors_new_mode(self):
        client = Mock()
        grasp.reanchor(client, True)
        client.step.assert_not_called()

    def test_grasp_accepts_new_mode_result_without_legacy_retries(self):
        fixture = grasp_fixture.GraspContinuousCorrectionTests()
        fixture.setUp()
        client = BoundPoseClient([grasp_fixture.ARRIVED], returned={'ok':True, 'controller':'external_tracking'})
        left, right, result = fixture.move(client)
        self.assertEqual(len(client.calls), 1)
        self.assertEqual(result['controller'], 'external_tracking')

    def test_grasp_rejects_result_from_wrong_controller(self):
        fixture = grasp_fixture.GraspContinuousCorrectionTests()
        fixture.setUp()
        client = BoundPoseClient([grasp_fixture.ARRIVED], returned={'ok':True, 'controller':'continuous_servo'})
        with self.assertRaisesRegex(RuntimeError, 'no retry or next stage'):
            fixture.move(client)
        self.assertEqual(len(client.calls), 1)

    def test_placement_failure_never_opens_gripper_or_returns_home(self):
        client = Mock()
        args = SimpleNamespace(execute=True, release_on_motion_failure=True, go_home_after_motion_failure=True)
        with patch.object(place, '_move', side_effect=RuntimeError('synthetic uncertainty')):
            with self.assertRaisesRegex(RuntimeError, 'synthetic uncertainty'):
                place._move_with_failure_release(client, side='left', active_target=None,
                    current_left=None, current_right=None, args=args, stage='place', report={})
        client.open_gripper.assert_not_called()
        client.go_home.assert_not_called()

    def test_unsafe_placement_flags_fail_before_client_creation(self):
        with patch.object(place, '_load_rpc_module') as load, redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError, 'cannot bypass'):
                place.main(['--side','left','--place-xyz','[0.5,0.2,0.1]',
                            '--execute','--release-on-motion-failure'])
        load.assert_not_called()

    def test_handover_alignment_does_not_reset_or_operate_grippers(self):
        path = Path(__file__).resolve().parents[1] / 'procedure_skills/dual_franka_handover_transition/skills/procedure-franka-handover-transition/scripts/handover_transition.py'
        spec = importlib.util.spec_from_file_location('external_handover_test', path)
        handover = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(handover)
        client = Mock()
        client.dual_robot_move_to_ee_pose.return_value = {'ok':True, 'controller':'external_tracking'}
        rpc = SimpleNamespace(DualFrankaRobotiqRpcClient=Mock(return_value=client))
        with patch.object(handover, '_load_rpc_client', return_value=rpc), \
             patch.object(handover, 'load_transition_targets', return_value=([0.]*6,[0.]*6,'fake')), \
             redirect_stdout(io.StringIO()):
            self.assertEqual(handover.main(['--active-arm','right','--execute','--no-transfer-release']), 0)
        client.step.assert_not_called()
        client.open_gripper.assert_not_called()
        client.close_gripper.assert_not_called()
        client.dual_robot_move_to_ee_pose.assert_called_once()


if __name__ == '__main__':
    unittest.main()
