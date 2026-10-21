"""Downward grasp geometry and stage ordering; all feedback is synthetic."""
from contextlib import redirect_stderr, redirect_stdout
import io
import json
import math
import os
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation as R

from test_grasp_p2p_retries import grasp, FakeMotionClient


POLICY = {'position_tolerance_m': .005, 'rotation_tolerance_rad': .2, 'warning_multiplier': 2}
RECORDED = np.array([.439556, -.251545, -.119902, -2.222801, -1.682882, -.224627])


class DownwardGraspTests(unittest.TestCase):
    def setUp(self):
        env = patch.dict(os.environ, {grasp.franka_rpc.ARRIVAL_POLICY_ENV: json.dumps(POLICY)})
        env.start()
        self.addCleanup(env.stop)

    def assert_down(self, rotvec):
        np.testing.assert_allclose(R.from_rotvec(rotvec).apply([0, 0, 1]), [0, 0, -1], atol=1e-12)

    def test_contact_gate_rejects_empty_stale_fault_and_inconsistent_native_feedback(self):
        def samples(contact=True):
            values = []
            for i in range(3):
                driver = {'schema':'aoe.robotiq-register-status.v1', 'sample_valid':True,
                    'sample_fresh':True, 'ready':True, 'activated':True, 'go_to_requested':True,
                    'object_detection_code':2 if contact else 3, 'sample_sequence':i+1,
                    'source_ros_timestamp_ns':int((100 + .2*i)*1e9), 'source_session_id':'test',
                    'contact_direction':'closing' if contact else None,
                    'requested_position_reached':not contact, 'object_detected':contact,
                    'registers':{'status_byte':185 if contact else 249, 'fault_byte':0,
                                 'requested_position_raw':230, 'position_raw':180 if contact else 230}}
                values.append({'left_arm':{'gripper':{'velocity':0., 'object_detected':contact, 'driver_status':driver}}})
            return values
        def check(values):
            client = unittest.mock.Mock()
            client.get_observation.side_effect = values
            with patch.object(grasp.time,'time',return_value=100.5), patch.object(grasp.time,'sleep'), redirect_stdout(io.StringIO()):
                return grasp.native_contact_before_lift(client, 'left_arm')
        self.assertTrue(check(samples()))
        self.assertFalse(check(samples(False)))
        for key, value in [('sample_fresh',False), ('sample_sequence',1), ('source_session_id','changed'),
                           ('requested_position_reached',True), ('source_ros_timestamp_ns',1)]:
            values = samples();values[-1]['left_arm']['gripper']['driver_status'][key] = value
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                check(values)
        values = samples();values[-1]['left_arm']['gripper']['driver_status']['registers']['fault_byte'] = 1
        with self.assertRaises(RuntimeError):
            check(values)

    def test_recorded_tilt_is_removed_without_changing_yaw(self):
        down = grasp.gripper_down_rotvec(RECORDED, 0)
        self.assert_down(down)
        self.assertAlmostEqual(R.from_rotvec(down).as_euler('xyz')[2],
                               R.from_rotvec(RECORDED[3:]).as_euler('xyz')[2])

    def test_recorded_half_turn_selects_centered_wrist_winding(self):
        current = [1.95618, -2.418704, -.010157]
        target = [2.410999, 2.014122, 0.]
        middle, route = grasp.wrist_yaw_waypoint(current, target, .3791298568)
        self.assertIsNotNone(middle)
        self.assertGreater(route['selected_yaw_delta_rad'], math.pi)
        self.assertLess(abs(route['projected_selected_joint7_rad']), abs(route['projected_shortest_joint7_rad']))
        self.assertFalse(route['full_arm_ik_verified'])
        self.assert_down(middle)
        for a, b in ((current, middle), (middle, target)):
            self.assertLess((R.from_rotvec(b) * R.from_rotvec(a).inv()).magnitude(), 1.8)
        # The opposite native wrist state already favors the shortest winding.
        self.assertIsNone(grasp.wrist_yaw_waypoint(current, target, -.4)[0])

    def test_wrist_route_does_not_add_winding_to_short_moves_or_tilted_tools(self):
        down = R.from_euler('xyz', [math.pi, 0, 0]).as_rotvec()
        short = R.from_euler('xyz', [math.pi, 0, .6]).as_rotvec()
        tilted = R.from_euler('xyz', [math.pi, .3, -3.]).as_rotvec()
        self.assertEqual(grasp.wrist_yaw_waypoint(down, short, 2.), (None, None))
        self.assertEqual(grasp.wrist_yaw_waypoint(down, tilted, 2.), (None, None))
        sideways = R.from_euler('xyz', [math.pi, .5, 0.]).as_rotvec()
        half_turn = R.from_euler('xyz', [math.pi, 0., -3.]).as_rotvec()
        self.assertEqual(grasp.wrist_yaw_waypoint(sideways, half_turn, 2.), (None, None))
        with self.assertRaises(ValueError):
            grasp.wrist_yaw_waypoint(down, short, math.nan)

    def test_recorded_right_home_tilt_still_selects_alternate_winding(self):
        current = [-2.406442, -1.744553, -.337942]
        target = [1.376076, -2.824185, 0.]
        middle, route = grasp.wrist_yaw_waypoint(current, target, -.49)
        self.assertIsNotNone(middle)
        self.assertGreater(route['start_downward_tilt_rad'], .15)
        self.assertLess(route['start_downward_tilt_rad'], .4)
        self.assertLess(route['selected_yaw_delta_rad'], -math.pi)
        self.assertLess(abs(route['projected_selected_joint7_rad']),
                        abs(route['projected_shortest_joint7_rad']))
        self.assertFalse(route['full_arm_ik_verified'])
        self.assert_down(middle)
        for a, b in ((current, middle), (middle, target)):
            self.assertLess((R.from_rotvec(b) * R.from_rotvec(a).inv()).magnitude(), 1.8)

    def test_contact_precision_scope_restores_transit_policy_even_on_failure(self):
        args = grasp.build_parser().parse_args(['--xyz','[0.4,0,-0.2]',
                                               '--approach-position-tolerance-m','.002'])
        before = os.environ[grasp.franka_rpc.ARRIVAL_POLICY_ENV]
        with redirect_stdout(io.StringIO()), self.assertRaisesRegex(RuntimeError, 'synthetic'):
            with grasp.descent_precision(args):
                policy = grasp.franka_rpc.arrival_policy(.005, .2)
                self.assertEqual(policy['precision_position_tolerance_m'], .002)
                self.assertEqual(policy['position_tolerance_m'], .005)
                self.assertEqual(policy['rotation_tolerance_rad'], .2)
                self.assertEqual(policy['warning_multiplier'], 2.)
                raise RuntimeError('synthetic')
        self.assertEqual(os.environ[grasp.franka_rpc.ARRIVAL_POLICY_ENV], before)
        args.approach_position_tolerance_m = None
        with grasp.descent_precision(args):
            self.assertEqual(os.environ[grasp.franka_rpc.ARRIVAL_POLICY_ENV], before)

    def test_object_alignment_is_vertical_for_both_axes_and_direction_policies(self):
        vector = np.array([.3412485766, .9315979405, .125198587])
        for roll in (-2.956, 2.8, .7):
            for axis in ('x', 'y'):
                for directional in (False, True):
                    with self.subTest(roll=roll, axis=axis, directional=directional):
                        pose = np.r_[RECORDED[:3], R.from_euler('xyz', [roll, -.33, 1.26]).as_rotvec()]
                        rotvec, alignment = grasp.gripper_parallel_rotvec(pose, vector, 0, axis, directional)
                        self.assert_down(rotvec)
                        projected = R.from_rotvec(rotvec).as_matrix()[:2, 0 if axis == 'x' else 1]
                        score = float(np.dot(projected / np.linalg.norm(projected), vector[:2] / np.linalg.norm(vector[:2])))
                        self.assertAlmostEqual(score if directional else abs(score), 1.)
                        self.assertAlmostEqual(alignment['parallel_score'], 1.)

    def test_undirected_yaw_uses_nearest_parallel_branch(self):
        current = np.r_[RECORDED[:3], R.from_euler('xyz', [2.9, -.2, math.pi - .01]).as_rotvec()]
        rotvec, result = grasp.gripper_parallel_rotvec(current, np.array([1., .01, 0]), 0, 'x', False)
        self.assert_down(rotvec)
        self.assertLess(abs(result['chosen_yaw_rad'] - result['current_yaw_rad']), .03)

    def test_explicit_standalone_pitch_override_and_invalid_vector(self):
        rotvec = grasp.gripper_down_rotvec(RECORDED, .15)
        tilt = math.acos(-R.from_rotvec(rotvec).as_matrix()[2, 2])
        self.assertAlmostEqual(tilt, .15)
        with self.assertRaisesRegex(ValueError, 'no usable xy direction'):
            grasp.gripper_parallel_rotvec(RECORDED, np.array([0, 0, 1]), 0, 'x', False)

    def test_fresh_orientation_check_preserves_two_band_policy_and_reports_tilt(self):
        target = R.from_euler('xyz', [math.pi, 0, 1.2]).as_rotvec()
        args = grasp.build_parser().parse_args(['--xyz', '[0.4,0,-0.2]'])
        for delta, status in ((.19, 'within_tolerance'), (.3, 'warning'), (.399, 'warning')):
            measured = np.r_[RECORDED[:3], (R.from_rotvec([delta, 0, 0]) * R.from_rotvec(target)).as_rotvec()]
            with redirect_stdout(io.StringIO()):
                result = grasp.check_grasp_orientation(measured, target, args, 'before_descent')
            self.assertEqual(result['status'], status)
            self.assertAlmostEqual(result['target_tilt_from_down_rad'], 0)
            self.assertAlmostEqual(result['measured_tilt_from_down_rad'], delta)
            self.assertEqual(result['stop_rotation_error_rad'], .4)
        for measured in (np.r_[RECORDED[:3], (R.from_rotvec([.401, 0, 0]) * R.from_rotvec(target)).as_rotvec()],
                         np.r_[RECORDED[:3], [np.nan, 0, 0]]):
            with redirect_stdout(io.StringIO()), self.assertRaises((RuntimeError, ValueError)):
                grasp.check_grasp_orientation(measured, target, args, 'before_descent')

    def run_main(self, *, side='right', drift_stage=None, keep=False, fast_transit=False, orient_first=False, contact=None, wrist_waypoint=False):
        client = FakeMotionClient([])
        client.left = RECORDED.tolist()
        client.right = RECORDED.tolist()
        calls = []
        def move(_client, stage, left, right, args, **kwargs):
            calls.append((stage, np.asarray(left).copy(), np.asarray(right).copy(), kwargs))
            client.left, client.right = list(left), list(right)
            if drift_stage and stage.startswith(drift_stage):
                pose = client.left if side == 'left' else client.right
                pose[3:] = (R.from_rotvec([.401, 0, 0]) * R.from_rotvec(pose[3:])).as_rotvec().tolist()
            return np.asarray(client.left), np.asarray(client.right), {'ok': True}
        argv = ['--xyz', '[0.436739,-0.242876,-0.214893]', '--arm', side, '--execute',
                '--orientation-vector', '[0.341248,0.931598,0.125199]', '--no-return-transition',
                '--after-close-sleep-sec', '0']
        if keep:
            argv.append('--keep-current-rotation')
        if orient_first:
            argv.append('--orient-before-xy')
        if wrist_waypoint:
            argv += ['--orient-before-xy', '--prefer-wrist-yaw-margin']
        if fast_transit:
            argv += ['--pregrasp-xy-max-translation-speed', '.1',
                     '--pregrasp-xy-max-translation-step', '.005', '--lift-after-grasp-m', '.04']
        if contact is not None:
            argv += ['--require-contact-before-lift', '--lift-after-grasp-m', '.04']
        with patch.object(grasp, 'DualFrankaRobotiqRpcClient', return_value=client), \
             patch.object(grasp, 'reanchor'), patch.object(grasp, 'move_stage', side_effect=move), \
             patch.object(grasp, 'close_gripper_and_confirm', return_value=({}, {'ok': True})) as close, \
             patch.object(grasp, 'native_contact_before_lift', return_value=contact), \
             patch.object(grasp, 'measured_wrist_yaw_waypoint', return_value=(R.from_euler('xyz',[math.pi,0,0]).as_rotvec(), {'full_arm_ik_verified':False})), \
             redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            if drift_stage:
                with self.assertRaisesRegex(RuntimeError, 'grasp orientation error'):
                    grasp.main(argv)
                close.assert_not_called()
            else:
                self.assertEqual(grasp.main(argv), 0)
                close.assert_called_once()
        self.assertTrue(client.closed)
        return calls

    def test_wrist_waypoint_precedes_final_orientation_and_xy_without_changing_goal(self):
        normal = self.run_main(orient_first=True)
        routed = self.run_main(wrist_waypoint=True)
        self.assertIn('wrist_yaw_waypoint', routed[0][0])
        self.assertIn('orient_before_xy', routed[1][0])
        self.assertIn('move_xy_keep_z', routed[2][0])
        np.testing.assert_allclose(routed[1][2], normal[0][2])

    def test_confirmed_empty_close_never_dispatches_the_requested_lift(self):
        self.assertFalse(any(c[0] == 'post_grasp_lift' for c in self.run_main(contact=False)))
        self.assertTrue(any(c[0] == 'post_grasp_lift' for c in self.run_main(contact=True)))

    def test_both_arms_align_down_before_descent_and_keep_the_target_during_descent(self):
        for side, index in (('left', 1), ('right', 2)):
            with self.subTest(side=side):
                calls = self.run_main(side=side)
                self.assertEqual(len(calls), 3)
                self.assertIn('stage2_align_gripper', calls[1][0])
                self.assert_down(calls[1][index][3:])
                np.testing.assert_allclose(calls[1][index][3:], calls[2][index][3:])
                np.testing.assert_allclose(calls[1][index][:2], calls[2][index][:2])
                self.assertLess(calls[2][index][2], calls[1][index][2])

    def test_optional_empty_orientation_precedes_horizontal_transit_without_changing_xyz(self):
        for side, index in (('left', 1), ('right', 2)):
            calls = self.run_main(side=side, orient_first=True)
            self.assertEqual(len(calls), 4)
            self.assertIn('stage0', calls[0][0])
            np.testing.assert_allclose(calls[0][index][:3], RECORDED[:3])
            self.assert_down(calls[0][index][3:])
            self.assertIn('stage1', calls[1][0])
            self.assert_down(calls[1][index][3:])
        calls = self.run_main(orient_first=True, drift_stage='stage0')
        self.assertEqual(len(calls), 1)

    def test_post_alignment_drift_blocks_descent_and_post_descent_drift_blocks_close(self):
        for stage, expected_calls in (('stage2', 2), ('stage3', 3)):
            with self.subTest(stage=stage):
                calls = self.run_main(drift_stage=stage)
                self.assertEqual(len(calls), expected_calls)

    def test_explicit_extraction_preserves_rotation_and_skips_alignment(self):
        calls = self.run_main(keep=True)
        self.assertEqual(len(calls), 2)
        np.testing.assert_allclose(calls[-1][2][3:], RECORDED[3:])

    def test_fast_transit_limits_do_not_leak_into_descent_or_carried_lift(self):
        calls = self.run_main(fast_transit=True)
        self.assertEqual(calls[0][3]['max_translation_speed'], .1)
        self.assertEqual(calls[0][3]['max_translation_step'], .005)
        self.assertTrue(any(c[0] == 'post_grasp_lift' for c in calls))
        for stage, left, right, options in calls[1:]:
            self.assertNotEqual(options.get('max_translation_speed'), .1, stage)
            self.assertNotEqual(options.get('max_translation_step'), .005, stage)


if __name__ == '__main__':
    unittest.main()
