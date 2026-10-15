"""Offline transport/guard tests; fake time and clients, no socket or process."""
import copy
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
import unittest
from contextlib import redirect_stderr
from unittest import mock


PATH = Path(__file__).resolve().parents[1] / 'atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/continuous_p2p.py'
spec = importlib.util.spec_from_file_location('test_atomic_continuous_p2p_adapter', PATH)
adapter = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = adapter
spec.loader.exec_module(adapter)
SIDES = adapter.SIDES
PROGRESS_PATH = Path(__file__).resolve().parents[1] / 'task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/p2p_progress.py'
progress_spec = importlib.util.spec_from_file_location('test_atomic_real_progress_guard', PROGRESS_PATH)
progress_module = importlib.util.module_from_spec(progress_spec)
progress_spec.loader.exec_module(progress_module)


class Clock:
    def __init__(self):
        self.now = 1.

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        assert duration >= 0
        self.now += duration


class FakeClient:
    def __init__(self, clock, *, left_saved=(.01, 0., 0., 0., 0., 0.),
                 right_saved=(0., .02, 0., 0., 0., .25)):
        self.clock = clock
        self.saved = dict(zip(SIDES, map(list, (left_saved, right_saved))))
        self.measured = {s: [0.] * 6 for s in SIDES}
        self.bias = {s: adapter.legacy._absolute_target_to_delta(self.measured[s], self.saved[s]) for s in SIDES}
        self.count, self.sequence = 40, 0
        self.session = 'owned-session'
        self.actions, self.send_times = [], []
        self.mutate = None
        self.raw_mutate = None
        self.step_error = None
        self.latency = .001
        self.frozen = False

    def get_observation(self):
        self.clock.now += .001
        self.sequence += 1
        value = {'rpc_metadata': {'protocol': 'aoe.franka-rpc.telemetry.v1',
                    'legacy_motion_enabled': True, 'native_snapshot_available': True,
                    'source_bound_torso_pose_available': True, 'diagnostics_errors': {}},
                 'command_target_state': {'available': True, 'server_session_id': self.session,
                    'step_count': self.count, 'arms': {}}}
        for side in SIDES:
            frame = side.removesuffix('_arm') + '_base'
            measured, saved = adapter._pose6(self.measured[side]), adapter._pose6(self.saved[side])
            native = {'diagnostics_complete': True, 'diagnostics_schema': 'libfranka-errors/v1',
                      'current_errors': dict.fromkeys(adapter.NATIVE_ERROR_FIELDS, False),
                      'robot_mode': 2, 'robot_mode_name': 'move', 'sensor_clock_progressing': True,
                      'age_ns': 1_000_000, 'sequence': self.sequence,
                      'source_ros_timestamp_ns': self.sequence * 1000000,
                      'joint_positions': [0.] * 7, 'joint_velocities': [0.] * 7,
                      'joint_efforts': [0.] * 7,
                      'wrench': {'frame': frame, 'force_n': [0.] * 3, 'torque_nm': [0.] * 3},
                      'torso_eef_pose': {'schema': 'aoe.franka-source-bound-pose.v1', 'frame': 'torso',
                          'native_frame': frame, 'source_sequence': self.sequence,
                          'source_ros_timestamp_ns': self.sequence * 1000000,
                          'position': list(measured.position_m), 'orientation_xyzw': list(measured.quaternion_xyzw)}}
            value[side] = {'native_robot_state': native, 'robot_state': {'joint_collision': [0.] * 7}}
            value['command_target_state']['arms'][side] = {'frame': 'torso',
                'position_m': list(saved.position_m), 'quaternion_xyzw': list(saved.quaternion_xyzw)}
        if self.mutate:
            self.mutate(value, self)
        return value

    def step(self, action):
        assert action is not None, 'No synchronization/reset permitted'
        self.actions.append(copy.deepcopy(action))
        self.send_times.append(self.clock.now)
        self.clock.now += self.latency
        if self.step_error:
            raise self.step_error
        for side in SIDES:
            motion = action[side]['motion']
            delta = motion['translation'] + motion['rotation_rotvec']
            self.saved[side] = adapter.legacy._pose_from_delta(self.saved[side], delta)
            if not self.frozen:
                # Constant world-frame displacement/rotation error of a simple
                # deterministic quasi-static plant, not a hardware simulation.
                self.measured[side] = adapter.legacy._pose_from_delta(self.saved[side], [-v for v in self.bias[side]])
        self.count += 1
        response = {'step_count': self.count, 'observation': {'raw': True}, 'done': False, 'info': {}}
        if self.raw_mutate:
            self.raw_mutate(response, self)
        return response


class ContinuousP2PTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.client = FakeClient(self.clock)
        self.patches = [mock.patch.object(adapter, 'time', self.clock),
                        mock.patch.dict(adapter.os.environ),
                        mock.patch('socket.socket', side_effect=AssertionError('network forbidden')),
                        mock.patch('subprocess.Popen', side_effect=AssertionError('process forbidden'))]
        for patch in self.patches:
            patch.start()
            self.addCleanup(patch.stop)
        adapter.os.environ.pop(adapter.TRANSLATION_LIMIT_ENV, None)

    def run_probe(self, goal=(.005, 0., 0., 0., 0., 0.), **kwargs):
        options = {'position_tolerance_m': .003, 'rotation_tolerance_rad': .1}
        options.update(kwargs)
        with redirect_stderr(io.StringIO()):
            return adapter.run(self.client, goal, [0.] * 6, **options)

    def test_raw_reply_without_ok_and_no_sync_preserve_hold_compensation(self):
        old_right = self.client.saved['right_arm'][:]
        result = self.run_probe()
        self.assertTrue(result['ok'])
        self.assertEqual(result['controller'], 'continuous_servo')
        self.assertGreater(result['commands'], 0)
        self.assertNotIn('ok', result['last_step'])
        self.assertEqual(result['final_step_count'] - result['initial_step_count'], result['commands'])
        self.assertEqual(self.client.saved['right_arm'], old_right)
        for action in self.client.actions:
            self.assertEqual(action['right_arm']['motion'], {'translation': [0.] * 3, 'rotation_rotvec': [0.] * 3})
        self.assertTrue(all(b - a >= .05 - 1e-12 for a, b in zip(self.client.send_times, self.client.send_times[1:])))
        self.assertEqual(len(result['hold_confirmation']), 3)

    def test_already_arrived_both_sides_is_read_only(self):
        result = self.run_probe(goal=[0.] * 6)
        self.assertEqual(result['commands'], 0)
        self.assertEqual(self.client.actions, [])
        self.assertIsNone(result['last_step'])
        self.assertEqual(result['held_sides'], list(SIDES))

    def test_arrived_but_moving_arm_is_rejected_without_reanchoring(self):
        self.client.mutate = lambda o, c: o['right_arm']['native_robot_state']['joint_velocities'].__setitem__(0, .02)
        with self.assertRaisesRegex(adapter.ContinuousP2PError, 'not stationary'):
            self.run_probe()
        self.assertFalse(self.client.actions)

    def test_distant_goal_uses_saved_nominal_then_servo_and_duration_honors_speed(self):
        result = self.run_probe(goal=(.06, 0., 0., 0., 0., 0.), duration_sec=.001,
                                max_translation_speed=.04, max_translation_step=.002)
        nominal = result['trajectory']
        self.assertEqual(nominal['left_start_pose'], [.01, 0., 0., 0., 0., 0.])
        self.assertGreaterEqual(nominal['duration_sec'], 1.875 * .05 / .04)
        self.assertEqual(nominal['profile'], 'quintic_s_curve')
        self.assertIn('nominal', {t['phase'] for t in result['timings']})
        self.assertIn('servo', {t['phase'] for t in result['timings']})
        self.assertLessEqual(result['final_error']['max_translation_error_m'], .003)

    def test_orientation_nominal_preserves_in_budget_translation_compensation(self):
        result = self.run_probe(goal=(.005, 0., 0., 0., 0., .5))
        target = result['trajectory']['left_target_pose']
        self.assertEqual(target[:3], [.01, 0., 0.])
        self.assertAlmostEqual(target[5], .5)
        self.assertEqual(result['trajectory']['max_translation_m'], 0.)

    def test_translation_nominal_preserves_in_budget_rotation_compensation(self):
        self.client = FakeClient(self.clock, left_saved=(.01, 0., 0., 0., 0., .15))
        result = self.run_probe(goal=(.06, 0., 0., 0., 0., .05))
        target = result['trajectory']['left_target_pose']
        self.assertEqual(target[:3], [.06, 0., 0.])
        self.assertAlmostEqual(target[5], .15)
        self.assertLess(result['trajectory']['max_rotation_rad'], 1e-12)

    def test_nominal_keeps_caps_with_the_same_roundoff_allowance_as_servo_math(self):
        for left_saved, goal in (
                ((.030000000001, 0., 0., 0., 0., 0.), (.005, 0., 0., 0., 0., 0.)),
                ((0., 0., 0., 0., 0., .3400000000009), (0., 0., 0., 0., 0., .14))):
            with self.subTest(left_saved=left_saved):
                self.client = FakeClient(self.clock, left_saved=left_saved)
                runner = adapter._Run(self.client, goal, [0.] * 6, False, None, None,
                                      {'position_tolerance_m': .003, 'rotation_tolerance_rad': .1})
                runner.started = self.clock.now
                with redirect_stderr(io.StringIO()):
                    runner.prepare()
                    # This is a legal existing saved target according to the
                    # exact controller that will consume it after nominal.
                    adapter.geometry.ContinuousPoseServo(runner.goals['left_arm'],
                                                         runner.saved['left_arm'], runner.config)
                    metadata = runner.nominal()
                self.assertEqual(metadata['steps'], 0)
                self.assertFalse(self.client.actions)
                self.assertEqual(self.client.saved['left_arm'], list(left_saved))

    def test_delta_goal_is_frozen_from_initial_native_not_saved(self):
        result = self.run_probe(delta=True)
        self.assertLessEqual(result['final_error']['left_translation_error_m'], .003)
        # Starting native x=0, requested +5mm, initial saved x=10mm.
        self.assertLess(self.client.measured['left_arm'][0], .008)

    def test_tighter_caller_limits_are_not_relaxed(self):
        result = self.run_probe(position_tolerance_m=.001, rotation_tolerance_rad=.02,
                                max_translation_speed=.001, max_rotation_speed=.005,
                                max_translation_step=.00004, max_rotation_step=.0001,
                                max_tracking_translation_bias_m=.012, max_tracking_rotation_bias_rad=.15)
        cfg = result['servo_config']
        self.assertEqual(cfg['position_tolerance_m'], .001)
        self.assertEqual(cfg['rotation_tolerance_rad'], .02)
        self.assertEqual(cfg['max_translation_step_m'], .00004)
        self.assertEqual(cfg['max_translation_bias_m'], .012)
        self.assertLessEqual(result['final_error']['max_translation_error_m'], .001)

    def test_explicit_none_can_reach_with_more_than_25mm_compensation(self):
        self.client = FakeClient(self.clock, left_saved=(.04, 0., 0., 0., 0., 0.))
        result = self.run_probe(max_tracking_translation_bias_m=None)
        self.assertTrue(result['ok'])
        self.assertIsNone(result['servo_config']['max_translation_bias_m'])
        self.assertEqual(result['trajectory']['steps'], 0)
        self.assertGreater(self.client.actions[0]['left_arm']['motion']['translation'][0], 0.)
        self.assertGreater(self.client.saved['left_arm'][0] - .005, .025)
        self.assertLessEqual(result['final_error']['left_translation_error_m'], .003)
        bound = result['bounds']['left_arm']
        self.assertAlmostEqual(bound['position_margin_m'], .002 * (30. + .2))
        self.assertEqual(bound['position_margin_basis'], 'servo_speed_times_budget_plus_max_dt')
        self.assertFalse(bound['workspace_authorization_bound'])

    def test_environment_none_overrides_old_caller_25mm_option(self):
        self.client = FakeClient(self.clock, left_saved=(.04, 0., 0., 0., 0., 0.))
        with mock.patch.dict(adapter.os.environ, {adapter.TRANSLATION_LIMIT_ENV: 'none'}):
            result = self.run_probe(max_tracking_translation_bias_m=.025)
        self.assertIsNone(result['servo_config']['max_translation_bias_m'])
        self.assertEqual(result['translation_limit_source'], 'environment')
        self.assertGreater(self.client.saved['left_arm'][0] - .005, .025)
        self.assertTrue(result['ok'])

    def test_explicit_finite_cap_above_25mm_is_not_silently_reduced(self):
        self.client = FakeClient(self.clock, left_saved=(.03, 0., 0., 0., 0., 0.))
        result = self.run_probe(max_tracking_translation_bias_m=.04)
        self.assertEqual(result['servo_config']['max_translation_bias_m'], .04)
        self.assertGreater(self.client.saved['left_arm'][0] - .005, .025)
        self.assertLessEqual(self.client.saved['left_arm'][0] - .005, .04)

    def test_uncapped_far_goal_nominal_carries_existing_bias_without_changing_goal(self):
        self.client = FakeClient(self.clock, left_saved=(.03, 0., 0., 0., 0., 0.))
        result = self.run_probe(goal=(.06, 0., 0., 0., 0., 0.), max_tracking_translation_bias_m=None)
        self.assertAlmostEqual(result['trajectory']['left_start_pose'][0], .03)
        self.assertAlmostEqual(result['trajectory']['left_target_pose'][0], .09)
        self.assertAlmostEqual(self.client.measured['left_arm'][0], .06)
        self.assertAlmostEqual(self.client.saved['left_arm'][0], .09)
        self.assertLessEqual(result['final_error']['left_translation_error_m'], .003)
        self.assertGreaterEqual(result['bounds']['left_arm']['high'][0], .09 + .0604 - 1e-12)

    def test_uncapped_near_goal_keeps_saved_bias_during_rotation_nominal(self):
        self.client = FakeClient(self.clock, left_saved=(.04, 0., 0., 0., 0., 0.))
        result = self.run_probe(goal=(.005, 0., 0., 0., 0., .5), max_tracking_translation_bias_m=None)
        self.assertAlmostEqual(result['trajectory']['left_target_pose'][0], .04)
        self.assertAlmostEqual(result['trajectory']['left_target_pose'][5], .5)
        self.assertEqual(result['trajectory']['max_translation_m'], 0.)
        self.assertGreater(self.client.saved['left_arm'][0] - .005, .025)

    def test_uncapped_translation_still_enforces_rotation_cap_and_deadline(self):
        self.client = FakeClient(self.clock, left_saved=(0., 0., 0., 0., 0., .34))
        with self.assertRaises(adapter.ContinuousP2PError) as raised:
            self.run_probe(goal=(0., 0., 0., 0., 0., .14), max_tracking_translation_bias_m=None)
        message = str(raised.exception)
        self.assertIn('30-second servo budget exhausted', message)
        detail = json.loads(message.split('residual=', 1)[1].split('; confirmed_commands=', 1)[0])
        self.assertIsNone(detail['translation_cap_m'])
        self.assertFalse(detail['at_cap']['left_arm']['translation'])
        self.assertTrue(detail['at_cap']['left_arm']['rotation'])
        self.assertLessEqual(detail['bias']['left_arm']['rotation_rad'], .2 + 2e-12)
        self.assertLess(self.clock.now, 31.1)

    def test_invalid_translation_limit_environment_rejected_before_transport(self):
        for value in ('', 'None', 'NaN', 'inf', '0.05', 'false'):
            with self.subTest(value=value), mock.patch.dict(adapter.os.environ, {adapter.TRANSLATION_LIMIT_ENV: value}):
                with self.assertRaisesRegex(ValueError, adapter.TRANSLATION_LIMIT_ENV):
                    self.run_probe()
        self.assertEqual(self.client.sequence, 0)

    def test_ack_failures_stop_after_one_attempt(self):
        for mutation in (lambda r, c: r.update(ok=False), lambda r, c: r.update(done=True),
                         lambda r, c: r.update(step_count=c.count + 1),
                         lambda r, c: r.update(info={'error': 'fault'})):
            with self.subTest(mutation=mutation):
                self.client = FakeClient(self.clock)
                self.client.raw_mutate = mutation
                with self.assertRaisesRegex(adapter.ContinuousP2PError, 'acknowledgement'):
                    self.run_probe()
                self.assertEqual(len(self.client.actions), 1)

    def test_timeout_never_replays(self):
        self.client.step_error = TimeoutError('unknown transport outcome')
        with self.assertRaisesRegex(adapter.ContinuousP2PError, 'unknown transport outcome'):
            self.run_probe()
        self.assertEqual(len(self.client.actions), 1)

    def test_session_change_after_ack_is_not_adopted(self):
        def mutation(o, c):
            if c.actions:
                o['command_target_state']['server_session_id'] = 'foreign'
        self.client.mutate = mutation
        with self.assertRaisesRegex(adapter.ContinuousP2PError, 'session/count'):
            self.run_probe()
        self.assertEqual(len(self.client.actions), 1)

    def test_saved_target_mismatch_after_ack_is_not_adopted(self):
        def mutation(o, c):
            if c.actions:
                o['command_target_state']['arms']['left_arm']['position_m'][0] += .0001
        self.client.mutate = mutation
        with self.assertRaisesRegex(adapter.ContinuousP2PError, 'saved target differs'):
            self.run_probe()
        self.assertEqual(len(self.client.actions), 1)

    def test_native_fault_missing_error_and_bad_source_binding_rejected(self):
        mutations = [lambda n: n['current_errors'].update(joint_reflex=True),
                     lambda n: n['current_errors'].pop('joint_reflex'),
                     lambda n: n['torso_eef_pose'].update(source_sequence=999),
                     lambda n: n.update(age_ns=250_000_001),
                     lambda n: n.update(robot_mode=4)]
        for mutation in mutations:
            with self.subTest(mutation=mutation):
                self.client = FakeClient(self.clock)
                self.client.mutate = lambda o, c: mutation(o['left_arm']['native_robot_state'])
                with self.assertRaises(adapter.ContinuousP2PError):
                    self.run_probe()
                self.assertFalse(self.client.actions)

    def test_nonfinite_joint_force_torque_and_pose_are_rejected(self):
        for field in ('joint_velocities', 'force_n', 'torque_nm', 'position'):
            with self.subTest(field=field):
                self.client = FakeClient(self.clock)
                def mutation(o, c):
                    native = o['left_arm']['native_robot_state']
                    owner = (native if field == 'joint_velocities' else native['torso_eef_pose']
                             if field == 'position' else native['wrench'])
                    owner[field][0] = math.nan
                self.client.mutate = mutation
                with self.assertRaisesRegex(adapter.ContinuousP2PError, 'finite'):
                    self.run_probe()
                self.assertFalse(self.client.actions)

    def test_servo_speed_limit_and_wrench_limits_stop_without_retry(self):
        for field, value in (('joint_velocities', .151), ('force_n', 10.01), ('torque_nm', 3.01)):
            with self.subTest(field=field):
                self.client = FakeClient(self.clock)
                def mutation(o, c):
                    if c.actions:
                        native = o['left_arm']['native_robot_state']
                        owner = native if field == 'joint_velocities' else native['wrench']
                        owner[field][0] = value
                self.client.mutate = mutation
                with self.assertRaisesRegex(adapter.ContinuousP2PError, 'limit exceeded'):
                    self.run_probe()
                self.assertEqual(len(self.client.actions), 1)

    def test_actual_pose_outside_fixed_corridor_stops(self):
        def mutation(o, c):
            if c.actions:
                o['left_arm']['native_robot_state']['torso_eef_pose']['position'][2] = .026
        self.client.mutate = mutation
        with self.assertRaisesRegex(adapter.ContinuousP2PError, 'corridor exceeded'):
            self.run_probe()
        self.assertEqual(len(self.client.actions), 1)

    def test_slow_ack_is_confirmed_but_never_followed_by_catchup(self):
        self.client.latency = .101
        with self.assertRaisesRegex(adapter.ContinuousP2PError, 'confirmed_commands=1'):
            self.run_probe()
        self.assertEqual(len(self.client.actions), 1)

    def test_final_correction_guard_once_and_feedback_every_snapshot(self):
        feedback, corrections = [], []
        result = self.run_probe(feedback_guard=lambda o: feedback.append(o), correction_guard=corrections.append)
        self.assertEqual(corrections, [result['final_error']])
        self.assertGreater(len(feedback), result['commands'] + 3)

    def test_real_task_progress_guard_accepts_the_single_final_report(self):
        guard = progress_module.P2PProgressGuard(.003, .1)
        decisions = []
        def check(report):
            decisions.append(guard.observe(report))
        result = self.run_probe(correction_guard=check)
        self.assertEqual(result['final_error']['correction_index'], 0)
        self.assertEqual(len(decisions), 1)
        self.assertTrue(decisions[0]['within_tolerance'])
        self.assertEqual(decisions[0]['correction_index'], 0)
        self.assertEqual(len(guard.best), 4)

    def test_external_feedback_guard_failure_stops_immediately(self):
        def guard(observation):
            if self.client.actions:
                raise RuntimeError('cancelled by supervisor')
        with self.assertRaisesRegex(adapter.ContinuousP2PError, 'cancelled by supervisor'):
            self.run_probe(feedback_guard=guard)
        self.assertEqual(len(self.client.actions), 1)

    def test_count_change_during_read_only_hold_rejected(self):
        def mutation(o, c):
            if c.sequence >= 4:
                o['command_target_state']['step_count'] += 1
        self.client.mutate = mutation
        with self.assertRaisesRegex(adapter.ContinuousP2PError, 'session/count'):
            self.run_probe(goal=[0.] * 6)
        self.assertFalse(self.client.actions)

    def test_stale_native_sequence_rejected_before_send(self):
        self.client.mutate = lambda o, c: o['left_arm']['native_robot_state'].update(sequence=1)
        with self.assertRaises(adapter.ContinuousP2PError):
            self.run_probe()
        self.assertFalse(self.client.actions)

    def test_slow_callback_before_first_send_cannot_use_stale_feedback(self):
        def guard(observation):
            self.clock.now += .11
        with self.assertRaisesRegex(adapter.ContinuousP2PError, 'Dispatch deadline'):
            self.run_probe(feedback_guard=guard)
        self.assertFalse(self.client.actions)

    def test_slow_native_read_rejected_before_send(self):
        original = self.client.get_observation
        def slow_read():
            self.clock.now += .11
            return original()
        self.client.get_observation = slow_read
        with self.assertRaisesRegex(adapter.ContinuousP2PError, 'Native observation read'):
            self.run_probe()
        self.assertFalse(self.client.actions)

    def test_hold_drift_is_failure_without_any_commands(self):
        def mutation(o, c):
            if c.sequence >= 4:
                o['left_arm']['native_robot_state']['torso_eef_pose']['position'][0] = .004
        self.client.mutate = mutation
        with self.assertRaisesRegex(adapter.ContinuousP2PError, 'held arm left'):
            self.run_probe(goal=[0.] * 6)
        self.assertFalse(self.client.actions)

    def test_slow_hold_third_snapshot_cannot_complete_after_deadline(self):
        def guard(observation):
            # prepare + servo precheck + the three regular hold observations.
            if self.client.sequence == 5:
                self.clock.now += 6.
        with self.assertRaisesRegex(adapter.ContinuousP2PError, 'within 6 seconds'):
            self.run_probe(goal=[0.] * 6, feedback_guard=guard)
        self.assertEqual(self.client.sequence, 5)
        self.assertFalse(self.client.actions)

    def test_slow_independent_hold_snapshot_cannot_complete_after_deadline(self):
        def guard(observation):
            if self.client.sequence == 6:
                self.clock.now += 6.
        with self.assertRaisesRegex(adapter.ContinuousP2PError, 'within 6 seconds'):
            self.run_probe(goal=[0.] * 6, feedback_guard=guard)
        self.assertEqual(self.client.sequence, 6)
        self.assertFalse(self.client.actions)

    def test_hold_confirmation_work_cannot_return_success_after_deadline(self):
        original = adapter._six
        delayed = False
        def slow_confirmation(value):
            nonlocal delayed
            if self.client.sequence == 6 and not delayed:
                self.clock.now += 6.
                delayed = True
            return original(value)
        with mock.patch.object(adapter, '_six', slow_confirmation):
            with self.assertRaisesRegex(adapter.ContinuousP2PError, 'within 6 seconds'):
                self.run_probe(goal=[0.] * 6)
        self.assertTrue(delayed)
        self.assertFalse(self.client.actions)

    def test_nonresponsive_plant_exhausts_budget_without_success(self):
        self.client.frozen = True
        with self.assertRaisesRegex(adapter.ContinuousP2PError, 'budget exhausted|no feasible progress'):
            self.run_probe()
        self.assertGreater(len(self.client.actions), 0)
        self.assertLess(self.clock.now, 32.)
        self.assertLessEqual(self.client.saved['left_arm'][0] - .005, .025 + 1e-12)

    def capped_client(self):
        self.client = FakeClient(self.clock, left_saved=(.03, 0., 0., 0., 0., 0.))
        return self.client

    def assert_only_cap_roundoff_step(self):
        # The math module permits 1e-12 m of geometric roundoff at the boundary.
        # Its first proposal consumes that slack. ALL subsequent cap feedback
        # must be read-only; this is not permission for repeated zero RPCs.
        self.assertEqual(len(self.client.actions), 1)
        increment = self.client.actions[0]['left_arm']['motion']['translation']
        self.assertLessEqual(math.hypot(*increment), 2e-12)
        self.assertEqual(self.client.count, 41)

    def test_cap_wait_can_converge_from_delayed_feedback_without_zero_steps(self):
        self.capped_client()
        def delayed_feedback(o, c):
            if c.sequence >= 8:
                o['left_arm']['native_robot_state']['torso_eef_pose']['position'][0] = .005
        self.client.mutate = delayed_feedback
        log = io.StringIO()
        with redirect_stderr(log):
            result = adapter.run(self.client, [.005, 0., 0., 0., 0., 0.], [0.] * 6,
                                 position_tolerance_m=.003, rotation_tolerance_rad=.1)
        self.assertTrue(result['ok'])
        self.assertEqual(result['commands'], 1)
        self.assert_only_cap_roundoff_step()
        self.assertGreaterEqual(self.client.sequence, 12)
        self.assertEqual(log.getvalue().count('"event": "p2p_servo_cap_warning"'), 1)

    def test_permanent_cap_wait_keeps_original_deadline_and_reports_residual(self):
        self.capped_client()
        with self.assertRaises(adapter.ContinuousP2PError) as raised:
            self.run_probe()
        message = str(raised.exception)
        self.assertIn('30-second servo budget exhausted', message)
        detail = json.loads(message.split('residual=', 1)[1].split('; confirmed_commands=', 1)[0])
        self.assertAlmostEqual(detail['final_error']['left_translation_error_m'], .005, places=10)
        self.assertIn('"translation_cap_m": 0.025', message)
        self.assertIn('"cap_wait_sides": ["left_arm"]', message)
        self.assertIn('"at_cap":', message)
        self.assertIn('"translation": true', message)
        self.assert_only_cap_roundoff_step()
        self.assertLessEqual(self.clock.now, 31.06)

    def test_cap_wait_rejects_external_command_and_native_fault(self):
        for fault in ('count', 'native', 'target'):
            with self.subTest(fault=fault):
                self.capped_client()
                def mutation(o, c):
                    if c.sequence >= 4:
                        if fault == 'count':
                            o['command_target_state']['step_count'] += 1
                        elif fault == 'target':
                            o['command_target_state']['arms']['left_arm']['position_m'][0] += .0001
                        else:
                            o['left_arm']['native_robot_state']['current_errors']['joint_reflex'] = True
                self.client.mutate = mutation
                with self.assertRaises(adapter.ContinuousP2PError):
                    self.run_probe()
                self.assertEqual(self.client.sequence, 4)
                self.assert_only_cap_roundoff_step()

    def test_cap_wait_can_resume_a_feasible_inward_step_without_slow_cycle_error(self):
        self.capped_client()
        def changed_error(o, c):
            if c.sequence >= 8:
                o['left_arm']['native_robot_state']['torso_eef_pose']['position'][0] = .009 if c.count == 41 else .005
        self.client.mutate = changed_error
        result = self.run_probe()
        self.assertTrue(result['ok'])
        self.assertGreater(result['commands'], 0)
        self.assertLess(self.client.actions[1]['left_arm']['motion']['translation'][0], 0.)
        self.assertGreater(self.client.send_times[1] - self.client.send_times[0], .1)
        self.assertGreater(result['timings'][1]['actual_send_interval_s'], .1)
        self.assertLess(result['timings'][1]['control_dt_s'], .1)
        self.assertLessEqual(result['final_error']['left_translation_error_m'], .003)

    def test_one_capped_arm_stays_fixed_while_other_arm_moves(self):
        self.capped_client()
        self.client.saved['right_arm'] = [0.] * 6
        self.client.bias['right_arm'] = [0.] * 6
        def delayed_left(o, c):
            if c.sequence >= 12:
                o['left_arm']['native_robot_state']['torso_eef_pose']['position'][0] = .005
        self.client.mutate = delayed_left
        with redirect_stderr(io.StringIO()):
            result = adapter.run(self.client, [.005, 0., 0., 0., 0., 0.],
                                 [0., .005, 0., 0., 0., 0.], position_tolerance_m=.003, rotation_tolerance_rad=.1)
        self.assertTrue(result['ok'])
        self.assertGreater(result['commands'], 0)
        self.assertLessEqual(math.hypot(*self.client.actions[0]['left_arm']['motion']['translation']), 2e-12)
        for action in self.client.actions[1:]:
            self.assertEqual(action['left_arm']['motion']['translation'], [0.] * 3)
        self.assertAlmostEqual(self.client.saved['left_arm'][0], .03, places=10)

    def test_cap_wait_read_crossing_deadline_cannot_succeed(self):
        self.capped_client()
        hold_start = None
        def callback(o):
            nonlocal hold_start
            if self.client.sequence == 2:
                hold_start = self.clock.now
            if self.client.sequence >= 3 and self.clock.now - hold_start > 29.9:
                self.clock.now += .09
                o['left_arm']['native_robot_state']['torso_eef_pose']['position'][0] = .005
        with self.assertRaisesRegex(adapter.ContinuousP2PError, 'budget exhausted'):
            self.run_probe(feedback_guard=callback)
        self.assert_only_cap_roundoff_step()

    def test_cap_wait_does_not_hide_a_single_slow_native_read(self):
        self.capped_client()
        original = self.client.get_observation
        def slow_wait_read():
            if self.client.sequence >= 3:
                self.clock.now += .101
            return original()
        self.client.get_observation = slow_wait_read
        with self.assertRaisesRegex(adapter.ContinuousP2PError, 'Native observation read exceeded'):
            self.run_probe()
        self.assert_only_cap_roundoff_step()

    def test_arrival_latched_then_lost_does_not_restart_correction(self):
        first_arrival = None
        commands_at_loss = None
        def later_drift(o, c):
            nonlocal first_arrival, commands_at_loss
            actual = o['left_arm']['native_robot_state']['torso_eef_pose']['position']
            if abs(actual[0] - .005) <= .003 and first_arrival is None:
                first_arrival = c.sequence
            if first_arrival is not None and c.sequence >= first_arrival + 5:
                if commands_at_loss is None:
                    commands_at_loss = len(c.actions)
                actual[0] = -.02
        self.client.mutate = later_drift
        with self.assertRaisesRegex(adapter.ContinuousP2PError, 'lost|arrival'):
            self.run_probe()
        self.assertIsNotNone(commands_at_loss)
        self.assertEqual(len(self.client.actions), commands_at_loss)

    def test_max_steps_is_total_limit_without_retry(self):
        with self.assertRaisesRegex(adapter.ContinuousP2PError, 'max_steps budget exhausted'):
            self.run_probe(max_steps=2)
        self.assertEqual(len(self.client.actions), 2)

    def test_bad_timing_and_sleep_options_rejected_before_transport(self):
        for kwargs in ({'sleep': False}, {'rate_hz': 5}, {'rate_hz': 0}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.run_probe(**kwargs)
        self.assertEqual(self.client.sequence, 0)


if __name__ == '__main__':
    unittest.main()
