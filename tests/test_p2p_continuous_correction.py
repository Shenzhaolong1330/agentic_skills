"""Guarded P2P correction against a saved-target simulator; no robot or network."""
import copy
import importlib.util
import io
from contextlib import redirect_stderr
from pathlib import Path
import unittest


PATH = Path(__file__).resolve().parents[1] / 'atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/dual_franka_robotiq_rpc_client.py'
spec = importlib.util.spec_from_file_location('continuous_correction_rpc', PATH)
rpc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rpc)
SIDES = ('left_arm', 'right_arm')


class SavedTargetPlant(rpc.DualFrankaRobotiqRpcClient):
    """Match the server's delta accumulation, with independently biased motion."""

    def __init__(self, bias=None, stuck=False):
        # Deliberately do not call the transport constructor.
        self.actual = {'left_arm': [0., .35, 0., 0., 0., 0.],
                       'right_arm': [0., -.38, 0., 0., 0., 0.]}
        self.target = {'left_arm': [.4, .2, .1, .1, 0., .2],
                       'right_arm': [.5, -.2, .1, 0., .1, .3]}
        self.bias = bias if bias is not None else [0., 0., 0., 0., 0., -.13]
        self.stuck = stuck
        self.actions, self.published = [], []
        self.reads = 0
        self.mutate_snapshot = None
        self.fail_motion = None
        self.reply_done = False

    def snapshot(self, kind):
        snapshot = {side: {'end_pose': self.actual[side].copy(), 'robot_state': {
            'diagnostics_complete': True, 'robot_mode': 2,
            'current_errors': {'joint_reflex': False},
        }} for side in SIDES}
        snapshot['command_target_state'] = {
            'available': True, 'server_session_id': 'fake-session',
            'step_count': len(self.actions),
            'arms': {side: {'frame': 'torso', 'position_m': pose[:3],
                            'quaternion_xyzw': list(rpc._rotvec_to_quat_xyzw(pose[3:]))}
                     for side, pose in self.target.items()},
        }
        snapshot['_feedback_kind'] = kind
        if self.mutate_snapshot:
            self.mutate_snapshot(snapshot, kind)
        return snapshot

    def get_observation(self):
        self.reads += 1
        return self.snapshot('initial' if not self.actions else 'correction')

    def step(self, action=None):
        self.actions.append(copy.deepcopy(action))
        if action is None:
            self.target = copy.deepcopy(self.actual)
        else:
            for side in SIDES:
                motion = action[side]['motion']
                self.target[side] = rpc._pose_from_delta(
                    self.target[side], motion['translation'] + motion['rotation_rotvec'])
            self.published.append(copy.deepcopy(self.target))
            if not self.stuck:
                self.actual = copy.deepcopy(self.target)
                self.actual['right_arm'] = rpc._pose_from_delta(self.target['right_arm'], self.bias)
            if self.fail_motion:
                raise self.fail_motion  # Applied command, but uncertain reply.
        return {'ok': True, 'done': self.reply_done,
                'observation': self.snapshot('sync' if action is None else 'motion'),
                'step_count': len(self.actions)}


class LegacyRawStepPlant(SavedTargetPlant):
    """The saved deployed server enriches get_observation, but not env.step."""

    def __init__(self):
        super().__init__()
        self.raw_reply_mutator = None
        self.read_mutator = None
        self.fail_read_at = None
        self.read_error = TimeoutError('complete observation read timed out')

    def step(self, action=None):
        reply = super().step(action)
        # The actual server's raw step contains legacy eef_pose and gripper
        # fields, without native_robot_state or command_target_state.
        reply['observation'] = {side: {'robot_state': {'eef_pose': {
            'position': self.actual[side][:3],
            'orientation_xyzw': list(rpc._rotvec_to_quat_xyzw(self.actual[side][3:])),
            'frame': 'torso',
        }}, 'gripper': {}} for side in SIDES}
        reply['reward'] = 0.
        reply['info'] = {}
        if self.raw_reply_mutator:
            self.raw_reply_mutator(reply, action)
        return reply

    def get_observation(self):
        snapshot = super().get_observation()
        if self.reads == self.fail_read_at:
            raise self.read_error
        if self.read_mutator:
            self.read_mutator(snapshot)
        return snapshot


def require_health(observation):
    # The application supplies its native health validator; this fake requires
    # explicit evidence rather than accepting absent fields as healthy.
    for side in SIDES:
        health = observation[side].get('robot_state', {})
        if (health.get('diagnostics_complete') is not True or health.get('robot_mode') != 2
                or not isinstance(health.get('current_errors'), dict)
                or any(health['current_errors'].values())):
            raise RuntimeError('missing or faulty health')


class ContinuousCorrectionTests(unittest.TestCase):
    def move(self, client, **overrides):
        options = dict(delta=False, smooth=True, sleep=False, rate_hz=10,
                       position_tolerance_m=.003, rotation_tolerance_rad=.03,
                       max_correction_iters=9, stall_policy='warn', feedback_guard=require_health)
        options.update(overrides)
        self.stderr = io.StringIO()
        with redirect_stderr(self.stderr):
            return client.dual_robot_move_to_ee_pose(
                client.actual['left_arm'], [.01, -.38, 0., 0., 0., 0.], **options)

    def test_continuous_corrections_converge_where_repeated_short_calls_do_not(self):
        old = SavedTargetPlant()
        for _ in range(3):
            result = self.move(old, max_correction_iters=2, stall_policy='stop', feedback_guard=None)
            self.assertFalse(result['ok'])
            self.assertAlmostEqual(result['final_error']['right_rotation_error_rad'], .07)
        self.assertEqual(sum(action is None for action in old.actions), 3)
        continuous = SavedTargetPlant()
        result = self.move(continuous)
        self.assertTrue(result['ok'])
        self.assertEqual(result['final_error']['correction_index'], 4)
        self.assertLessEqual(result['final_error']['right_rotation_error_rad'], .03)
        self.assertEqual(sum(action is None for action in continuous.actions), 1)
        self.assertEqual(result['tolerances']['position_tolerance_m'], .003)
        self.assertEqual(result['tolerances']['rotation_tolerance_rad'], .03)

    def test_every_returned_observation_runs_feedback_guard(self):
        client = SavedTargetPlant()
        seen = []
        def guard(observation):
            require_health(observation)
            seen.append(observation['_feedback_kind'])
        result = self.move(client, feedback_guard=guard)
        self.assertTrue(result['ok'])
        self.assertEqual(seen[:2], ['initial', 'sync'])
        self.assertEqual(seen.count('motion'), len(client.published))
        self.assertEqual(seen.count('correction'), len(result['corrections']))
        self.assertEqual(len(seen), client.reads + len(client.actions))

    def test_stall_warn_continues_same_call_but_default_stop_remains_compatible(self):
        options = dict(stall_detection_side='right_arm', max_stalled_correction_iters=1,
                       stall_translation_epsilon_m=1., stall_rotation_epsilon_rad=1.)
        old = self.move(SavedTargetPlant(), stall_policy='stop', feedback_guard=None, **options)
        self.assertFalse(old['ok'])
        self.assertEqual(old['final_error']['correction_index'], 0)
        client = SavedTargetPlant()
        result = self.move(client, **options)
        self.assertTrue(result['ok'])
        self.assertIn('p2p_stall_warning:', self.stderr.getvalue())
        self.assertEqual(sum(action is None for action in client.actions), 1)

    def test_stuck_plant_warns_until_budget_and_is_never_successful(self):
        client = SavedTargetPlant(stuck=True)
        result = self.move(client, max_correction_iters=2, stall_detection_side='right_arm',
                           max_stalled_correction_iters=1)
        self.assertFalse(result['ok'])
        self.assertEqual(result['final_error']['correction_index'], 2)
        self.assertEqual(self.stderr.getvalue().count('p2p_stall_warning:'), 3)
        self.assertEqual(sum(action is None for action in client.actions), 1)

    def test_compensation_bound_does_not_release_an_out_of_tolerance_result(self):
        client = SavedTargetPlant(stuck=True)
        result = self.move(client, max_tracking_translation_bias_m=.005,
                           stall_detection_side='right_arm', max_stalled_correction_iters=1)
        self.assertFalse(result['ok'])
        self.assertEqual(result['stalled']['reason'], 'tracking_compensation_limit')
        self.assertLess(result['final_error']['correction_index'], 9)
        self.assertLessEqual(max(p['right_arm'][0] for p in client.published), .015 + 1e-9)
        self.assertEqual(sum(action is None for action in client.actions), 1)

    def test_warn_requires_callable_guard_and_smooth_before_any_rpc(self):
        for override in ({'feedback_guard': None}, {'feedback_guard': True},
                         {'stall_policy': 'ignore'}, {'smooth': False},
                         {'correction_guard': True}):
            with self.subTest(override=override):
                client = SavedTargetPlant()
                with self.assertRaises(ValueError):
                    self.move(client, **override)
                self.assertEqual(client.actions, [])
                self.assertEqual(client.reads, 0)

    def test_guard_fault_or_timeout_at_each_observation_aborts_without_replay(self):
        for phase in ('initial', 'sync', 'motion', 'correction'):
            for error in (RuntimeError('fault'), TimeoutError('guard timed out')):
                with self.subTest(phase=phase, error=type(error).__name__):
                    client = SavedTargetPlant()
                    at_failure = []
                    def guard(observation):
                        require_health(observation)
                        if observation['_feedback_kind'] == phase:
                            at_failure.append((len(client.actions), client.reads))
                            raise error
                    with self.assertRaises(type(error)) as caught:
                        self.move(client, feedback_guard=guard)
                    self.assertIs(caught.exception, error)
                    self.assertEqual(at_failure, [(len(client.actions), client.reads)])
                    if phase == 'initial':
                        self.assertEqual(client.actions, [])
                    elif phase == 'sync':
                        self.assertEqual(client.actions, [None])
                    elif phase == 'motion':
                        self.assertEqual(len(client.actions), 2)

    def test_missing_health_at_initial_or_motion_is_rejected(self):
        for phase in ('initial', 'motion'):
            with self.subTest(phase=phase):
                client = SavedTargetPlant()
                def omit_health(observation, kind):
                    if kind == phase:
                        observation['right_arm'].pop('robot_state')
                client.mutate_snapshot = omit_health
                with self.assertRaisesRegex(RuntimeError, 'missing or faulty health'):
                    self.move(client)
                self.assertEqual(len(client.actions), 0 if phase == 'initial' else 2)

    def test_correction_guard_runs_on_each_measurement_including_arrival_and_stops_on_failure(self):
        client = SavedTargetPlant()
        calls = []
        result = self.move(client, correction_guard=lambda error: calls.append(copy.deepcopy(error)))
        self.assertEqual([error['correction_index'] for error in calls], [0, 1, 2, 3, 4])
        self.assertEqual(result['final_error']['correction_index'], 4)
        for error in (RuntimeError('error increasing'), TimeoutError('guard timed out')):
            client = SavedTargetPlant()
            at_failure = []
            def guard(final_error):
                at_failure.append((len(client.actions), client.reads))
                raise error
            with self.assertRaises(type(error)) as caught:
                self.move(client, correction_guard=guard)
            self.assertIs(caught.exception, error)
            self.assertEqual(at_failure, [(len(client.actions), client.reads)])
            self.assertNotIn('p2p_correction_start:', self.stderr.getvalue())

    def test_already_within_tolerance_reports_arrival_without_another_command(self):
        calls = []
        result = self.move(SavedTargetPlant(bias=[0.] * 6), correction_guard=lambda error: calls.append(error))
        self.assertTrue(result['ok'])
        self.assertEqual(result['final_error']['correction_index'], 0)
        self.assertEqual(calls, [result['final_error']])
        self.assertNotIn('p2p_correction_start:', self.stderr.getvalue())

    def test_exhausted_budget_still_checks_final_physical_error(self):
        client = SavedTargetPlant()
        calls = []
        def guard(final_error):
            calls.append(final_error['correction_index'])
            if final_error['correction_index'] == 1:
                raise RuntimeError('final physical error increased')
        with self.assertRaisesRegex(RuntimeError, 'final physical error increased'):
            self.move(client, max_correction_iters=1, correction_guard=guard)
        self.assertEqual(calls, [0, 1])
        self.assertEqual(self.stderr.getvalue().count('p2p_correction_start:'), 1)

    def test_applied_command_timeout_is_not_retried(self):
        client = SavedTargetPlant()
        client.fail_motion = TimeoutError('applied command, reply lost')
        with self.assertRaises(TimeoutError) as caught:
            self.move(client)
        self.assertIs(caught.exception, client.fail_motion)
        self.assertEqual(len(client.actions), 2)
        self.assertEqual(len(client.published), 1)

    def test_initial_command_metadata_is_required_before_sync(self):
        for field, value in (('available', False), ('server_session_id', ''),
                             ('step_count', True), ('step_count', -1)):
            with self.subTest(field=field, value=value):
                client = SavedTargetPlant()
                def corrupt(observation, kind):
                    observation['command_target_state'][field] = value
                client.mutate_snapshot = corrupt
                with self.assertRaisesRegex(RuntimeError, 'missing or invalid'):
                    self.move(client)
                self.assertEqual(client.actions, [])

    def test_session_count_frame_and_expected_pose_changes_abort_without_retry(self):
        for phase in ('sync', 'motion', 'correction'):
            for corruption in ('session', 'count', 'frame', 'pose'):
                # A sync establishes a new saved target; its pose may differ
                # from the old target by design. Stream/read poses may not.
                if phase == 'sync' and corruption == 'pose':
                    continue
                with self.subTest(phase=phase, corruption=corruption):
                    client = SavedTargetPlant()
                    at_failure = []
                    def corrupt(observation, kind):
                        if kind != phase:
                            return
                        at_failure.append((len(client.actions), client.reads))
                        state = observation['command_target_state']
                        if corruption == 'session':
                            state['server_session_id'] = 'another-session'
                        elif corruption == 'count':
                            state['step_count'] += 1
                        elif corruption == 'frame':
                            state['arms']['left_arm']['frame'] = 'left_base'
                        else:
                            state['arms']['left_arm']['position_m'][0] += .002
                    client.mutate_snapshot = corrupt
                    with self.assertRaises(RuntimeError):
                        self.move(client)
                    self.assertEqual(at_failure, [(len(client.actions), client.reads)])

    def test_done_sync_reply_prevents_trajectory(self):
        client = SavedTargetPlant()
        client.reply_done = True
        with self.assertRaisesRegex(RuntimeError, 'reported done'):
            self.move(client)
        self.assertEqual(client.actions, [None])

    def test_explicit_read_after_step_supports_deployed_raw_replies(self):
        client = LegacyRawStepPlant()
        result = self.move(client, feedback_mode='read_after_step')
        self.assertTrue(result['ok'])
        self.assertEqual(sum(action is None for action in client.actions), 1)
        self.assertEqual(client.reads, 1 + len(client.actions) + len(result['corrections']))
        # Explicit selection matters: default mode must not silently weaken a
        # health check or invent a fresh snapshot when raw feedback is missing.
        default = LegacyRawStepPlant()
        with self.assertRaisesRegex(RuntimeError, 'missing or faulty health'):
            self.move(default)
        self.assertEqual(default.reads, 1)
        self.assertEqual(default.actions, [None])

    def test_read_after_step_timeout_does_not_replay_sync_or_motion(self):
        for fail_read_at, expected_actions in ((2, 1), (3, 2)):
            with self.subTest(fail_read_at=fail_read_at):
                client = LegacyRawStepPlant()
                client.fail_read_at = fail_read_at
                with self.assertRaises(TimeoutError) as caught:
                    self.move(client, feedback_mode='read_after_step')
                self.assertIs(caught.exception, client.read_error)
                self.assertEqual(len(client.actions), expected_actions)
                self.assertEqual(client.reads, fail_read_at)

    def test_read_after_step_concurrent_command_is_not_mistaken_for_ack(self):
        for count_shift in (-1, 1):
            with self.subTest(count_shift=count_shift):
                client = LegacyRawStepPlant()
                def concurrent(snapshot):
                    if client.reads == 3:
                        snapshot['command_target_state']['step_count'] += count_shift
                client.read_mutator = concurrent
                with self.assertRaisesRegex(RuntimeError, 'step_count is not continuous'):
                    self.move(client, feedback_mode='read_after_step')
                self.assertEqual(len(client.actions), 2)
                self.assertEqual(client.reads, 3)

    def test_failed_raw_reply_never_gets_hidden_by_a_later_health_read(self):
        for phase in ('sync', 'motion'):
            for bad in ({'ok': False}, {'error': 'fault'}, {'info': {'error': 'fault'}},
                        {'done': True}, {'step_count': True}, {'step_count': 9000}):
                with self.subTest(phase=phase, bad=bad):
                    client = LegacyRawStepPlant()
                    def corrupt(reply, action):
                        if (action is None) == (phase == 'sync'):
                            reply.update(bad)
                    client.raw_reply_mutator = corrupt
                    with self.assertRaises(RuntimeError):
                        self.move(client, feedback_mode='read_after_step')
                    self.assertEqual(len(client.actions), 1 if phase == 'sync' else 2)
                    self.assertEqual(client.reads, 1 if phase == 'sync' else 2)

    def test_sync_health_read_must_pass_before_first_trajectory_command(self):
        client = LegacyRawStepPlant()
        def unhealthy(snapshot):
            if client.reads == 2:
                snapshot['right_arm'].pop('robot_state')
        client.read_mutator = unhealthy
        with self.assertRaisesRegex(RuntimeError, 'missing or faulty health'):
            self.move(client, feedback_mode='read_after_step')
        self.assertEqual(client.actions, [None])
        self.assertEqual(client.reads, 2)

    def test_read_after_step_is_explicit_guarded_and_smooth_only(self):
        for options in ({'feedback_mode': 'auto'},
                        {'feedback_mode': 'read_after_step', 'feedback_guard': None, 'stall_policy': 'stop'},
                        {'feedback_mode': 'read_after_step', 'smooth': False}):
            with self.subTest(options=options):
                client = LegacyRawStepPlant()
                with self.assertRaises(ValueError):
                    self.move(client, **options)
                self.assertEqual(client.actions, [])
                self.assertEqual(client.reads, 0)

    def test_sync_measurement_drift_cannot_shift_nominal_command_endpoint(self):
        class SyncMeasurementDrift(SavedTargetPlant):
            def step(self, action=None):
                reply = super().step(action)
                if action is None:
                    for side in SIDES:
                        self.actual[side] = rpc._pose_from_delta(
                            self.actual[side], [.0004, -.0003, .0002, .002, -.001, .003])
                    reply['observation'] = self.snapshot('sync')
                return reply

        for mode in ('step_reply', 'read_after_step'):
            with self.subTest(mode=mode):
                client = SyncMeasurementDrift(bias=[0.] * 6)
                left_goal = client.actual['left_arm'].copy()
                result = self.move(client, feedback_mode=mode, max_correction_iters=0)
                self.assertTrue(result['ok'])
                for side, goal in (('left_arm', left_goal), ('right_arm', [.01, -.38, 0., 0., 0., 0.])):
                    error = rpc._motion_norms(rpc._absolute_target_to_delta(goal, client.target[side]))
                    self.assertLess(error[0], 1e-10)
                    self.assertLess(error[1], 1e-10)
                self.assertEqual(result['final_error']['correction_index'], 0)
                self.assertEqual(sum(action is None for action in client.actions), 1)

    def test_delta_goal_remains_relative_to_sync_measurement_not_saved_target(self):
        class SyncMeasurementDrift(SavedTargetPlant):
            def step(self, action=None):
                reply = super().step(action)
                if action is None:
                    self.actual['right_arm'] = rpc._pose_from_delta(
                        self.actual['right_arm'], [.0004, 0., 0., 0., 0., .003])
                    reply['observation'] = self.snapshot('sync')
                return reply

        client = SyncMeasurementDrift(bias=[0.] * 6)
        with redirect_stderr(io.StringIO()):
            result = client.dual_robot_move_to_ee_pose(
                [0.] * 6, [.01, 0., 0., 0., 0., .02], delta=True, smooth=True, sleep=False,
                stall_policy='warn', feedback_mode='read_after_step', feedback_guard=require_health,
                max_correction_iters=0)
        self.assertTrue(result['ok'])
        self.assertAlmostEqual(client.target['right_arm'][0], .0104)
        self.assertAlmostEqual(client.target['right_arm'][5], .023)


if __name__ == '__main__':
    unittest.main()
