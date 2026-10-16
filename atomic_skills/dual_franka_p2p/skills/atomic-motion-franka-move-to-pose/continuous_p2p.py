"""Opt-in continuous P2P transport adapter; no reset, reanchor, replay or gripper RPC.

The fixed nominal goal is measured in torso coordinates. Distant goals use a
quintic command trajectory from the EXISTING saved target, followed immediately
by the bounded slow outer servo. The envelope is a local geometric bound, not a
collision-free path proof. Contact insertion must retain its separate force
policy. Resource ownership and independent ROS receipt checks belong to callers.
"""
from __future__ import annotations

from collections import deque
from dataclasses import asdict
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time


def _neighbor(filename, name):
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(filename))
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


geometry = _neighbor('continuous_pose_servo.py', '_franka_atomic_continuous_geometry')
legacy = _neighbor('dual_franka_robotiq_rpc_client.py', '_franka_atomic_p2p_geometry')
Pose = geometry.Pose
SIDES = ('left_arm', 'right_arm')
SERVO_BUDGET_S = 30.
MAX_SETTLE_REACQUISITIONS = 3
REACQUISITION_ERROR_RATIO = 2.
TRANSLATION_LIMIT_ENV = 'FRANKA_P2P_TRANSLATION_BIAS_LIMIT_M'
NATIVE_ERROR_FIELDS = frozenset('''joint_position_limits_violation cartesian_position_limits_violation
self_collision_avoidance_violation joint_velocity_violation cartesian_velocity_violation
force_control_safety_violation joint_reflex cartesian_reflex max_goal_pose_deviation_violation
max_path_pose_deviation_violation cartesian_velocity_profile_safety_violation
joint_position_motion_generator_start_pose_invalid joint_motion_generator_position_limits_violation
joint_motion_generator_velocity_limits_violation joint_motion_generator_velocity_discontinuity
joint_motion_generator_acceleration_discontinuity cartesian_position_motion_generator_start_pose_invalid
cartesian_motion_generator_elbow_limit_violation cartesian_motion_generator_velocity_limits_violation
cartesian_motion_generator_velocity_discontinuity cartesian_motion_generator_acceleration_discontinuity
cartesian_motion_generator_elbow_sign_inconsistent cartesian_motion_generator_start_elbow_invalid
cartesian_motion_generator_joint_position_limits_violation cartesian_motion_generator_joint_velocity_limits_violation
cartesian_motion_generator_joint_velocity_discontinuity cartesian_motion_generator_joint_acceleration_discontinuity
cartesian_position_motion_generator_invalid_frame force_controller_desired_force_tolerance_violation
controller_torque_discontinuity start_elbow_sign_inconsistent communication_constraints_violation
power_limit_violation joint_p2p_insufficient_torque_for_planning tau_j_range_violation instability_detected'''.split())


class ContinuousP2PError(RuntimeError):
    """A failed/uncertain continuous run must not be retried as a tolerance miss."""


def _emit(event, **data):
    print(json.dumps({'event': event, 'controller': 'continuous_servo', **data},
                     ensure_ascii=False, allow_nan=False), file=sys.stderr, flush=True)


def _vector(value, size, name):
    return geometry._finite_vector(value, size, name)


def _pose6(value):
    value = _vector(value, 6, 'torso pose')
    return Pose(value[:3], geometry._quat_from_rotvec(value[3:]))


def _six(value):
    return list(value.position_m) + legacy._quat_to_rotvec(value.quaternion_xyzw)


def _error(actual, goal):
    return geometry._error(actual, goal)


def _same(actual, expected):
    error = _error(actual, expected)
    return error['translation_m'] <= 1e-6 and error['rotation_rad'] <= 1e-6


def _positive(value, name):
    return geometry._positive(value, name)


class _Run:
    def __init__(self, client, left, right, delta, feedback_guard, correction_guard, options):
        self.client, self.inputs, self.delta = client, dict(zip(SIDES, (left, right))), delta
        self.feedback_guard, self.correction_guard = feedback_guard, correction_guard
        self.options = options
        if options.get('sleep', True) is not True:
            raise ValueError('Continuous P2P requires real timed dispatch; sleep=False is unsupported')
        for guard in (feedback_guard, correction_guard):
            if guard is not None and not callable(guard):
                raise ValueError('Continuous P2P guards must be callable')
        self.rate = min(20., _positive(options.get('rate_hz', 50.), 'rate_hz'))
        self.period = 1. / self.rate
        if self.period >= .2:
            raise ValueError('Continuous P2P rate_hz must exceed 5 Hz to leave feedback timing margin')
        self.max_cycle = min(.2, 2. * self.period)
        self.max_steps = options.get('max_steps', 3000)
        if type(self.max_steps) is not int or self.max_steps <= 0:
            raise ValueError('max_steps must be a positive integer')
        translation_limit = options.get('max_tracking_translation_bias_m', .025)
        env_limit = os.environ.get(TRANSLATION_LIMIT_ENV)
        if env_limit is not None:
            if env_limit != 'none':
                raise ValueError(f'{TRANSLATION_LIMIT_ENV} must be absent or exactly "none"')
            translation_limit = None
        if translation_limit is not None:
            translation_limit = _positive(translation_limit, 'translation bias')
        self.translation_limit_source = 'environment' if env_limit is not None else 'argument_or_default'
        # This selects the approach profile by physical distance. It never
        # limits compensation, clips a target or resets the saved command.
        self.nominal_approach_distance_m = _positive(options.get('nominal_approach_distance_m', .025),
                                                    'nominal approach distance')
        self.config = geometry.ServoConfig(
            translation_integral_gain_per_s=.3, rotation_integral_gain_per_s=.3,
            max_translation_speed_m_s=min(.002, _positive(options.get('max_translation_speed', .05), 'translation speed')),
            max_rotation_speed_rad_s=min(.01, _positive(options.get('max_rotation_speed', .4), 'rotation speed')),
            max_translation_step_m=self._step('max_translation_step', .00025),
            max_rotation_step_rad=self._step('max_rotation_step', .002),
            max_translation_acceleration_m_s2=.005, max_rotation_acceleration_rad_s2=.02,
            max_translation_bias_m=translation_limit,
            max_rotation_bias_rad=min(.2, _positive(options.get('max_tracking_rotation_bias_rad', .2), 'rotation bias')),
            position_tolerance_m=min(.003, _positive(options.get('position_tolerance_m', .0015), 'position tolerance')),
            rotation_tolerance_rad=min(.1, _positive(options.get('rotation_tolerance_rad', .01), 'rotation tolerance')),
            max_dt_s=.2)
        self.phase, self.session, self.count = 'preflight', None, None
        self.saved, self.initial, self.goals, self.bounds, self.held = {}, {}, {}, {}, set()
        self.measured = {}
        self.nominal_targets = {}
        self.sequences, self.stamps = {}, {}
        self.commands = self.attempts = 0
        self.last_step, self.last_dispatch = None, None
        self.last_control_tick = None
        self.cap_wait_sides = set()
        self.settle_reacquisitions = 0
        self.timings, self.progress = [], []
        self.last_log = -math.inf

    def _step(self, key, limit):
        value = self.options.get(key)
        return limit if value is None else min(limit, _positive(value, key))

    def within(self, measured, side):
        error = _error(measured, self.goals[side])
        return (error['translation_m'] <= self.config.position_tolerance_m
                and error['rotation_rad'] <= self.config.rotation_tolerance_rad)

    def snapshot(self, *, expected=None, count=None):
        read_started = time.monotonic()
        observation = self.client.get_observation()
        self.feedback_received = time.monotonic()
        if self.feedback_received - read_started > self.max_cycle:
            raise ContinuousP2PError('Native observation read exceeded cycle limit')
        if not isinstance(observation, dict):
            raise ContinuousP2PError('Missing complete native observation')
        metadata = observation.get('rpc_metadata', {})
        if (metadata.get('protocol') != 'aoe.franka-rpc.telemetry.v1'
                or any(metadata.get(k) is not True for k in ('legacy_motion_enabled',
                    'native_snapshot_available', 'source_bound_torso_pose_available'))
                or metadata.get('diagnostics_errors') != {}):
            raise ContinuousP2PError('Reviewed native diagnostic protocol is unavailable')
        state = observation.get('command_target_state', {})
        session, actual_count = state.get('server_session_id'), state.get('step_count')
        if (state.get('available') is not True or not isinstance(session, str) or not session
                or type(actual_count) is not int or actual_count < 0
                or (self.session is not None and session != self.session)
                or (count is not None and actual_count != count)):
            raise ContinuousP2PError('Saved command session/count is missing or changed')
        measured, saved, velocities = {}, {}, {}
        for side in SIDES:
            data = observation.get(side, {})
            native = data.get('native_robot_state', {})
            errors = native.get('current_errors', {})
            if (native.get('diagnostics_complete') is not True
                    or native.get('diagnostics_schema') != 'libfranka-errors/v1'
                    or not isinstance(errors, dict) or not NATIVE_ERROR_FIELDS.issubset(errors)
                    or any(v is not False for v in errors.values())
                    or type(native.get('robot_mode')) is not int
                    or (native.get('robot_mode'), native.get('robot_mode_name')) not in ((1, 'idle'), (2, 'move'))
                    or native.get('sensor_clock_progressing') is not True):
                raise ContinuousP2PError(f'{side}: incomplete native diagnostics or fault/mode change')
            age = native.get('age_ns')
            if type(age) not in (int, float) or not math.isfinite(age) or not 0 <= age <= 250_000_000:
                raise ContinuousP2PError(f'{side}: stale or invalid native age')
            sequence, stamp = native.get('sequence'), native.get('source_ros_timestamp_ns')
            if (type(sequence) is not int or sequence <= self.sequences.get(side, 0)
                    or type(stamp) is not int or stamp <= self.stamps.get(side, 0)):
                raise ContinuousP2PError(f'{side}: native sequence/timestamp did not advance')
            _vector(native.get('joint_positions'), 7, f'{side} joint positions')
            _vector(native.get('joint_efforts'), 7, f'{side} joint efforts')
            velocities[side] = _vector(native.get('joint_velocities'), 7, f'{side} joint velocities')
            frame = side.removesuffix('_arm') + '_base'
            torso = native.get('torso_eef_pose', {})
            if (torso.get('schema') != 'aoe.franka-source-bound-pose.v1'
                    or torso.get('frame') != 'torso' or torso.get('native_frame') != frame
                    or type(torso.get('source_sequence')) is not int or torso['source_sequence'] != sequence
                    or type(torso.get('source_ros_timestamp_ns')) is not int or torso['source_ros_timestamp_ns'] != stamp):
                raise ContinuousP2PError(f'{side}: torso pose is not bound to the fresh native sample')
            measured[side] = Pose(_vector(torso.get('position'), 3, 'native position'),
                                  _vector(torso.get('orientation_xyzw'), 4, 'native quaternion'))
            wrench = native.get('wrench', {})
            if wrench.get('frame') != frame:
                raise ContinuousP2PError(f'{side}: unexpected wrench frame')
            force = _vector(wrench.get('force_n'), 3, 'force')
            torque = _vector(wrench.get('torque_nm'), 3, 'torque')
            if self.phase in ('servo', 'hold') and (max(map(abs, velocities[side])) > .15
                    or math.hypot(*force) > 10. or math.hypot(*torque) > 3.):
                raise ContinuousP2PError(f'{side}: servo joint speed or wrench limit exceeded')
            # Some servers expose these only in the legacy diagnostic view.
            for owner in (native, data.get('robot_state', {})):
                for key in ('joint_collision', 'cartesian_collision', 'joint_collisions', 'cartesian_collisions'):
                    if key in owner:
                        flags = owner[key]
                        if not isinstance(flags, (list, tuple)) or not flags or any(
                                type(v) not in (int, float, bool) or not math.isfinite(v) or v != 0 for v in flags):
                            raise ContinuousP2PError(f'{side}: collision flag or invalid collision telemetry')
            command = state.get('arms', {}).get(side, {})
            if command.get('frame') != 'torso':
                raise ContinuousP2PError(f'{side}: saved command frame is not torso')
            saved[side] = Pose(_vector(command.get('position_m'), 3, 'saved position'),
                              _vector(command.get('quaternion_xyzw'), 4, 'saved quaternion'))
            if expected is not None and not _same(saved[side], expected[side]):
                raise ContinuousP2PError(f'{side}: saved target differs from the acknowledged proposal')
            if self.bounds:
                for value in (measured[side], saved[side]):
                    self.check_bounds(side, value)
                if side in self.held and not self.within(measured[side], side):
                    raise ContinuousP2PError(f'{side}: held arm left its fixed arrival tolerance')
        if self.feedback_guard is not None:
            self.feedback_guard(observation)
        self.session, self.count = session, actual_count
        self.sequences = {s: observation[s]['native_robot_state']['sequence'] for s in SIDES}
        self.stamps = {s: observation[s]['native_robot_state']['source_ros_timestamp_ns'] for s in SIDES}
        self.saved, self.measured, self.velocities = saved, measured, velocities
        self.observation = observation
        return observation

    def check_bounds(self, side, value):
        bound = self.bounds[side]
        if any(not low - 1e-12 <= v <= high + 1e-12
               for v, low, high in zip(value.position_m, bound['low'], bound['high'])):
            raise ContinuousP2PError(f'{side}: fixed position corridor exceeded')
        if _error(self.initial[side], value)['rotation_rad'] > bound['rotation_from_initial_rad'] + 1e-12:
            raise ContinuousP2PError(f'{side}: fixed orientation corridor exceeded')

    def prepare(self):
        self.snapshot()
        self.initial = dict(self.measured)
        self.goals = {side: _pose6(legacy._pose_from_delta(_six(self.initial[side]), self.inputs[side]))
                      if self.delta else _pose6(self.inputs[side]) for side in SIDES}
        for side in SIDES:
            if self.within(self.initial[side], side):
                if max(map(abs, self.velocities[side])) >= .01:
                    raise ContinuousP2PError(f'{side}: already-at-goal arm is not stationary; saved compensation preserved')
                self.held.add(side)
            self.nominal_targets[side] = self.nominal_target(side)
            poses = (self.initial[side], self.saved[side], self.goals[side], self.nominal_targets[side])
            cap = self.config.max_translation_bias_m
            margin = (self.config.max_translation_speed_m_s * (SERVO_BUDGET_S + self.config.max_dt_s)
                      if cap is None else cap)
            self.bounds[side] = {
                'low': [min(p.position_m[i] for p in poses) - margin for i in range(3)],
                'high': [max(p.position_m[i] for p in poses) + margin for i in range(3)],
                'position_margin_m': margin,
                'position_margin_basis': 'servo_speed_times_budget_plus_max_dt' if cap is None else 'translation_bias_cap',
                'workspace_authorization_bound': False,
                'rotation_from_initial_rad': min(math.pi, max(_error(self.initial[side], p)['rotation_rad']
                                                              for p in poses) + self.config.max_rotation_bias_rad)}
        self.start_count = self.count
        self.initial_saved = dict(self.saved)
        _emit('p2p_servo_start', goals={s: _six(p) for s, p in self.goals.items()},
              held_sides=sorted(self.held), bounds=self.bounds, servo_config=asdict(self.config),
              period_s=self.period, max_cycle_s=self.max_cycle, servo_budget_s=SERVO_BUDGET_S,
              translation_limit_source=self.translation_limit_source,
              nominal_approach_distance_m=self.nominal_approach_distance_m,
              nominal_targets={s: _six(p) for s, p in self.nominal_targets.items()},
              correction_guard_policy='final_success_only',
              legacy_corrections_and_settle_replaced=True, safety_qualified=False)

    def scheduled_dt(self, *, deadline=None):
        # The controller also advances through read-only cap waits. Its real
        # observation/control interval is separate from the last command time.
        reference = self.last_control_tick if self.phase == 'servo' else self.last_dispatch
        earliest = max((reference + self.period if reference is not None else -math.inf),
                       (self.last_dispatch + self.period if self.last_dispatch is not None else -math.inf))
        time.sleep(max(0., earliest - time.monotonic()))
        now = time.monotonic()
        dt = self.period if reference is None else now - reference
        if dt > self.max_cycle + 1e-9:
            raise ContinuousP2PError('Feedback cycle too slow; no catch-up command')
        if deadline is not None:
            self.check_servo_deadline(deadline)
        if self.phase == 'servo':
            self.last_control_tick = now
        return dt

    def check_servo_deadline(self, deadline):
        if time.monotonic() >= deadline:
            bias = {s: _error(self.goals[s], self.saved[s]) for s in SIDES}
            detail = {'final_error': self.error_report(),
                      'bias': bias,
                      'at_cap': {s: {'translation': self.config.max_translation_bias_m is not None
                                     and b['translation_m'] >= self.config.max_translation_bias_m - 1e-9,
                                     'rotation': b['rotation_rad'] >= self.config.max_rotation_bias_rad - 1e-9}
                                 for s, b in bias.items()},
                      'translation_cap_m': self.config.max_translation_bias_m,
                      'rotation_cap_rad': self.config.max_rotation_bias_rad,
                      'cap_wait_sides': sorted(self.cap_wait_sides)}
            raise ContinuousP2PError('Continuous 30-second servo budget exhausted; residual=' +
                                     json.dumps(detail, allow_nan=False))

    def note_cap_wait(self, side, update):
        if side not in self.cap_wait_sides:
            self.cap_wait_sides.add(side)
            _emit('p2p_servo_cap_warning', side=side, errors=update.errors, bias=update.bias,
                  at_cap=update.at_cap, command_preserved=True,
                  message='Bounded command stopped before arrival; read fresh feedback within original budget')

    def dispatch(self, desired, *, dt, deadline=None):
        if self.commands >= self.max_steps:
            raise ContinuousP2PError('Continuous P2P max_steps budget exhausted')
        action = {}
        for side in SIDES:
            self.check_bounds(side, desired[side])
            increment = ([0.] * 6 if side in self.held else
                         legacy._absolute_target_to_delta(_six(self.saved[side]), _six(desired[side])))
            action[side] = legacy._motion_action_from_delta(increment)
        # Never catch up after slow computation or read-only waiting. Waiting at
        # a cap can legitimately make actual_send_interval_s large; the fresh
        # controller cycle and each individual RPC still have strict deadlines.
        if self.last_dispatch is not None:
            time.sleep(max(0., self.last_dispatch + self.period - time.monotonic()))
        before = time.monotonic()
        interval = None if self.last_dispatch is None else before - self.last_dispatch
        elapsed_cycle = (dt + before - self.last_control_tick if self.phase == 'servo'
                         and self.last_control_tick is not None else interval)
        if ((elapsed_cycle is not None and elapsed_cycle > self.max_cycle + 1e-9)
                or before - self.feedback_received > self.max_cycle):
            raise ContinuousP2PError('Dispatch deadline exceeded before RPC; no catch-up command')
        if deadline is not None:
            self.check_servo_deadline(deadline)
        expected_count = self.count + 1
        self.attempts += 1
        response = self.client.step(action)
        ack = time.monotonic()
        self.last_dispatch = before
        if (not isinstance(response, dict) or response.get('ok') is False or response.get('error')
                or response.get('done') is True or not isinstance(response.get('info', {}), dict)
                or response.get('info', {}).get('error') or type(response.get('step_count')) is not int
                or response['step_count'] != expected_count):
            raise ContinuousP2PError('Unconfirmed raw step acknowledgement; no replay or further command')
        self.snapshot(expected=desired, count=expected_count)
        self.commands += 1
        self.last_step = response  # Preserve the actual raw ACK, including absence of `ok`.
        finished = time.monotonic()
        timing = {'phase': self.phase, 'step_count': self.count, 'control_dt_s': dt,
                  'actual_send_interval_s': interval, 'rpc_s': ack - before,
                  'ack_and_feedback_s': finished - before}
        self.timings.append(timing)
        if finished - before > self.max_cycle:
            raise ContinuousP2PError('RPC plus native feedback exceeded cycle limit; no further command')
        return timing

    def nominal_target(self, side):
        """Freeze a target while keeping compensation and nominal goal distinct."""
        saved, goal = self.saved[side], self.goals[side]
        if side in self.held:
            return saved
        error = _error(saved, goal)
        cap = self.config.max_translation_bias_m
        position = saved.position_m
        if cap is None:
            if _error(self.initial[side], goal)['translation_m'] > self.nominal_approach_distance_m:
                # Carry the existing world-frame bias to the new physical goal.
                # This changes only the nominal command trajectory endpoint;
                # the measured arrival goal remains exactly the caller's goal.
                position = tuple(g + c - m for g, c, m in
                                 zip(goal.position_m, saved.position_m, self.initial[side].position_m))
        elif error['translation_m'] > cap + geometry._ROUNDING:
            position = goal.position_m
        rotation = (goal.quaternion_xyzw
                    if error['rotation_rad'] > self.config.max_rotation_bias_rad + geometry._ROUNDING
                    else saved.quaternion_xyzw)
        return Pose(position, rotation)

    def nominal(self):
        targets = dict(self.nominal_targets)
        translation = max(_error(self.saved[s], targets[s])['translation_m'] for s in SIDES)
        rotation = max(_error(self.saved[s], targets[s])['rotation_rad'] for s in SIDES)
        if translation <= 1e-12 and rotation <= 1e-12:
            return {'steps': 0, 'profile': 'existing_saved_command_preserved'}
        self.phase = 'nominal'
        # An explicit short duration never overrides the caller's velocity limit.
        velocity = _positive(self.options.get('max_translation_speed', .05), 'translation speed')
        angular = _positive(self.options.get('max_rotation_speed', .4), 'rotation speed')
        duration = max(1.875 * translation / velocity, 1.875 * rotation / angular,
                       float(self.options.get('min_duration_sec', .5)),
                       _positive(self.options['duration_sec'], 'duration') if self.options.get('duration_sec') is not None else 0.)
        waypoints, metadata = legacy._plan_smooth_absolute_trajectory(
            *(_six(self.saved[s]) for s in SIDES), *(_six(targets[s]) for s in SIDES),
            duration_sec=duration, rate_hz=self.rate, max_translation_speed=velocity,
            max_rotation_speed=angular, max_translation_step=self.options.get('max_translation_step', .003),
            max_rotation_step=self.options.get('max_rotation_step', .025),
            min_duration_sec=self.options.get('min_duration_sec', .5), max_steps=self.max_steps)
        for waypoint in waypoints:
            dt = self.scheduled_dt()
            desired = {s: _pose6(waypoint[s.removesuffix('_arm') + '_pose']) for s in SIDES}
            self.dispatch(desired, dt=dt)
            self.log_progress()
        return metadata

    def error_report(self):
        return legacy._pose_error_report(*(_six(self.measured[s]) for s in SIDES),
                                         *(_six(self.goals[s]) for s in SIDES))

    def log_progress(self, updates=None):
        now = time.monotonic()
        if now - self.last_log >= .25:
            report = {'phase': self.phase, 'step_count': self.count, 'commands': self.commands,
                      'elapsed_monotonic_s': now - self.started, 'final_error': self.error_report(),
                      'bias': {s: _error(self.goals[s], self.saved[s]) for s in SIDES}}
            if updates:
                report['servo'] = {s: u.to_dict() for s, u in updates.items()}
            self.progress.append(report)
            _emit('p2p_servo_progress', **report)
            self.last_log = now

    def reacquire(self, servos, sides, deadline, *, pending=None):
        """Resume this closed loop after small verified drift, never replay P2P."""
        self.check_servo_deadline(deadline)
        if not sides or any(s not in servos for s in sides):
            raise ContinuousP2PError('Arrival drift affects a held arm; no reacquisition')
        for side in sides:
            error = _error(self.measured[side], self.goals[side])
            # This is a recovery envelope, never an expanded arrival tolerance.
            if (error['translation_m'] > REACQUISITION_ERROR_RATIO * self.config.position_tolerance_m
                    or error['rotation_rad'] > REACQUISITION_ERROR_RATIO * self.config.rotation_tolerance_rad):
                raise ContinuousP2PError(f'{side}: arrival drift exceeds bounded reacquisition envelope')
        if self.settle_reacquisitions >= MAX_SETTLE_REACQUISITIONS:
            raise ContinuousP2PError(f'Arrival did not stabilize within {MAX_SETTLE_REACQUISITIONS} settle reacquisitions')
        for side in sides:
            if pending is not None:
                servos[side].confirm_command(self.saved[side])
            servos[side].resume_after_settle_drift()
        self.settle_reacquisitions += 1
        _emit('p2p_servo_reacquire', phase=self.phase, sides=sides,
              attempt=self.settle_reacquisitions, max_attempts=MAX_SETTLE_REACQUISITIONS,
              final_error=self.error_report(), remaining_servo_s=deadline - time.monotonic(),
              goal_preserved=True, saved_compensation_preserved=True, step_count=self.count)

    def hold(self, *, overall_deadline=None):
        self.phase = 'hold'
        deadline, window = time.monotonic() + 6., deque(maxlen=3)
        if overall_deadline is not None:
            deadline = min(deadline, overall_deadline)
        held = dict(self.saved)

        def check_deadline():
            if time.monotonic() >= deadline:
                raise ContinuousP2PError('Stopped-command hold did not settle within 6 seconds')

        while time.monotonic() < deadline:
            self.snapshot(expected=held, count=self.count)
            check_deadline()
            if not all(self.within(self.measured[s], s) for s in SIDES):
                return None  # Fresh, validated drift; caller decides whether to reacquire.
            window.append((time.monotonic(), dict(self.measured), dict(self.velocities)))
            if (len(window) == 3 and window[-1][0] - window[0][0] >= .5
                    and all(max(abs(v) for sample in window for v in sample[2][s]) < .01
                            and max(math.dist(a[1][s].position_m, b[1][s].position_m)
                                    for a in window for b in window) <= .00015 for s in SIDES)):
                # A fourth, independent fresh read must still match the held count,
                # command, native stationarity and physical goal.
                time.sleep(self.period)
                check_deadline()
                self.snapshot(expected=held, count=self.count)
                check_deadline()
                if not all(self.within(self.measured[s], s) for s in SIDES):
                    return None
                if (any(max(map(abs, self.velocities[s])) >= .01
                               or any(math.dist(self.measured[s].position_m, sample[1][s].position_m) > .00015
                                      for sample in window) for s in SIDES)):
                    raise ContinuousP2PError('Independent final arrival/stationarity check failed')
                confirmation = [{'monotonic_s': stamp, 'poses': {s: _six(p) for s, p in poses.items()}}
                                for stamp, poses, _ in window]
                check_deadline()
                return confirmation
            time.sleep(min(.25, max(0., deadline - time.monotonic())))
        raise ContinuousP2PError('Stopped-command hold did not settle within 6 seconds')

    def execute(self):
        self.started = time.monotonic()
        self.prepare()
        nominal_metadata = self.nominal()
        self.phase = 'servo'
        # Recheck the stricter slow-servo limits before sending any correction.
        time.sleep(self.period if self.last_dispatch is None else
                   max(0., self.last_dispatch + self.period - time.monotonic()))
        self.snapshot(expected=self.saved, count=self.count)
        servos = {s: geometry.ContinuousPoseServo(self.goals[s], self.saved[s], self.config)
                  for s in SIDES if s not in self.held}
        deadline = time.monotonic() + SERVO_BUDGET_S
        while True:
            dt = self.scheduled_dt(deadline=deadline)
            updates = {s: servo.update(self.measured[s], dt) for s, servo in servos.items()}
            drifted = [s for s, u in updates.items()
                       if u.command_stationary and u.arrival_latched and not u.within_tolerance]
            if drifted:
                self.reacquire(servos, drifted, deadline, pending=updates)
                for side in drifted:
                    updates[side] = servos[side].update(self.measured[side], dt)
            for side, update in updates.items():
                if update.command_stationary and not update.within_tolerance:
                    if update.arrival_latched:
                        raise ContinuousP2PError(f'{side}: stopped arm lost arrival; no restart')
                    if not any(update.at_cap.values()):
                        raise ContinuousP2PError(f'{side}: no feasible progress before arrival outside a cap')
                    self.note_cap_wait(side, update)
            if all(update.command_stationary for update in updates.values()):
                for side, update in updates.items():
                    servos[side].confirm_command(self.saved[side])
                if all(u.within_tolerance and u.arrival_latched for u in updates.values()):
                    confirmation = self.hold(overall_deadline=deadline + 6.)
                    if confirmation is not None:
                        break
                    drifted = [s for s in SIDES if not self.within(self.measured[s], s)]
                    self.reacquire(servos, drifted, deadline)
                    self.phase = 'servo'
                    # Hold reads are not servo integration intervals. Start a
                    # fresh timed cycle using the just-validated observation.
                    self.last_control_tick = None
                    continue
                # Keep the last checked saved targets. Do not send zero steps,
                # reset the integral, or renew the original 30-second budget.
                self.log_progress(updates)
                time.sleep(max(0., min(self.last_control_tick + self.period, deadline) - time.monotonic()))
                self.check_servo_deadline(deadline)
                self.snapshot(expected=self.saved, count=self.count)
                self.check_servo_deadline(deadline)
                continue
            desired = {s: updates[s].command if s in updates else self.saved[s] for s in SIDES}
            self.dispatch(desired, dt=dt, deadline=deadline)
            for side in servos:
                servos[side].confirm_command(self.saved[side])
            self.log_progress(updates)
        final_error = self.error_report()
        # The legacy progress callback receives exactly one final report, not
        # a fabricated sequence of discrete continuous-servo correction rounds.
        final_error['correction_index'] = 0
        if self.correction_guard is not None:
            self.correction_guard(final_error)
        result = {'ok': True, 'mode': 'continuous_servo', 'controller': 'continuous_servo',
                  'final_error': final_error, 'last_step': self.last_step, 'stalled': None,
                  'trajectory': nominal_metadata, 'corrections': [], 'continuous_progress': self.progress,
                  'commands': self.commands, 'dispatch_attempts': self.attempts,
                  'initial_step_count': self.start_count, 'final_step_count': self.count,
                  'server_session_id': self.session, 'timings': self.timings,
                  'held_sides': sorted(self.held), 'hold_confirmation': confirmation,
                  'settle_reacquisitions': self.settle_reacquisitions,
                  'bounds': self.bounds, 'frame': 'torso', 'servo_config': asdict(self.config),
                  'translation_limit_source': self.translation_limit_source,
                  'nominal_approach_distance_m': self.nominal_approach_distance_m,
                  'tolerances': {'position_tolerance_m': self.config.position_tolerance_m,
                                 'rotation_tolerance_rad': self.config.rotation_tolerance_rad,
                                 'settle_time_sec': 0., 'max_correction_iters': 0},
                  'correction_guard_policy': 'final_success_only',
                  'physical_stop_ack': False, 'ros_receipt_verified': False}
        _emit('p2p_servo_arrived', commands=self.commands, final_error=final_error,
              step_count=self.count, hold_samples=3)
        return result


def run(client, left_pose, right_pose, *, delta=False, feedback_guard=None, correction_guard=None, **kwargs):
    """Run one synchronous opt-in P2P; every failure raises, with no replay."""
    runner = _Run(client, left_pose, right_pose, delta, feedback_guard, correction_guard, kwargs)
    try:
        return runner.execute()
    except Exception as error:
        # Keep execution uncertainty separate from physical arrival failure.
        detail = (f'Continuous P2P failed in {runner.phase}: {error}; '
                  f'confirmed_commands={runner.commands}, dispatch_attempts={runner.attempts}; no automatic retry')
        _emit('p2p_servo_failed', phase=runner.phase, message=str(error),
              confirmed_commands=runner.commands, dispatch_attempts=runner.attempts,
              last_validated_error=runner.error_report() if all(s in runner.measured and s in runner.goals for s in SIDES) else None,
              settle_reacquisitions=runner.settle_reacquisitions)
        raise ContinuousP2PError(detail) from error
