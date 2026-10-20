"""Opt-in full-path free-space outer loop over the existing pose RPC.

Reuses the reviewed transport, telemetry, ownership, bounds and hold checks.
Never changes the RPC server, impedance parameters, raw step() or policy loop.
"""
from dataclasses import asdict, replace
import importlib.util
import os
from pathlib import Path
import sys


def _neighbor(filename, name):
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(filename))
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


base = _neighbor('continuous_p2p.py', '_franka_skill_continuous_p2p')
tracking = _neighbor('external_tracking.py', '_franka_external_tracking_math')
continuity = _neighbor('tracking_continuity.py', '_franka_tracking_continuity')
SIDES = base.SIDES


class TrackingRun(base._Run):
    controller = 'external_tracking'

    def settle_preflight(self):
        if all(max(map(abs, values)) < .01 for values in self.velocities.values()):
            return
        started = base.time.monotonic()
        count, saved = self.count, dict(self.saved)
        stable = 0
        while base.time.monotonic() - started < 5.:
            base.time.sleep(.1)
            # Passive settling only. Any new command, changed saved target,
            # stale feedback, session change or native fault aborts here.
            self.snapshot(expected=saved, count=count)
            stable = stable + 1 if all(max(map(abs, v)) < .01 for v in self.velocities.values()) else 0
            if stable >= 3:
                base._emit('p2p_preflight_settled', waited_s=base.time.monotonic()-started,
                           step_count=count, commands=0)
                return
        raise base.ContinuousP2PError('External tracking start did not settle within 5 seconds; no command sent')

    def initially_arrived(self, side):
        if self.profile.name != 'fast_tracking':
            return super().initially_arrived(side)
        error = base._error(self.initial[side], self.goals[side])
        # An accepted warning is not a reason to skip a requested correction.
        # Hold the passive arm only inside the nominal task tolerance, matching
        # the servo's capture criterion rather than its wider stop boundary.
        return (error['translation_m'] <= self.arrival_policy.get('precision_position_tolerance_m',
                                                                self.arrival_policy['position_tolerance_m'])
                and error['rotation_rad'] <= self.arrival_policy['rotation_tolerance_rad'])

    def check_bounds(self, side, value):
        if (getattr(self, 'profile', None) is not None and
                self.profile.endpoint_orientation_only and side not in self.held):
            bound = self.bounds[side]
            if any(not low - 1e-12 <= v <= high + 1e-12
                   for v, low, high in zip(value.position_m, bound['low'], bound['high'])):
                raise base.ContinuousP2PError(f'{side}: fixed position corridor exceeded')
            # The servo still bounds angular command speed, acceleration and
            # compensation. Actual attitude is constrained at final arrival.
            return
        return super().check_bounds(side, value)

    def nominal_target(self, side):
        # Used ONLY to establish the original fixed observation corridor.
        # Actual references are generated from measured pose during execution.
        if side in self.held:
            return self.saved[side]
        offset = base._error(self.initial[side], self.saved[side])
        return base.Pose(tracking.g._add(self.goals[side].position_m, offset['translation_vector_m']),
                         tracking.g._integrate(self.goals[side].quaternion_xyzw,
                                               offset['rotation_vector_rad'], True))

    def check_servo_deadline(self, deadline):
        if base.time.monotonic() >= deadline:
            detail = {s: {'path_fraction': servo.virtual_time / servo.duration,
                          'governor_scale': servo.governor_scale,
                          'tracking_error': servo.telemetry.get('tracking_error'),
                          'rotation_governor_rad': servo.rotation_governor_limits(),
                          'command_bias': base._error(servo.reference, self.saved[s])}
                      for s, servo in getattr(self, 'servos', {}).items()}
            raise base.ContinuousP2PError('External tracking budget exhausted; no reference or correction restart; '
                                          + base.json.dumps(detail, allow_nan=False))

    def log_progress(self, updates=None):
        now = base.time.monotonic()
        if now - self.last_log < .25:
            return
        report = {'phase': self.phase, 'step_count': self.count, 'commands': self.commands,
                  'elapsed_monotonic_s': now - self.started, 'final_error': self.error_report(),
                  'tracking': {s: servo.telemetry for s, servo in self.servos.items()}}
        if updates:
            report['servo'] = {s: u.to_dict() for s, u in updates.items()}
        self.progress.append(report)
        base._emit('p2p_servo_progress', controller=self.controller, **report)
        self.last_log = now

    def execute(self):
        profile = tracking.feedback_profile(os.environ.get('FRANKA_P2P_TRACKING_PROFILE', 'conservative'))
        self.profile = profile
        self.started = base.time.monotonic()
        self.prepare()
        if profile.endpoint_orientation_only:
            for side in SIDES:
                if side not in self.held:
                    self.bounds[side]['rotation_from_initial_rad'] = None
                    self.bounds[side]['orientation_constraint'] = 'endpoint_only'
        if any(max(map(abs, self.velocities[s])) >= .01 for s in SIDES):
            raise base.ContinuousP2PError('External tracking requires a stationary start')
        config = tracking.command_config(self.config,
            self.options.get('max_translation_speed', .05), self.options.get('max_rotation_speed', .4),
            self.options.get('max_translation_step') or .003,
            self.options.get('max_rotation_step') or .025, profile=profile.name, period_s=self.period)
        config = replace(config,
            translation_integral_gain_per_s=profile.translation_gain_per_s,
            rotation_integral_gain_per_s=profile.rotation_gain_per_s)
        minimum = max(base._positive(self.options.get('min_duration_sec', .5), 'minimum duration'),
                      base._positive(self.options['duration_sec'], 'duration')
                      if self.options.get('duration_sec') is not None else .5)
        path_rotation_tolerance = min(config.rotation_tolerance_rad, self.arrival_policy['rotation_tolerance_rad'])
        prior = continuity.read(self.client) if profile.name == 'fast_tracking' else {}
        reference_limit = continuity.reference_tolerance(prior, config, path_rotation_tolerance)
        references = (continuity.matching_references(prior,
            session=self.session, count=self.count, saved=self.saved, measured=self.measured,
            config=config, nominal_rotation=path_rotation_tolerance, geometry=tracking.g)
            if profile.name == 'fast_tracking' else {})
        starts = {s: (base.Pose(self.initial[s].position_m, references[s]) if s in references else
                      tracking.reference_start(self.initial[s], self.goals[s], config, profile)) for s in SIDES}
        duration = max(tracking.path_duration(
                           starts[s], self.goals[s], config, minimum=minimum)
                       for s in SIDES)
        budget = duration + base.SERVO_BUDGET_S
        self.servos = {s: tracking.TrackingServo(self.initial[s], self.goals[s], self.saved[s], config, duration,
                       profile=profile.name, path_rotation_tolerance_rad=path_rotation_tolerance,
                       nominal_position_tolerance_m=self.arrival_policy.get('precision_position_tolerance_m',
                                                                           self.arrival_policy['position_tolerance_m']),
                       confirmed_orientation_reference=references.get(s),
                       confirmed_rotation_tolerance_rad=reference_limit)
                       for s in SIDES if s not in self.held}
        rotation_limits = next(iter(self.servos.values())).rotation_governor_limits() if self.servos else (None, None)
        # An honest planning estimate, not a deadline or physical success claim.
        performance = {'target_s': 5., 'nominal_motion_s': duration,
                       'minimum_hold_s': .5, 'nominal_with_hold_s': duration + .5,
                       'nominal_exceeds_target': duration + .5 > 5.,
                       'step_and_rate_accounted': True}
        base._emit('p2p_tracking_config', controller=self.controller, nominal_duration_s=duration,
                   total_budget_s=budget, command_config=asdict(config),
                   performance=performance, wrench_policy=self.wrench_policy,
                   feedback_profile=asdict(profile),
                   feedback_translation_speed_m_s=profile.translation_speed_m_s,
                   feedback_rotation_speed_rad_s=profile.rotation_speed_rad_s,
                   governor={'position_slow_m': profile.position_slow_m, 'position_pause_m': profile.position_pause_m,
                             'rotation_slow_rad': rotation_limits[0],
                             'rotation_pause_rad': rotation_limits[1],
                             'orientation_constraint': 'endpoint_only' if profile.endpoint_orientation_only else 'whole_path',
                             'joint_speed_governor_enabled': False,
                             'joint_speed_stop_limit_rad_s': None},
                   impedance_parameters_changed=False, contact_policy=False)
        self.phase = 'servo'
        base.time.sleep(self.period)
        self.snapshot(expected=self.saved, count=self.count)
        deadline = base.time.monotonic() + budget
        warning_since = {}
        warning_captures = []
        while True:
            dt = self.scheduled_dt(deadline=deadline)
            # Both arms share path progress: a lagging arm slows the coordinated
            # reference rather than allowing the other arm to run ahead.
            scale = min((servo.tracking_scale(self.measured[s])
                         for s, servo in self.servos.items()), default=1.)
            within_all = all(self.within(self.measured[s], s) for s in SIDES)
            capture_allowed = (not (profile.endpoint_orientation_only or profile.precise_capture) or
                               all(max(map(abs, self.velocities[s])) < .01 for s in SIDES))
            warning_capture = {}
            for side, servo in self.servos.items():
                eligible = (within_all and capture_allowed
                            and servo.stationary_warning_candidate(self.measured[side]))
                if eligible:
                    warning_since.setdefault(side, base.time.monotonic())
                else:
                    warning_since.pop(side, None)
                warning_capture[side] = eligible and base.time.monotonic() - warning_since[side] >= .75
                if warning_capture[side]:
                    item = {'side':side, 'stationary_dwell_s':base.time.monotonic()-warning_since[side],
                            'error':base._error(self.measured[side], self.goals[side]),
                            'path_fraction':servo.virtual_time/servo.duration}
                    warning_captures.append(item)
                    base._emit('p2p_endpoint_warning_capture', **item)
            all_arrived = within_all and all(all(servo.capture_components(self.measured[s],
                allow_warning=warning_capture[s])) for s, servo in self.servos.items())
            updates = {s: servo.update(self.measured[s], dt, reference_scale=scale,
                                      allow_early_arrival=all_arrived,
                                      allow_capture=capture_allowed,
                                      allow_warning_capture=warning_capture[s])
                       for s, servo in self.servos.items()}
            for side, servo in self.servos.items():
                servo.telemetry.update(measured_joint_speed_rad_s=max(map(abs, self.velocities[side])))
            drifted = [s for s, u in updates.items()
                       if u.command_stationary and u.arrival_latched and not u.within_tolerance]
            if drifted:
                self.reacquire(self.servos, drifted, deadline, pending=updates)
                for side in drifted:
                    updates[side] = self.servos[side].update(self.measured[side], dt,
                        allow_capture=capture_allowed)
            if all(u.command_stationary for u in updates.values()):
                for side in self.servos:
                    self.servos[side].confirm_command(self.saved[side])
                if all(u.arrival_latched and u.within_tolerance for u in updates.values()):
                    confirmation = self.hold(overall_deadline=deadline + 6.)
                    if confirmation is not None:
                        break
                    drifted = [s for s in SIDES if not self.within(self.measured[s], s)]
                    self.reacquire(self.servos, drifted, deadline)
                    self.phase = 'servo'
                    self.last_control_tick = None
                    continue
                self.log_progress(updates)
                base.time.sleep(max(0., min(self.last_control_tick + self.period, deadline) - base.time.monotonic()))
                self.check_servo_deadline(deadline)
                self.snapshot(expected=self.saved, count=self.count)
                self.check_servo_deadline(deadline)
                continue
            desired = {s: updates[s].command if s in updates else self.saved[s] for s in SIDES}
            self.dispatch(desired, dt=dt, deadline=deadline)
            for side in self.servos:
                self.servos[side].confirm_command(self.saved[side])
            self.log_progress(updates)
        final_error = self.error_report()
        final_error['correction_index'] = 0
        policy = getattr(self, 'arrival_policy', None)
        arrival = (base.legacy.arrival_check(final_error, policy, emit=True)
                   if policy and policy['configured'] else None)
        if arrival is not None and not arrival['accepted']:
            raise base.ContinuousP2PError('Final measured arrival exceeds the explicit stop tolerance')
        if self.correction_guard is not None:
            self.correction_guard(final_error)
        if profile.name == 'fast_tracking':
            records = {}
            for side in SIDES:
                reference = self.servos[side].reference if side in self.servos else starts[side]
                valid = base._error(reference, self.saved[side])['rotation_rad'] <= config.max_rotation_bias_rad + 4e-12
                records[side] = {'saved': asdict(self.saved[side]), 'reference': asdict(reference) if valid else None}
            receipt = {'version': 1, 'server_session_id': self.session,
                       'step_count': self.count, 'arms': records,
                       'accepted_rotation_tolerance_rad': (
                           config.rotation_tolerance_rad if not arrival or arrival['status'] == 'warning'
                           else self.arrival_policy['rotation_tolerance_rad'])}
            continuity.write(self.client, receipt)
        result = {'ok': True, 'mode': self.controller, 'controller': self.controller,
                  'final_error': final_error, 'last_step': self.last_step, 'stalled': None,
                  'arrival_policy': policy, 'arrival_check': arrival,
                  'endpoint_warning_captures': warning_captures,
                  'trajectory': {'profile': 'feedback_governed_quintic', 'duration_sec': duration},
                  'corrections': [], 'continuous_progress': self.progress,
                  'commands': self.commands, 'dispatch_attempts': self.attempts,
                  'initial_step_count': self.start_count, 'final_step_count': self.count,
                  'server_session_id': self.session, 'timings': self.timings,
                  'held_sides': sorted(self.held), 'hold_confirmation': confirmation,
                  'settle_reacquisitions': self.settle_reacquisitions,
                  'bounds': self.bounds, 'frame': 'torso', 'servo_config': asdict(config),
                  'total_budget_s': budget, 'impedance_parameters_changed': False,
                  'performance': {**performance, 'actual_call_s': base.time.monotonic() - self.started,
                                  'met_target': base.time.monotonic() - self.started <= 5.},
                  'feedback_profile': asdict(profile),
                  'orientation_constraint': 'endpoint_only' if profile.endpoint_orientation_only else 'whole_path',
                  'translation_limit_source': self.translation_limit_source,
                  'correction_guard_policy': 'final_success_only',
                  'tolerances': {'position_tolerance_m': self.config.position_tolerance_m,
                                 'rotation_tolerance_rad': self.config.rotation_tolerance_rad},
                  'physical_stop_ack': False, 'ros_receipt_verified': False}
        base._emit('p2p_servo_arrived', controller=self.controller, commands=self.commands,
                   final_error=final_error, step_count=self.count, hold_samples=3,
                   performance=result['performance'],
                   orientation_references={s: servo.orientation_reference for s, servo in self.servos.items()})
        return result


def run(client, left_pose, right_pose, *, delta=False, feedback_guard=None, correction_guard=None, **kwargs):
    runner = TrackingRun(client, left_pose, right_pose, delta, feedback_guard, correction_guard, kwargs)
    try:
        return runner.execute()
    except Exception as error:
        base._emit('p2p_servo_failed', controller=runner.controller, phase=runner.phase, message=str(error),
                   confirmed_commands=runner.commands, dispatch_attempts=runner.attempts,
                   last_validated_error=runner.error_report()
                   if all(s in runner.measured and s in runner.goals for s in SIDES) else None,
                   tracking={s: servo.telemetry for s, servo in getattr(runner, 'servos', {}).items()})
        raise base.ContinuousP2PError(f'External tracking failed in {runner.phase}: {error}; '
            f'confirmed_commands={runner.commands}, dispatch_attempts={runner.attempts}; no automatic retry') from error
