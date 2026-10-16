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
SIDES = base.SIDES


class TrackingRun(base._Run):
    controller = 'external_tracking'

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
            raise base.ContinuousP2PError('External tracking budget exhausted; no reference or correction restart')

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
        self.started = base.time.monotonic()
        self.prepare()
        if any(max(map(abs, self.velocities[s])) >= .01 for s in SIDES):
            raise base.ContinuousP2PError('External tracking requires a stationary start')
        config = tracking.command_config(self.config,
            self.options.get('max_translation_speed', .05), self.options.get('max_rotation_speed', .4),
            self.options.get('max_translation_step') or .003,
            self.options.get('max_rotation_step') or .025)
        config = replace(config,
            translation_integral_gain_per_s=profile.translation_gain_per_s,
            rotation_integral_gain_per_s=profile.rotation_gain_per_s)
        minimum = max(base._positive(self.options.get('min_duration_sec', .5), 'minimum duration'),
                      base._positive(self.options['duration_sec'], 'duration')
                      if self.options.get('duration_sec') is not None else .5)
        duration = max(tracking.path_duration(self.initial[s], self.goals[s], config, minimum=minimum)
                       for s in SIDES)
        budget = duration + base.SERVO_BUDGET_S
        self.servos = {s: tracking.TrackingServo(self.initial[s], self.goals[s], self.saved[s], config, duration, profile=profile.name)
                       for s in SIDES if s not in self.held}
        base._emit('p2p_tracking_config', controller=self.controller, nominal_duration_s=duration,
                   total_budget_s=budget, command_config=asdict(config),
                   feedback_profile=asdict(profile),
                   feedback_translation_speed_m_s=profile.translation_speed_m_s,
                   feedback_rotation_speed_rad_s=profile.rotation_speed_rad_s,
                   governor={'position_slow_m': .003, 'position_pause_m': .009,
                             'rotation_slow_rad': .03, 'rotation_pause_rad': .08},
                   impedance_parameters_changed=False, contact_policy=False)
        self.phase = 'servo'
        base.time.sleep(self.period)
        self.snapshot(expected=self.saved, count=self.count)
        deadline = base.time.monotonic() + budget
        while True:
            dt = self.scheduled_dt(deadline=deadline)
            # Both arms share path progress: a lagging arm slows the coordinated
            # reference rather than allowing the other arm to run ahead.
            scale = min((servo.tracking_scale(self.measured[s])
                         for s, servo in self.servos.items()), default=1.)
            all_arrived = all(self.within(self.measured[s], s) for s in SIDES)
            updates = {s: servo.update(self.measured[s], dt, reference_scale=scale,
                                      allow_early_arrival=all_arrived)
                       for s, servo in self.servos.items()}
            drifted = [s for s, u in updates.items()
                       if u.command_stationary and u.arrival_latched and not u.within_tolerance]
            if drifted:
                self.reacquire(self.servos, drifted, deadline, pending=updates)
                for side in drifted:
                    updates[side] = self.servos[side].update(self.measured[side], dt)
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
        result = {'ok': True, 'mode': self.controller, 'controller': self.controller,
                  'final_error': final_error, 'last_step': self.last_step, 'stalled': None,
                  'arrival_policy': policy, 'arrival_check': arrival,
                  'trajectory': {'profile': 'feedback_governed_quintic', 'duration_sec': duration},
                  'corrections': [], 'continuous_progress': self.progress,
                  'commands': self.commands, 'dispatch_attempts': self.attempts,
                  'initial_step_count': self.start_count, 'final_step_count': self.count,
                  'server_session_id': self.session, 'timings': self.timings,
                  'held_sides': sorted(self.held), 'hold_confirmation': confirmation,
                  'settle_reacquisitions': self.settle_reacquisitions,
                  'bounds': self.bounds, 'frame': 'torso', 'servo_config': asdict(config),
                  'total_budget_s': budget, 'impedance_parameters_changed': False,
                  'feedback_profile': asdict(profile),
                  'translation_limit_source': self.translation_limit_source,
                  'correction_guard_policy': 'final_success_only',
                  'tolerances': {'position_tolerance_m': self.config.position_tolerance_m,
                                 'rotation_tolerance_rad': self.config.rotation_tolerance_rad},
                  'physical_stop_ack': False, 'ros_receipt_verified': False}
        base._emit('p2p_servo_arrived', controller=self.controller, commands=self.commands,
                   final_error=final_error, step_count=self.count, hold_samples=3)
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
