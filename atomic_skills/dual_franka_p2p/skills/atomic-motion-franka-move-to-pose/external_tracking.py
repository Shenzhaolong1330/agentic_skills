"""Free-space reference governor; changes pose references, never impedance gains.

The acknowledged command is the integral state. A quintic reference advances
only as fast as measured tracking permits. Feedback acts during the whole move,
including rotation during translation. No RPC, timing loop or contact policy is
implemented here. These command bounds are not a physical stability proof.
"""
from dataclasses import dataclass, replace
import importlib.util
from pathlib import Path
import sys


def _geometry():
    name = '_franka_atomic_continuous_geometry'
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name('continuous_pose_servo.py'))
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


g = _geometry()


def bounded_tangent_escape(current, center, previous, desired, stopped, *, dt,
                           speed, step, acceleration, cap):
    """Find a descending angular direction when radial projection stalls.

    Only a stationary command at the existing cap is eligible. The original
    projector rechecks the candidate's speed, step, acceleration, endpoint and
    complete braking reserve. Nothing is committed here; the normal ACK path
    still owns all state updates. Pure outward demand remains stopped.
    """
    if (cap is None or g._norm(previous) > g._STATIONARY
            or g._norm(stopped) > max(g._STATIONARY, 4 * g._ROUNDING / dt)):
        return stopped
    bias = g._rotation_delta(center, current)
    radius = g._norm(bias)
    if radius < cap - 2e-9 or radius > cap + 4 * g._ROUNDING:
        return stopped
    radial = g._scale(bias, 1. / radius)
    outward = sum(a * b for a, b in zip(desired, radial))
    if outward <= 0.:
        return stopped
    tangent = g._sub(desired, g._scale(radial, outward))
    if g._norm(tangent) <= g._STATIONARY:
        return stopped
    tangent = g._limit(tangent, min(speed, step / dt, acceleration * dt))
    velocity = g._norm(tangent)
    # A chord needs inward curvature reserve while moving along the cap. This
    # conservative candidate is still accepted only by the exact SO(3) checks.
    inward = 2. * velocity * velocity * (dt + velocity / (2. * acceleration)) / cap
    candidate = g._sub(tangent, g._scale(radial, inward))
    try:
        result = g._bounded_velocity(current, center, previous, candidate,
            dt=dt, speed=speed, step=step, acceleration=acceleration, cap=cap, angular=True)
    except g.NoFeasibleStep:
        return stopped
    if sum(a * b for a, b in zip(desired, result)) <= 0.:
        return stopped
    return result


@dataclass(frozen=True)
class FeedbackProfile:
    name: str
    translation_gain_per_s: float
    translation_speed_m_s: float
    rotation_gain_per_s: float = .3
    rotation_speed_rad_s: float = .01
    settled_rotation_deadband_rad: float = 0.
    command_translation_speed_m_s: float = .02
    command_translation_step_m: float = .001
    command_translation_acceleration_m_s2: float = .04
    rotation_pause_rad: float = .08
    rotation_slow_rad: float = .03
    use_satisfied_goal_orientation: bool = False
    endpoint_orientation_only: bool = False
    command_rotation_speed_rad_s: float = .08
    command_rotation_step_rad: float = .004
    command_rotation_acceleration_rad_s2: float = .16
    position_slow_m: float = .003
    position_pause_m: float = .009
    precise_capture: bool = False
    tolerance_scaled_governor: bool = False
    anti_windup_gain_per_s: float = 0.


PROFILES = {
    'conservative': FeedbackProfile('conservative', .3, .002),
    'responsive': FeedbackProfile('responsive', .6, .004, settled_rotation_deadband_rad=.03),
    # Explicit research opt-in. Native fault/wrench/arrival checks remain
    # unchanged; a faster reference is not a guarantee of faster real motion.
    'fast_free_space': FeedbackProfile(
        'fast_free_space', 2., .02, rotation_gain_per_s=1.,
        rotation_speed_rad_s=.04, settled_rotation_deadband_rad=.01,
        command_translation_speed_m_s=.10, command_translation_step_m=.005,
        command_translation_acceleration_m_s2=.20, rotation_pause_rad=.09, rotation_slow_rad=.07,
        use_satisfied_goal_orientation=True),
}
PROFILES['fast_endpoint'] = replace(PROFILES['fast_free_space'], name='fast_endpoint',
    endpoint_orientation_only=True, translation_gain_per_s=3., translation_speed_m_s=.03,
    rotation_gain_per_s=3., rotation_speed_rad_s=.12,
    command_rotation_speed_rad_s=.12, command_rotation_step_rad=.006,
    command_rotation_acceleration_rad_s2=.4, position_slow_m=.006, position_pause_m=.015)
# Production free-space candidate: retain the measured orientation corridor and
# an angular governor, with the tested faster parallel correction/capture policy.
# Unlike endpoint-only motion, this also applies to orientation-constrained paths.
PROFILES['fast_tracking'] = replace(PROFILES['fast_endpoint'], name='fast_tracking',
    endpoint_orientation_only=False, precise_capture=True, tolerance_scaled_governor=True,
    anti_windup_gain_per_s=2., command_translation_speed_m_s=.06,
    command_translation_step_m=.003, command_translation_acceleration_m_s2=.10,
    command_rotation_acceleration_rad_s2=.20)


def feedback_profile(name='conservative'):
    """Named, bounded outer-loop profiles; no inner impedance tuning."""
    if name not in PROFILES:
        raise ValueError('Tracking profile must be one of: ' + ', '.join(PROFILES))
    return PROFILES[name]


def command_config(feedback_config, translation_speed=.05, rotation_speed=.4,
                   translation_step=.003, rotation_step=.025, *, profile='conservative', period_s=.05):
    """Total reference + correction limits; keep the existing bias/arrival policy."""
    limits = feedback_profile(profile)
    period = g._positive(period_s, 'control period')
    linear_step = min(limits.command_translation_step_m, g._positive(translation_step, 'translation step'))
    angular_step = min(limits.command_rotation_step_rad, g._positive(rotation_step, 'rotation step'))
    return replace(feedback_config,
        max_translation_speed_m_s=min(limits.command_translation_speed_m_s, g._positive(translation_speed, 'translation speed'), linear_step / period),
        max_rotation_speed_rad_s=min(limits.command_rotation_speed_rad_s, g._positive(rotation_speed, 'rotation speed'), angular_step / period),
        max_translation_step_m=linear_step,
        max_rotation_step_rad=angular_step,
        max_translation_acceleration_m_s2=limits.command_translation_acceleration_m_s2,
        max_rotation_acceleration_rad_s2=limits.command_rotation_acceleration_rad_s2)


def path_duration(initial, goal, config, *, minimum=.5):
    error = g._error(initial, goal)
    # Leave command-speed headroom for feedback; no caller speed is exceeded.
    return max(g._positive(minimum, 'minimum duration'),
               1.875 * error['translation_m'] / (config.max_translation_speed_m_s * .8),
               1.875 * error['rotation_rad'] / (config.max_rotation_speed_rad_s * .8),
               (5.773502692 * error['translation_m'] / config.max_translation_acceleration_m_s2)**.5,
               (5.773502692 * error['rotation_rad'] / config.max_rotation_acceleration_rad_s2)**.5)


def reference_start(initial, goal, config, profile, *, saved=None, rotation_tolerance_rad=None):
    # A warning/stop band is not evidence that the requested rotation is already
    # satisfied. Anchor only inside the nominal path tolerance AND the existing
    # saved-command cap; otherwise track a real path from the measured start.
    tolerance = min(config.rotation_tolerance_rad,
        g._positive(rotation_tolerance_rad, 'path rotation tolerance')
        if rotation_tolerance_rad is not None else config.rotation_tolerance_rad)
    anchor = g.Pose(initial.position_m, goal.quaternion_xyzw)
    compatible = (saved is None or g._error(anchor, saved)['rotation_rad']
                  <= config.max_rotation_bias_rad + 4 * g._ROUNDING)
    if (profile.use_satisfied_goal_orientation and compatible
            and g._error(initial, goal)['rotation_rad'] <= tolerance):
        return anchor
    return initial


class TrackingServo(g.ContinuousPoseServo):
    def __init__(self, initial, goal, saved, config, duration, *, profile="conservative",
                 path_rotation_tolerance_rad=None, nominal_position_tolerance_m=None,
                 confirmed_orientation_reference=None, confirmed_rotation_tolerance_rad=None):
        # Initial offset is inherited from actual saved state, never cleared or
        # excluded from the configured total bias cap.
        self.feedback = feedback_profile(profile)
        # Use nominal task tolerance, never its wider warning/stop band, to
        # govern transit. The final acceptance policy remains independent.
        self.path_rotation_tolerance_rad = min(config.rotation_tolerance_rad,
            g._positive(path_rotation_tolerance_rad, 'path rotation tolerance')
            if path_rotation_tolerance_rad is not None else config.rotation_tolerance_rad)
        self.nominal_position_tolerance_m = min(config.position_tolerance_m,
            g._positive(nominal_position_tolerance_m, 'nominal position tolerance')
            if nominal_position_tolerance_m is not None else config.position_tolerance_m)
        # A satisfied orientation can use the requested goal as its fixed
        # reference. This preserves a previous call's valid compensation instead
        # of moving its cap centre to the slightly different measured pose.
        # Neither the saved command nor the cap is changed, and both physical
        # arrival and the complete saved-to-reference bias are checked below.
        if not isinstance(goal, g.Pose) or goal.frame != 'torso':
            raise ValueError('goal must be a validated torso Pose')
        self.orientation_reference = 'measured_start'
        anchored = reference_start(initial, goal, config, self.feedback, saved=saved,
            rotation_tolerance_rad=self.path_rotation_tolerance_rad)
        if confirmed_orientation_reference is not None:
            # This reference is accepted only with the driver's complete prior
            # success receipt and fresh matching command/session evidence.
            reference = g.Pose(initial.position_m, confirmed_orientation_reference)
            confirmed_limit = (
                g._positive(confirmed_rotation_tolerance_rad, 'confirmed arrival tolerance')
                if confirmed_rotation_tolerance_rad is not None
                else min(config.rotation_tolerance_rad, self.path_rotation_tolerance_rad))
            if g._error(initial, reference)['rotation_rad'] > confirmed_limit:
                raise ValueError('Confirmed orientation reference no longer matches measured pose')
            anchored = reference
            self.orientation_reference = 'confirmed_previous_reference'
        if anchored is not initial:
            initial = anchored
            if confirmed_orientation_reference is None:
                self.orientation_reference = 'already_satisfied_goal'
        super().__init__(initial, saved, config)
        self._validate_pose(goal)
        self.initial = initial
        self.initial_bias = g._error(initial, saved)
        self.final_goal = goal
        self.duration = g._positive(duration, 'duration')
        self.reference = initial
        self.virtual_time = 0.
        self.governor_scale = 0.
        self._pending_path = None
        # A translation call whose orientation is already comfortably inside
        # tolerance must not keep integrating harmless attitude noise. Preserve
        # its saved compensation; resume angular feedback on meaningful drift.
        deadband = min(self.feedback.settled_rotation_deadband_rad,
                       .5 * config.rotation_tolerance_rad)
        self.orientation_deadband_rad = (deadband if
            g._error(initial, goal)['rotation_rad'] <= deadband else 0.)
        self.telemetry = {}

    def _reference_at(self, virtual_time):
        fraction = min(1., virtual_time / self.duration)
        blend = fraction**3 * (10. - 15. * fraction + 6. * fraction**2)
        delta = g._error(self.initial, self.final_goal)
        return g.Pose(g._add(self.initial.position_m, g._scale(delta['translation_vector_m'], blend)),
                      g._integrate(self.initial.quaternion_xyzw,
                                   g._scale(delta['rotation_vector_rad'], blend), True))

    def tracking_scale(self, measured):
        self._validate_pose(measured)
        error = g._error(measured, self.reference)
        position_scale = max(0., min(1.,
            (self.feedback.position_pause_m - error['translation_m']) /
            (self.feedback.position_pause_m - self.feedback.position_slow_m)))
        if self.feedback.endpoint_orientation_only:
            # The task constrains attitude at the endpoint only. Keep angular
            # feedback running in parallel, but never stall XYZ on path attitude.
            return position_scale
        slow, pause = self.rotation_governor_limits()
        return min(position_scale, max(0., min(1.,
                          (pause - error['rotation_rad']) / (pause - slow))))

    def rotation_governor_limits(self):
        slow, pause = self.feedback.rotation_slow_rad, self.feedback.rotation_pause_rad
        if self.feedback.tolerance_scaled_governor:
            # Measured tracking error and command compensation are different
            # quantities. A saturated compensator can still track within the
            # task's nominal tolerance. Do not stall an acceptable translation
            # at a hidden 0.9 * nominal threshold. Slow in the explicitly
            # configured warning band; pause at its actual acceptance boundary.
            # Strict callers without a warning band still slow before their
            # nominal boundary. Command bias remains independently bounded.
            nominal = self.path_rotation_tolerance_rad
            pause = min(self.config.rotation_tolerance_rad, 2. * nominal)
            slow = nominal if pause > nominal else .6 * nominal
            return slow, pause
        if self.feedback.name == 'fast_free_space':
            slow = min(slow, .7 * self.config.rotation_tolerance_rad)
            pause = min(pause, .9 * self.config.rotation_tolerance_rad)
        return slow, pause

    def stationary_warning_candidate(self, measured):
        """A completed, ACKed path stalled inside the explicit warning band."""
        if (self.feedback.name != 'fast_tracking' or self._arrival_latched
                or self.virtual_time < self.duration
                or any(g._norm(v) > g._STATIONARY for v in self._velocity.values())):
            return False
        error = g._error(measured, self.final_goal)
        within = (error['translation_m'] <= self.config.position_tolerance_m
                  and error['rotation_rad'] <= self.config.rotation_tolerance_rad)
        return within and not all(self.capture_components(measured))

    def capture_components(self, measured, *, allow_warning=False):
        error = g._error(measured, self.final_goal)
        position = self.config.position_tolerance_m
        rotation = self.config.rotation_tolerance_rad
        if self.feedback.endpoint_orientation_only or self.feedback.precise_capture:
            # Brake well inside the acceptance boundary. Orientation correction
            # can move XYZ slightly; stopping feedback at 3 mm invited repeated
            # hold/reacquire cycles in hardware. This tightens capture only.
            position = min(.001, position * .4)
            rotation = min(.08, rotation * .8)
        if self.feedback.name == 'fast_tracking' and not allow_warning:
            # Capture already requires measured rest in the driver. Do not add
            # a hidden tighter endpoint requirement that can deadlock at a
            # compensation cap despite satisfying the user's nominal tolerance.
            position = self.nominal_position_tolerance_m
            rotation = self.path_rotation_tolerance_rad
        if self.feedback.name == 'fast_tracking' and allow_warning:
            position, rotation = self.config.position_tolerance_m, self.config.rotation_tolerance_rad
        return error['translation_m'] <= position, error['rotation_rad'] <= rotation

    def update(self, measured, dt, *, reference_scale=None, allow_early_arrival=True,
               feedback_scale=1., allow_capture=True, allow_warning_capture=False):
        if self._pending is not None:
            raise RuntimeError('A command proposal is pending confirmation; do not replay or advance')
        dt = g._positive(dt, 'dt')
        if dt > self.config.max_dt_s:
            raise ValueError('dt exceeds max_dt_s; do not integrate stale time')
        self._validate_pose(measured)
        feedback_scale = g._finite_vector([feedback_scale], 1, 'feedback scale')[0]
        if not 0 <= feedback_scale <= 1:
            raise ValueError('feedback scale must be within [0, 1]')
        tracking_error = g._error(measured, self.reference)
        # Slow before the fixed physical orientation corridor is approached.
        requested_scale = self.tracking_scale(measured)
        if reference_scale is not None:
            value = g._finite_vector([reference_scale], 1, 'reference scale')[0]
            if not 0 <= value <= 1:
                raise ValueError('reference scale must be within [0, 1]')
            requested_scale = min(requested_scale, value)
        # Do not smooth a safety slowdown into several extra forward steps.
        # Command acceleration/braking remains bounded by _bounded_velocity.
        # Only accelerate the virtual reference gradually after recovery.
        scale = min(requested_scale, self.governor_scale + 2. * dt)
        virtual_time = min(self.duration, self.virtual_time + dt * scale)
        reference = self._reference_at(virtual_time)
        final_error = g._error(measured, self.final_goal)
        position_in = final_error['translation_m'] <= self.config.position_tolerance_m
        rotation_in = final_error['rotation_rad'] <= self.config.rotation_tolerance_rad
        within = position_in and rotation_in
        position_captured, rotation_captured = self.capture_components(measured, allow_warning=allow_warning_capture)
        latched = self._arrival_latched or (allow_capture and position_captured and rotation_captured and
                   (virtual_time >= self.duration or allow_early_arrival))
        if latched:
            # This is free-space P2P, not timed path playback. Actual final-goal
            # arrival wins over a governor-stalled internal path clock. Freeze
            # the reference while braking so saved bias and its bounds persist.
            reference = self.reference
            virtual_time = self.virtual_time
            scale = 0.
        feedforward = g._error(self.reference, reference)
        reference_error = g._error(measured, reference)
        velocities = {}
        inward_projection = []
        for key, vector, current, center, speed, step, acceleration, cap, angular, correction_speed, gain in (
            ('translation_m_s', 'translation_vector_m', self._command.position_m, reference.position_m,
             self.config.max_translation_speed_m_s, self.config.max_translation_step_m,
             self.config.max_translation_acceleration_m_s2, self.config.max_translation_bias_m, False,
             self.feedback.translation_speed_m_s, self.feedback.translation_gain_per_s),
            ('rotation_rad_s', 'rotation_vector_rad', self._command.quaternion_xyzw, reference.quaternion_xyzw,
             self.config.max_rotation_speed_rad_s, self.config.max_rotation_step_rad,
             self.config.max_rotation_acceleration_rad_s2, self.config.max_rotation_bias_rad, True,
             self.feedback.rotation_speed_rad_s, self.feedback.rotation_gain_per_s),
        ):
            arrived_component = position_captured if not angular else rotation_captured
            error_vector = reference_error[vector]
            forward = feedforward[vector]
            if angular and self.orientation_deadband_rad > 0.:
                # Smooth dead zone: no sudden kick at its boundary, and no
                # nominal angular feedforward for an already-satisfied axis.
                magnitude = g._norm(error_vector)
                factor = max(0., 1. - self.orientation_deadband_rad / magnitude) if magnitude else 0.
                error_vector = g._scale(error_vector, factor)
                forward = g._ZERO
            feedback = (g._ZERO if virtual_time >= self.duration and arrived_component else
                        g._scale(g._limit(g._scale(error_vector, gain), correction_speed), feedback_scale))
            if self.feedback.anti_windup_gain_per_s and feedback_scale < 1.:
                # A paused reference does not stop the plant pursuing the
                # accumulated equilibrium offset. Back-calculate only the NEW
                # integral correction through bounded increments; preserve the
                # acknowledged static compensation inherited at call start.
                bias_vector = g._error(reference, self._command)[vector]
                excess = g._sub(bias_vector, self.initial_bias[vector])
                # Acceleration limiting can leave the command BEHIND its
                # reference. That is not windup: never "unwind" it by issuing
                # extra forward motion while the joint governor is paused.
                aligned = sum(a * b for a, b in zip(excess, error_vector)) > 0.
                unwind = (g._scale(excess, -self.feedback.anti_windup_gain_per_s * (1. - feedback_scale))
                          if aligned else g._ZERO)
                feedback = g._limit(g._add(feedback, unwind), correction_speed)
            desired = g._ZERO if latched else g._add(g._scale(forward, 1. / dt), feedback)
            # Leave a numerical reserve before the physical cap. This does not
            # enlarge a cap or relax the saved-command acknowledgement check.
            projection_cap = cap
            if cap is not None and g._distance(current, center, angular) < cap - 1e-9:
                projection_cap = cap - 1e-9
            try:
                velocities[key] = g._bounded_velocity(current, center, self._velocity[key], desired,
                    dt=dt, speed=speed, step=step, acceleration=acceleration, cap=projection_cap, angular=angular)
                if angular and self.feedback.name == 'fast_tracking' and not latched:
                    velocities[key] = bounded_tangent_escape(current, center, self._velocity[key],
                        desired, velocities[key], dt=dt, speed=speed, step=step,
                        acceleration=acceleration, cap=projection_cap)
            except g.NoFeasibleStep:
                # The cap centre moves with the reference. At a saturated
                # inherited command, both the desired outward correction and
                # zero-velocity braking can lie outside the NEW sphere, even
                # though a bounded inward step is feasible. Test that direction
                # through the same projector before declaring infeasibility.
                # This is proposal math only: no RPC, time advance or ACK retry.
                old_center = self.reference.quaternion_xyzw if angular else self.reference.position_m
                if (cap is None or g._distance(old_center, center, angular) <= g._ROUNDING
                        or g._distance(current, old_center, angular) > cap + 4 * g._ROUNDING):
                    raise
                inward = g._rotation_delta(current, center) if angular else g._sub(center, current)
                try:
                    velocities[key] = g._bounded_velocity(current, center, self._velocity[key],
                        g._scale(inward, 1. / dt), dt=dt, speed=speed, step=step,
                        acceleration=acceleration, cap=projection_cap, angular=angular)
                except g.NoFeasibleStep:
                    # A moving cap may also need more deceleration than this
                    # tick permits. Hold the acknowledged reference for one
                    # proposal and brake within its existing reserved space.
                    # No command or clock has been committed above. With zero
                    # reference advance this fallback cannot recurse again.
                    update = self.update(measured, dt, reference_scale=0.,
                        allow_early_arrival=allow_early_arrival,
                        feedback_scale=feedback_scale, allow_capture=allow_capture,
                        allow_warning_capture=allow_warning_capture)
                    self.telemetry['reference_paused_for_projection'] = key
                    return update
                inward_projection.append(key)
        translation = g._scale(velocities['translation_m_s'], dt)
        rotation = g._scale(velocities['rotation_rad_s'], dt)
        command = g.Pose(g._integrate(self._command.position_m, translation, False),
                         g._integrate(self._command.quaternion_xyzw, rotation, True))
        # The pending reference is used by the inherited ACK/braking checks.
        # No reference clock or integral command is committed before the ACK.
        self._goal = reference
        bias = self._validate_bias(command)
        stationary = all(g._norm(v) <= g._STATIONARY for v in velocities.values())
        at_cap = {'translation': self.config.max_translation_bias_m is not None and
                  bias['translation_m'] >= self.config.max_translation_bias_m - 2e-9,
                  'rotation': bias['rotation_rad'] >= self.config.max_rotation_bias_rad - 2e-9}
        status = ('holding' if stationary else 'braking') if latched else (
            'correcting' if virtual_time >= self.duration else 'governor_wait' if scale <= 1e-12 else 'approaching')
        self.telemetry = {'reference': {'position_m': reference.position_m,
                                       'quaternion_xyzw': reference.quaternion_xyzw},
                          'tracking_error': tracking_error, 'governor_scale': scale,
                          'requested_scale': requested_scale, 'path_fraction': virtual_time / self.duration,
                          'stage': status, 'bias_frame': 'current_reference',
                          'orientation_deadband_rad': self.orientation_deadband_rad,
                          'orientation_reference': self.orientation_reference,
                          'capture_ready': position_captured and rotation_captured,
                          'stationary_warning_capture': bool(allow_warning_capture),
                          'anti_windup_active': bool(self.feedback.anti_windup_gain_per_s and feedback_scale < 1.),
                          'moving_reference_inward_projection': inward_projection,
                          'command_to_measured': g._error(measured, command)}
        self._pending_path = (reference, virtual_time, scale)
        self._pending = g.ServoUpdate(command, translation + rotation, final_error, bias, within,
            stationary, at_cap, status, velocities, dt, latched)
        return self._pending

    def confirm_command(self, saved):
        super().confirm_command(saved)
        self.reference, self.virtual_time, self.governor_scale = self._pending_path
        self._pending_path = None
