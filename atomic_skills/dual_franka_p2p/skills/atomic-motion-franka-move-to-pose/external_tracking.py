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


@dataclass(frozen=True)
class FeedbackProfile:
    name: str
    translation_gain_per_s: float
    translation_speed_m_s: float
    rotation_gain_per_s: float = .3
    rotation_speed_rad_s: float = .01
    settled_rotation_deadband_rad: float = 0.


PROFILES = {
    'conservative': FeedbackProfile('conservative', .3, .002),
    'responsive': FeedbackProfile('responsive', .6, .004, settled_rotation_deadband_rad=.03),
}


def feedback_profile(name='conservative'):
    """Named, bounded outer-loop profiles; no inner impedance tuning."""
    if name not in PROFILES:
        raise ValueError('Tracking profile must be conservative or responsive')
    return PROFILES[name]


def command_config(feedback_config, translation_speed=.05, rotation_speed=.4,
                   translation_step=.003, rotation_step=.025):
    """Total reference + correction limits; keep the existing bias/arrival policy."""
    return replace(feedback_config,
        max_translation_speed_m_s=min(.02, g._positive(translation_speed, 'translation speed')),
        max_rotation_speed_rad_s=min(.08, g._positive(rotation_speed, 'rotation speed')),
        max_translation_step_m=min(.001, g._positive(translation_step, 'translation step')),
        max_rotation_step_rad=min(.004, g._positive(rotation_step, 'rotation step')),
        max_translation_acceleration_m_s2=.04, max_rotation_acceleration_rad_s2=.16)


def path_duration(initial, goal, config, *, minimum=.5):
    error = g._error(initial, goal)
    # Leave command-speed headroom for feedback; no caller speed is exceeded.
    return max(g._positive(minimum, 'minimum duration'),
               1.875 * error['translation_m'] / (config.max_translation_speed_m_s * .8),
               1.875 * error['rotation_rad'] / (config.max_rotation_speed_rad_s * .8))


class TrackingServo(g.ContinuousPoseServo):
    def __init__(self, initial, goal, saved, config, duration, *, profile="conservative"):
        # Initial offset is inherited from actual saved state, never cleared or
        # excluded from the configured total bias cap.
        self.feedback = feedback_profile(profile)
        super().__init__(initial, saved, config)
        self._validate_pose(goal)
        self.initial = initial
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
        return max(0., min(1., (.009 - error['translation_m']) / .006,
                          (.08 - error['rotation_rad']) / .05))

    def update(self, measured, dt, *, reference_scale=None, allow_early_arrival=True):
        if self._pending is not None:
            raise RuntimeError('A command proposal is pending confirmation; do not replay or advance')
        dt = g._positive(dt, 'dt')
        if dt > self.config.max_dt_s:
            raise ValueError('dt exceeds max_dt_s; do not integrate stale time')
        self._validate_pose(measured)
        tracking_error = g._error(measured, self.reference)
        # Slow before the fixed physical orientation corridor is approached.
        requested_scale = self.tracking_scale(measured)
        if reference_scale is not None:
            value = g._finite_vector([reference_scale], 1, 'reference scale')[0]
            if not 0 <= value <= 1:
                raise ValueError('reference scale must be within [0, 1]')
            requested_scale = min(requested_scale, value)
        scale = self.governor_scale + max(-2. * dt, min(2. * dt, requested_scale - self.governor_scale))
        virtual_time = min(self.duration, self.virtual_time + dt * scale)
        reference = self._reference_at(virtual_time)
        final_error = g._error(measured, self.final_goal)
        position_in = final_error['translation_m'] <= self.config.position_tolerance_m
        rotation_in = final_error['rotation_rad'] <= self.config.rotation_tolerance_rad
        within = position_in and rotation_in
        latched = self._arrival_latched or (within and (virtual_time >= self.duration or allow_early_arrival))
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
            arrived_component = position_in if not angular else rotation_in
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
                        g._limit(g._scale(error_vector, gain), correction_speed))
            desired = g._ZERO if latched else g._add(g._scale(forward, 1. / dt), feedback)
            # Leave a numerical reserve before the physical cap. This does not
            # enlarge a cap or relax the saved-command acknowledgement check.
            projection_cap = cap
            if cap is not None and g._distance(current, center, angular) < cap - 1e-9:
                projection_cap = cap - 1e-9
            velocities[key] = g._bounded_velocity(current, center, self._velocity[key], desired,
                dt=dt, speed=speed, step=step, acceleration=acceleration, cap=projection_cap, angular=angular)
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
                          'command_to_measured': g._error(measured, command)}
        self._pending_path = (reference, virtual_time, scale)
        self._pending = g.ServoUpdate(command, translation + rotation, final_error, bias, within,
            stationary, at_cap, status, velocities, dt, latched)
        return self._pending

    def confirm_command(self, saved):
        super().confirm_command(saved)
        self.reference, self.virtual_time, self.governor_scale = self._pending_path
        self._pending_path = None
