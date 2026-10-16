"""Self-contained geometry for the bounded, continuous Cartesian outer servo.

The integral state is the last ACKNOWLEDGED saved command, never a separate
unbounded accumulator. Each update proposes one small increment; confirm_command
commits it only after the driver checks the RPC acknowledgement and observation.
There is no RPC, synchronization, robot reset, sleep, clock or arrival dwell here.

The nominal goal stays fixed. Quaternion increments left-compose in torso frame.
Velocity projection includes a conservative straight-line braking reserve inside
each configured goal-centred bias ball. The translation radius may explicitly be
disabled with None; speed, step, acceleration and the rotation radius still apply.
These limits bound generated commands, not physical motion
or whole-arm clearance. Stability on a real delayed robot is NOT established by
this candidate or its simulations; native health, timing, dwell and faults belong
to the driver. A distant goal needs a separately verified nominal approach.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
import math
from typing import Any, Iterable

# Self-contained geometry: the atomic skill must not import research/runtime modules.
def _finite_vector(values: Iterable[float], length: int, name: str) -> tuple[float, ...]:
    try:
        raw = tuple(values)
        if len(raw) != length or any(isinstance(value, bool) for value in raw):
            raise ValueError
        result = tuple(float(value) for value in raw)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{name} requires {length} finite numbers") from error
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} requires {length} finite numbers")
    return result



@dataclass(frozen=True)
class Pose:
    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    frame: str = "torso"

    def __post_init__(self) -> None:
        position = _finite_vector(self.position_m, 3, "position_m")
        quaternion = _finite_vector(self.quaternion_xyzw, 4, "quaternion_xyzw")
        size = math.hypot(*quaternion)
        if not math.isfinite(size) or abs(size - 1.0) > 1e-6:
            raise ValueError("quaternion_xyzw must be a unit quaternion (within 1e-6)")
        if not isinstance(self.frame, str) or not self.frame or self.frame != self.frame.strip():
            raise ValueError("frame must be explicit and nonempty")
        # Copy caller-owned lists and remove only tiny unit-quaternion roundoff.
        object.__setattr__(self, "position_m", position)
        object.__setattr__(self, "quaternion_xyzw", tuple(value / size for value in quaternion))



_ZERO = (0., 0., 0.)
_ROUNDING = 1e-12
_STATIONARY = 1e-12


class NoFeasibleStep(ValueError):
    """No proposed increment can honor all command bounds; do not send one."""


def _add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _scale(v, factor):
    return tuple(x * factor for x in v)


def _norm(v):
    return math.hypot(*v)


def _limit(v, maximum):
    size = _norm(v)
    return _scale(v, maximum / size) if size > maximum else tuple(v)


def _positive(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label} must be positive and finite')
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{label} must be positive and finite')
    return float(value)


def _quat_multiply(a, b):
    x, y, z, w = a
    u, v, t, s = b
    q = (w*u + x*s + y*t - z*v, w*v - x*t + y*s + z*u,
         w*t + x*v - y*u + z*s, w*s - x*u - y*v - z*t)
    return _scale(q, 1. / _norm(q))


def _quat_inverse(q):
    return (-q[0], -q[1], -q[2], q[3])


def _quat_from_rotvec(v):
    angle = _norm(v)
    if angle <= 1e-15:
        q = (*_scale(v, .5), 1.)
        return _scale(q, 1. / _norm(q))
    return (*_scale(v, math.sin(angle / 2.) / angle), math.cos(angle / 2.))


def _rotation_delta(actual, target):
    """Shortest Log(target * actual^-1), expressed in the shared base frame."""
    q = _quat_multiply(target, _quat_inverse(actual))
    if q[3] < 0:
        q = _scale(q, -1.)
    size = _norm(q[:3])
    if size <= 1e-15:
        return _scale(q[:3], 2.)
    return _scale(q[:3], 2. * math.atan2(size, q[3]) / size)


def _integrate(value, increment, angular):
    return _quat_multiply(_quat_from_rotvec(increment), value) if angular else _add(value, increment)


def _distance(value, goal, angular):
    return _norm(_rotation_delta(goal, value)) if angular else _norm(_sub(value, goal))


def _error(actual, target):
    translation = _sub(target.position_m, actual.position_m)
    rotation = _rotation_delta(actual.quaternion_xyzw, target.quaternion_xyzw)
    return {'translation_m': _norm(translation), 'rotation_rad': _norm(rotation),
            'translation_vector_m': translation, 'rotation_vector_rad': rotation}


@dataclass(frozen=True)
class ServoConfig:
    translation_integral_gain_per_s: float = .6
    rotation_integral_gain_per_s: float = .6
    max_translation_speed_m_s: float = .004
    max_rotation_speed_rad_s: float = .03
    max_translation_step_m: float = .00025
    max_rotation_step_rad: float = .002
    max_translation_acceleration_m_s2: float = .01
    max_rotation_acceleration_rad_s2: float = .08
    max_translation_bias_m: float | None = .025
    max_rotation_bias_rad: float = .20
    position_tolerance_m: float = .003
    rotation_tolerance_rad: float = .1
    max_dt_s: float = .2

    def __post_init__(self):
        for field in fields(self):
            if field.name == 'max_translation_bias_m' and getattr(self, field.name) is None:
                continue
            object.__setattr__(self, field.name, _positive(getattr(self, field.name), field.name))
        # The angular braking proof uses the geodesic convexity of a small SO(3)
        # ball. No configurable cap may silently enter the antipodal region.
        if self.max_rotation_bias_rad >= math.pi / 2:
            raise ValueError('max_rotation_bias_rad must be less than pi/2')
        if self.rotation_tolerance_rad >= math.pi:
            raise ValueError('rotation_tolerance_rad must be less than pi')


@dataclass(frozen=True)
class ServoUpdate:
    command: Pose
    proposed_delta: tuple[float, ...]
    errors: dict[str, Any]
    bias: dict[str, Any]
    within_tolerance: bool
    command_stationary: bool
    at_cap: dict[str, bool]
    status: str
    command_velocity: dict[str, tuple[float, ...]]
    dt_s: float
    arrival_latched: bool

    def to_dict(self):
        return {
            'command': {'position_m': self.command.position_m,
                        'quaternion_xyzw': self.command.quaternion_xyzw, 'frame': self.command.frame},
            'proposed_delta': self.proposed_delta, 'errors': dict(self.errors), 'bias': dict(self.bias),
            'within_tolerance': self.within_tolerance, 'command_stationary': self.command_stationary,
            'at_cap': dict(self.at_cap), 'status': self.status,
            'command_velocity': dict(self.command_velocity), 'dt_s': self.dt_s,
            'arrival_latched': self.arrival_latched,
        }


def _bounded_velocity(current, goal, previous, desired, *, dt, speed, step, acceleration, cap, angular):
    """Project velocity without silently changing the committed integral state.

    From the proposed endpoint reserve v^2/(2a) along the same direction. The
    discrete maximal-deceleration path travels no farther than that continuous
    reserve, including a shortened final step. Convex XYZ / small SO(3) balls
    contain the entire segment when its endpoints are inside. This also allows
    inward travel from a saturated boundary without accumulating outward bias.
    A None cap skips only the radius and radial braking-reserve constraints.
    """
    speed_limit = min(speed, step / dt)
    desired = _limit(desired, speed_limit)
    slew = _add(previous, _limit(_sub(desired, previous), acceleration * dt))
    old_speed = _norm(previous)
    brake = _scale(previous, max(0., old_speed - acceleration * dt) / old_speed) if old_speed else _ZERO

    def feasible(velocity):
        magnitude = _norm(velocity)
        if (magnitude > speed_limit + _ROUNDING
                or _norm(_sub(velocity, previous)) > acceleration * dt + _ROUNDING):
            return False
        if cap is None:
            return True
        endpoint = _integrate(current, _scale(velocity, dt), angular)
        stopping_point = _integrate(endpoint, _scale(velocity, magnitude / (2. * acceleration)), angular)
        return (_distance(endpoint, goal, angular) <= cap + _ROUNDING
                and _distance(stopping_point, goal, angular) <= cap + _ROUNDING)

    if feasible(slew):
        return slew
    if not feasible(brake):
        raise NoFeasibleStep('No feasible command step preserves speed, acceleration and braking bounds')
    # Both endpoints respect the convex speed/acceleration constraints. Keep a
    # verified feasible lower endpoint; no assumption about global optimality is
    # required for this deliberately conservative line search.
    low, high = 0., 1.
    result = brake
    for _ in range(60):
        fraction = (low + high) / 2.
        candidate = _add(brake, _scale(_sub(slew, brake), fraction))
        if feasible(candidate):
            low, result = fraction, candidate
        else:
            high = fraction
    if _norm(result) <= _STATIONARY and feasible(_ZERO):
        return _ZERO
    return result


class ContinuousPoseServo:
    """One fixed local goal and its acknowledged command history.

    update() creates a pending proposal, without changing confirmed command or
    velocity. The driver must validate health/session/count and call
    confirm_command(saved_pose) after one successful RPC; it must stop on an
    uncertain reply, failed read or failed confirmation. There is intentionally
    no retry/replay or automatic adoption of an external command.
    """

    def __init__(self, goal: Pose, confirmed_command: Pose, config: ServoConfig | None = None):
        self.config = config if config is not None else ServoConfig()
        if not isinstance(self.config, ServoConfig):
            raise ValueError('config must be ServoConfig')
        if not isinstance(goal, Pose) or goal.frame != 'torso':
            raise ValueError('goal must be a validated torso Pose')
        self._goal = goal
        self._validate_pose(confirmed_command)
        self._validate_bias(confirmed_command)
        self._command = confirmed_command
        self._velocity = {'translation_m_s': _ZERO, 'rotation_rad_s': _ZERO}
        self._arrival_latched = False
        self._pending = None

    @property
    def goal(self):
        return self._goal

    @property
    def confirmed_command(self):
        return self._command

    @property
    def pending(self):
        return self._pending

    def _validate_pose(self, pose):
        if not isinstance(pose, Pose) or pose.frame != self._goal.frame:
            raise ValueError('expected a validated Pose in the fixed goal frame')

    def _validate_bias(self, command):
        bias = _error(self._goal, command)
        translation_cap = self.config.max_translation_bias_m
        if ((translation_cap is not None and bias['translation_m'] > translation_cap + _ROUNDING)
                or bias['rotation_rad'] > self.config.max_rotation_bias_rad + _ROUNDING):
            raise ValueError('Saved command exceeds the fixed nominal-goal bias cap; use a separate approach')
        return bias

    def update(self, measured: Pose, dt: float) -> ServoUpdate:
        if self._pending is not None:
            raise RuntimeError('A command proposal is pending confirmation; do not replay or advance')
        dt = _positive(dt, 'dt')
        if dt > self.config.max_dt_s:
            raise ValueError('dt exceeds max_dt_s; do not integrate stale time or replay missed steps')
        self._validate_pose(measured)
        config = self.config
        errors = _error(measured, self._goal)
        position_in = errors['translation_m'] <= config.position_tolerance_m
        rotation_in = errors['rotation_rad'] <= config.rotation_tolerance_rad
        within = position_in and rotation_in
        arrival_latched = self._arrival_latched or within
        translation_desired = (_ZERO if arrival_latched or position_in else
                               _scale(errors['translation_vector_m'], config.translation_integral_gain_per_s))
        rotation_desired = (_ZERO if arrival_latched or rotation_in else
                            _scale(errors['rotation_vector_rad'], config.rotation_integral_gain_per_s))
        translation_velocity = _bounded_velocity(
            self._command.position_m, self._goal.position_m, self._velocity['translation_m_s'], translation_desired,
            dt=dt, speed=config.max_translation_speed_m_s, step=config.max_translation_step_m,
            acceleration=config.max_translation_acceleration_m_s2, cap=config.max_translation_bias_m, angular=False)
        rotation_velocity = _bounded_velocity(
            self._command.quaternion_xyzw, self._goal.quaternion_xyzw, self._velocity['rotation_rad_s'], rotation_desired,
            dt=dt, speed=config.max_rotation_speed_rad_s, step=config.max_rotation_step_rad,
            acceleration=config.max_rotation_acceleration_rad_s2, cap=config.max_rotation_bias_rad, angular=True)
        translation_delta, rotation_delta = _scale(translation_velocity, dt), _scale(rotation_velocity, dt)
        command = Pose(_integrate(self._command.position_m, translation_delta, False),
                       _integrate(self._command.quaternion_xyzw, rotation_delta, True), self._goal.frame)
        bias = self._validate_bias(command)
        stationary = _norm(translation_velocity) <= _STATIONARY and _norm(rotation_velocity) <= _STATIONARY
        at_cap = {'translation': (config.max_translation_bias_m is not None
                                  and bias['translation_m'] >= config.max_translation_bias_m - 1e-9),
                  'rotation': bias['rotation_rad'] >= config.max_rotation_bias_rad - 1e-9}
        status = ('holding' if stationary else 'braking') if arrival_latched else (
            'at_cap' if any(at_cap.values()) else 'tracking')
        proposal = ServoUpdate(command, translation_delta + rotation_delta, errors, bias, within, stationary,
                               at_cap, status, {'translation_m_s': translation_velocity,
                                                'rotation_rad_s': rotation_velocity}, dt, arrival_latched)
        self._pending = proposal
        return proposal

    def resume_after_settle_drift(self) -> None:
        """Reopen tracking only after the driver verifies bounded healthy drift.

        Preserve the goal, acknowledged command and zero velocity. This is not
        a transport retry or an adoption of a new saved target. The driver owns
        the unchanged deadline, recovery count and fresh-feedback checks.
        """
        if self._pending is not None:
            raise RuntimeError('Cannot resume with an unconfirmed proposal')
        if not self._arrival_latched:
            raise RuntimeError('Cannot resume before a latched arrival')
        if any(_norm(value) > _STATIONARY for value in self._velocity.values()):
            raise RuntimeError('Cannot resume before command braking completes')
        self._arrival_latched = False

    def confirm_command(self, actual_saved_pose: Pose) -> None:
        if self._pending is None:
            raise RuntimeError('No pending command to confirm')
        self._validate_pose(actual_saved_pose)
        proposal = self._pending
        discrepancy = _error(actual_saved_pose, proposal.command)
        if discrepancy['translation_m'] > 1e-6 or discrepancy['rotation_rad'] > 1e-6:
            raise ValueError('Acknowledged saved command differs from the pending proposal')
        self._validate_bias(actual_saved_pose)
        actual_delta = _error(self._command, actual_saved_pose)
        velocities = {'translation_m_s': _scale(actual_delta['translation_vector_m'], 1. / proposal.dt_s),
                      'rotation_rad_s': _scale(actual_delta['rotation_vector_rad'], 1. / proposal.dt_s)}
        for key, speed, step, acceleration in (
            ('translation_m_s', self.config.max_translation_speed_m_s, self.config.max_translation_step_m,
             self.config.max_translation_acceleration_m_s2),
            ('rotation_rad_s', self.config.max_rotation_speed_rad_s, self.config.max_rotation_step_rad,
             self.config.max_rotation_acceleration_rad_s2),
        ):
            # Recheck the ACTUAL saved increment, not only the proposal, allowing
            # roundoff in quaternion conversion rather than a physical tolerance.
            if (_norm(velocities[key]) > min(speed, step / proposal.dt_s) + 1e-10
                    or _norm(_sub(velocities[key], self._velocity[key])) > acceleration * proposal.dt_s + 1e-10):
                raise ValueError('Acknowledged command violates velocity or acceleration bounds')
        for key, current, goal, cap, acceleration, angular in (
            ('translation_m_s', actual_saved_pose.position_m, self._goal.position_m,
             self.config.max_translation_bias_m, self.config.max_translation_acceleration_m_s2, False),
            ('rotation_rad_s', actual_saved_pose.quaternion_xyzw, self._goal.quaternion_xyzw,
             self.config.max_rotation_bias_rad, self.config.max_rotation_acceleration_rad_s2, True),
        ):
            if cap is None:
                continue
            stopping_point = _integrate(
                current, _scale(velocities[key], _norm(velocities[key]) / (2. * acceleration)), angular)
            # Reconstructing velocity from the acknowledged quaternion adds one
            # more roundoff step compared with the original velocity proposal.
            if _distance(stopping_point, goal, angular) > cap + 2. * _ROUNDING:
                raise ValueError('Acknowledged command consumes the reserved braking distance')
        # Roundoff must not resurrect a command velocity after an exact hold.
        for key in velocities:
            if _norm(velocities[key]) <= _STATIONARY:
                velocities[key] = _ZERO
        self._command = actual_saved_pose
        self._velocity = velocities
        self._arrival_latched = proposal.arrival_latched
        self._pending = None
