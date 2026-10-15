"""Pure per-stage P2P progress state; no robot client, motion or I/O.

Create one guard before a continuous correction sequence. Its first report must
be the measured error AFTER the main trajectory, not the pre-move starting pose.
Keep that same guard for subsequent corrections: recreating it would discard the
historical-best protection against small cumulative drift. This guard neither
changes arrival tolerances nor qualifies robot health or permits gripper actions.
"""

from __future__ import annotations

from collections.abc import Mapping
import json
import math
from numbers import Integral, Real


_COMPONENTS = {
    "left_translation_error_m": "position",
    "right_translation_error_m": "position",
    "left_rotation_error_rad": "rotation",
    "right_rotation_error_rad": "rotation",
}


def _finite_nonnegative(value, name, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite {'positive' if positive else 'nonnegative'} number")
    try:
        value = float(value)
    except (OverflowError, ValueError) as error:
        raise ValueError(f"{name} must be finite") from error
    if not math.isfinite(value) or value < 0 or (positive and value == 0):
        raise ValueError(f"{name} must be a finite {'positive' if positive else 'nonnegative'} number")
    return value


class P2PProgressGuard:
    """Track four independent physical errors against their historical minima.

    A component fails only if it is outside its original arrival tolerance AND
    exceeds its historical best plus that component's physical deadband. An
    in-tolerance component never causes this guard to reject a report. Accepted
    observations update each historical best even for sub-deadband improvements.
    Invalid/rejected observations leave every best value unchanged.
    """

    def __init__(self, position_tolerance_m, rotation_tolerance_rad,
                 position_deadband_m=0.0005, rotation_deadband_rad=0.005):
        self._tolerances = {
            "position": _finite_nonnegative(position_tolerance_m, "position_tolerance_m", positive=True),
            "rotation": _finite_nonnegative(rotation_tolerance_rad, "rotation_tolerance_rad", positive=True),
        }
        self._deadbands = {
            "position": _finite_nonnegative(position_deadband_m, "position_deadband_m"),
            "rotation": _finite_nonnegative(rotation_deadband_rad, "rotation_deadband_rad"),
        }
        self._best = None

    @property
    def best(self):
        """Return a copy so caller-owned log objects cannot reset the guard."""
        return None if self._best is None else dict(self._best)

    def observe(self, report):
        if not isinstance(report, Mapping):
            raise ValueError("P2P error report must be a mapping")
        try:
            current = {name: _finite_nonnegative(report[name], name) for name in _COMPONENTS}
            correction_index = report["correction_index"]
        except KeyError as error:
            raise ValueError(f"P2P error report is missing {error.args[0]}") from error
        if isinstance(correction_index, bool) or not isinstance(correction_index, Integral) or correction_index < 0:
            raise ValueError("correction_index must be a nonnegative integer")

        baseline = self._best is None
        previous_best = None if baseline else dict(self._best)
        outside = [name for name, kind in _COMPONENTS.items() if current[name] > self._tolerances[kind]]
        increased = [] if baseline else [
            name for name in outside
            if current[name] > self._best[name] + self._deadbands[_COMPONENTS[name]]
        ]
        if increased:
            # Do not move the reference upwards after any rejected observation.
            detail = {
                "correction_index": int(correction_index),
                "increased_components": increased,
                "best": previous_best,
                "current": current,
                "deadbands": {"position_m": self._deadbands["position"],
                              "rotation_rad": self._deadbands["rotation"]},
                "tolerances": {"position_m": self._tolerances["position"],
                               "rotation_rad": self._tolerances["rotation"]},
            }
            raise RuntimeError("p2p_error_increased: " + json.dumps(detail, allow_nan=False, sort_keys=True))

        improved = [] if baseline else [
            name for name, kind in _COMPONENTS.items()
            if current[name] < self._best[name] - self._deadbands[kind]
        ]
        self._best = current.copy() if baseline else {
            name: min(self._best[name], current[name]) for name in _COMPONENTS
        }
        within = not outside
        status = "within_tolerance" if within else "baseline" if baseline else "improving" if improved else "plateau"
        return {
            "status": status,
            "baseline": baseline,
            "plateau": status == "plateau",
            "improving": status == "improving",
            "within_tolerance": within,
            "correction_index": int(correction_index),
            "best": dict(self._best),
            "current": current,
            "previous_best": previous_best,
            "outside_tolerance_components": outside,
            "improved_components": improved,
            "deadbands": {"position_m": self._deadbands["position"],
                          "rotation_rad": self._deadbands["rotation"]},
            "tolerances": {"position_m": self._tolerances["position"],
                           "rotation_rad": self._tolerances["rotation"]},
        }
