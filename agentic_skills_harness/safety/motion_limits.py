from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping


@dataclass(frozen=True)
class MotionLimits:
    max_translation_speed_m_s: float = 0.03
    max_rotation_speed_rad_s: float = 0.15
    max_translation_step_m: float = 0.010
    max_rotation_step_rad: float = 0.035
    max_timeout_s: float = 30.0
    max_relative_displacement_m: float = 0.010

    def __post_init__(self) -> None:
        for name in ("max_translation_speed_m_s", "max_rotation_speed_rad_s", "max_translation_step_m", "max_rotation_step_rad", "max_timeout_s", "max_relative_displacement_m"):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)) or float(value) <= 0:
                raise ValueError(f"{name} must be finite and positive")

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "MotionLimits":
        allowed = {"max_translation_speed_m_s", "max_rotation_speed_rad_s", "max_translation_step_m", "max_rotation_step_rad", "max_timeout_s", "max_relative_displacement_m"}
        unknown = set(data) - allowed
        if unknown: raise ValueError(f"unknown motion limit fields: {sorted(unknown)}")
        return cls(**dict(data))

    def validate(self, *, translation_speed_m_s: float, rotation_speed_rad_s: float, timeout_s: float, translation_step_m: float | None = None, rotation_step_rad: float | None = None) -> None:
        checks = ((translation_speed_m_s, self.max_translation_speed_m_s, "translation_speed"), (rotation_speed_rad_s, self.max_rotation_speed_rad_s, "rotation_speed"), (timeout_s, self.max_timeout_s, "timeout"))
        if translation_step_m is not None: checks += ((translation_step_m, self.max_translation_step_m, "translation_step"),)
        if rotation_step_rad is not None: checks += ((rotation_step_rad, self.max_rotation_step_rad, "rotation_step"),)
        for value, limit, name in checks:
            if not math.isfinite(float(value)) or float(value) < 0 or float(value) > limit:
                raise ValueError(f"{name}_limit_exceeded")

    def to_dict(self) -> dict[str, float]:
        return {key: float(value) for key, value in self.__dict__.items()}
