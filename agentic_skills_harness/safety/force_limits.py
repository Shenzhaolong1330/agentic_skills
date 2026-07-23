from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping


@dataclass(frozen=True)
class ForceLimits:
    max_force_n: float = 10.0
    max_torque_nm: float = 1.0
    max_distance_m: float = 0.010
    max_timeout_s: float = 10.0

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            if not math.isfinite(float(value)) or float(value) <= 0:
                raise ValueError(f"{name} must be finite and positive")

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "ForceLimits":
        allowed = set(cls.__dataclass_fields__)
        if set(data) - allowed: raise ValueError("unknown force limit field")
        return cls(**dict(data))

    def validate(self, *, force_n: float, torque_nm: float, distance_m: float, timeout_s: float) -> None:
        for name, value, limit in (("force", force_n, self.max_force_n), ("torque", torque_nm, self.max_torque_nm), ("distance", distance_m, self.max_distance_m), ("timeout", timeout_s, self.max_timeout_s)):
            if not math.isfinite(float(value)) or float(value) <= 0 or float(value) > limit:
                raise ValueError(f"{name}_limit_exceeded")
