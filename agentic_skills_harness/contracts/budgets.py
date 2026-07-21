from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from .serialization import ContractValidationError, reject_unknown, require_number, to_plain


@dataclass(frozen=True)
class ExecutionBudget:
    max_nodes: int = 64
    max_depth: int = 16
    max_elapsed_s: float = 300.0
    max_tool_calls: int = 64
    max_replans: int = 4
    max_recovery_actions: int = 2
    max_same_error_retries: int = 2
    no_progress_limit: int = 3

    def __post_init__(self) -> None:
        positive = ("max_nodes", "max_depth", "max_elapsed_s", "max_tool_calls")
        nonnegative = ("max_replans", "max_recovery_actions", "max_same_error_retries", "no_progress_limit")
        for name in positive:
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)) or value <= 0:
                raise ContractValidationError(f"{name} must be a finite positive number")
            if name != "max_elapsed_s" and int(value) != value:
                raise ContractValidationError(f"{name} must be an integer")
        for name in nonnegative:
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)) or value < 0:
                raise ContractValidationError(f"{name} must be a finite non-negative number")
            if int(value) != value:
                raise ContractValidationError(f"{name} must be an integer")
        for name in (*positive, *nonnegative):
            if name != "max_elapsed_s":
                object.__setattr__(self, name, int(getattr(self, name)))
        object.__setattr__(self, "max_elapsed_s", float(self.max_elapsed_s))

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ExecutionBudget":
        names = ("max_nodes", "max_depth", "max_elapsed_s", "max_tool_calls", "max_replans", "max_recovery_actions", "max_same_error_retries", "no_progress_limit")
        reject_unknown(data, names, names)
        return cls(**data)

    def to_dict(self) -> dict[str, Any]:
        return to_plain(self)
