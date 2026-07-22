from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from ..contracts.enums import ErrorCode
from ..contracts.errors import ErrorInfo
from ..dispatch.error_mapping import make_error


@dataclass
class ExecutionError(Exception):
    code: ErrorCode | str
    message: str
    details: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        super().__init__(self.message)

    def to_error_info(self) -> ErrorInfo:
        return make_error(self.code, self.message, details=self.details, source="graph_executor")


class InvalidStateTransition(ExecutionError):
    def __init__(self, current: str, target: str) -> None:
        super().__init__(ErrorCode.INVALID_STATE_TRANSITION, f"invalid node state transition: {current} -> {target}", {"current": current, "target": target})


class CheckpointError(ExecutionError):
    pass


class EventLogError(ExecutionError):
    pass
