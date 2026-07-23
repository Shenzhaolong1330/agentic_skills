from __future__ import annotations

from enum import Enum


class EStopState(str, Enum):
    CLEAR = "CLEAR"
    ESTOP = "ESTOP"
    UNSAFE = "UNSAFE"
    UNKNOWN = "UNKNOWN"


class EStopGuard:
    def allow_action(self, state: EStopState | str) -> bool:
        return (state if isinstance(state, EStopState) else EStopState(state)) == EStopState.CLEAR

    def allow_auto_recovery(self, state: EStopState | str) -> bool:
        return self.allow_action(state)

    def require_clear(self, state: EStopState | str) -> None:
        if not self.allow_action(state):
            raise PermissionError("estop_or_unsafe")
