from __future__ import annotations

from enum import Enum


class HeldObjectState(str, Enum):
    NONE_CONFIRMED = "NONE_CONFIRMED"
    HOLDING_CONFIRMED = "HOLDING_CONFIRMED"
    HOLDING_TENTATIVE = "HOLDING_TENTATIVE"
    UNKNOWN = "UNKNOWN"


class HeldObjectGuard:
    def allow_recovery(self, state: HeldObjectState | str) -> bool:
        return (state if isinstance(state, HeldObjectState) else HeldObjectState(state)) == HeldObjectState.NONE_CONFIRMED

    def allow_release(self, *, object_not_held: bool, object_supported: bool, receiving_gripper_verified: bool = False, explicit_empty_test: bool = False) -> bool:
        return bool(object_not_held or object_supported or receiving_gripper_verified or explicit_empty_test)

    def require_recovery(self, state: HeldObjectState | str) -> None:
        if not self.allow_recovery(state):
            raise PermissionError("held_object_guard")
