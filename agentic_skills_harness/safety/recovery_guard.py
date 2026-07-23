from __future__ import annotations

from .estop_guard import EStopState
from .held_object_guard import HeldObjectState


class RecoveryGuard:
    def allow(self, *, estop_state: EStopState | str, held_object_state: HeldObjectState | str, current_error_known: bool, allowed_as_recovery: bool) -> tuple[bool, str]:
        estop = estop_state if isinstance(estop_state, EStopState) else EStopState(estop_state)
        held = held_object_state if isinstance(held_object_state, HeldObjectState) else HeldObjectState(held_object_state)
        if estop != EStopState.CLEAR: return False, "estop_or_unsafe"
        if held != HeldObjectState.NONE_CONFIRMED: return False, "held_object_guard"
        if not current_error_known: return False, "current_error_unknown"
        if not allowed_as_recovery: return False, "recovery_not_allowed"
        return True, "allowed"
