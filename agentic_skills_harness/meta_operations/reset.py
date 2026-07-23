from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from ..safety.estop_guard import EStopState
from ..safety.held_object_guard import HeldObjectState
from ..safety.recovery_guard import RecoveryGuard


@dataclass(frozen=True)
class ResetHomeContract:
    """High-risk home/reset contract; no implicit resume or release."""

    def execute(self, controller: Any, *, hardware_allowed: bool, execute: bool, allowed_as_recovery: bool, estop_state: EStopState | str, held_object_state: HeldObjectState | str) -> dict[str, Any]:
        if not hardware_allowed: return {"ok": False, "status": "DENIED", "reason": "hardware_allowed_required", "continued": False}
        if not execute: return {"ok": False, "status": "DENIED", "reason": "execute_required_for_side_effects", "continued": False}
        allowed, reason = RecoveryGuard().allow(estop_state=estop_state, held_object_state=held_object_state, current_error_known=True, allowed_as_recovery=allowed_as_recovery)
        if not allowed: return {"ok": False, "status": "DENIED", "reason": reason, "continued": False}
        reset = getattr(controller, "reset_home", None) or getattr(controller, "home", None)
        if not callable(reset): return {"ok": False, "status": "CAPABILITY_UNSUPPORTED", "reason": "reset_home_api_unavailable", "continued": False}
        reset()
        return {"ok": True, "status": "SUCCEEDED", "continued": False, "world_state_requires_reobserve": True, "holding_invalidated": True}
