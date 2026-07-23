from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Protocol
from datetime import datetime, timezone

from ..safety.estop_guard import EStopGuard, EStopState
from ..safety.held_object_guard import HeldObjectGuard, HeldObjectState


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _call(client: Any, name: str) -> Any:
    method = getattr(client, name, None)
    return method() if callable(method) else None


def observe_robot_state(client: Any, *, source: str = "robot_state_adapter") -> dict[str, Any]:
    """Read only adapter; unavailable fields remain explicitly UNKNOWN."""
    if client is None: return {"ok": False, "data_integrity": "UNKNOWN", "source": source, "timestamp": _now(), "errors": ["robot_client_unavailable"]}
    values: dict[str, Any] = {}
    for key, method in (("joint_positions", "get_joint_positions"), ("ee_pose", "get_ee_pose"), ("gripper_state", "get_gripper_state"), ("wrench", "get_wrench"), ("controller_state", "get_controller_state"), ("current_errors", "get_current_errors"), ("robot_mode", "get_robot_mode"), ("estop_state", "get_estop_state"), ("motion_state", "get_motion_state")):
        try: values[key] = _call(client, method) if callable(getattr(client, method, None)) else "UNKNOWN"
        except Exception as exc: values[key] = "UNKNOWN"; values.setdefault("errors", []).append(f"{method}:{type(exc).__name__}")
    values.setdefault("errors", [])
    values.update({"ok": True, "source": source, "timestamp": _now(), "moving": values.get("motion_state") in {"MOVING", "RUNNING"}, "data_integrity": "OK" if all(value != "UNKNOWN" for key, value in values.items() if key not in {"errors"}) else "PARTIAL"})
    return values


@dataclass(frozen=True)
class RobotStateAdapter:
    client: Any

    def observe(self) -> dict[str, Any]:
        return observe_robot_state(self.client)


@dataclass(frozen=True)
class SafeStopContract:
    def execute(self, controller: Any, *, timeout_s: float = 2.0) -> dict[str, Any]:
        stop = getattr(controller, "stop", None) or getattr(controller, "hold", None) or getattr(controller, "cancel_motion", None)
        if not callable(stop): return {"ok": False, "status": "CAPABILITY_UNSUPPORTED", "verified": False, "reason": "stop_api_unavailable"}
        stop()
        state = observe_robot_state(controller)
        motion_state = state.get("motion_state")
        verified = motion_state in {"STOPPED", "HOLD", "IDLE"}
        return {"ok": verified, "status": "SUCCEEDED" if verified else "UNKNOWN", "verified": verified, "motion_state": motion_state, "did_not_reset_home": True, "did_not_open_gripper": True}


@dataclass(frozen=True)
class FaultRecoveryContract:
    def execute(self, controller: Any, *, estop_state: EStopState | str, held_object_state: HeldObjectState | str, current_error_known: bool, allowed_as_recovery: bool) -> dict[str, Any]:
        allowed, reason = __import__("agentic_skills_harness.safety.recovery_guard", fromlist=["RecoveryGuard"]).RecoveryGuard().allow(estop_state=estop_state, held_object_state=held_object_state, current_error_known=current_error_known, allowed_as_recovery=allowed_as_recovery)
        if not allowed: return {"ok": False, "status": "DENIED", "reason": reason, "continued": False}
        recover = getattr(controller, "recover_fault", None) or getattr(controller, "recover", None) or getattr(controller, "clear_fault", None)
        if not callable(recover): return {"ok": False, "status": "CAPABILITY_UNSUPPORTED", "reason": "recovery_api_unavailable", "continued": False}
        recover()
        return {"ok": True, "status": "SUCCEEDED", "state_after": observe_robot_state(controller), "continued": False, "world_state_requires_reobserve": True}
