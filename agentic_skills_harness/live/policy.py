from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

from .readiness import ReadinessRegistry


@dataclass(frozen=True)
class LivePreflightRequest:
    capability_id: str
    mode: str
    hardware_allowed: bool
    execute: bool
    has_side_effects: bool
    required_acceptance_level: str
    capability_version: str
    adapter_digest: str
    input_schema_digest: str
    output_schema_digest: str
    hardware_fingerprint_digest: str
    calibration_hash: str
    workspace_digest: str
    robot_health: str = "UNKNOWN"
    estop_or_unsafe: bool = True
    resources_available: bool = False
    input_valid: bool = False
    workspace_valid: bool = False
    speed_valid: bool = False
    force_valid: bool = True
    held_object_safe: bool = False
    verifier_available: bool = False
    recovery_allowed: bool = False


@dataclass(frozen=True)
class LivePreflightResult:
    allowed: bool
    reasons: tuple[str, ...] = ()
    backend_calls_allowed: int = 0
    evidence_updated: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {"allowed": self.allowed, "reasons": list(self.reasons), "backend_calls_allowed": self.backend_calls_allowed, "evidence_updated": self.evidence_updated}


class LivePreflightPolicy:
    """Fail-closed checks for a future atomic live runner.

    This policy deliberately does not call a backend and cannot be used to
    switch the generic GraphExecutor into live mode.
    """

    def __init__(self, readiness: ReadinessRegistry) -> None:
        self.readiness = readiness

    def evaluate(self, request: LivePreflightRequest) -> LivePreflightResult:
        reasons: list[str] = []
        if request.mode != "live": reasons.append("mode_live_required")
        if not request.hardware_allowed: reasons.append("hardware_allowed_required")
        if request.has_side_effects and not request.execute: reasons.append("execute_required_for_side_effects")
        try:
            ok, reason = self.readiness.can_live(request.capability_id, acceptance_level=request.required_acceptance_level, hardware_fingerprint_digest=request.hardware_fingerprint_digest, calibration_hash=request.calibration_hash, capability_version=request.capability_version, adapter_digest=request.adapter_digest, input_schema_digest=request.input_schema_digest, output_schema_digest=request.output_schema_digest, workspace_digest=request.workspace_digest)
            if not ok: reasons.append(reason)
        except Exception:
            reasons.append("readiness_missing")
        if request.robot_health != "READY": reasons.append("robot_health_not_ready")
        if request.estop_or_unsafe: reasons.append("estop_or_unsafe")
        if not request.resources_available: reasons.append("resources_unavailable")
        if not request.input_valid: reasons.append("input_schema_invalid")
        if not request.workspace_valid: reasons.append("workspace_invalid")
        if not request.speed_valid: reasons.append("speed_limits_invalid")
        if not request.force_valid: reasons.append("force_limits_invalid")
        if not request.held_object_safe: reasons.append("held_object_guard")
        if not request.verifier_available: reasons.append("verifier_unavailable")
        if request.has_side_effects and request.recovery_allowed is False and request.required_acceptance_level == "H7_RECOVERY": reasons.append("recovery_authorization_required")
        return LivePreflightResult(not reasons, tuple(dict.fromkeys(reasons)), int(not reasons), False)

    def require(self, request: LivePreflightRequest) -> None:
        result = self.evaluate(request)
        if not result.allowed:
            raise PermissionError("live preflight rejected: " + ", ".join(result.reasons))
