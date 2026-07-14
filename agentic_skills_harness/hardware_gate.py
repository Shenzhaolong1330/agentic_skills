from __future__ import annotations

from dataclasses import dataclass
import os
from typing import Any

from .types import SkillContext, SkillMode, to_plain


@dataclass
class GateDecision:
    allowed: bool
    reason: str
    mode: str
    requires_hardware: bool
    opens_camera: bool
    connects_robot_rpc: bool
    moves_robot: bool
    controls_gripper: bool
    allowed_as_recovery: bool
    token_checked: bool
    manifest_checked: bool
    planned_only: bool = False

    def to_dict(self) -> dict[str, Any]:
        return to_plain(self)


class HardwareGate:
    def __init__(self, manifest: dict[str, Any]):
        self.manifest = manifest
        gate = manifest.get("hardware_gate", {})
        self.env_token_name = gate.get("env_token_name", "AGENTIC_SKILLS_HARDWARE_TOKEN")

    def evaluate(self, context: SkillContext, entrypoint: dict[str, Any], *, recovery: bool = False) -> GateDecision:
        requires_hardware = bool(entrypoint.get("requires_hardware"))
        decision_base = {
            "mode": str(context.mode.value if isinstance(context.mode, SkillMode) else context.mode),
            "requires_hardware": requires_hardware,
            "opens_camera": bool(entrypoint.get("opens_camera")),
            "connects_robot_rpc": bool(entrypoint.get("connects_robot_rpc")),
            "moves_robot": bool(entrypoint.get("moves_robot")),
            "controls_gripper": bool(entrypoint.get("controls_gripper")),
            "allowed_as_recovery": bool(entrypoint.get("allowed_as_recovery")),
            "manifest_checked": True,
        }
        mode = context.mode if isinstance(context.mode, SkillMode) else SkillMode(str(context.mode))
        if mode in (SkillMode.MOCK, SkillMode.DRY_RUN, SkillMode.FROM_ARTIFACTS):
            return GateDecision(
                allowed=False,
                reason=f"{mode.value} mode records a plan and never executes hardware",
                token_checked=False,
                planned_only=True,
                **decision_base,
            )
        if not requires_hardware:
            return GateDecision(allowed=True, reason="entrypoint does not require hardware", token_checked=False, **decision_base)
        if recovery and not bool(entrypoint.get("allowed_as_recovery")):
            return GateDecision(allowed=False, reason="entrypoint is not allowed as recovery", token_checked=False, **decision_base)
        if not context.execute:
            return GateDecision(allowed=False, reason="live hardware requires execute=true", token_checked=False, **decision_base)
        if not context.hardware_allowed:
            return GateDecision(allowed=False, reason="live hardware requires hardware_allowed=true", token_checked=False, **decision_base)
        env_token = os.environ.get(self.env_token_name)
        if not env_token or not context.operator_token:
            return GateDecision(allowed=False, reason="operator token missing", token_checked=True, **decision_base)
        if context.operator_token != env_token:
            return GateDecision(allowed=False, reason="operator token mismatch", token_checked=True, **decision_base)
        return GateDecision(allowed=True, reason="hardware gate passed", token_checked=True, **decision_base)
