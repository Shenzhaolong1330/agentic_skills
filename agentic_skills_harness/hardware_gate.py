from __future__ import annotations

from dataclasses import dataclass
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
    hardware_allowed_checked: bool
    execute_checked: bool
    manifest_checked: bool
    planned_only: bool = False

    def to_dict(self) -> dict[str, Any]:
        return to_plain(self)


def _context_mode(context: SkillContext) -> SkillMode:
    return context.mode if isinstance(context.mode, SkillMode) else SkillMode(str(context.mode))


def _entrypoint_flags(entrypoint: dict[str, Any]) -> dict[str, bool]:
    return {
        "opens_camera": bool(entrypoint.get("opens_camera")),
        "connects_robot_rpc": bool(entrypoint.get("connects_robot_rpc")),
        "moves_robot": bool(entrypoint.get("moves_robot")),
        "controls_gripper": bool(entrypoint.get("controls_gripper")),
        "changes_robot_state": bool(entrypoint.get("changes_robot_state")),
        "physical_side_effects": bool(entrypoint.get("physical_side_effects")),
        "allowed_as_recovery": bool(entrypoint.get("allowed_as_recovery")),
    }


def evaluate_hardware_authorization(
    context: SkillContext,
    entrypoint: dict[str, Any],
    *,
    recovery: bool = False,
) -> GateDecision:
    """Evaluate the single hardware authorization policy.

    This function is intentionally free of I/O. Callers must evaluate it before
    importing device adapters, starting services, opening cameras, or creating
    RPC clients.
    """

    flags = _entrypoint_flags(entrypoint)
    requires_hardware = bool(entrypoint.get("requires_hardware")) or any(
        flags[name] for name in ("opens_camera", "connects_robot_rpc", "moves_robot", "controls_gripper", "changes_robot_state")
    )
    has_side_effects = any(
        flags[name] for name in ("moves_robot", "controls_gripper", "changes_robot_state", "physical_side_effects")
    )
    mode = _context_mode(context)
    base = {
        "mode": mode.value,
        "requires_hardware": requires_hardware,
        "opens_camera": flags["opens_camera"],
        "connects_robot_rpc": flags["connects_robot_rpc"],
        "moves_robot": flags["moves_robot"],
        "controls_gripper": flags["controls_gripper"],
        "allowed_as_recovery": flags["allowed_as_recovery"],
        "manifest_checked": True,
    }
    if mode in (SkillMode.MOCK, SkillMode.DRY_RUN, SkillMode.FROM_ARTIFACTS) and requires_hardware:
        return GateDecision(
            allowed=False,
            reason="non_live_mode_planned_only",
            hardware_allowed_checked=False,
            execute_checked=False,
            planned_only=True,
            **base,
        )
    if not requires_hardware:
        return GateDecision(
            allowed=True,
            reason="hardware_gate_passed",
            hardware_allowed_checked=False,
            execute_checked=False,
            **base,
        )
    if mode != SkillMode.LIVE:
        return GateDecision(
            allowed=False,
            reason="non_live_mode_planned_only",
            hardware_allowed_checked=False,
            execute_checked=False,
            planned_only=True,
            **base,
        )
    if not bool(context.hardware_allowed):
        return GateDecision(
            allowed=False,
            reason="hardware_allowed_required",
            hardware_allowed_checked=True,
            execute_checked=False,
            **base,
        )
    if recovery and not flags["allowed_as_recovery"]:
        return GateDecision(
            allowed=False,
            reason="recovery_not_allowed",
            hardware_allowed_checked=True,
            execute_checked=has_side_effects,
            **base,
        )
    if has_side_effects and not bool(context.execute):
        return GateDecision(
            allowed=False,
            reason="execute_required_for_side_effects",
            hardware_allowed_checked=True,
            execute_checked=True,
            **base,
        )
    return GateDecision(
        allowed=True,
        reason="hardware_gate_passed",
        hardware_allowed_checked=True,
        execute_checked=has_side_effects,
        **base,
    )


class HardwareGate:
    def __init__(self, manifest: dict[str, Any]):
        self.manifest = manifest

    def evaluate(self, context: SkillContext, entrypoint: dict[str, Any], *, recovery: bool = False) -> GateDecision:
        return evaluate_hardware_authorization(context, entrypoint, recovery=recovery)
