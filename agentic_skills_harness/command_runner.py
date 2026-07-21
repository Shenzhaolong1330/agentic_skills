from __future__ import annotations

from dataclasses import dataclass, field
import json
import subprocess
from typing import Any

from .hardware_gate import HardwareGate
from .types import SkillContext, SkillMode, to_plain


@dataclass
class CommandPlan:
    argv: list[str]
    entrypoint_ref: str
    requires_hardware: bool = False
    opens_camera: bool = False
    connects_robot_rpc: bool = False
    moves_robot: bool = False
    controls_gripper: bool = False
    allowed_as_recovery: bool = False
    would_execute: bool = False
    executed: bool = False
    recovery: bool = False
    reason: str = ""
    gate_decision: dict[str, Any] | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return to_plain(self)


@dataclass
class CommandResult:
    plan: CommandPlan
    returncode: int | None = None
    stdout: str = ""
    stderr: str = ""
    executed: bool = False
    planned_only: bool = False
    abnormal_robot_state_detected: bool = False
    raw_json: Any = None
    gate_decision: dict[str, Any] | None = None

    def to_dict(self) -> dict[str, Any]:
        return to_plain(self)


class CommandRunner:
    def __init__(self, manifest: dict[str, Any], gate: HardwareGate | None = None):
        self.manifest = manifest
        self.gate = gate or HardwareGate(manifest)

    def run(
        self,
        command: list[str],
        *,
        context: SkillContext,
        metadata: dict[str, Any] | None = None,
        safety_flags: dict[str, Any] | None = None,
        entrypoint_ref: str,
        entrypoint: dict[str, Any],
        recovery: bool = False,
    ) -> CommandResult:
        safety = safety_flags or {}
        plan = CommandPlan(
            argv=list(command),
            entrypoint_ref=entrypoint_ref,
            requires_hardware=bool(entrypoint.get("requires_hardware")),
            opens_camera=bool(entrypoint.get("opens_camera")),
            connects_robot_rpc=bool(entrypoint.get("connects_robot_rpc")),
            moves_robot=bool(entrypoint.get("moves_robot")),
            controls_gripper=bool(entrypoint.get("controls_gripper")),
            allowed_as_recovery=bool(entrypoint.get("allowed_as_recovery")),
            would_execute=bool(context.execute and context.mode == SkillMode.LIVE),
            recovery=bool(recovery),
            metadata={**safety, **(metadata or {})},
        )
        decision = self.gate.evaluate(context, entrypoint, recovery=recovery)
        plan.reason = decision.reason
        plan.gate_decision = decision.to_dict()
        if context.mode != SkillMode.LIVE or not decision.allowed:
            return CommandResult(
                plan=plan,
                planned_only=True,
                executed=False,
                gate_decision=decision.to_dict(),
                abnormal_robot_state_detected=False,
            )
        completed = subprocess.run(list(command), text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
        plan.executed = True
        result = CommandResult(
            plan=plan,
            returncode=int(completed.returncode),
            stdout=completed.stdout,
            stderr=completed.stderr,
            executed=True,
            planned_only=False,
            gate_decision=decision.to_dict(),
        )
        result.abnormal_robot_state_detected = classify_command_result_for_robot_abnormal(result)
        try:
            result.raw_json = json.loads(completed.stdout)
        except json.JSONDecodeError:
            result.raw_json = None
        return result


def classify_command_result_for_robot_abnormal(result: CommandResult | dict[str, Any]) -> bool:
    data = result.to_dict() if hasattr(result, "to_dict") else result
    text = " ".join(
        str(data.get(key, "")) for key in ("stdout", "stderr", "returncode")
    ).lower()
    raw = data.get("raw_json")
    if isinstance(raw, dict):
        status = raw.get("status_before") or raw.get("status") or raw
        if isinstance(status, dict):
            classification = str(status.get("classification") or status.get("state") or "").lower()
            if classification in {"needs_reset", "abnormal", "fault", "unreachable"}:
                return True
            if status.get("requires_reset") or status.get("reset_recommended"):
                return True
        if raw.get("abnormal_robot_state_detected"):
            return True
    robot_markers = [
        "robot abnormal",
        "robot fault",
        "controller fault",
        "recoverable error",
        "needs_reset",
        "current_errors",
        "reflex",
        "automatic_error_recovery",
        "unreachable",
    ]
    perception_markers = ["object was not found", "low confidence", "no candidates"]
    if any(marker in text for marker in perception_markers):
        return False
    return any(marker in text for marker in robot_markers)
