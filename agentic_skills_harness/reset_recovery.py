from __future__ import annotations

from pathlib import Path
from typing import Any

from .hardware_gate import HardwareGate
from .manifest import find_entrypoint
from .robot_health import RobotHealthMonitor
from .types import (
    HeldObjectState,
    ResetOutcome,
    ResetRecoveryResult,
    RobotHealthState,
    RobotHealthStatus,
    SkillContext,
    SkillMode,
    read_json,
)

RESET_SKILL = "atomic-state-dual-franka-reset"
RESET_ENTRYPOINT = "check_and_reset_ensure"


class ResetRecoveryController:
    def __init__(self, manifest: dict[str, Any], repo_root: str | Path | None = None):
        self.manifest = manifest
        self.repo_root = Path(repo_root or Path(__file__).resolve().parents[1])
        self.health_monitor = RobotHealthMonitor(manifest, repo_root=self.repo_root)
        self.gate = HardwareGate(manifest)

    def should_reset(self, health: RobotHealthStatus | None, stage_result: Any, context: SkillContext) -> bool:
        if not context.auto_reset_on_abnormal:
            return False
        if health and (health.requires_reset or health.state in {RobotHealthState.ABNORMAL, RobotHealthState.FAULT, RobotHealthState.UNREACHABLE}):
            return True
        if isinstance(stage_result, dict) and stage_result.get("abnormal_robot_state_detected"):
            return True
        return False

    def build_reset_command(self, context: SkillContext) -> list[str]:
        entrypoint = find_entrypoint(self.manifest, RESET_SKILL, RESET_ENTRYPOINT)
        script = self.repo_root / entrypoint["path"]
        command = ["python3", str(script), "ensure", "--compact"]
        if context.robot_server and ":" in context.robot_server:
            host, port = context.robot_server.rsplit(":", 1)
            command.extend(["--server-host", host, "--server-port", port])
        if context.mode == SkillMode.LIVE and context.execute:
            command.append("--execute")
        return command

    def perform_auto_reset(
        self,
        context: SkillContext,
        *,
        reason: str,
        held_object_state: HeldObjectState,
        before_health: RobotHealthStatus | None = None,
    ) -> ResetRecoveryResult:
        attempt_index = context.reset_attempt_count + 1
        max_attempts = int(context.max_auto_reset_attempts)
        held_risk = held_object_state not in {HeldObjectState.NONE, HeldObjectState.RELEASED}
        if context.reset_attempt_count >= max_attempts:
            return ResetRecoveryResult(
                ok=False,
                outcome=ResetOutcome.RESET_EXCEEDED_MAX_ATTEMPTS,
                attempted=False,
                executed=False,
                planned_only=False,
                before_health=before_health,
                attempt_index=attempt_index,
                max_attempts=max_attempts,
                held_object_state=held_object_state,
                held_object_risk=held_risk,
                aborted_after_reset=True,
                errors=[f"max auto reset attempts exceeded while handling {reason}"],
            )
        context.reset_attempt_count += 1
        command = self.build_reset_command(context)
        mode = context.mode if isinstance(context.mode, SkillMode) else SkillMode(str(context.mode))
        if mode == SkillMode.MOCK:
            after = RobotHealthStatus(ok=True, state=RobotHealthState.READY, source="after_mock_reset", raw_result={"mock_reset": True})
            return ResetRecoveryResult(
                ok=True,
                outcome=ResetOutcome.MOCK_RESET_OK,
                attempted=True,
                executed=False,
                planned_only=False,
                before_health=before_health,
                after_health=after,
                command=command,
                attempt_index=attempt_index,
                max_attempts=max_attempts,
                held_object_state=held_object_state,
                held_object_risk=held_risk,
                resumed_after_reset=not held_risk or context.resume_after_held_object_reset,
                aborted_after_reset=held_risk and not context.resume_after_held_object_reset,
                warnings=["held object state requires reverification after reset"] if held_risk else [],
            )
        if mode == SkillMode.DRY_RUN:
            after = RobotHealthStatus(ok=False, state=RobotHealthState.UNKNOWN, source="after_dry_run_reset_plan", raw_result={"planned_reset": command})
            return ResetRecoveryResult(
                ok=True,
                outcome=ResetOutcome.PLANNED_ONLY,
                attempted=True,
                executed=False,
                planned_only=True,
                before_health=before_health,
                after_health=after,
                command=command,
                attempt_index=attempt_index,
                max_attempts=max_attempts,
                held_object_state=held_object_state,
                held_object_risk=held_risk,
                resumed_after_reset=not held_risk,
                aborted_after_reset=held_risk,
                warnings=["dry_run planned reset only; no hardware was executed"],
            )
        if mode == SkillMode.FROM_ARTIFACTS:
            if context.reset_recovery_json:
                loaded = read_json(context.reset_recovery_json)
                return ResetRecoveryResult(
                    ok=bool(loaded.get("ok")),
                    outcome=ResetOutcome(str(loaded.get("outcome", ResetOutcome.PLANNED_ONLY.value))),
                    attempted=bool(loaded.get("attempted", True)),
                    executed=bool(loaded.get("executed", False)),
                    planned_only=bool(loaded.get("planned_only", True)),
                    command=loaded.get("command", command),
                    attempt_index=attempt_index,
                    max_attempts=max_attempts,
                    held_object_state=held_object_state,
                    held_object_risk=held_risk,
                    artifacts={"loaded_reset_artifact": loaded},
                )
            return ResetRecoveryResult(
                ok=True,
                outcome=ResetOutcome.PLANNED_ONLY,
                attempted=True,
                executed=False,
                planned_only=True,
                before_health=before_health,
                command=command,
                attempt_index=attempt_index,
                max_attempts=max_attempts,
                held_object_state=held_object_state,
                held_object_risk=held_risk,
                warnings=["from_artifacts had no reset artifact; recorded reset plan only"],
            )
        entrypoint = find_entrypoint(self.manifest, RESET_SKILL, RESET_ENTRYPOINT)
        decision = self.gate.evaluate(context, entrypoint, recovery=True)
        if not decision.allowed:
            return ResetRecoveryResult(
                ok=False,
                outcome=ResetOutcome.RESET_DENIED_BY_GATE,
                attempted=True,
                executed=False,
                planned_only=bool(decision.planned_only),
                before_health=before_health,
                command=command,
                gate_decision=decision.to_dict(),
                attempt_index=attempt_index,
                max_attempts=max_attempts,
                held_object_state=held_object_state,
                held_object_risk=held_risk,
                aborted_after_reset=True,
                errors=[decision.reason],
            )
        return ResetRecoveryResult(
            ok=False,
            outcome=ResetOutcome.RESET_OK,
            attempted=True,
            executed=True,
            planned_only=False,
            before_health=before_health,
            command=command,
            gate_decision=decision.to_dict(),
            attempt_index=attempt_index,
            max_attempts=max_attempts,
            held_object_state=held_object_state,
            held_object_risk=held_risk,
            warnings=["live execution path is command-planned; actual subprocess is owned by CommandRunner"],
        )

    def verify_after_reset(self, context: SkillContext) -> RobotHealthStatus:
        return self.health_monitor.check_health(context, source="after_reset")

    def decide_resume_or_abort(self, context: SkillContext, reset_result: ResetRecoveryResult, current_stage: str) -> dict[str, Any]:
        if not reset_result.ok:
            return {"resume": False, "abort": True, "stopped_reason": "reset_failed_or_denied"}
        if reset_result.held_object_risk and not context.resume_after_held_object_reset:
            return {
                "resume": False,
                "abort": True,
                "stopped_reason": "reset_while_holding_tube_requires_reverification",
                "stage": current_stage,
            }
        return {"resume": True, "abort": False, "stage": current_stage}
