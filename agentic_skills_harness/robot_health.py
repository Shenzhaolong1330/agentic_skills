from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .manifest import find_entrypoint
from .types import RobotHealthState, RobotHealthStatus, SkillContext, SkillMode, read_json

RESET_SKILL = "atomic-state-dual-franka-reset"
STATUS_ENTRYPOINT = "check_and_reset_status"


class RobotHealthMonitor:
    def __init__(self, manifest: dict[str, Any], repo_root: str | Path | None = None):
        self.manifest = manifest
        self.repo_root = Path(repo_root or Path(__file__).resolve().parents[1])

    def build_status_command(self, context: SkillContext) -> list[str]:
        entrypoint = find_entrypoint(self.manifest, RESET_SKILL, STATUS_ENTRYPOINT)
        script = self.repo_root / entrypoint["path"]
        command = ["python3", str(script), "status", "--compact"]
        if context.robot_server and ":" in context.robot_server:
            host, port = context.robot_server.rsplit(":", 1)
            command.extend(["--server-host", host, "--server-port", port])
        return command

    def check_health(self, context: SkillContext, source: str = "preflight") -> RobotHealthStatus:
        mode = context.mode if isinstance(context.mode, SkillMode) else SkillMode(str(context.mode))
        injected = (context.mock_robot_health or "").lower()
        if mode in (SkillMode.MOCK, SkillMode.DRY_RUN) and injected in {"abnormal", "fault", "unreachable", "estop"}:
            state = {
                "abnormal": RobotHealthState.ABNORMAL,
                "fault": RobotHealthState.FAULT,
                "unreachable": RobotHealthState.UNREACHABLE,
                "estop": RobotHealthState.ESTOP_OR_UNSAFE,
            }[injected]
            return RobotHealthStatus(
                ok=False,
                state=state,
                source=f"{source}:{mode.value}:mock_injection",
                raw_result={"mock_robot_health": injected, "command_plan": self.build_status_command(context)},
                requires_reset=state in {RobotHealthState.ABNORMAL, RobotHealthState.FAULT, RobotHealthState.UNREACHABLE},
                reset_recommended=state in {RobotHealthState.ABNORMAL, RobotHealthState.FAULT, RobotHealthState.UNREACHABLE},
            )
        if mode == SkillMode.MOCK:
            return RobotHealthStatus(ok=True, state=RobotHealthState.READY, source=f"{source}:mock", raw_result={"mock_robot_health": "ready"})
        if mode == SkillMode.DRY_RUN:
            return RobotHealthStatus(
                ok=False,
                state=RobotHealthState.UNKNOWN,
                source=f"{source}:dry_run",
                raw_result={"planned_command": self.build_status_command(context)},
                requires_reset=False,
                reset_recommended=False,
                warnings=["dry_run does not connect robot RPC"],
            )
        if mode == SkillMode.FROM_ARTIFACTS:
            if not context.robot_health_json:
                return RobotHealthStatus(
                    ok=False,
                    state=RobotHealthState.UNKNOWN,
                    source=f"{source}:from_artifacts",
                    errors=["robot_health_json was not provided"],
                )
            return self.classify_health(read_json(context.robot_health_json), source=f"{source}:from_artifacts")
        # Live execution is intentionally command-plan based here; task runner must call CommandRunner with HardwareGate.
        return RobotHealthStatus(
            ok=False,
            state=RobotHealthState.UNKNOWN,
            source=f"{source}:live_plan",
            raw_result={"planned_command": self.build_status_command(context)},
            warnings=["live health check requires HardwareGate and CommandRunner"],
        )

    def parse_health_output(self, raw_result: Any) -> RobotHealthStatus:
        if isinstance(raw_result, str):
            try:
                raw_result = json.loads(raw_result)
            except json.JSONDecodeError:
                return self.classify_health(raw_result, source="raw_text")
        return self.classify_health(raw_result, source="raw_json")

    def classify_health(self, raw_json_or_text: Any, source: str = "artifact") -> RobotHealthStatus:
        if isinstance(raw_json_or_text, RobotHealthStatus):
            return raw_json_or_text
        if isinstance(raw_json_or_text, str):
            text = raw_json_or_text.lower()
            if any(marker in text for marker in ("needs_reset", "abnormal", "fault", "reflex", "unreachable")):
                return RobotHealthStatus(ok=False, state=RobotHealthState.ABNORMAL, source=source, raw_result=raw_json_or_text, requires_reset=True, reset_recommended=True)
            return RobotHealthStatus(ok=False, state=RobotHealthState.UNKNOWN, source=source, raw_result=raw_json_or_text)
        data = raw_json_or_text if isinstance(raw_json_or_text, dict) else {}
        status = data.get("status_before") if isinstance(data.get("status_before"), dict) else data
        classification = str(status.get("classification") or status.get("state") or "").upper()
        mapping = {
            "HEALTHY": RobotHealthState.READY,
            "READY": RobotHealthState.READY,
            "WARNING": RobotHealthState.WARNING,
            "NEEDS_RESET": RobotHealthState.ABNORMAL,
            "ABNORMAL": RobotHealthState.ABNORMAL,
            "FAULT": RobotHealthState.FAULT,
            "UNKNOWN": RobotHealthState.UNKNOWN,
            "UNREACHABLE": RobotHealthState.UNREACHABLE,
            "MANUAL_INTERVENTION": RobotHealthState.ESTOP_OR_UNSAFE,
            "USER_STOPPED": RobotHealthState.ESTOP_OR_UNSAFE,
        }
        state = mapping.get(classification, RobotHealthState.UNKNOWN)
        requires_reset = bool(status.get("requires_reset") or status.get("needs_reset") or state in {RobotHealthState.ABNORMAL, RobotHealthState.FAULT, RobotHealthState.UNREACHABLE})
        return RobotHealthStatus(
            ok=state in {RobotHealthState.READY, RobotHealthState.WARNING} and not requires_reset,
            state=state,
            source=source,
            raw_result=raw_json_or_text,
            errors=[str(item) for item in status.get("errors", [])] if isinstance(status.get("errors"), list) else [],
            warnings=[str(item) for item in status.get("warnings", [])] if isinstance(status.get("warnings"), list) else [],
            requires_reset=requires_reset,
            reset_recommended=requires_reset,
        )
