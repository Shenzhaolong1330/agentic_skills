from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from pathlib import Path
from typing import Any, Mapping
import json


class AcceptanceLevel(IntEnum):
    H0_CONFIG = 0
    H1_READ_ONLY = 1
    H2_GRIPPER_EMPTY = 2
    H3_MOTION_P2P = 3
    H4_MOTION_RELATIVE = 4
    H5_STOP = 5
    H6_GRASP_VERIFICATION = 6
    H7_RECOVERY = 7
    H8_RESET_HOME = 8


@dataclass(frozen=True)
class AcceptanceStep:
    step_id: str
    level: str
    capability_id: str
    description: str
    side_effects: tuple[str, ...] = ()
    requires_operator_check: bool = True

    def to_dict(self) -> dict[str, Any]:
        return {"step_id": self.step_id, "level": self.level, "capability_id": self.capability_id, "description": self.description, "side_effects": list(self.side_effects), "requires_operator_check": self.requires_operator_check}


@dataclass(frozen=True)
class AcceptanceObservation:
    step_id: str
    timestamp: str
    automatic_checks: Mapping[str, Any]
    operator_checks: Mapping[str, Any]
    notes: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return {"step_id": self.step_id, "timestamp": self.timestamp, "automatic_checks": dict(self.automatic_checks), "operator_checks": dict(self.operator_checks), "notes": list(self.notes)}


@dataclass(frozen=True)
class AcceptancePlan:
    plan_id: str
    plan_version: str
    steps: tuple[AcceptanceStep, ...]
    graph_live_enabled: bool = False

    def __post_init__(self) -> None:
        if self.graph_live_enabled:
            raise ValueError("S10 acceptance plans cannot enable GraphExecutor live")
        seen: set[int] = set()
        for step in self.steps:
            level = AcceptanceLevel[step.level]
            if int(level) in seen:
                raise ValueError("acceptance plan contains duplicate levels")
            seen.add(int(level))

    def to_dict(self) -> dict[str, Any]:
        return {"plan_id": self.plan_id, "plan_version": self.plan_version, "graph_live_enabled": self.graph_live_enabled, "steps": [item.to_dict() for item in self.steps]}


@dataclass(frozen=True)
class AcceptanceResult:
    capability_id: str
    capability_version: str
    adapter_digest: str
    input_schema_digest: str
    output_schema_digest: str
    hardware_fingerprint_digest: str
    workspace_digest: str
    calibration_hash: str
    acceptance_level: str
    automatic_checks: Mapping[str, Any]
    operator_checks: Mapping[str, Any]
    passed: bool
    failed_reasons: tuple[str, ...] = ()
    started_at: str = ""
    ended_at: str = ""
    real_hardware: bool = False

    def __post_init__(self) -> None:
        # Promotion, rather than construction, enforces real-hardware evidence.
        # Keeping the model permissive lets tests exercise fake passed payloads
        # and prove that promotion still rejects them.
        return None

    def to_dict(self) -> dict[str, Any]:
        return {"capability_id": self.capability_id, "capability_version": self.capability_version, "adapter_digest": self.adapter_digest, "input_schema_digest": self.input_schema_digest, "output_schema_digest": self.output_schema_digest, "hardware_fingerprint_digest": self.hardware_fingerprint_digest, "workspace_digest": self.workspace_digest, "calibration_hash": self.calibration_hash, "acceptance_level": self.acceptance_level, "automatic_checks": dict(self.automatic_checks), "operator_checks": dict(self.operator_checks), "passed": self.passed, "failed_reasons": list(self.failed_reasons), "started_at": self.started_at, "ended_at": self.ended_at, "real_hardware": self.real_hardware}


def default_acceptance_plan() -> AcceptancePlan:
    values = (
        ("H0_CONFIG", "config", "H0 configuration and safety preparation", ()),
        ("H1_READ_ONLY", "observe.robot_state", "sample robot and scene state", ()),
        ("H2_GRIPPER_EMPTY", "gripper.command", "empty-gripper open/close/status checks", ("may move gripper",)),
        ("H3_MOTION_P2P", "motion.move_to_pose", "bounded fixed acceptance poses", ("moves robot",)),
        ("H4_MOTION_RELATIVE", "motion.move_relative", "bounded relative motion", ("moves robot",)),
        ("H5_STOP", "motion.safe_stop", "verify fixed stop binding", ("stops motion",)),
        ("H6_GRASP_VERIFICATION", "gripper.verify_grasp", "independent grasp evidence", ("may move gripper",)),
        ("H7_RECOVERY", "robot.recover_fault", "benign recoverable fault only", ("recovery",)),
        ("H8_RESET_HOME", "procedure.reset_home", "final empty-gripper home reset", ("moves robot", "may release object")),
    )
    return AcceptancePlan("s10-hardware-acceptance", "1.0.0", tuple(AcceptanceStep(f"{level.lower()}", level, capability, description, side_effects) for level, capability, description, side_effects in values))


def write_json(path: str | Path, value: Mapping[str, Any]) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(dict(value), ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
