from __future__ import annotations

from dataclasses import dataclass, field, fields, is_dataclass
from datetime import datetime, timezone
from enum import Enum
import json
from pathlib import Path
from typing import Any


class StrEnum(str, Enum):
    def __str__(self) -> str:
        return self.value


class SkillMode(StrEnum):
    MOCK = "mock"
    DRY_RUN = "dry_run"
    FROM_ARTIFACTS = "from_artifacts"
    LIVE = "live"


class TaskState(StrEnum):
    INIT = "INIT"
    PREFLIGHT_ROBOT_HEALTH_CHECK = "PREFLIGHT_ROBOT_HEALTH_CHECK"
    AUTO_RESET_RECOVERY = "AUTO_RESET_RECOVERY"
    RESET_RECOVERY_VERIFY = "RESET_RECOVERY_VERIFY"
    LOCATE_TUBE = "LOCATE_TUBE"
    SELECT_ARM = "SELECT_ARM"
    MOVE_TO_PREGRASP = "MOVE_TO_PREGRASP"
    ALIGN_GRASP_POSE = "ALIGN_GRASP_POSE"
    GRASP_TUBE_BODY = "GRASP_TUBE_BODY"
    HANDOVER = "HANDOVER"
    LOCATE_RACK = "LOCATE_RACK"
    NON_HOLDER_HOME = "NON_HOLDER_HOME"
    HOLDER_ALIGN_RACK_ABOVE = "HOLDER_ALIGN_RACK_ABOVE"
    WRIST_LOCATE_HOLE = "WRIST_LOCATE_HOLE"
    SELECT_HOLE = "SELECT_HOLE"
    INSERT_ALIGN_REFINE = "INSERT_ALIGN_REFINE"
    INSERT_DESCEND = "INSERT_DESCEND"
    RELEASE = "RELEASE"
    RETURN_HOME = "RETURN_HOME"
    POST_STAGE_ROBOT_HEALTH_CHECK = "POST_STAGE_ROBOT_HEALTH_CHECK"
    COMPLETE = "COMPLETE"
    ABORT = "ABORT"


class RobotHealthState(StrEnum):
    UNKNOWN = "UNKNOWN"
    READY = "READY"
    WARNING = "WARNING"
    ABNORMAL = "ABNORMAL"
    FAULT = "FAULT"
    ESTOP_OR_UNSAFE = "ESTOP_OR_UNSAFE"
    UNREACHABLE = "UNREACHABLE"


class ResetOutcome(StrEnum):
    NOT_NEEDED = "NOT_NEEDED"
    PLANNED_ONLY = "PLANNED_ONLY"
    MOCK_RESET_OK = "MOCK_RESET_OK"
    RESET_OK = "RESET_OK"
    RESET_FAILED = "RESET_FAILED"
    RESET_DENIED_BY_GATE = "RESET_DENIED_BY_GATE"
    RESET_SKIPPED_HELD_OBJECT_POLICY = "RESET_SKIPPED_HELD_OBJECT_POLICY"
    RESET_EXCEEDED_MAX_ATTEMPTS = "RESET_EXCEEDED_MAX_ATTEMPTS"


class HeldObjectState(StrEnum):
    NONE = "NONE"
    TUBE_BODY_SELECTED_ARM = "TUBE_BODY_SELECTED_ARM"
    TUBE_HANDOVER_BOTH_ARMS = "TUBE_HANDOVER_BOTH_ARMS"
    TUBE_HEAD_HOLDER_ARM = "TUBE_HEAD_HOLDER_ARM"
    TUBE_INSERTED_NOT_RELEASED = "TUBE_INSERTED_NOT_RELEASED"
    RELEASED = "RELEASED"
    UNKNOWN = "UNKNOWN"


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def to_plain(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value):
        result = {}
        for item_field in fields(value):
            item = getattr(value, item_field.name)
            result[item_field.name] = to_plain(item)
        return result
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): to_plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_plain(item) for item in value]
    return value


@dataclass
class Pose6D:
    frame: str
    xyz_m: list[float]
    source: str
    rotvec_rad: list[float] | None = None
    rpy_rad: list[float] | None = None
    quaternion_xyzw: list[float] | None = None
    timestamp: str | None = None
    confidence: float | None = None
    quality: dict[str, Any] | None = None

    def to_dict(self) -> dict[str, Any]:
        return to_plain(self)


@dataclass
class Error6DoF:
    translation_m: float
    rotation_rad: float
    translation_threshold_m: float
    rotation_threshold_rad: float
    within_threshold: bool
    source: str

    def to_dict(self) -> dict[str, Any]:
        return to_plain(self)


@dataclass
class RobotHealthStatus:
    ok: bool
    state: RobotHealthState
    source: str
    checked_at: str = field(default_factory=utc_now_iso)
    raw_result: Any = None
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    requires_reset: bool = False
    reset_recommended: bool = False
    reset_blocked_reason: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return to_plain(self)


@dataclass
class ResetRecoveryResult:
    ok: bool
    outcome: ResetOutcome
    attempted: bool
    executed: bool
    planned_only: bool
    before_health: RobotHealthStatus | None = None
    after_health: RobotHealthStatus | None = None
    command: list[str] | None = None
    gate_decision: dict[str, Any] | None = None
    attempt_index: int = 0
    max_attempts: int = 1
    held_object_state: HeldObjectState = HeldObjectState.NONE
    held_object_risk: bool = False
    resumed_after_reset: bool = False
    aborted_after_reset: bool = False
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    artifacts: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return to_plain(self)


@dataclass
class StageResult:
    stage: TaskState | str
    ok: bool
    status: str = "ok"
    retry_count: int = 0
    reset_attempted: bool = False
    reset_result: ResetRecoveryResult | dict[str, Any] | None = None
    inputs: dict[str, Any] = field(default_factory=dict)
    outputs: dict[str, Any] = field(default_factory=dict)
    metrics: dict[str, Any] = field(default_factory=dict)
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    artifacts: dict[str, Any] = field(default_factory=dict)
    started_at: str = field(default_factory=utc_now_iso)
    ended_at: str | None = None

    def close(self) -> "StageResult":
        self.ended_at = utc_now_iso()
        return self

    def to_dict(self) -> dict[str, Any]:
        return to_plain(self)


@dataclass
class SkillContext:
    run_id: str
    mode: SkillMode = SkillMode.MOCK
    hardware_allowed: bool = False
    execute: bool = False
    artifact_dir: str = ""
    manifest_path: str = "skill_manifest.json"
    robot_server: str | None = None
    frames: dict[str, Any] = field(default_factory=dict)
    thresholds: dict[str, Any] = field(default_factory=dict)
    recovery_policy: dict[str, Any] = field(default_factory=dict)
    reset_attempt_count: int = 0
    max_auto_reset_attempts: int = 1
    held_object_state: HeldObjectState = HeldObjectState.NONE
    calibration_snapshot: dict[str, Any] | None = None
    auto_reset_on_abnormal: bool = True
    resume_after_held_object_reset: bool = False
    mock_robot_health: str = "ready"
    robot_health_json: str | None = None
    reset_recovery_json: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return to_plain(self)


@dataclass
class SkillResult:
    ok: bool
    status: str
    outputs: dict[str, Any] = field(default_factory=dict)
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return to_plain(self)


@dataclass
class TaskResult:
    ok: bool
    task_state: TaskState
    completion_flag: bool
    physical_verified: bool
    context: SkillContext
    stages: list[StageResult | dict[str, Any]] = field(default_factory=list)
    outputs: dict[str, Any] = field(default_factory=dict)
    reset_recovery: list[ResetRecoveryResult | dict[str, Any]] = field(default_factory=list)
    stopped_reason: str | None = None
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    artifacts: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return to_plain(self)


def write_json(path: str | Path, obj: Any) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(to_plain(obj), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def read_json(path: str | Path) -> Any:
    return json.loads(Path(path).read_text(encoding="utf-8"))
