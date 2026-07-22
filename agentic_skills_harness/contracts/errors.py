from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .enums import ErrorCategory, ErrorCode, ErrorSeverity
from .serialization import (
    ContractValidationError,
    ensure_jsonable,
    reject_unknown,
    require_bool,
    require_enum,
    require_object,
    require_string,
    stable_dumps,
    to_plain,
)




def _entry(category: ErrorCategory, severity: ErrorSeverity, retryable: bool, safe_to_replan: bool, requires_human: bool) -> dict[str, Any]:
    return {
        "category": category,
        "severity": severity,
        "retryable": retryable,
        "safe_to_replan": safe_to_replan,
        "requires_human": requires_human,
    }


ERROR_CATALOG: dict[ErrorCode, dict[str, Any]] = {
    ErrorCode.UNKNOWN_ERROR: _entry(ErrorCategory.UNKNOWN, ErrorSeverity.FATAL, False, False, True),
    ErrorCode.INVALID_INPUT: _entry(ErrorCategory.INPUT, ErrorSeverity.FATAL, False, True, False),
    ErrorCode.SCHEMA_VALIDATION_FAILED: _entry(ErrorCategory.INPUT, ErrorSeverity.FATAL, False, True, False),
    ErrorCode.CAPABILITY_NOT_FOUND: _entry(ErrorCategory.INPUT, ErrorSeverity.FATAL, False, True, False),
    ErrorCode.CAPABILITY_UNSUPPORTED: _entry(ErrorCategory.INPUT, ErrorSeverity.FATAL, False, True, False),
    ErrorCode.CAPABILITY_EXECUTION_FAILED: _entry(ErrorCategory.INTERNAL, ErrorSeverity.RECOVERABLE, True, True, False),
    ErrorCode.AUTHORIZATION_DENIED: _entry(ErrorCategory.AUTHORIZATION, ErrorSeverity.UNSAFE, False, False, True),
    ErrorCode.PERCEPTION_NOT_FOUND: _entry(ErrorCategory.PERCEPTION, ErrorSeverity.RECOVERABLE, True, True, False),
    ErrorCode.PERCEPTION_LOW_CONFIDENCE: _entry(ErrorCategory.PERCEPTION, ErrorSeverity.RECOVERABLE, True, True, False),
    ErrorCode.PERCEPTION_STALE: _entry(ErrorCategory.PERCEPTION, ErrorSeverity.RECOVERABLE, True, True, False),
    ErrorCode.FRAME_OR_CALIBRATION_INVALID: _entry(ErrorCategory.FRAME, ErrorSeverity.FATAL, False, True, True),
    ErrorCode.PRECONDITION_FALSE: _entry(ErrorCategory.PRECONDITION, ErrorSeverity.RECOVERABLE, False, True, False),
    ErrorCode.MOTION_IK_INFEASIBLE: _entry(ErrorCategory.MOTION, ErrorSeverity.RECOVERABLE, False, True, False),
    ErrorCode.MOTION_COLLISION_RISK: _entry(ErrorCategory.MOTION, ErrorSeverity.UNSAFE, False, False, True),
    ErrorCode.MOTION_TARGET_NOT_REACHED: _entry(ErrorCategory.MOTION, ErrorSeverity.RECOVERABLE, True, True, False),
    ErrorCode.MOTION_TIMEOUT: _entry(ErrorCategory.TIMEOUT, ErrorSeverity.RECOVERABLE, True, True, False),
    ErrorCode.CONTACT_FORCE_EXCEEDED: _entry(ErrorCategory.MOTION, ErrorSeverity.UNSAFE, False, False, True),
    ErrorCode.GRASP_NOT_CONFIRMED: _entry(ErrorCategory.GRIPPER, ErrorSeverity.RECOVERABLE, True, True, False),
    ErrorCode.OBJECT_DROPPED: _entry(ErrorCategory.GRIPPER, ErrorSeverity.UNSAFE, False, False, True),
    ErrorCode.RELEASE_NOT_CONFIRMED: _entry(ErrorCategory.GRIPPER, ErrorSeverity.RECOVERABLE, True, True, False),
    ErrorCode.ROBOT_WARNING: _entry(ErrorCategory.ROBOT_STATE, ErrorSeverity.WARNING, True, True, False),
    ErrorCode.ROBOT_FAULT: _entry(ErrorCategory.ROBOT_STATE, ErrorSeverity.FATAL, False, False, True),
    ErrorCode.ROBOT_UNREACHABLE: _entry(ErrorCategory.ROBOT_STATE, ErrorSeverity.RECOVERABLE, True, False, True),
    ErrorCode.ROBOT_ESTOP_OR_UNSAFE: _entry(ErrorCategory.ROBOT_STATE, ErrorSeverity.UNSAFE, False, False, True),
    ErrorCode.VERIFICATION_FAILED: _entry(ErrorCategory.VERIFICATION, ErrorSeverity.RECOVERABLE, True, True, False),
    ErrorCode.RECOVERY_FAILED: _entry(ErrorCategory.RECOVERY, ErrorSeverity.UNSAFE, False, False, True),
    ErrorCode.RESOURCE_CONFLICT: _entry(ErrorCategory.RESOURCE, ErrorSeverity.RECOVERABLE, True, True, False),
    ErrorCode.BUDGET_EXCEEDED: _entry(ErrorCategory.TIMEOUT, ErrorSeverity.FATAL, False, False, True),
    ErrorCode.NO_PROGRESS: _entry(ErrorCategory.TIMEOUT, ErrorSeverity.RECOVERABLE, False, True, False),
    ErrorCode.INVALID_STATE_TRANSITION: _entry(ErrorCategory.INTERNAL, ErrorSeverity.FATAL, False, False, True),
    ErrorCode.EXECUTION_PRECONDITION_FAILED: _entry(ErrorCategory.PRECONDITION, ErrorSeverity.RECOVERABLE, False, True, False),
    ErrorCode.INPUT_BINDING_FAILED: _entry(ErrorCategory.INPUT, ErrorSeverity.FATAL, False, True, False),
    ErrorCode.NODE_TIMEOUT: _entry(ErrorCategory.TIMEOUT, ErrorSeverity.RECOVERABLE, True, True, False),
    ErrorCode.TASK_TIMEOUT: _entry(ErrorCategory.TIMEOUT, ErrorSeverity.FATAL, False, False, True),
    ErrorCode.EXECUTION_INTERRUPTED: _entry(ErrorCategory.INTERNAL, ErrorSeverity.FATAL, False, False, True),
    ErrorCode.CHECKPOINT_CORRUPT: _entry(ErrorCategory.INTERNAL, ErrorSeverity.FATAL, False, False, True),
    ErrorCode.CHECKPOINT_MISMATCH: _entry(ErrorCategory.INPUT, ErrorSeverity.FATAL, False, False, True),
    ErrorCode.EVENT_LOG_CORRUPT: _entry(ErrorCategory.INTERNAL, ErrorSeverity.FATAL, False, False, True),
    ErrorCode.RESOURCE_ACQUISITION_FAILED: _entry(ErrorCategory.RESOURCE, ErrorSeverity.RECOVERABLE, True, True, False),
    ErrorCode.REPLAN_REQUIRED: _entry(ErrorCategory.INPUT, ErrorSeverity.RECOVERABLE, False, True, False),
    ErrorCode.HUMAN_ACTION_REQUIRED: _entry(ErrorCategory.AUTHORIZATION, ErrorSeverity.RECOVERABLE, False, False, True),
    ErrorCode.NO_TERMINAL_PATH: _entry(ErrorCategory.INPUT, ErrorSeverity.FATAL, False, True, False),
    ErrorCode.NON_DETERMINISTIC_GRAPH: _entry(ErrorCategory.INPUT, ErrorSeverity.FATAL, False, True, False),
    ErrorCode.PARALLEL_EXECUTION_NOT_SUPPORTED_IN_S7: _entry(ErrorCategory.INPUT, ErrorSeverity.FATAL, False, True, False),
}

ERROR_DIRECTORY = ERROR_CATALOG


def error_catalog() -> dict[str, dict[str, Any]]:
    return {code.value: to_plain(value) for code, value in ERROR_CATALOG.items()}


def standardize_error_code(code: str | ErrorCode, details: dict[str, Any] | None = None) -> tuple[ErrorCode, dict[str, Any]]:
    if isinstance(code, ErrorCode):
        return code, dict(details or {})
    if not isinstance(code, str):
        raise ContractValidationError("error code must be a string")
    try:
        return ErrorCode(code), dict(details or {})
    except ValueError:
        normalized = dict(details or {})
        normalized.setdefault("original_code", code)
        return ErrorCode.UNKNOWN_ERROR, normalized


@dataclass(frozen=True)
class ErrorInfo:
    code: ErrorCode | str = ErrorCode.UNKNOWN_ERROR
    category: ErrorCategory = ErrorCategory.UNKNOWN
    severity: ErrorSeverity = ErrorSeverity.FATAL
    message: str = ""
    retryable: bool = False
    safe_to_replan: bool = False
    requires_human: bool = True
    state_invalidated: bool = False
    details: dict[str, Any] = field(default_factory=dict)
    source: str = "unknown"

    def __post_init__(self) -> None:
        code, details = standardize_error_code(self.code, self.details)
        catalog = ERROR_CATALOG[code]
        if code == ErrorCode.UNKNOWN_ERROR:
            object.__setattr__(self, "category", catalog["category"])
            object.__setattr__(self, "severity", catalog["severity"])
            object.__setattr__(self, "retryable", False)
            object.__setattr__(self, "safe_to_replan", False)
            object.__setattr__(self, "requires_human", True)
        else:
            object.__setattr__(self, "category", require_enum(self.category, ErrorCategory, "category"))
            object.__setattr__(self, "severity", require_enum(self.severity, ErrorSeverity, "severity"))
            object.__setattr__(self, "retryable", require_bool(self.retryable, "retryable"))
            object.__setattr__(self, "safe_to_replan", require_bool(self.safe_to_replan, "safe_to_replan"))
            object.__setattr__(self, "requires_human", require_bool(self.requires_human, "requires_human"))
        object.__setattr__(self, "code", code)
        object.__setattr__(self, "message", require_string(self.message, "message", non_empty=True))
        object.__setattr__(self, "state_invalidated", require_bool(self.state_invalidated, "state_invalidated"))
        object.__setattr__(self, "details", ensure_jsonable(require_object(details, "details"), "details"))
        object.__setattr__(self, "source", require_string(self.source, "source", non_empty=True))

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ErrorInfo":
        fields = (
            "code", "category", "severity", "message", "retryable", "safe_to_replan",
            "requires_human", "state_invalidated", "details", "source",
        )
        reject_unknown(data, fields, fields)
        return cls(
            code=data["code"], category=data["category"], severity=data["severity"],
            message=data["message"], retryable=data["retryable"], safe_to_replan=data["safe_to_replan"],
            requires_human=data["requires_human"], state_invalidated=data["state_invalidated"],
            details=data["details"], source=data["source"],
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "code": self.code.value,
            "category": self.category.value,
            "severity": self.severity.value,
            "message": self.message,
            "retryable": self.retryable,
            "safe_to_replan": self.safe_to_replan,
            "requires_human": self.requires_human,
            "state_invalidated": self.state_invalidated,
            "details": ensure_jsonable(self.details, "details"),
            "source": self.source,
        }

    def to_json(self) -> str:
        return stable_dumps(self.to_dict())
