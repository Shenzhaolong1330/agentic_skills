"""Generic, task-independent execution contracts."""

from .budgets import ExecutionBudget
from .enums import (
    ActionStatus,
    CapabilityKind,
    ErrorCategory,
    ErrorCode,
    ErrorSeverity,
    NodeStatus,
    ResourceMode,
    RiskClass,
    TaskTerminalState,
)
from .errors import ErrorInfo, error_catalog
from .resources import ResourceRequirement, StateInvalidation
from .results import ActionResult, ObservationResult, VerificationResult

__all__ = [
    "ActionResult",
    "ActionStatus",
    "CapabilityKind",
    "ErrorCategory",
    "ErrorCode",
    "ErrorInfo",
    "ErrorSeverity",
    "ExecutionBudget",
    "NodeStatus",
    "ObservationResult",
    "ResourceMode",
    "ResourceRequirement",
    "RiskClass",
    "StateInvalidation",
    "TaskTerminalState",
    "VerificationResult",
    "error_catalog",
]
