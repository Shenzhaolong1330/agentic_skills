from __future__ import annotations

from pathlib import Path
import json

import pytest

from agentic_skills_harness.contracts import (
    ActionResult,
    ActionStatus,
    CapabilityKind,
    ErrorCode,
    ErrorInfo,
    ErrorSeverity,
    ExecutionBudget,
    NodeStatus,
    ObservationResult,
    ResourceMode,
    ResourceRequirement,
    RiskClass,
    StateInvalidation,
    TaskTerminalState,
    VerificationResult,
)
from agentic_skills_harness.contracts.serialization import ContractValidationError, utc_now_iso


def error() -> ErrorInfo:
    return ErrorInfo(code=ErrorCode.INVALID_INPUT, category="INPUT", severity="FATAL", message="bad input", retryable=False, safe_to_replan=True, requires_human=False, state_invalidated=False, details={}, source="test")


def action(**changes) -> ActionResult:
    values = dict(capability_id="motion.move_to_pose", status=ActionStatus.SUCCEEDED, command_accepted=True, command_executed=True, planned_only=False, controller_target_reached=None, effect_observed=None, goal_verified=False, verification=None, outputs={}, errors=(), warnings=(), metrics={}, artifacts={}, started_at=utc_now_iso(), ended_at=None)
    values.update(changes)
    return ActionResult(**values)


def test_enum_values_are_stable_strings():
    assert NodeStatus.RUNNING.value == "RUNNING"
    assert TaskTerminalState.NEEDS_HUMAN.value == "NEEDS_HUMAN"
    assert CapabilityKind.OBSERVATION.value == "observation"
    assert RiskClass.READ_ONLY_HARDWARE.value == "READ_ONLY_HARDWARE"
    assert ResourceMode.EXCLUSIVE.value == "exclusive"


def test_error_round_trip_and_unknown_code_is_conservative():
    unknown = ErrorInfo(code="vendor_fault_99", message="external", details={}, source="vendor")
    assert unknown.code is ErrorCode.UNKNOWN_ERROR
    assert unknown.details["original_code"] == "vendor_fault_99"
    decoded = ErrorInfo.from_dict(unknown.to_dict())
    assert decoded.to_dict() == unknown.to_dict()
    json.dumps(decoded.to_dict(), allow_nan=False)


def test_result_round_trips():
    verification = VerificationResult(True, "goal.reached", "observation", ({"value": True},), 1.0, utc_now_iso())
    result = action(goal_verified=True, verification=verification)
    assert ActionResult.from_dict(result.to_dict()).to_dict() == result.to_dict()
    observation = ObservationResult("robot.observe_health", True, {"state": "ready"}, utc_now_iso(), 3.0, "test", (), (), {})
    assert ObservationResult.from_dict(observation.to_dict()).to_dict() == observation.to_dict()
    assert VerificationResult.from_dict(verification.to_dict()).to_dict() == verification.to_dict()


def test_action_success_does_not_infer_physical_success_from_returncode():
    result = ActionResult.from_command_result("motion.move_to_pose", 0)
    assert result.status is ActionStatus.SUCCEEDED
    assert result.controller_target_reached is None
    assert result.effect_observed is None
    assert result.goal_verified is False


@pytest.mark.parametrize("changes", [
    {"planned_only": True, "command_executed": True},
    {"status": ActionStatus.PLANNED_ONLY, "planned_only": False},
    {"status": ActionStatus.DENIED, "command_executed": True},
    {"status": ActionStatus.TIMEOUT, "effect_observed": True},
    {"goal_verified": True, "verification": None},
    {"status": ActionStatus.SUCCEEDED, "errors": (ErrorInfo(code=ErrorCode.ROBOT_FAULT, category="ROBOT_STATE", severity="FATAL", message="fault", retryable=False, safe_to_replan=False, requires_human=True, state_invalidated=True, details={}, source="test"),)},
])
def test_action_invariants_reject_unsafe_combinations(changes):
    with pytest.raises((ContractValidationError, ValueError)):
        action(**changes)


def test_resource_and_invalidation_round_trip():
    resource = ResourceRequirement("robot.dual_franka.state", ResourceMode.SHARED, "state")
    invalidation = StateInvalidation(("robot.health",), "reset", "robot", True)
    assert ResourceRequirement.from_dict(resource.to_dict()).to_dict() == resource.to_dict()
    assert StateInvalidation.from_dict(invalidation.to_dict()).to_dict() == invalidation.to_dict()
