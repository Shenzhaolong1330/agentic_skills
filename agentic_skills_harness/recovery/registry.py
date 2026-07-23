from __future__ import annotations

"""First-party, deterministic recovery policy registry."""

from dataclasses import replace
from typing import Any, Iterable, Mapping

from ..contracts.enums import ErrorCode, ErrorCategory, ErrorSeverity, RiskClass
from ..contracts.serialization import ContractValidationError
from .models import (
    HeldObjectEvidence,
    RecoveryCandidate,
    RecoveryContext,
    RecoveryDisposition,
    RecoveryStrategy,
    RecoveryStrategyId,
)


_RISK_ORDER = {
    RiskClass.NONE: 0,
    RiskClass.READ_ONLY_HARDWARE: 1,
    RiskClass.MOTION: 2,
    RiskClass.GRIPPER: 3,
    RiskClass.CONTACT: 4,
    RiskClass.RELEASE: 5,
    RiskClass.RECOVERY: 6,
    RiskClass.HIGH_RISK: 7,
}


def _value(value: Any) -> str:
    return value.value if hasattr(value, "value") else str(value)


def _error_code(context: RecoveryContext) -> str:
    return context.error_info.code.value


def _capability(registry: Any, capability_id: str) -> Any | None:
    if registry is None:
        return None
    getter = getattr(registry, "get", None)
    return getter(capability_id) if callable(getter) else None


class RecoveryPolicyRegistry:
    """Closed registry for recovery strategies.

    Production registries accept only first-party registration.  Tests may use
    ``production=False`` to construct an isolated registry, which still runs
    all structural validation.
    """

    def __init__(self, capability_registry: Any | None = None, *, production: bool = True) -> None:
        self.capability_registry = capability_registry
        self.production = production
        self._strategies: dict[str, RecoveryStrategy] = {}

    def register(self, strategy: RecoveryStrategy | Mapping[str, Any], *, first_party: bool = False) -> RecoveryStrategy:
        if self.production and not first_party:
            raise PermissionError("production recovery policies must be first-party registered")
        item = strategy if isinstance(strategy, RecoveryStrategy) else RecoveryStrategy.from_dict(strategy)
        key = item.strategy_id.value
        if key in self._strategies:
            raise ValueError(f"duplicate recovery strategy ID: {key}")
        self._strategies[key] = item
        errors = self.validate()
        if errors:
            self._strategies.pop(key, None)
            raise ContractValidationError("invalid recovery strategy: " + "; ".join(errors))
        return item

    def get(self, strategy_id: RecoveryStrategyId | str) -> RecoveryStrategy | None:
        return self._strategies.get(_value(strategy_id))

    def list(self) -> tuple[RecoveryStrategy, ...]:
        return tuple(sorted(self._strategies.values(), key=lambda item: (-item.priority, item.strategy_id.value)))

    def validate(self) -> list[str]:
        errors: list[str] = []
        for strategy in self._strategies.values():
            for capability_id in (*strategy.allowed_capability_ids, *strategy.required_capability_ids):
                if self.capability_registry is not None and _capability(self.capability_registry, capability_id) is None:
                    errors.append(f"{strategy.strategy_id.value}: capability does not exist: {capability_id}")
            if strategy.strategy_id == RecoveryStrategyId.RESET_HOME:
                if not strategy.required_capability_ids:
                    errors.append("RESET_HOME: must reference an allowed_as_recovery capability")
                for capability_id in strategy.required_capability_ids:
                    capability = _capability(self.capability_registry, capability_id)
                    if capability is not None and not bool(getattr(capability, "allowed_as_recovery", False)):
                        errors.append(f"RESET_HOME: capability is not allowed_as_recovery: {capability_id}")
            if strategy.strategy_id == RecoveryStrategyId.CONTROLLER_RECOVERY:
                for capability_id in strategy.required_capability_ids:
                    capability = _capability(self.capability_registry, capability_id)
                    if capability is not None and not bool(getattr(capability, "allowed_as_recovery", False)):
                        errors.append(f"CONTROLLER_RECOVERY: capability is not allowed_as_recovery: {capability_id}")
        return sorted(set(errors))

    def find_candidates(self, context: RecoveryContext | Mapping[str, Any]) -> tuple[RecoveryStrategy, ...]:
        item = context if isinstance(context, RecoveryContext) else RecoveryContext.from_dict(context)
        code = _error_code(item)
        category = item.error_info.category.value
        result = []
        for strategy in self.list():
            if strategy.trigger_error_codes and code not in strategy.trigger_error_codes:
                continue
            if strategy.trigger_error_categories and category not in strategy.trigger_error_categories:
                continue
            if not strategy.trigger_error_codes and not strategy.trigger_error_categories:
                result.append(strategy)
                continue
            result.append(strategy)
        return tuple(result)


def _strategy(
    strategy_id: RecoveryStrategyId,
    disposition: RecoveryDisposition,
    priority: int,
    *,
    codes: Iterable[ErrorCode] = (),
    categories: Iterable[ErrorCategory] = (),
    required: Iterable[str] = (),
    reobserve: bool = False,
    retreat: bool = False,
    max_attempts: int = 1,
    risk: RiskClass = RiskClass.NONE,
    template: str | None = None,
    notes: str = "",
) -> RecoveryStrategy:
    return RecoveryStrategy(
        strategy_id=strategy_id,
        disposition=disposition,
        priority=priority,
        trigger_error_codes=tuple(code.value for code in codes),
        trigger_error_categories=tuple(category.value for category in categories),
        required_capability_ids=tuple(required),
        requires_reobserve=reobserve,
        requires_safe_retreat=retreat,
        max_attempts=max_attempts,
        risk_class=risk,
        template_id=template,
        notes=notes,
    )


def build_first_party_recovery_policy_registry(capability_registry: Any | None = None) -> RecoveryPolicyRegistry:
    """Build the platform minimum strategy set in stable priority order."""

    registry = RecoveryPolicyRegistry(capability_registry, production=True)
    all_perception = (ErrorCode.PERCEPTION_NOT_FOUND, ErrorCode.PERCEPTION_LOW_CONFIDENCE, ErrorCode.PERCEPTION_STALE)
    for strategy in (
        _strategy(RecoveryStrategyId.REOBSERVE, RecoveryDisposition.RECOVERY_SUBGRAPH, 100, codes=all_perception, reobserve=True, template="reobserve", notes="Obtain fresh perception evidence before using a target pose."),
        _strategy(RecoveryStrategyId.SAFE_RETREAT, RecoveryDisposition.RECOVERY_SUBGRAPH, 95, codes=(ErrorCode.MOTION_TARGET_NOT_REACHED, ErrorCode.MOTION_TIMEOUT, ErrorCode.GRASP_NOT_CONFIRMED, ErrorCode.VERIFICATION_FAILED, ErrorCode.RELEASE_NOT_CONFIRMED, ErrorCode.OBJECT_DROPPED, ErrorCode.CONTACT_FORCE_EXCEEDED), retreat=True, reobserve=True, template="safe-retreat-then-reobserve"),
        _strategy(RecoveryStrategyId.RETRY_SAME_PARAMETERS, RecoveryDisposition.LOCAL_RETRY, 90, codes=(ErrorCode.MOTION_TARGET_NOT_REACHED, ErrorCode.MOTION_TIMEOUT, ErrorCode.ROBOT_WARNING), max_attempts=1),
        _strategy(RecoveryStrategyId.ALTERNATE_CANDIDATE, RecoveryDisposition.REPLAN_REMAINDER, 80, codes=all_perception + (ErrorCode.MOTION_IK_INFEASIBLE,), reobserve=True, template="recompile-remainder"),
        _strategy(RecoveryStrategyId.ALTERNATE_ARM, RecoveryDisposition.REPLAN_REMAINDER, 79, codes=(ErrorCode.MOTION_IK_INFEASIBLE, ErrorCode.GRASP_NOT_CONFIRMED, ErrorCode.MOTION_TARGET_NOT_REACHED), reobserve=True, template="recompile-remainder"),
        _strategy(RecoveryStrategyId.RECOMPILE_REMAINDER, RecoveryDisposition.REPLAN_REMAINDER, 70, categories=(ErrorCategory.PERCEPTION, ErrorCategory.MOTION, ErrorCategory.VERIFICATION), reobserve=True, template="recompile-remainder"),
        _strategy(RecoveryStrategyId.CONTROLLER_RECOVERY, RecoveryDisposition.RECOVERY_SUBGRAPH, 60, codes=(ErrorCode.ROBOT_FAULT, ErrorCode.ROBOT_UNREACHABLE), required=("robot.recover_reset_home",), reobserve=True, template="controller-recovery-then-reobserve", risk=RiskClass.RECOVERY),
        _strategy(RecoveryStrategyId.RESET_HOME, RecoveryDisposition.RECOVERY_SUBGRAPH, 50, codes=(ErrorCode.ROBOT_FAULT, ErrorCode.ROBOT_UNREACHABLE), required=("robot.recover_reset_home",), reobserve=True, template="reset-home-then-reobserve", risk=RiskClass.RECOVERY),
        _strategy(RecoveryStrategyId.REQUEST_HUMAN, RecoveryDisposition.NEEDS_HUMAN, 10, categories=tuple(ErrorCategory), template="request-human", notes="Stop and create a human action request."),
        _strategy(RecoveryStrategyId.ABORT, RecoveryDisposition.TERMINATE, 0, categories=tuple(ErrorCategory), template="abort"),
    ):
        # The manifest capability may be absent in an isolated unit test.  The
        # registry remains useful there; validation is strict when a registry
        # was supplied.
        try:
            registry.register(strategy, first_party=True)
        except ContractValidationError:
            if capability_registry is not None:
                raise
            registry._strategies[strategy.strategy_id.value] = strategy
    return registry


def platform_strategy_allowed(context: RecoveryContext, strategy: RecoveryStrategy) -> tuple[bool, str | None]:
    """Apply platform guards that task policy is never allowed to weaken."""

    code = context.error_info.code
    held = context.held_object_evidence
    sid = strategy.strategy_id
    if context.mode == "live":
        return False, "live recovery is disabled in S8/S9"
    if code == ErrorCode.ROBOT_ESTOP_OR_UNSAFE or context.error_info.severity == ErrorSeverity.UNSAFE and code in {ErrorCode.ROBOT_ESTOP_OR_UNSAFE, ErrorCode.MOTION_COLLISION_RISK, ErrorCode.CONTACT_FORCE_EXCEEDED}:
        if sid not in {RecoveryStrategyId.REQUEST_HUMAN, RecoveryStrategyId.ABORT}:
            return False, "unsafe or E-stop errors allow only human request or abort"
    if code in {ErrorCode.PERCEPTION_NOT_FOUND, ErrorCode.PERCEPTION_LOW_CONFIDENCE, ErrorCode.PERCEPTION_STALE} and sid in {RecoveryStrategyId.CONTROLLER_RECOVERY, RecoveryStrategyId.RESET_HOME}:
        return False, "perception failures must not trigger reset or controller recovery"
    if code == ErrorCode.FRAME_OR_CALIBRATION_INVALID and sid in {RecoveryStrategyId.RETRY_SAME_PARAMETERS, RecoveryStrategyId.CONTROLLER_RECOVERY, RecoveryStrategyId.RESET_HOME}:
        return False, "invalid frame/calibration cannot be fixed by repeating motion or reset"
    if code in {ErrorCode.BUDGET_EXCEEDED, ErrorCode.NO_PROGRESS} and sid not in {RecoveryStrategyId.REQUEST_HUMAN, RecoveryStrategyId.ABORT}:
        return False, "budget exhaustion and no-progress are terminal for automatic recovery"
    if code == ErrorCode.OBJECT_DROPPED and sid in {RecoveryStrategyId.RETRY_SAME_PARAMETERS, RecoveryStrategyId.CONTROLLER_RECOVERY, RecoveryStrategyId.RESET_HOME}:
        return False, "dropped-object state cannot reuse the old pose"
    if sid in {RecoveryStrategyId.CONTROLLER_RECOVERY, RecoveryStrategyId.RESET_HOME} and held != HeldObjectEvidence.NONE_CONFIRMED:
        return False, "reset/controller recovery is not automatic with confirmed, tentative, or unknown holding"
    if sid == RecoveryStrategyId.RETRY_SAME_PARAMETERS:
        if code in {ErrorCode.PERCEPTION_STALE, ErrorCode.FRAME_OR_CALIBRATION_INVALID, ErrorCode.MOTION_COLLISION_RISK, ErrorCode.OBJECT_DROPPED, ErrorCode.ROBOT_ESTOP_OR_UNSAFE, ErrorCode.BUDGET_EXCEEDED, ErrorCode.NO_PROGRESS, ErrorCode.VERIFICATION_FAILED, ErrorCode.RELEASE_NOT_CONFIRMED}:
            return False, "same-parameter retry is forbidden for this error"
        if context.error_info.requires_human or not context.error_info.retryable:
            return False, "error is not retryable without human review"
    if strategy.requires_reobserve and context.error_info.code == ErrorCode.PERCEPTION_STALE:
        # The selector must choose the reobserve subgraph, not pretend stale
        # evidence is current.  This guard is informational for REOBSERVE and
        # keeps any non-observing strategy out.
        if sid not in {RecoveryStrategyId.REOBSERVE, RecoveryStrategyId.SAFE_RETREAT, RecoveryStrategyId.RECOMPILE_REMAINDER, RecoveryStrategyId.ALTERNATE_CANDIDATE, RecoveryStrategyId.ALTERNATE_ARM}:
            return False, "stale perception requires a fresh observation"
    return True, None
