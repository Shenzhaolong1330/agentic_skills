from __future__ import annotations

from dataclasses import replace
from typing import Any, Callable, Mapping

from ..capability import CapabilityContract
from ..contracts.enums import ActionStatus, ErrorCode, ErrorSeverity
from ..contracts.errors import ErrorInfo
from ..contracts.results import ActionResult, ObservationResult, VerificationResult
from ..contracts.serialization import utc_now_iso
from ..dispatch.error_mapping import make_error
from ..dispatch.models import DispatchContext, DispatchRequest
from ..world.models import FactStatus
from ..world.predicates import PredicateEngine, PredicateResult, PredicateSpec
from ..world.store import WorldStateStore


TaskVerifier = Callable[[ActionResult | ObservationResult, WorldStateStore, tuple[Any, ...]], bool | PredicateResult | VerificationResult]


class VerifierEngine:
    def __init__(self, capability_registry: Any, *, dispatcher: Any | None = None, predicate_engine: PredicateEngine | None = None, max_verifier_depth: int = 1) -> None:
        self.registry = capability_registry
        self.dispatcher = dispatcher
        self.predicates = predicate_engine or PredicateEngine()
        self.max_verifier_depth = max_verifier_depth
        self._task_handlers: dict[str, TaskVerifier] = {}

    def register_task_handler(self, capability_id: str, handler: TaskVerifier) -> None:
        self.registry.require(capability_id)
        if not callable(handler):
            raise TypeError("task verifier handler must be callable")
        self._task_handlers[capability_id] = handler

    def validate_contracts(self) -> list[str]:
        errors: list[str] = []
        for capability in self.registry.list(include_internal=True, include_legacy=True):
            verifier = capability.verifier
            if verifier.type == "capability":
                target = self.registry.get(verifier.capability_id or "")
                if target is None:
                    errors.append(f"{capability.capability_id}: verifier capability missing")
                elif target.kind.value not in {"observation", "compute"}:
                    errors.append(f"{capability.capability_id}: verifier target is not observation/check")
                elif target.capability_id == capability.capability_id:
                    errors.append(f"{capability.capability_id}: verifier cycle")
        return errors

    @staticmethod
    def _failed(predicate_id: str, method: str, reason: str, *, code: ErrorCode = ErrorCode.VERIFICATION_FAILED, evidence: tuple[Any, ...] = ()) -> VerificationResult:
        return VerificationResult(False, predicate_id, method, evidence, None, utc_now_iso(), (make_error(code, reason),), ())

    def verify_effect(self, capability: CapabilityContract, result: ActionResult | ObservationResult, world: WorldStateStore, *, evidence: tuple[Any, ...] = (), context: DispatchContext | None = None, _depth: int = 0) -> VerificationResult:
        if _depth > self.max_verifier_depth:
            return self._failed(capability.capability_id, "capability", "verifier depth exceeded")
        if isinstance(result, ActionResult) and any(error.severity in (ErrorSeverity.FATAL, ErrorSeverity.UNSAFE) for error in result.errors):
            return self._failed(capability.capability_id, "error_guard", "fatal or unsafe action error blocks verification", code=ErrorCode.VERIFICATION_FAILED)
        verifier = capability.verifier
        if verifier.type == "none":
            return self._failed(capability.capability_id, "none", "physical capability has no verifier") if capability.physical_side_effects or capability.moves_robot or capability.controls_gripper else VerificationResult(True, capability.capability_id, "none", ("no_physical_effect",), 1.0, utc_now_iso(), (), ())
        if verifier.type == "output_schema":
            if capability.physical_side_effects or capability.moves_robot or capability.controls_gripper:
                return self._failed(capability.capability_id, "output_schema", "output schema proves structure only; physical effect remains unverified")
            return VerificationResult(isinstance(result, ObservationResult) and result.ok, capability.capability_id, "output_schema", evidence or (result.to_dict(),), 1.0 if isinstance(result, ObservationResult) and result.ok else None, utc_now_iso(), (), ())
        if verifier.type == "task_specific":
            handler = self._task_handlers.get(capability.capability_id)
            if handler is None:
                return self._failed(capability.capability_id, "task_specific", "task_specific_verifier_not_registered")
            try:
                value = handler(result, world, tuple(evidence))
            except Exception as exc:
                return self._failed(capability.capability_id, "task_specific", f"task-specific verifier failed: {exc}")
            if isinstance(value, VerificationResult):
                return value
            if isinstance(value, PredicateResult):
                return VerificationResult(value.satisfied, value.predicate_id, "task_specific", tuple(value.evidence_fact_ids), None, value.evaluated_at, () if value.satisfied else (make_error(ErrorCode.VERIFICATION_FAILED, value.reason or "predicate not satisfied"),), ())
            return VerificationResult(bool(value), capability.capability_id, "task_specific", tuple(evidence) or ("task_handler",) if value else (), 1.0 if value else None, utc_now_iso(), (), ()) if value else self._failed(capability.capability_id, "task_specific", "task-specific verifier returned false")
        if verifier.type == "capability":
            if self.dispatcher is None:
                return self._failed(capability.capability_id, "capability", "verifier dispatcher is not configured")
            target = self.registry.get(verifier.capability_id or "")
            if target is None or target.kind.value not in {"observation", "compute"} or target.capability_id == capability.capability_id:
                return self._failed(capability.capability_id, "capability", "invalid or cyclic verifier capability")
            try:
                observed = self.dispatcher.dispatch(DispatchRequest(target.capability_id, {}), context or DispatchContext())
            except Exception as exc:
                return self._failed(capability.capability_id, "capability", f"verifier dispatch failed: {exc}")
            if isinstance(observed, ObservationResult) and observed.ok:
                return VerificationResult(True, capability.capability_id, f"capability:{target.capability_id}", tuple(evidence) + (observed.to_dict(),), 1.0, utc_now_iso(), (), ())
            return self._failed(capability.capability_id, "capability", "independent verifier observation failed")
        return self._failed(capability.capability_id, "unknown", "unsupported verifier type")

    def verify_goal(self, predicate: PredicateSpec | Mapping[str, Any], world: WorldStateStore, *, action_result: ActionResult | None = None, capability: CapabilityContract | None = None, now: str | None = None) -> VerificationResult:
        if action_result is not None and any(error.severity in (ErrorSeverity.FATAL, ErrorSeverity.UNSAFE) for error in action_result.errors):
            return self._failed(getattr(predicate, "predicate_id", "goal"), "goal_predicate", "fatal or unsafe error blocks goal verification")
        spec = predicate if isinstance(predicate, PredicateSpec) else PredicateSpec.from_dict(predicate)
        evaluated = self.predicates.evaluate(spec, world, now=now)
        if not evaluated.satisfied:
            return VerificationResult(False, evaluated.predicate_id, "predicate_engine", tuple(evaluated.evidence_fact_ids), None, evaluated.evaluated_at, (make_error(ErrorCode.VERIFICATION_FAILED, evaluated.reason or "goal predicate not satisfied", details={"errors": list(evaluated.errors)}),), ())
        facts = [world.get_fact(fact_id) for fact_id in evaluated.evidence_fact_ids]
        facts = [fact for fact in facts if fact is not None]
        if not facts or any(fact.status in {FactStatus.INVALIDATED, FactStatus.TENTATIVE} or not fact.is_fresh(evaluated.evaluated_at) for fact in facts):
            return self._failed(spec.predicate_id, "predicate_engine", "goal evidence is stale, invalidated, or tentative")
        confidence_values = [fact.confidence for fact in facts if fact.confidence is not None]
        confidence = min(confidence_values) if confidence_values else 1.0
        return VerificationResult(True, evaluated.predicate_id, "predicate_engine", tuple(evaluated.evidence_fact_ids), confidence, evaluated.evaluated_at, (), ())


def apply_effect_verification(result: ActionResult, verification: VerificationResult) -> ActionResult:
    if not isinstance(result, ActionResult) or not isinstance(verification, VerificationResult):
        raise TypeError("apply_effect_verification expects ActionResult and VerificationResult")
    return replace(result, effect_observed=verification.verified, verification=verification)


def apply_goal_verification(result: ActionResult, verification: VerificationResult) -> ActionResult:
    if not isinstance(result, ActionResult) or not isinstance(verification, VerificationResult):
        raise TypeError("apply_goal_verification expects ActionResult and VerificationResult")
    if not verification.verified or any(error.severity in (ErrorSeverity.FATAL, ErrorSeverity.UNSAFE) for error in result.errors):
        return replace(result, goal_verified=False, verification=verification)
    return replace(result, goal_verified=True, verification=verification)
