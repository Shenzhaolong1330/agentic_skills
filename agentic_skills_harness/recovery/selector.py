from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from ..contracts.enums import ErrorCode, RiskClass
from ..planning.canonical import digest
from .models import RecoveryCandidate, RecoveryContext, RecoveryDecision, RecoveryDisposition, RecoveryStrategy
from .registry import RecoveryPolicyRegistry, _RISK_ORDER, platform_strategy_allowed


def _envelope(context: RecoveryContext) -> Mapping[str, Any]:
    return context.original_envelope


def _remaining(context: RecoveryContext, key: str, default: int = 0) -> int:
    value = context.remaining_budget.get(key, default)
    try:
        return max(0, int(value))
    except (TypeError, ValueError):
        return 0


def _history_count(context: RecoveryContext, *, strategy_id: str | None = None, error_code: str | None = None) -> int:
    count = 0
    for item in context.recovery_history:
        if strategy_id is not None and str(item.get("strategy_id")) != strategy_id:
            continue
        if error_code is not None and str(item.get("error_code")) != error_code:
            continue
        count += 1
    return count


@dataclass(frozen=True)
class RecoverySelector:
    registry: RecoveryPolicyRegistry

    def _reject(self, strategy: RecoveryStrategy, reasons: list[str]) -> RecoveryCandidate:
        return RecoveryCandidate(strategy, False, tuple(sorted(set(reasons))))

    def _evaluate(self, context: RecoveryContext, strategy: RecoveryStrategy) -> RecoveryCandidate:
        reasons: list[str] = []
        envelope = _envelope(context)
        allowed = set(envelope.get("allowed_capabilities", ()))
        forbidden = set(envelope.get("forbidden_capabilities", ()))
        if strategy.allowed_node_kinds and context.failed_node_kind not in strategy.allowed_node_kinds:
            reasons.append("failed node kind is outside strategy allowlist")
        if strategy.allowed_capability_ids and context.failed_capability_id not in strategy.allowed_capability_ids:
            reasons.append("failed capability is outside strategy allowlist")
        if any(item in forbidden for item in strategy.required_capability_ids):
            reasons.append("required recovery capability is forbidden by original envelope")
        if allowed and any(item not in allowed for item in strategy.required_capability_ids):
            reasons.append("recovery capability would expand the original allowlist")
        for capability_id in strategy.required_capability_ids:
            if self.registry.capability_registry is not None and self.registry.capability_registry.get(capability_id) is None:
                reasons.append(f"required capability is unavailable: {capability_id}")
        risk_ceiling = envelope.get("risk_ceiling", RiskClass.NONE.value)
        try:
            ceiling = RiskClass(str(risk_ceiling))
        except ValueError:
            reasons.append("original envelope has an invalid risk ceiling")
        else:
            if _RISK_ORDER[strategy.risk_class if isinstance(strategy.risk_class, RiskClass) else RiskClass(str(strategy.risk_class))] > _RISK_ORDER[ceiling]:
                reasons.append("recovery strategy exceeds original risk ceiling")
        if context.mode not in {"mock", "dry_run", "from_artifacts"}:
            reasons.append("only offline modes are supported")
        allowed_by_platform, platform_reason = platform_strategy_allowed(context, strategy)
        if not allowed_by_platform and platform_reason:
            reasons.append(platform_reason)
        if strategy.disposition == RecoveryDisposition.LOCAL_RETRY:
            if _remaining(context, "same_error_retries", int(envelope.get("max_same_error_retries", 0))) <= 0:
                reasons.append("same-error retry budget is exhausted")
        elif strategy.disposition == RecoveryDisposition.RECOVERY_SUBGRAPH:
            if _remaining(context, "recovery_actions", int(envelope.get("max_recovery_actions", 0))) <= 0:
                reasons.append("recovery budget is exhausted")
        elif strategy.disposition == RecoveryDisposition.REPLAN_REMAINDER:
            if _remaining(context, "replans", int(envelope.get("max_replans", 0))) <= 0:
                reasons.append("replan budget is exhausted")
        sid = strategy.strategy_id.value
        if _history_count(context, strategy_id=sid) >= strategy.max_attempts:
            reasons.append("strategy attempt budget is exhausted")
        if context.latest_progress_fingerprint and context.recovery_history:
            latest = context.recovery_history[-1]
            if latest.get("progress_fingerprint") == context.latest_progress_fingerprint and str(latest.get("strategy_id")) == sid:
                reasons.append("same recovery decision and world state made no progress")
        forbidden_when = strategy.forbidden_when
        held_forbidden = forbidden_when.get("held_object_evidence")
        if held_forbidden and context.held_object_evidence.value in {str(item) for item in (held_forbidden if isinstance(held_forbidden, list) else [held_forbidden])}:
            reasons.append("held-object evidence matches strategy forbidden_when guard")
        if not reasons:
            return RecoveryCandidate(strategy, True, ())
        return self._reject(strategy, reasons)

    def select(self, context: RecoveryContext | Mapping[str, Any]) -> RecoveryDecision:
        item = context if isinstance(context, RecoveryContext) else RecoveryContext.from_dict(context)
        candidates = tuple(self._evaluate(item, strategy) for strategy in self.registry.find_candidates(item))
        eligible = tuple(candidate for candidate in candidates if candidate.eligible)
        rejected = tuple(candidate for candidate in candidates if not candidate.eligible)
        ordered = tuple(sorted(eligible, key=lambda candidate: (-candidate.strategy.priority, candidate.strategy.strategy_id.value)))
        selected: RecoveryStrategy | None = None
        reason = "no eligible recovery strategy"
        if ordered:
            top_priority = ordered[0].strategy.priority
            tied = tuple(candidate for candidate in ordered if candidate.strategy.priority == top_priority)
            if len(tied) == 1:
                selected = tied[0].strategy
                reason = "selected highest-priority eligible strategy"
            else:
                reason = "same-priority recovery strategies are ambiguous; fail closed"
        requires_human = selected is None or selected.disposition == RecoveryDisposition.NEEDS_HUMAN
        if selected is not None and selected.strategy_id.value == "ABORT":
            requires_human = False
        decision_id = "decision_" + digest({
            "run_id": item.run_id,
            "plan_hash": item.plan_hash,
            "failed_node_id": item.failed_node_id,
            "error_code": item.error_info.code.value,
            "world_snapshot": item.world_snapshot,
            "held_object_evidence": item.held_object_evidence.value,
            "eligible": [candidate.strategy.strategy_id.value for candidate in ordered],
        })[:24]
        return RecoveryDecision(
            decision_id=decision_id,
            selected_strategy=selected,
            eligible_candidates=ordered,
            rejected_candidates=rejected,
            reason=reason,
            remaining_budget=item.remaining_budget,
            requires_reobserve=bool(selected and selected.requires_reobserve),
            requires_human=requires_human,
            terminal_if_failed=True,
        )
