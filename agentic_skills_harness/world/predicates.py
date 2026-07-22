from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Mapping

from ..contracts.serialization import ContractValidationError, ensure_jsonable, reject_unknown, require_list, require_number, require_string, stable_dumps, utc_now_iso
from .models import FactStatus, WorldFact
from .store import WorldStateStore


_FORBIDDEN_KEYS = {"code", "script", "expression", "callback", "eval", "exec", "__import__"}


def _reject_code_fields(value: Any) -> None:
    if isinstance(value, dict):
        for key, item in value.items():
            if isinstance(key, str) and key.lower() in _FORBIDDEN_KEYS:
                raise ContractValidationError(f"predicate field is not permitted: {key}")
            _reject_code_fields(item)
    elif isinstance(value, list):
        for item in value:
            _reject_code_fields(item)


SUPPORTED_OPERATORS = {"exists", "equals", "not_equals", "greater_than", "less_than", "within_range", "fresh", "confidence_at_least", "frame_equals", "pose_error_within", "holding", "supported_by", "inside", "robot_health_is", "resource_available", "all", "any", "not"}


@dataclass(frozen=True)
class PredicateSpec:
    operator: str
    operands: tuple[Any, ...] = ()
    predicate_id: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "operator", require_string(self.operator, "operator", non_empty=True))
        if self.operator not in SUPPORTED_OPERATORS:
            raise ContractValidationError(f"unknown predicate operator: {self.operator}")
        values = tuple(self.operands) if isinstance(self.operands, (list, tuple)) else None
        if values is None:
            raise ContractValidationError("predicate operands must be an array")
        if len(values) > 128:
            raise ContractValidationError("predicate tree is too large")
        normalized = tuple(ensure_jsonable(value, "predicate operand") for value in values)
        for value in normalized:
            _reject_code_fields(value)
        object.__setattr__(self, "operands", normalized)
        if self.predicate_id:
            object.__setattr__(self, "predicate_id", require_string(self.predicate_id, "predicate_id", non_empty=True))
        else:
            object.__setattr__(self, "predicate_id", self.operator)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any], *, depth: int = 0) -> "PredicateSpec":
        if depth > 8:
            raise ContractValidationError("predicate nesting depth exceeded")
        reject_unknown(data, ("operator", "operands", "predicate_id"), ("operator", "operands"))
        operands = []
        for value in require_list(data["operands"], "operands"):
            if isinstance(value, dict) and "operator" in value:
                operands.append(cls.from_dict(value, depth=depth + 1).to_dict())
            else:
                operands.append(value)
        return cls(data["operator"], tuple(operands), data.get("predicate_id", ""))

    def to_dict(self) -> dict[str, Any]:
        return {"operator": self.operator, "operands": list(self.operands), "predicate_id": self.predicate_id}


@dataclass(frozen=True)
class PredicateResult:
    satisfied: bool
    predicate_id: str
    operator: str
    evidence_fact_ids: tuple[str, ...] = ()
    reason: str = ""
    errors: tuple[str, ...] = ()
    evaluated_at: str = field(default_factory=utc_now_iso)

    def to_dict(self) -> dict[str, Any]:
        return {"satisfied": self.satisfied, "predicate_id": self.predicate_id, "operator": self.operator, "evidence_fact_ids": list(self.evidence_fact_ids), "reason": self.reason, "errors": list(self.errors), "evaluated_at": self.evaluated_at}

    def to_json(self) -> str:
        return stable_dumps(self.to_dict())


def _selector(value: Any) -> dict[str, Any] | None:
    if not isinstance(value, dict):
        return None
    if "selector" in value:
        selected = value["selector"]
        return dict(selected) if isinstance(selected, dict) else None
    if any(key in value for key in ("fact_id", "subject_id", "entity_id", "predicate", "source_capability_id")):
        return dict(value)
    return None


class PredicateEngine:
    def __init__(self, *, max_depth: int = 8, max_nodes: int = 128) -> None:
        self.max_depth = max_depth
        self.max_nodes = max_nodes

    def _facts(self, selector: Any, store: WorldStateStore, *, now: str, fresh: bool = False) -> tuple[WorldFact, ...]:
        selected = _selector(selector)
        if selected is None:
            return ()
        fact_id = selected.get("fact_id")
        if fact_id:
            fact = store.get_fact(fact_id)
            return (fact,) if fact is not None and (not fresh or fact.is_fresh(now)) else ()
        facts = store.query(predicate=selected.get("predicate"), entity_id=selected.get("entity_id", selected.get("subject_id")), source_capability_id=selected.get("source_capability_id"), fresh=fresh, now=now)
        if selected.get("frame") is not None:
            facts = tuple(fact for fact in facts if fact.frame == selected["frame"])
        return facts

    @staticmethod
    def _value(fact: WorldFact) -> Any:
        return fact.value

    def evaluate(self, spec: PredicateSpec | Mapping[str, Any], store: WorldStateStore, *, now: str | None = None) -> PredicateResult:
        try:
            spec = spec if isinstance(spec, PredicateSpec) else PredicateSpec.from_dict(spec)
        except Exception as exc:
            return PredicateResult(False, "invalid", "invalid", reason="invalid predicate specification", errors=(str(exc),), evaluated_at=now or utc_now_iso())
        current = now or utc_now_iso()
        try:
            return self._evaluate(spec, store, current, 0, [0])
        except Exception as exc:
            return PredicateResult(False, spec.predicate_id, spec.operator, reason="predicate evaluation failed", errors=(str(exc),), evaluated_at=current)

    def _evaluate(self, spec: PredicateSpec, store: WorldStateStore, now: str, depth: int, count: list[int]) -> PredicateResult:
        count[0] += 1
        if depth > self.max_depth or count[0] > self.max_nodes:
            return PredicateResult(False, spec.predicate_id, spec.operator, reason="predicate limits exceeded", errors=("predicate_limits_exceeded",), evaluated_at=now)
        op, args = spec.operator, spec.operands
        if op in {"all", "any"}:
            nested = [PredicateSpec.from_dict(item, depth=depth + 1) if isinstance(item, dict) else None for item in args]
            if any(item is None for item in nested):
                return PredicateResult(False, spec.predicate_id, op, reason="logical operands must be predicates", errors=("invalid_logical_operand",), evaluated_at=now)
            results = [self._evaluate(item, store, now, depth + 1, count) for item in nested if item is not None]
            satisfied = all(item.satisfied for item in results) if op == "all" else any(item.satisfied for item in results)
            if not args and op == "any":
                satisfied = False
            evidence = tuple(dict.fromkeys(fact for item in results for fact in item.evidence_fact_ids))
            return PredicateResult(satisfied, spec.predicate_id, op, evidence, "all predicates satisfied" if satisfied else "logical predicate not satisfied", tuple(error for item in results for error in item.errors), now)
        if op == "not":
            if len(args) != 1 or not isinstance(args[0], dict):
                return PredicateResult(False, spec.predicate_id, op, reason="not requires one nested predicate", errors=("invalid_not_operand",), evaluated_at=now)
            child = self._evaluate(PredicateSpec.from_dict(args[0], depth=depth + 1), store, now, depth + 1, count)
            return PredicateResult(not child.satisfied, spec.predicate_id, op, child.evidence_fact_ids, "negated predicate", child.errors, now)
        if op == "exists":
            facts = self._facts(args[0] if len(args) == 1 else {}, store, now=now)
            return PredicateResult(bool(facts), spec.predicate_id, op, tuple(fact.fact_id for fact in facts), "fact exists" if facts else "fact missing", (), now)
        if op in {"fresh", "confidence_at_least", "frame_equals"}:
            facts = self._facts(args[0] if args else {}, store, now=now, fresh=op == "fresh")
            if not facts:
                return PredicateResult(False, spec.predicate_id, op, reason="fact missing or stale", evaluated_at=now)
            fact = facts[0]
            if op == "fresh":
                satisfied = fact.is_fresh(now)
            elif op == "confidence_at_least":
                satisfied = fact.confidence is not None and len(args) > 1 and fact.confidence >= float(args[1])
            else:
                satisfied = len(args) > 1 and fact.frame == args[1]
            return PredicateResult(satisfied, spec.predicate_id, op, (fact.fact_id,), "predicate satisfied" if satisfied else "fact does not meet requirement", (), now)
        if op == "pose_error_within":
            facts = self._facts(args[0] if args else {}, store, now=now, fresh=True)
            if not facts or len(args) < 3 or not isinstance(facts[0].value, dict):
                return PredicateResult(False, spec.predicate_id, op, tuple(fact.fact_id for fact in facts), reason="pose error evidence missing or malformed", evaluated_at=now)
            value = facts[0].value
            satisfied = float(value.get("translation_m", float("inf"))) <= float(args[1]) and float(value.get("rotation_rad", float("inf"))) <= float(args[2])
            return PredicateResult(satisfied, spec.predicate_id, op, (facts[0].fact_id,), "pose error within limits" if satisfied else "pose error exceeds limits", (), now)
        if op in {"holding", "supported_by", "inside", "robot_health_is", "resource_available"}:
            predicate = op
            selected = _selector(args[0]) if args else {}
            facts = self._facts({**(selected or {}), "predicate": predicate}, store, now=now, fresh=True)
            if not facts and op == "robot_health_is" and len(args) > 1:
                facts = self._facts({"predicate": "robot.health", "entity_id": selected.get("entity_id") if selected else None}, store, now=now, fresh=True)
            if not facts:
                return PredicateResult(False, spec.predicate_id, op, reason="relation or state fact missing", evaluated_at=now)
            expected = args[1] if len(args) > 1 else None
            satisfied = expected is None or any(self._value(fact) == expected or fact.object == expected for fact in facts)
            return PredicateResult(satisfied, spec.predicate_id, op, tuple(fact.fact_id for fact in facts), "relation satisfied" if satisfied else "relation value mismatch", (), now)
        if op in {"equals", "not_equals", "greater_than", "less_than", "within_range"}:
            facts = self._facts(args[0] if args else {}, store, now=now, fresh=True)
            if not facts or len(args) < 2:
                return PredicateResult(False, spec.predicate_id, op, tuple(fact.fact_id for fact in facts), reason="comparison evidence missing", evaluated_at=now)
            actual = self._value(facts[0])
            expected = args[1]
            try:
                if op == "equals":
                    satisfied = actual == expected
                elif op == "not_equals":
                    satisfied = actual != expected
                elif op == "greater_than":
                    satisfied = actual > expected
                elif op == "less_than":
                    satisfied = actual < expected
                else:
                    lower, upper = (args[1], args[2]) if len(args) > 2 else (expected[0], expected[1])
                    satisfied = lower <= actual <= upper
            except (TypeError, IndexError, KeyError):
                return PredicateResult(False, spec.predicate_id, op, (facts[0].fact_id,), reason="comparison types are incompatible", errors=("invalid_comparison",), evaluated_at=now)
            return PredicateResult(satisfied, spec.predicate_id, op, (facts[0].fact_id,), "comparison satisfied" if satisfied else "comparison false", (), now)
        return PredicateResult(False, spec.predicate_id, op, reason="unsupported operator", errors=("unsupported_operator",), evaluated_at=now)
