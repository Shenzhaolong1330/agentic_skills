from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable

from ..capability import CapabilityContract
from ..contracts.resources import StateInvalidation
from ..contracts.results import ActionResult
from .models import FactStatus, WorldFact
from .store import WorldStateStore


PLATFORM_RESET_KEYS = ("robot.pose", "robot.state", "gripper.state", "relation.holding", "object.held_state", "object.dynamic_pose")


@dataclass(frozen=True)
class InvalidationReport:
    event: str
    invalidated_fact_ids: tuple[str, ...]
    requires_reobserve: bool

    def to_dict(self) -> dict[str, Any]:
        return {"event": self.event, "invalidated_fact_ids": list(self.invalidated_fact_ids), "requires_reobserve": self.requires_reobserve}


class InvalidationEngine:
    def __init__(self, store: WorldStateStore) -> None:
        self.store = store

    @staticmethod
    def _key_matches(fact: WorldFact, key: str) -> bool:
        predicate = fact.predicate.lower()
        subject = fact.subject.entity_id.lower()
        entity_type = fact.subject.entity_type.lower()
        key = key.lower()
        if key.startswith("fact:"):
            return fact.fact_id == key[5:]
        if key.startswith("subject_prefix:"):
            return subject.startswith(key[15:])
        if key.startswith("predicate:"):
            return predicate == key[10:]
        if key.startswith("predicate_prefix:"):
            return predicate.startswith(key[17:])
        if key.startswith("entity:"):
            return fact.subject.entity_id == key[7:]
        if key.startswith("source_capability:"):
            return (fact.source_capability_id or "") == key[19:]
        if key in {"robot.pose", "robot.state"}:
            return ("robot" in entity_type or "robot" in subject) and ("pose" in predicate or "state" in predicate or "health" in predicate)
        if key == "gripper.state":
            return "gripper" in entity_type or "gripper" in subject or "gripper" in predicate
        if key == "relation.holding":
            return predicate in {"holding", "relation.holding"} or "holding" in predicate
        if key == "object.held_state":
            return "held" in predicate or "holding" in predicate
        if key == "object.dynamic_pose":
            return ("pose" in predicate or "position" in predicate) and ("object" in entity_type or fact.metadata.get("dynamic", False))
        if key == "world.observations":
            return fact.metadata.get("observation", False) is True or (fact.source_capability_id or "").startswith(("perception.", "state.", "robot.observe", "gripper.observe"))
        return predicate == key or predicate.startswith(key + ".")

    def apply(self, invalidation: StateInvalidation | dict[str, Any]) -> InvalidationReport:
        item = invalidation if isinstance(invalidation, StateInvalidation) else StateInvalidation.from_dict(invalidation)
        selected = [fact for fact in self.store.all_latest() if any(self._key_matches(fact, key) for key in item.keys)]
        invalidated = tuple(self.store.invalidate(fact_ids=(fact.fact_id for fact in selected), reason=item.reason))
        return InvalidationReport(item.scope, tuple(fact.fact_id for fact in invalidated), item.requires_reobserve)

    def invalidate_for_capability(self, capability: CapabilityContract, result: ActionResult | None = None, *, arguments: dict[str, Any] | None = None) -> InvalidationReport:
        if result is not None and not result.command_executed:
            return InvalidationReport("not_executed", (), False)
        capability_id = capability.capability_id.lower()
        recovery = capability.kind.value == "recovery" or any(word in capability_id for word in ("reset", "recover", "home"))
        if recovery:
            selected = [fact for fact in self.store.all_latest() if any(self._key_matches(fact, key) for key in PLATFORM_RESET_KEYS)]
            invalidated = self.store.invalidate(fact_ids=(fact.fact_id for fact in selected), reason="platform reset/recovery invalidation")
            return InvalidationReport("reset", tuple(fact.fact_id for fact in invalidated), True)
        operation = str((arguments or {}).get("operation", (arguments or {}).get("command", ""))).lower()
        if capability.controls_gripper and operation in {"open", "release"}:
            selected = [fact for fact in self.store.all_latest() if self._key_matches(fact, "relation.holding") or self._key_matches(fact, "object.held_state")]
            invalidated = self.store.invalidate(fact_ids=(fact.fact_id for fact in selected), reason="gripper open/release requires independent observation")
            return InvalidationReport("gripper_open_or_release", tuple(fact.fact_id for fact in invalidated), True)
        if capability.moves_robot:
            selected = [fact for fact in self.store.all_latest() if self._key_matches(fact, "robot.pose")]
            invalidated = self.store.invalidate(fact_ids=(fact.fact_id for fact in selected), reason="robot motion changes pose freshness")
            return InvalidationReport("robot_motion", tuple(fact.fact_id for fact in invalidated), True)
        return InvalidationReport("none", (), False)
