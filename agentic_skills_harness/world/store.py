from __future__ import annotations

from collections.abc import Iterable
from dataclasses import replace
from typing import Any

from ..contracts.serialization import utc_now_iso
from .clock import Clock, SystemClock
from .models import EntityRef, FactStatus, WorldFact
from .snapshot import WorldSnapshot


class WorldStateStore:
    def __init__(self, *, clock: Clock | None = None) -> None:
        self.clock = clock or SystemClock()
        self._history: dict[str, list[WorldFact]] = {}
        self._revision = 0

    @property
    def current_revision(self) -> int:
        return self._revision

    def _next_revision(self) -> int:
        self._revision += 1
        return self._revision

    def add_fact(self, fact: WorldFact) -> WorldFact:
        if not isinstance(fact, WorldFact):
            raise TypeError("add_fact expects WorldFact")
        revision = self._next_revision()
        stored = replace(fact, revision=revision)
        self._history.setdefault(fact.fact_id, []).append(stored)
        return stored

    def upsert_fact(self, fact: WorldFact) -> WorldFact:
        return self.add_fact(fact)

    def get_fact(self, fact_id: str, *, include_invalidated: bool = False) -> WorldFact | None:
        values = self._history.get(fact_id, ())
        if not values:
            return None
        fact = values[-1]
        return fact if include_invalidated or fact.status != FactStatus.INVALIDATED else None

    def all_latest(self, *, include_invalidated: bool = False) -> tuple[WorldFact, ...]:
        values = [fact for facts in self._history.values() if facts for fact in [facts[-1]]]
        if not include_invalidated:
            values = [fact for fact in values if fact.status != FactStatus.INVALIDATED]
        return tuple(sorted(values, key=lambda fact: (fact.fact_id, fact.revision)))

    def query(self, *, subject: EntityRef | str | None = None, predicate: str | None = None, status: FactStatus | str | None = None, source_capability_id: str | None = None, include_invalidated: bool = False, fresh: bool = False, now: str | None = None, entity_id: str | None = None) -> tuple[WorldFact, ...]:
        expected_status = None if status is None else (status if isinstance(status, FactStatus) else FactStatus(status))
        result = []
        for fact in self.all_latest(include_invalidated=include_invalidated):
            if expected_status is not None and fact.status != expected_status:
                continue
            if isinstance(subject, EntityRef) and fact.subject != subject:
                continue
            if isinstance(subject, str) and fact.subject.entity_id != subject:
                continue
            if entity_id is not None and fact.subject.entity_id != entity_id:
                continue
            if predicate is not None and fact.predicate != predicate:
                continue
            if source_capability_id is not None and fact.source_capability_id != source_capability_id:
                continue
            if fresh and not fact.is_fresh(now or utc_now_iso()):
                continue
            result.append(fact)
        return tuple(result)

    def invalidate(self, *, fact_ids: Iterable[str] = (), predicate: str | None = None, entity_id: str | None = None, subject_prefix: str | None = None, source_capability_id: str | None = None, predicate_prefix: str | None = None, reason: str = "state invalidated") -> tuple[WorldFact, ...]:
        fact_id_set = set(fact_ids)
        selected = []
        for fact in self.all_latest(include_invalidated=False):
            if fact_id_set and fact.fact_id not in fact_id_set:
                continue
            if predicate is not None and fact.predicate != predicate:
                continue
            if predicate_prefix is not None and not fact.predicate.startswith(predicate_prefix):
                continue
            if entity_id is not None and fact.subject.entity_id != entity_id:
                continue
            if subject_prefix is not None and not fact.subject.entity_id.startswith(subject_prefix):
                continue
            if source_capability_id is not None and fact.source_capability_id != source_capability_id:
                continue
            selected.append(fact)
        invalidated = []
        for fact in selected:
            invalidated.append(self.add_fact(replace(fact, status=FactStatus.INVALIDATED, metadata={**dict(fact.metadata), "invalidation_reason": reason})))
        return tuple(invalidated)

    def snapshot(self) -> WorldSnapshot:
        return WorldSnapshot(self._revision, self.all_latest(include_invalidated=True))

    def restore(self, snapshot: WorldSnapshot) -> None:
        if not isinstance(snapshot, WorldSnapshot):
            snapshot = WorldSnapshot.from_dict(snapshot)
        self._history = {fact.fact_id: [fact] for fact in snapshot.facts}
        self._revision = snapshot.revision
