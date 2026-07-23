from __future__ import annotations

from typing import Any

from agentic_skills_harness.contracts.serialization import utc_now_iso
from agentic_skills_harness.world.models import EntityRef, FactStatus, WorldFact
from agentic_skills_harness.world.store import WorldStateStore


TUBE_ID = "tube_1"
RACK_ID = "rack_1"
HOLE_ID = "rack_hole_1"
LEFT_GRIPPER = "left_gripper"
RIGHT_GRIPPER = "right_gripper"
ROBOT_ID = "robot"


def add_fact(store: WorldStateStore, fact_id: str, *, entity_id: str, entity_type: str, predicate: str, value: Any, object_value: Any = None, confidence: float = 1.0, status: FactStatus = FactStatus.VERIFIED, ttl_s: float = 300.0) -> WorldFact:
    now = utc_now_iso()
    return store.add_fact(WorldFact(fact_id, EntityRef(entity_id, entity_type), predicate, value if object_value is None else object_value, value, status, confidence, "base", now, None, "task.pick_tube_insert_rack.fixture", "1.0.0", (), None, 0, {"fixture": True}, ttl_s))


def populate_success_fixture(store: WorldStateStore, *, success: bool = True) -> WorldStateStore:
    """Populate only evidence facts; commands never create these relations."""

    add_fact(store, "fixture.tube.observation", entity_id=TUBE_ID, entity_type="tube", predicate="tube.observation", value=success)
    add_fact(store, "fixture.rack.observation", entity_id=RACK_ID, entity_type="rack", predicate="rack.observation", value=success)
    add_fact(store, "fixture.hole.observation", entity_id=HOLE_ID, entity_type="rack_hole", predicate="hole.observation", value=success)
    add_fact(store, "fixture.grasp.evidence", entity_id=TUBE_ID, entity_type="tube", predicate="grasp.evidence", value=success)
    add_fact(store, "fixture.handover.evidence", entity_id=TUBE_ID, entity_type="tube", predicate="handover.evidence", value=success)
    add_fact(store, "fixture.insertion.evidence", entity_id=TUBE_ID, entity_type="tube", predicate="insertion.evidence", value=success)
    add_fact(store, "fixture.release.evidence", entity_id=TUBE_ID, entity_type="tube", predicate="release.evidence", value=success)
    add_fact(store, "fixture.inside", entity_id=TUBE_ID, entity_type="tube", predicate="inside", value=success, object_value={"entity_id": HOLE_ID, "entity_type": "rack_hole"})
    add_fact(store, "fixture.supported", entity_id=TUBE_ID, entity_type="tube", predicate="supported_by", value=success, object_value={"entity_id": HOLE_ID, "entity_type": "rack_hole"})
    add_fact(store, "fixture.holding.left", entity_id=LEFT_GRIPPER, entity_type="gripper", predicate="holding", value=False)
    add_fact(store, "fixture.holding.right", entity_id=RIGHT_GRIPPER, entity_type="gripper", predicate="holding", value=False)
    add_fact(store, "fixture.robot.health", entity_id=ROBOT_ID, entity_type="robot", predicate="robot.health", value="READY")
    return store


def reset_invalidate_fixture(store: WorldStateStore) -> tuple[str, ...]:
    return tuple(store.invalidate(fact_ids=(fact.fact_id for fact in store.all_latest()), reason="fixture reset invalidation"))


__all__ = ["HOLE_ID", "LEFT_GRIPPER", "RACK_ID", "RIGHT_GRIPPER", "ROBOT_ID", "TUBE_ID", "add_fact", "populate_success_fixture", "reset_invalidate_fixture"]
