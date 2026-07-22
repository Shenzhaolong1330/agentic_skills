from __future__ import annotations

from pathlib import Path

from agentic_skills_harness.world import EntityRef, FactStatus, FakeClock, InvalidationEngine, PredicateEngine, PredicateSpec, WorldFact, WorldStateStore
from agentic_skills_harness.verification import VerifierEngine, apply_effect_verification, apply_goal_verification
from agentic_skills_harness.contracts import ActionResult, ActionStatus


def fact(fact_id, predicate, value, *, entity_id="entity-1", entity_type="object", status=FactStatus.VERIFIED, observed_at="2026-01-01T00:00:00Z", valid_until="2026-01-01T00:10:00Z", confidence=0.9, metadata=None):
    return WorldFact(fact_id, EntityRef(entity_id, entity_type), predicate, None, value, status, confidence, None, observed_at, valid_until, "observation.check", "1.0.0", (), "calibration", 0, metadata or {})


def test_world_fact_snapshot_and_freshness_with_fake_clock():
    clock = FakeClock("2026-01-01T00:05:00Z")
    store = WorldStateStore(clock=clock)
    stored = store.add_fact(fact("pose-1", "object.pose", {"x": 1}, entity_id="object-1"))
    assert stored.revision == 1
    assert store.query(predicate="object.pose", fresh=True, now="2026-01-01T00:05:00Z")
    clock.advance(600)
    assert store.query(predicate="object.pose", fresh=True, now="2026-01-01T00:15:00Z") == ()
    snapshot = store.snapshot()
    restored = WorldStateStore()
    restored.restore(snapshot)
    assert restored.snapshot().to_json() == snapshot.to_json()


def test_predicates_are_deterministic_and_do_not_guess_missing_facts():
    store = WorldStateStore()
    store.add_fact(fact("health-1", "robot.health", "READY", entity_id="robot-1", entity_type="robot"))
    store.add_fact(fact("value-1", "resource.available", 3, entity_id="resource-1", entity_type="resource"))
    engine = PredicateEngine()
    ready = engine.evaluate(PredicateSpec("robot_health_is", ({"predicate": "robot.health", "entity_id": "robot-1"}, "READY")), store, now="2026-01-01T00:05:00Z")
    missing = engine.evaluate(PredicateSpec("exists", ({"predicate": "robot.pose", "entity_id": "robot-1"},)), store, now="2026-01-01T00:05:00Z")
    assert ready.satisfied is True
    assert missing.satisfied is False


def test_predicate_security_rejects_code_expression_and_deep_tree():
    try:
        PredicateSpec.from_dict({"operator": "eval", "operands": []})
        assert False
    except ValueError:
        pass
    try:
        PredicateSpec.from_dict({"operator": "equals", "operands": [{"expression": "__import__('os')"}, 1]})
        assert False
    except ValueError:
        pass


def test_reset_invalidation_removes_holding_pose_and_gripper_facts():
    store = WorldStateStore()
    store.add_fact(fact("holding-1", "holding", True, entity_id="object-1"))
    store.add_fact(fact("pose-1", "object.pose", {"x": 1}, entity_id="object-1", metadata={"dynamic": True}))
    store.add_fact(fact("gripper-1", "gripper.state", "CLOSED", entity_id="gripper-1", entity_type="gripper"))
    store.add_fact(fact("robot-1", "robot.pose", {"x": 0}, entity_id="robot-1", entity_type="robot"))
    from dataclasses import replace
    from agentic_skills_harness.capability import CapabilityContract
    from agentic_skills_harness.manifest import load_manifest
    from agentic_skills_harness.registry import CapabilityRegistry
    root = Path(__file__).resolve().parents[1]
    capability = CapabilityRegistry.from_manifest(load_manifest(root / "skill_manifest.json"), repo_root=root).require("recovery.robot_reset")
    result = ActionResult("recovery.robot_reset", ActionStatus.SUCCEEDED, True, True, False, None, None, False, None, {}, (), (), {}, {}, "2026-01-01T00:00:00Z", "2026-01-01T00:00:01Z")
    report = InvalidationEngine(store).invalidate_for_capability(capability, result)
    assert len(report.invalidated_fact_ids) == 4
    assert store.query(predicate="holding", status=FactStatus.VERIFIED) == ()
    assert store.query(predicate="object.pose", fresh=True) == ()


def test_verifier_separates_effect_and_goal_and_guards_limited_action():
    root = Path(__file__).resolve().parents[1]
    from agentic_skills_harness.manifest import load_manifest
    from agentic_skills_harness.registry import CapabilityRegistry
    registry = CapabilityRegistry.from_manifest(load_manifest(root / "skill_manifest.json"), repo_root=root)
    capability = registry.require("motion.move_to_pose")
    store = WorldStateStore()
    action = ActionResult("motion.move_to_pose", ActionStatus.SUCCEEDED, True, True, False, None, None, False, None, {}, (), (), {"returncode": 0}, {}, "2026-01-01T00:00:00Z", "2026-01-01T00:00:01Z")
    engine = VerifierEngine(registry)
    effect = engine.verify_effect(capability, action, store)
    assert effect.verified is False
    assert apply_effect_verification(action, effect).effect_observed is False
    assert apply_goal_verification(action, effect).goal_verified is False
    store.add_fact(fact("ready-1", "robot.health", "READY", entity_id="robot-1", entity_type="robot"))
    goal = engine.verify_goal(PredicateSpec("equals", ({"predicate": "robot.health", "entity_id": "robot-1"}, "READY")), store, now="2026-01-01T00:05:00Z")
    assert goal.verified is True
    assert apply_goal_verification(action, goal).goal_verified is True
