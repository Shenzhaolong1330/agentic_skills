from __future__ import annotations

from agentic_skills_harness.planning.goals import EntitySpec, EvidenceRequirement, GoalKind, GoalSpec
from agentic_skills_harness.world.predicates import PredicateSpec

from .facts import HOLE_ID, LEFT_GRIPPER, RACK_ID, RIGHT_GRIPPER, ROBOT_ID, TUBE_ID


def _equals(predicate: str, entity_id: str, value: object, predicate_id: str) -> dict:
    return PredicateSpec("equals", ({"predicate": predicate, "entity_id": entity_id}, value), predicate_id).to_dict()


def canonical_goal_spec(*, goal_id: str = "pick_tube_insert_rack.single_tube.goal") -> GoalSpec:
    success = PredicateSpec(
        "all",
        (
            _equals("inside", TUBE_ID, True, "tube_inside_target_hole"),
            _equals("supported_by", TUBE_ID, True, "tube_supported_by_rack"),
            _equals("holding", LEFT_GRIPPER, False, "left_gripper_released"),
            _equals("holding", RIGHT_GRIPPER, False, "right_gripper_released"),
            _equals("release.evidence", TUBE_ID, True, "fresh_release_evidence"),
            _equals("insertion.evidence", TUBE_ID, True, "fresh_insertion_evidence"),
            _equals("robot.health", ROBOT_ID, "READY", "robot_ready"),
        ),
        "pick_tube_insert_rack.goal_satisfied",
    )
    evidence = tuple(
        EvidenceRequirement(item_id, {"predicate": predicate, "entity_id": entity_id}, "VERIFIED", True, 0.9, "base", item_id)
        for item_id, predicate, entity_id in (
            ("inside_evidence", "inside", TUBE_ID), ("supported_evidence", "supported_by", TUBE_ID),
            ("release_evidence", "release.evidence", TUBE_ID), ("insertion_evidence", "insertion.evidence", TUBE_ID),
            ("robot_health_evidence", "robot.health", ROBOT_ID),
        )
    )
    return GoalSpec(
        goal_id=goal_id, goal_kind=GoalKind.PHYSICAL_STATE_CHANGE,
        description="Pick one allowed loose tube, insert it into a supported rack hole, and verify the final offline evidence.",
        entities=(EntitySpec(TUBE_ID, "tube", {"loose": True}), EntitySpec(RACK_ID, "rack"), EntitySpec(HOLE_ID, "rack_hole"), EntitySpec(LEFT_GRIPPER, "gripper"), EntitySpec(RIGHT_GRIPPER, "gripper")),
        success_predicate=success, required_evidence=evidence,
        metadata={"task_id": "pick_tube_insert_rack", "task_version": "1.0.0", "offline_physical_verification": False},
    )


__all__ = ["canonical_goal_spec"]
