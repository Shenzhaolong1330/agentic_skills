from __future__ import annotations

from typing import Any

from agentic_skills_harness.planning.graph import EdgeCondition, GraphEdge, GraphNode, NodeKind, TaskGraph
from agentic_skills_harness.contracts.enums import ResourceMode
from agentic_skills_harness.contracts.resources import ResourceRequirement

from .capabilities import ACTION_CAPABILITIES, COMPUTE_CAPABILITIES, OBSERVE_CAPABILITY
from .facts import HOLE_ID, LEFT_GRIPPER, RACK_ID, ROBOT_ID, TUBE_ID
from .goals import canonical_goal_spec


def _equals(predicate: str, entity_id: str, value: object, predicate_id: str) -> dict[str, Any]:
    return {"operator": "equals", "operands": [{"predicate": predicate, "entity_id": entity_id}, value], "predicate_id": predicate_id}


def _fact_projection(node_id: str, fact_id: str, predicate: str, entity_id: str, value_path: str = "/fresh") -> dict[str, Any]:
    return {"fact_id": f"observation.{node_id}.{fact_id}", "predicate": predicate, "subject": {"entity_id": entity_id, "entity_type": "task_entity"}, "value_path": value_path, "confidence": 1.0, "frame": "base", "freshness_ttl_s": 300.0}


def canonical_task_graph(*, graph_id: str = "pick_tube_insert_rack.single_tube.graph", goal_id: str | None = None) -> TaskGraph:
    goal_id = goal_id or canonical_goal_spec().goal_id
    nodes: list[GraphNode] = []
    resource = (ResourceRequirement("task.pick_tube_insert_rack.compute", ResourceMode.SHARED, "task-local deterministic computation"),)
    def observe(node_id: str, label: str, entity_id: str) -> None:
        nodes.append(GraphNode(node_id, NodeKind.OBSERVE, OBSERVE_CAPABILITY, {}, resource_requirements=resource, metadata={"stage": label, "fact_projections": (_fact_projection(node_id, "fresh", f"observation.{label}", entity_id),)}))
    def compute(node_id: str, capability_id: str, label: str) -> None:
        nodes.append(GraphNode(node_id, NodeKind.COMPUTE, capability_id, {}, resource_requirements=resource, metadata={"stage": label, "operation": "literal", "value": {"stage": label, "deterministic": True}}))
    def action(node_id: str, capability_id: str, label: str) -> None:
        nodes.append(GraphNode(node_id, NodeKind.ACT, capability_id, {}, resource_requirements=resource, metadata={"stage": label, "offline_plan": True}))
    def check(node_id: str, label: str, predicate: dict[str, Any] | None = None) -> None:
        metadata: dict[str, Any] = {"stage": label}
        if predicate is not None:
            metadata["predicate"] = predicate
        nodes.append(GraphNode(node_id, NodeKind.CHECK, metadata=metadata))
    def verify(node_id: str, label: str, predicate: dict[str, Any]) -> None:
        nodes.append(GraphNode(node_id, NodeKind.VERIFY, metadata={"stage": label, "goal_predicate": predicate}))

    observe("observe_robot_health", "robot_health", ROBOT_ID)
    check("check_robot_ready", "robot_ready", _equals("robot.health", ROBOT_ID, "READY", "robot_ready_check"))
    observe("observe_tube", "tube", TUBE_ID)
    check("verify_tube_observation", "tube_observation", _equals("tube.observation", TUBE_ID, True, "tube_observation_check"))
    compute("compute_tube_geometry", COMPUTE_CAPABILITIES[0], "extract_tube_geometry")
    compute("select_arm", COMPUTE_CAPABILITIES[1], "select_arm")
    compute("build_grasp_plan", COMPUTE_CAPABILITIES[2], "build_pregrasp_grasp_plan")
    action("move_to_pregrasp", ACTION_CAPABILITIES["move"], "move_to_pregrasp")
    action("align_grasp_pose", ACTION_CAPABILITIES["move"], "align_grasp_pose")
    action("close_gripper", ACTION_CAPABILITIES["grasp"], "close_gripper")
    verify("verify_grasp", "verify_grasp", _equals("grasp.evidence", TUBE_ID, True, "grasp_verified"))
    action("handover_procedure", ACTION_CAPABILITIES["handover"], "handover")
    verify("verify_handover", "verify_handover", _equals("handover.evidence", TUBE_ID, True, "handover_verified"))
    observe("observe_rack", "rack", RACK_ID)
    action("move_non_holder_safe", ACTION_CAPABILITIES["move"], "move_non_holder_safe")
    compute("select_empty_hole", COMPUTE_CAPABILITIES[3], "select_highest_confidence_hole")
    verify("verify_hole", "verify_hole", _equals("hole.observation", HOLE_ID, True, "hole_verified"))
    action("align_holder_above_rack", ACTION_CAPABILITIES["move"], "align_holder_above_rack")
    compute("refine_insertion_alignment", COMPUTE_CAPABILITIES[0], "refine_insertion_alignment")
    action("perform_insertion", ACTION_CAPABILITIES["insert"], "perform_insertion_plan")
    verify("verify_insertion", "verify_insertion", _equals("insertion.evidence", TUBE_ID, True, "insertion_verified"))
    action("release_gripper", ACTION_CAPABILITIES["release"], "release_gripper")
    verify("verify_release", "verify_release", _equals("release.evidence", TUBE_ID, True, "release_verified"))
    action("retract_to_safe", ACTION_CAPABILITIES["retract"], "retract_to_safe")
    verify("verify_final_goal", "verify_final_goal", canonical_goal_spec(goal_id=goal_id).success_predicate.to_dict())

    edges = tuple(GraphEdge(f"edge_{index:02d}", nodes[index].node_id, nodes[index + 1].node_id, EdgeCondition.DEFAULT, priority=0) for index in range(len(nodes) - 1))
    return TaskGraph("pick_tube_insert_rack.single_tube.graph" if graph_id is None else graph_id, goal_id, nodes[0].node_id, tuple(nodes), edges, (nodes[-1].node_id,), {"task_id": "pick_tube_insert_rack", "task_version": "1.0.0", "canonical_single_tube": True, "opaque_single_node": False, "supported_modes": ["mock", "dry_run", "from_artifacts"]})


__all__ = ["canonical_task_graph"]
