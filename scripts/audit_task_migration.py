#!/usr/bin/env python3
"""Audit the offline pick_tube_insert_rack migration against the legacy stages."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.registry import CapabilityRegistry
from task_skills.pick_tube_insert_rack.agentic.definition import PICK_TUBE_INSERT_RACK


STAGES = (
    ("health preflight", ("observe_robot_health", "check_robot_ready"), "robot.observe_health", "task.pick_tube_insert_rack.observe.fixture", "fact:robot.health", "ROBOT_FAULT -> REQUEST_HUMAN/ABORT", "MIGRATED", "offline fixture only"),
    ("locate tube", ("observe_tube", "verify_tube_observation"), "perception.locate_object_3d", "task.pick_tube_insert_rack.observe.fixture", "fact:tube.observation", "PERCEPTION_NOT_FOUND -> REOBSERVE", "MIGRATED", "legacy live perception remains gated"),
    ("select arm", ("select_arm",), "task_pick_tube_insert_rack_logic.select_arm_for_tail_side", "task.pick_tube_insert_rack.compute.select_arm", "selected_arm fact", "MOTION_IK_INFEASIBLE -> ALTERNATE_ARM", "MIGRATED", "deterministic offline selection"),
    ("pregrasp", ("compute_tube_geometry", "build_grasp_plan", "move_to_pregrasp", "align_grasp_pose"), "build_pregrasp_and_grasp_plan", "task.pick_tube_insert_rack.compute.build_grasp_plan", "pose evidence", "motion failure -> bounded replan", "MIGRATED", "plan-only"),
    ("grasp", ("close_gripper", "verify_grasp"), "gripper.command", "task.pick_tube_insert_rack.action.plan_grasp", "grasp.evidence", "GRASP_NOT_CONFIRMED -> retreat/reobserve", "MIGRATED", "command never asserts holding"),
    ("handover", ("handover_procedure", "verify_handover"), "procedure.handover_transition", "task.pick_tube_insert_rack.action.plan_handover", "handover.evidence", "verification failure -> human/replan", "MIGRATED", "offline procedure contract"),
    ("locate rack", ("observe_rack",), "tube_insertion_skill observation", "task.pick_tube_insert_rack.observe.fixture", "rack.observation", "PERCEPTION_NOT_FOUND -> REOBSERVE", "MIGRATED", "fixture replay"),
    ("locate/select hole", ("select_empty_hole", "verify_hole"), "select_highest_confidence_hole", "task.pick_tube_insert_rack.compute.select_hole", "hole.observation", "low confidence -> alternate candidate", "MIGRATED", "bounded candidate selection"),
    ("insertion", ("align_holder_above_rack", "refine_insertion_alignment", "perform_insertion", "verify_insertion"), "insert_down_from_current_pose.py", "task.pick_tube_insert_rack.action.plan_insert", "insertion.evidence/inside", "verification failure -> safe retreat, reobserve, replan", "MIGRATED", "no lift-only retry"),
    ("release", ("release_gripper", "verify_release"), "gripper.command", "task.pick_tube_insert_rack.action.plan_release", "release.evidence/holding=false", "RELEASE_NOT_CONFIRMED -> reobserve/human", "MIGRATED", "release command is not support evidence"),
    ("retract/home", ("retract_to_safe",), "motion.go_home", "task.pick_tube_insert_rack.action.plan_retract", "safe pose evidence", "reset never auto-resumes held state", "MIGRATED", "offline plan only"),
    ("completion", ("verify_final_goal",), "legacy completion_flag", "goal predicate verifier", "inside/supported_by/release/health", "VERIFICATION_FAILED -> bounded recovery", "MIGRATED", "physical verification remains false offline"),
    ("abnormal recovery", (), "legacy reset wrapper", "RecoveryPolicyRegistry/RecoveryOrchestrator", "invalidation + fresh observation", "E-stop -> human/abort", "LEGACY_LIVE_UNMIGRATED", "no real reset/controller execution in S9"),
)

RECOVERY_MAP = {
    "health preflight": "REQUEST_HUMAN/ABORT; controller recovery only with safe evidence",
    "locate tube": "REOBSERVE",
    "select arm": "ALTERNATE_ARM or bounded replan",
    "pregrasp": "SAFE_RETREAT/RECOMPILE_REMAINDER",
    "grasp": "SAFE_RETREAT -> REOBSERVE",
    "handover": "REQUEST_HUMAN or RECOMPILE_REMAINDER",
    "locate rack": "REOBSERVE",
    "locate/select hole": "ALTERNATE_CANDIDATE",
    "insertion": "SAFE_RETREAT -> REOBSERVE -> RECOMPILE_REMAINDER",
    "release": "REOBSERVE or REQUEST_HUMAN",
    "retract/home": "SAFE_RETREAT; never infer held state",
    "completion": "RECOMPILE_REMAINDER or REQUEST_HUMAN",
    "abnormal recovery": "REQUEST_HUMAN/ABORT; no live reset in S9",
}


def audit() -> dict:
    manifest = load_manifest(ROOT / "skill_manifest.json")
    base = CapabilityRegistry.from_manifest(manifest, repo_root=ROOT)
    report, _registry, _goal, _envelope, graph, context = PICK_TUBE_INSERT_RACK.compile(base, manifest, mode="mock")
    entries = [{"legacy_stage": stage, "new_graph_nodes": list(nodes), "legacy_capability_or_script": legacy, "new_capability_id": new, "verification_mapping": verification, "failure_mapping": failure, "recovery_mapping": RECOVERY_MAP[stage], "migration_status": status, "known_limitation": limitation} for stage, nodes, legacy, new, verification, failure, status, limitation in STAGES]
    return {"task_id": PICK_TUBE_INSERT_RACK.task_id, "task_version": PICK_TUBE_INSERT_RACK.task_version, "graph_id": graph.graph_id, "graph_node_count": len(graph.nodes), "opaque_single_node_graph": len(graph.nodes) <= 1, "unreviewed_stage_count": sum(item["migration_status"] in {"UNREVIEWED", "UNKNOWN"} for item in entries), "entries": entries, "internal_capability_allowlist": list(context.internal_capability_allowlist), "compile_ok": report.ok, "offline_modes": list(PICK_TUBE_INSERT_RACK.supported_modes), "live_enabled": False, "physical_execution_performed": False, "physical_goal_verified": False}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-json", type=Path, default=None)
    args = parser.parse_args(argv)
    value = audit()
    rendered = json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(rendered + "\n", encoding="utf-8")
    return 0 if value["compile_ok"] and not value["opaque_single_node_graph"] and value["unreviewed_stage_count"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
