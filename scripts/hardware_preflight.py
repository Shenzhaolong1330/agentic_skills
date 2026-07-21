#!/usr/bin/env python3
"""Run a fixed, side-effect-free hardware authorization preflight."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from agentic_skills_harness.hardware_gate import HardwareGate
from agentic_skills_harness.manifest import find_entrypoint, load_manifest
from agentic_skills_harness.types import SkillContext, SkillMode


OPERATIONS = {
    "task": ("task-pick-tube-insert-rack", "task_runner", False),
    "single": ("task-pick-tube-insert-rack", "single_flow", False),
    "full": ("task-pick-tube-insert-rack", "full_flow", False),
    "object-locator": ("atomic-perception-locate-object-3d", "object-locator-json", False),
    "motion": ("atomic-motion-franka-move-to-pose", "move_to_pose", False),
    "gripper": ("atomic-gripper-franka-open-close", "gripper_control", False),
    "gripper-status": ("atomic-gripper-franka-open-close", "gripper_status", False),
    "status": ("atomic-state-dual-franka-reset", "check_and_reset_status", True),
    "ensure": ("atomic-state-dual-franka-reset", "check_and_reset_ensure", True),
    "reset": ("procedure-robot-reset-home", "run_robot_reset", True),
    "recover": ("procedure-robot-reset-home", "run_robot_recover", True),
    "home": ("procedure-robot-reset-home", "run_robot_go_home", False),
    "viewer": ("atomic-state-realsense-viewer", "view_state_realsense", False),
    "viewer-reset": ("atomic-state-realsense-viewer", "view_state_realsense_reset", False),
    "handover": ("procedure-franka-handover-transition", "handover_transition", False),
    "grasp": ("task-pick-tube-insert-rack", "task_live_grasp_handover", False),
    "insert": ("task-pick-tube-insert-rack", "task_live_insert_flow", False),
}


def evaluate_operation(
    operation: str,
    *,
    mode: str,
    hardware_allowed: bool,
    execute: bool,
    manifest_path: Path,
):
    manifest = load_manifest(manifest_path)
    skill_name, entrypoint_name, recovery = OPERATIONS[operation]
    entrypoint = find_entrypoint(manifest, skill_name, entrypoint_name)
    context = SkillContext(
        run_id="hardware_preflight",
        mode=SkillMode(mode),
        hardware_allowed=bool(hardware_allowed),
        execute=bool(execute),
        manifest_path=str(manifest_path),
    )
    decision = HardwareGate(manifest).evaluate(context, entrypoint, recovery=recovery)
    return manifest, entrypoint, decision


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--operation", choices=sorted(OPERATIONS), required=True)
    parser.add_argument("--mode", choices=[item.value for item in SkillMode], default=SkillMode.MOCK.value)
    parser.add_argument("--hardware-allowed", action="store_true", help="Allow access to real hardware in live mode.")
    parser.add_argument("--execute", action="store_true", help="Allow physical side effects in live mode.")
    parser.add_argument("--manifest", type=Path, default=REPO_ROOT / "skill_manifest.json")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    _, entrypoint, decision = evaluate_operation(
        args.operation,
        mode=args.mode,
        hardware_allowed=args.hardware_allowed,
        execute=args.execute,
        manifest_path=args.manifest,
    )
    print(json.dumps({"operation": args.operation, "entrypoint": entrypoint["name"], "decision": decision.to_dict()}, indent=2))
    return 0 if decision.allowed or decision.planned_only else 1


if __name__ == "__main__":
    raise SystemExit(main())
