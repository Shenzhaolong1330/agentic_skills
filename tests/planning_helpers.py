from __future__ import annotations

import json
from pathlib import Path

from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.registry import CapabilityRegistry
from agentic_skills_harness.planning import ExecutionEnvelope, GoalSpec, GraphNode, TaskGraph


ROOT = Path(__file__).resolve().parents[1]


def registry() -> CapabilityRegistry:
    return CapabilityRegistry.from_manifest(load_manifest(ROOT / "skill_manifest.json"), repo_root=ROOT)


def example(name: str) -> dict:
    return json.loads((ROOT / "examples/planning" / name).read_text(encoding="utf-8"))


def observation_contract():
    r = registry()
    cap = r.require("robot.observe_health")
    goal = GoalSpec.from_dict(example("observe_object.goal.json"))
    envelope = ExecutionEnvelope.from_dict(example("dry_run.envelope.json"))
    graph = TaskGraph.from_dict(example("observe_object.graph.json"))
    return r, goal, envelope, graph, cap
