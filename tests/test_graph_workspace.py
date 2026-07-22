from __future__ import annotations

import unittest

from agentic_skills_harness.planning import TaskGraphCompiler
from tests.planning_helpers import example, registry


class GraphWorkspaceTests(unittest.TestCase):
    def test_literal_workspace_violation_is_rejected(self):
        r = registry()
        graph = example("move_to_pose.graph.json")
        graph["nodes"][1]["arguments"]["xyz_m"] = [9.0, 9.0, 9.0]
        report = TaskGraphCompiler(r).compile(example("move_to_pose.goal.json"), example("dry_run.envelope.json"), graph)
        self.assertIn("WORKSPACE_VIOLATION", {item.code for item in report.issues})

    def test_dynamic_pose_requires_guard(self):
        r = registry()
        graph = example("move_to_pose.graph.json")
        node = graph["nodes"][1]
        node["arguments"].pop("xyz_m")
        node["input_bindings"] = [{"binding_id": "pose", "target_argument_path": "/xyz_m", "source_type": "WORLD_FACT", "world_fact_selector": {"predicate": "object.pose"}}]
        report = TaskGraphCompiler(r).compile(example("move_to_pose.goal.json"), example("dry_run.envelope.json"), graph)
        self.assertIn("DYNAMIC_POSE_WITHOUT_WORKSPACE_GUARD", {item.code for item in report.issues})

