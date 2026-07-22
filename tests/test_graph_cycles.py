from __future__ import annotations

import unittest

from agentic_skills_harness.planning import TaskGraphCompiler
from tests.planning_helpers import example, registry


class GraphCycleTests(unittest.TestCase):
    def test_bounded_recovery_cycle_compiles(self):
        r = registry()
        report = TaskGraphCompiler(r, manifest=__import__("agentic_skills_harness.manifest", fromlist=["load_manifest"]).load_manifest("skill_manifest.json")).compile(example("bounded_recovery.goal.json"), example("dry_run.envelope.json"), example("bounded_recovery.graph.json"))
        self.assertTrue(report.ok, [item.to_dict() for item in report.issues])

    def test_unbounded_cycle_is_rejected(self):
        r = registry()
        goal = example("observe_object.goal.json")
        envelope = example("dry_run.envelope.json")
        graph = {"graph_id": "cycle", "goal_id": goal["goal_id"], "entry_node_id": "n", "nodes": [{"node_id": "n", "kind": "OBSERVE", "capability_id": "robot.observe_health", "resource_requirements": [{"resource_id": "robot.dual_franka.state", "mode": "shared", "description": "state"}]}], "edges": [{"edge_id": "loop", "source_node_id": "n", "target_node_id": "n", "condition": "FAILURE"}], "terminal_nodes": ["n"]}
        report = TaskGraphCompiler(r).compile(goal, envelope, graph)
        self.assertIn("UNBOUNDED_CYCLE", {item.code for item in report.issues})
