from __future__ import annotations

import time
import unittest

from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.planning import TaskGraphCompiler
from tests.planning_helpers import example, registry


class CompilerPerformanceTests(unittest.TestCase):
    def test_100_node_graph_is_bounded_and_deterministic(self):
        r = registry()
        graph = {"graph_id": "large", "goal_id": "observe_robot_health_fixture", "entry_node_id": "n0", "nodes": [], "edges": [], "terminal_nodes": ["n99"]}
        resource = {"resource_id": "robot.dual_franka.state", "mode": "shared", "description": "state"}
        for index in range(100):
            graph["nodes"].append({"node_id": f"n{index}", "kind": "OBSERVE", "capability_id": "robot.observe_health", "arguments": {}, "resource_requirements": [resource]})
            if index:
                graph["edges"].append({"edge_id": f"e{index}", "source_node_id": f"n{index-1}", "target_node_id": f"n{index}", "condition": "SUCCESS"})
        envelope = example("dry_run.envelope.json")
        envelope.update({"allowed_capabilities": ["robot.observe_health"], "max_nodes": 100, "max_depth": 100, "max_tool_calls": 100, "max_recovery_actions": 0})
        goal = example("observe_object.goal.json")
        compiler = TaskGraphCompiler(r, manifest=load_manifest("skill_manifest.json"))
        compiler.compile(goal, envelope, graph)
        timings = []
        hashes = []
        for _ in range(5):
            start = time.perf_counter()
            report = compiler.compile(goal, envelope, graph)
            timings.append(time.perf_counter() - start)
            self.assertTrue(report.ok, [item.to_dict() for item in report.issues])
            hashes.append(report.compiled_graph.plan_hash)
        self.assertEqual(len(set(hashes)), 1)
        self.assertLess(max(timings), 1.0)
