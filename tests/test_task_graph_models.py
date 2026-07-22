from __future__ import annotations

import unittest

from agentic_skills_harness.planning import GraphNode, NodeKind, RetryPolicy, TaskGraph


class TaskGraphModelTests(unittest.TestCase):
    def test_all_node_kinds_are_closed(self):
        for kind in ("OBSERVE", "COMPUTE", "CHECK", "ACT", "VERIFY", "RECOVER", "APPROVAL", "HUMAN_ACTION"):
            self.assertEqual(GraphNode("n" + kind, kind).kind.value, kind)

    def test_retry_is_bounded(self):
        with self.assertRaises(Exception):
            RetryPolicy(0)
        with self.assertRaises(Exception):
            RetryPolicy(65)
        graph = TaskGraph.from_dict({"graph_id": "g", "goal_id": "goal", "entry_node_id": "n", "nodes": [{"node_id": "n", "kind": "CHECK", "preconditions": [{"operator": "exists", "operands": [{"predicate": "x"}]}]}], "terminal_nodes": ["n"]})
        self.assertEqual(graph.nodes[0].retry_policy.max_attempts, 1)
