from __future__ import annotations

import unittest

from agentic_skills_harness.planning import TaskGraphCompiler
from tests.planning_helpers import example, registry


class GraphVerifierTests(unittest.TestCase):
    def test_physical_action_without_verifier_is_rejected(self):
        r = registry()
        graph = example("move_to_pose.graph.json")
        graph["nodes"][1].pop("verifier")
        report = TaskGraphCompiler(r).compile(example("move_to_pose.goal.json"), example("dry_run.envelope.json"), graph)
        self.assertIn("MISSING_VERIFIER", {item.code for item in report.issues})

    def test_returncode_verifier_is_rejected(self):
        r = registry()
        graph = example("move_to_pose.graph.json")
        graph["nodes"][1]["verifier"] = {"mode": "returncode", "source": "action_result"}
        report = TaskGraphCompiler(r).compile(example("move_to_pose.goal.json"), example("dry_run.envelope.json"), graph)
        self.assertIn("VERIFIER_WEAKENS_CONTRACT", {item.code for item in report.issues})
