from __future__ import annotations

import unittest

from agentic_skills_harness.planning import TaskGraphCompiler
from tests.planning_helpers import example, registry


class GraphResourceTests(unittest.TestCase):
    def test_missing_required_resource_is_rejected(self):
        r = registry()
        graph = example("observe_object.graph.json")
        graph["nodes"][0]["resource_requirements"] = []
        report = TaskGraphCompiler(r).compile(example("observe_object.goal.json"), example("dry_run.envelope.json"), graph)
        self.assertIn("RESOURCE_REQUIREMENT_MISSING", {item.code for item in report.issues})
