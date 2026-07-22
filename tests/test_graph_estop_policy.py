from __future__ import annotations

import unittest

from agentic_skills_harness.planning import TaskGraphCompiler
from tests.planning_helpers import example, registry


class GraphEstopPolicyTests(unittest.TestCase):
    def test_estop_cannot_route_to_recovery(self):
        r = registry()
        graph = example("rejection_cases/estop_auto_recovery.graph.json")
        report = TaskGraphCompiler(r).compile(example("bounded_recovery.goal.json"), example("dry_run.envelope.json"), graph)
        self.assertIn("ESTOP_AUTO_RECOVERY_FORBIDDEN", {item.code for item in report.issues})
