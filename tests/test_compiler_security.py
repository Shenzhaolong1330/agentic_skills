from __future__ import annotations

import unittest

from agentic_skills_harness.planning import TaskGraphCompiler
from tests.planning_helpers import example, registry


class CompilerSecurityTests(unittest.TestCase):
    def test_malicious_planning_fields_are_rejected(self):
        r = registry()
        accepted = 0
        for key in ("executable", "argv", "env", "adapter", "backend", "script", "cwd", "execute", "hardware_allowed", "operator_token") * 4:
            goal = example("observe_object.goal.json")
            goal["metadata"] = {key: "injected"}
            report = TaskGraphCompiler(r).compile(goal, example("dry_run.envelope.json"), example("observe_object.graph.json"))
            accepted += int(report.ok)
        self.assertEqual(accepted, 0)

