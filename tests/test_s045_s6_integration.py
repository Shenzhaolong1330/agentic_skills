from __future__ import annotations

import unittest

from agentic_skills_harness.dispatch.coverage import audit_adapter_coverage
from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.planning import TaskGraphCompiler
from tests.planning_helpers import example, registry


class S045S6IntegrationTests(unittest.TestCase):
    def test_adapter_and_compiler_contracts_share_manifest(self):
        r = registry()
        coverage = audit_adapter_coverage(r)
        self.assertEqual(coverage["reviewed_capabilities"], len(r.list(include_internal=True, include_legacy=True)))
        report = TaskGraphCompiler(r, manifest=load_manifest("skill_manifest.json")).compile(example("move_to_pose.goal.json"), example("dry_run.envelope.json"), example("move_to_pose.graph.json"))
        self.assertTrue(report.ok)
        self.assertEqual(report.compiled_graph.target_mode, "dry_run")

