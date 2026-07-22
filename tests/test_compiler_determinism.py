from __future__ import annotations

import unittest

from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.planning import TaskGraphCompiler
from tests.planning_helpers import example, registry


class CompilerDeterminismTests(unittest.TestCase):
    def test_same_input_has_same_hash(self):
        r = registry()
        c = TaskGraphCompiler(r, manifest=load_manifest("skill_manifest.json"))
        hashes = [c.compile(example("move_to_pose.goal.json"), example("dry_run.envelope.json"), example("move_to_pose.graph.json")).compiled_graph.plan_hash for _ in range(5)]
        self.assertEqual(len(set(hashes)), 1)

    def test_manifest_change_changes_digest_and_hash(self):
        r = registry()
        manifest = load_manifest("skill_manifest.json")
        c1 = TaskGraphCompiler(r, manifest=manifest)
        c2 = TaskGraphCompiler(r, manifest={**manifest, "version": "0.2.changed"})
        p1 = c1.compile(example("observe_object.goal.json"), example("dry_run.envelope.json"), example("observe_object.graph.json")).compiled_graph
        p2 = c2.compile(example("observe_object.goal.json"), example("dry_run.envelope.json"), example("observe_object.graph.json")).compiled_graph
        self.assertNotEqual(p1.plan_hash, p2.plan_hash)

