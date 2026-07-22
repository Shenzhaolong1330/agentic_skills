from __future__ import annotations

import unittest

from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.planning import TaskGraphCompiler
from tests.planning_helpers import example, registry


class TaskGraphCompilerTests(unittest.TestCase):
    def test_valid_examples_are_static(self):
        r = registry()
        compiler = TaskGraphCompiler(r, manifest=load_manifest("skill_manifest.json"))
        for goal, graph in (("observe_object.goal.json", "observe_object.graph.json"), ("move_to_pose.goal.json", "move_to_pose.graph.json"), ("bounded_recovery.goal.json", "bounded_recovery.graph.json")):
            report = compiler.compile(example(goal), example("dry_run.envelope.json"), example(graph))
            self.assertTrue(report.ok, (goal, [item.to_dict() for item in report.issues]))
            payload = report.compiled_graph.to_dict()
            forbidden = {"executable", "argv", "env", "hardware_allowed", "execute", "adapter", "backend"}
            def keys(value):
                if isinstance(value, dict):
                    return set(value).union(*(keys(item) for item in value.values()))
                if isinstance(value, list):
                    return set().union(*(keys(item) for item in value))
                return set()
            self.assertFalse(forbidden & keys(payload))

    def test_unknown_and_unsupported_capabilities_are_rejected(self):
        r = registry()
        goal = example("observe_object.goal.json")
        envelope = example("dry_run.envelope.json")
        graph = example("observe_object.graph.json")
        graph["nodes"][0]["capability_id"] = "unknown.capability"
        report = TaskGraphCompiler(r).compile(goal, envelope, graph)
        self.assertIn("UNKNOWN_CAPABILITY", {item.code for item in report.issues})
        graph["nodes"][0]["capability_id"] = "perception.locate_object_3d"
        report = TaskGraphCompiler(r).compile(goal, envelope, graph)
        self.assertIn("CAPABILITY_UNSUPPORTED", {item.code for item in report.issues})

    def test_live_plan_only_is_rejected(self):
        r = registry()
        envelope = example("dry_run.envelope.json")
        envelope["target_mode"] = "live"
        report = TaskGraphCompiler(r).compile(example("move_to_pose.goal.json"), envelope, example("move_to_pose.graph.json"))
        self.assertIn("CAPABILITY_PLAN_ONLY_FOR_LIVE", {item.code for item in report.issues})
