from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from agentic_skills_harness.execution import GraphExecutor
from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.registry import CapabilityRegistry
from agentic_skills_harness.world.store import WorldStateStore
from task_skills.pick_tube_insert_rack.agentic.definition import PICK_TUBE_INSERT_RACK
from task_skills.pick_tube_insert_rack.agentic.dispatcher import PickTubeOfflineDispatcher
from task_skills.pick_tube_insert_rack.agentic.facts import populate_success_fixture


ROOT = Path(__file__).resolve().parents[1]


class PickTubeAgenticMigrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest = load_manifest(ROOT / "skill_manifest.json")
        cls.base_registry = CapabilityRegistry.from_manifest(cls.manifest, repo_root=ROOT)
        cls.report, cls.registry, cls.goal, cls.envelope, cls.graph, cls.context = PICK_TUBE_INSERT_RACK.compile(cls.base_registry, cls.manifest, mode="mock")

    def test_compiles_non_opaque_graph(self):
        self.assertTrue(self.report.ok, self.report.to_dict())
        self.assertGreaterEqual(len(self.graph.nodes), 20)
        self.assertFalse(self.graph.metadata["opaque_single_node"])
        self.assertTrue(any(node.kind.value == "OBSERVE" for node in self.graph.nodes))
        self.assertTrue(any(node.kind.value == "VERIFY" for node in self.graph.nodes))

    def test_internal_capabilities_are_not_public(self):
        public_ids = {item.capability_id for item in self.registry.list()}
        self.assertNotIn("task.pick_tube_insert_rack.compute.extract_geometry", public_ids)
        self.assertIn("task.pick_tube_insert_rack.compute.extract_geometry", self.context.internal_capability_allowlist)

    def test_mock_verifies_simulated_goal_without_physical_flags(self):
        world = populate_success_fixture(WorldStateStore())
        with tempfile.TemporaryDirectory() as root:
            result = GraphExecutor(self.report.compiled_graph, registry=self.registry, manifest=self.manifest, dispatcher=PickTubeOfflineDispatcher(), mode="mock", artifact_dir=root, world_state=world).run()
        self.assertEqual(result.outcome.value, "GOAL_VERIFIED")
        self.assertTrue(result.goal_verified)
        self.assertFalse(result.physical_execution_performed)
        self.assertFalse(result.physical_goal_verified)

    def test_dry_run_is_plan_only(self):
        world = populate_success_fixture(WorldStateStore())
        dry_report, dry_registry, _goal, _envelope, _graph, _context = PICK_TUBE_INSERT_RACK.compile(self.base_registry, self.manifest, mode="dry_run")
        with tempfile.TemporaryDirectory() as root:
            result = GraphExecutor(dry_report.compiled_graph, registry=dry_registry, manifest=self.manifest, dispatcher=PickTubeOfflineDispatcher(), mode="dry_run", artifact_dir=root, world_state=world).run()
        self.assertEqual(result.outcome.value, "PLAN_COMPLETED")
        self.assertFalse(result.goal_verified)
        self.assertFalse(result.physical_execution_performed)

    def test_action_dispatch_does_not_create_holding_or_support(self):
        world = WorldStateStore()
        dispatcher = PickTubeOfflineDispatcher()
        action = dispatcher.dispatch(type("Request", (), {"capability_id": "task.pick_tube_insert_rack.action.plan_grasp"})(), type("Context", (), {"mode": "mock"})())
        self.assertFalse(action.command_executed)
        self.assertEqual(world.query(predicate="holding"), ())
        self.assertEqual(world.query(predicate="supported_by"), ())

    def test_live_mode_is_rejected_by_task_definition(self):
        with self.assertRaises(ValueError):
            PICK_TUBE_INSERT_RACK.build_envelope(mode="live")


if __name__ == "__main__":
    unittest.main()
