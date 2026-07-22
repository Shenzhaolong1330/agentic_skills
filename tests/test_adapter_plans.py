from __future__ import annotations

from pathlib import Path
import unittest

from agentic_skills_harness.dispatch import CapabilityDispatcher, DispatchContext, DispatchRequest, FakeBackend
from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.registry import CapabilityRegistry


ROOT = Path(__file__).resolve().parents[1]


class AdapterPlanDispatcherTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.registry = CapabilityRegistry.from_manifest(load_manifest(ROOT / "skill_manifest.json"), repo_root=ROOT)

    def test_dry_run_does_not_call_backend(self):
        backend = FakeBackend()
        dispatcher = CapabilityDispatcher(self.registry, backend=backend)
        result = dispatcher.dispatch(DispatchRequest("gripper.command", {"operation": "open", "side": "left"}), DispatchContext(mode="dry_run"))
        self.assertEqual(backend.calls, 0)
        self.assertTrue(result.planned_only)

    def test_mock_does_not_call_backend(self):
        backend = FakeBackend()
        dispatcher = CapabilityDispatcher(self.registry, backend=backend)
        result = dispatcher.dispatch(DispatchRequest("robot.observe_health", {}), DispatchContext(mode="mock"))
        self.assertEqual(backend.calls, 0)
        self.assertTrue(result.artifacts.get("planned_only"))

    def test_live_hardware_is_disabled_even_when_context_flags_are_true(self):
        backend = FakeBackend()
        dispatcher = CapabilityDispatcher(self.registry, backend=backend)
        result = dispatcher.dispatch(DispatchRequest("motion.move_to_pose", {"frame": "base", "xyz_m": [0.2, 0.0, 0.3], "source": "fixture", "side": "left"}), DispatchContext(mode="live", hardware_allowed=True, execute=True))
        self.assertEqual(backend.calls, 0)
        self.assertFalse(result.command_executed)
        self.assertFalse(result.planned_only)
        self.assertTrue(result.errors)
