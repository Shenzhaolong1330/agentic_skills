from __future__ import annotations

from pathlib import Path
import unittest

from agentic_skills_harness.dispatch import DispatchContext, DispatchRequest
from agentic_skills_harness.dispatch.adapters.fixed import build_fixed_adapter_registry
from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.registry import CapabilityRegistry


ROOT = Path(__file__).resolve().parents[1]


class AdapterContextFieldTests(unittest.TestCase):
    def test_context_controls_mode_and_execute_flags(self):
        registry = CapabilityRegistry.from_manifest(load_manifest(ROOT / "skill_manifest.json"), repo_root=ROOT)
        adapter = build_fixed_adapter_registry(registry)["gripper.command"]
        cap = registry.require("gripper.command")
        plan = adapter.build_plan(DispatchRequest("gripper.command", {"operation": "open", "side": "left"}), cap, DispatchContext(mode="dry_run", hardware_allowed=True, execute=True, robot_server="127.0.0.1:4242"))
        self.assertIn("--mode", plan.argv)
        self.assertIn("--hardware-allowed", plan.argv)
        self.assertIn("--execute", plan.argv)
        self.assertNotIn("--execute", DispatchRequest("gripper.command", {"operation": "open", "side": "left"}).to_dict()["arguments"])
