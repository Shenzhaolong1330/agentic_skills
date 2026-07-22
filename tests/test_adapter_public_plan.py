from __future__ import annotations

from pathlib import Path
import unittest

from agentic_skills_harness.dispatch import DispatchContext, DispatchRequest
from agentic_skills_harness.dispatch.adapters.fixed import build_fixed_adapter_registry
from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.registry import CapabilityRegistry


ROOT = Path(__file__).resolve().parents[1]


class PublicPlanTests(unittest.TestCase):
    def test_public_plan_redacts_absolute_paths(self):
        registry = CapabilityRegistry.from_manifest(load_manifest(ROOT / "skill_manifest.json"), repo_root=ROOT)
        cap = registry.require("state.capture_realsense")
        plan = build_fixed_adapter_registry(registry)[cap.capability_id].build_plan(DispatchRequest(cap.capability_id, {}), cap, DispatchContext(mode="dry_run", artifact_dir="/tmp/adapter-public-plan"))
        public = plan.to_public_dict()
        self.assertFalse(any(str(value).startswith(("/home/", "/tmp/")) for value in public["argv"]))
        self.assertNotIn("cwd", public)

