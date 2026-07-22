from __future__ import annotations

import json
from pathlib import Path
import unittest

from agentic_skills_harness.dispatch import DispatchContext, DispatchRequest
from agentic_skills_harness.dispatch.adapters.fixed import FirstPartyFixedAdapter, build_fixed_adapter_registry
from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.registry import CapabilityRegistry


ROOT = Path(__file__).resolve().parents[1]


class FixedAdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.registry = CapabilityRegistry.from_manifest(load_manifest(ROOT / "skill_manifest.json"), repo_root=ROOT)
        cls.adapters = build_fixed_adapter_registry(cls.registry)

    def plan(self, capability_id, arguments=None):
        capability = self.registry.require(capability_id)
        adapter = self.adapters[capability_id]
        self.assertIsInstance(adapter, FirstPartyFixedAdapter)
        return adapter.build_plan(DispatchRequest(capability_id, arguments or {}, request_id="test-request"), capability, DispatchContext(mode="dry_run", artifact_dir="/tmp/fixed-adapter-test"))

    def test_core_bindings_build_complete_plans(self):
        cases = {
            "motion.move_to_pose": {"frame": "base", "xyz_m": [0.2, 0.0, 0.3], "source": "fixture", "side": "left"},
            "gripper.command": {"operation": "open", "side": "left"},
            "robot.observe_health": {},
            "state.capture_realsense": {},
            "procedure.handover_transition": {"holder_side": "left"},
        }
        for capability_id, arguments in cases.items():
            plan = self.plan(capability_id, arguments)
            self.assertTrue(plan.argv)
            self.assertEqual(plan.capability_version, "1.0.0")
            self.assertTrue(plan.adapter_id.startswith("fixed."))
            self.assertTrue(plan.planned_only)
            self.assertTrue(plan.output_parser)
            self.assertTrue(plan.error_mapping)

    def test_pose_is_one_json_argument(self):
        plan = self.plan("motion.move_to_pose", {"frame": "base", "xyz_m": [0.2, 0.0, 0.3], "rotvec_rad": [0.0, 0.1, 0.0], "source": "fixture", "side": "right"})
        self.assertIn("--right-pose", plan.argv)
        pose = plan.argv[plan.argv.index("--right-pose") + 1]
        self.assertEqual(json.loads(pose), [0.2, 0.0, 0.3, 0.0, 0.1, 0.0])

    def test_request_cannot_change_binding(self):
        for key in ("executable", "argv", "adapter", "backend", "env", "cwd", "execute", "hardware_allowed", "config_path", "output_path", "reset_script", "client_path"):
            with self.assertRaises(Exception):
                DispatchRequest.from_dict({"capability_id": "gripper.command", "arguments": {key: "injected"}})

    def test_unsupported_capability_has_no_plan(self):
        adapter = self.adapters["perception.locate_object_3d"]
        with self.assertRaises(Exception):
            adapter.build_plan(DispatchRequest("perception.locate_object_3d", {}), self.registry.require("perception.locate_object_3d"), DispatchContext(mode="dry_run"))
