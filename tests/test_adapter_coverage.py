from __future__ import annotations

from pathlib import Path
import unittest

from agentic_skills_harness.dispatch.coverage import audit_adapter_coverage
from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.registry import CapabilityRegistry


ROOT = Path(__file__).resolve().parents[1]


class AdapterCoverageTests(unittest.TestCase):
    def test_manifest_coverage_is_complete(self):
        registry = CapabilityRegistry.from_manifest(load_manifest(ROOT / "skill_manifest.json"), repo_root=ROOT)
        result = audit_adapter_coverage(registry)
        self.assertEqual(result["total_capabilities"], 18)
        self.assertEqual(result["reviewed_capabilities"], 18)
        self.assertEqual(result["unreviewed"], 0)
        self.assertGreaterEqual(result["core_capabilities_with_plan"], 5)
        self.assertEqual(result["live_hardware_supported"], 0)
        self.assertEqual(result["duplicate_adapter_bindings"], 0)
