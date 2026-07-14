import os
import unittest
from pathlib import Path

from agentic_skills_harness.hardware_gate import HardwareGate
from agentic_skills_harness.manifest import find_entrypoint, load_manifest
from agentic_skills_harness.types import SkillContext, SkillMode


ROOT = Path(__file__).resolve().parents[1]


class HardwareGateTests(unittest.TestCase):
    def setUp(self):
        self.manifest = load_manifest(ROOT / "skill_manifest.json")
        self.entry = find_entrypoint(self.manifest, "atomic-state-dual-franka-reset", "check_and_reset_ensure")
        self.gate = HardwareGate(self.manifest)

    def context(self, mode=SkillMode.MOCK, execute=False, allowed=False, token=None):
        return SkillContext("test", mode=mode, execute=execute, hardware_allowed=allowed, operator_token=token, operator_token_present=bool(token))

    def test_non_live_modes_plan_only(self):
        for mode in (SkillMode.MOCK, SkillMode.DRY_RUN, SkillMode.FROM_ARTIFACTS):
            decision = self.gate.evaluate(self.context(mode=mode), self.entry, recovery=True)
            self.assertFalse(decision.allowed)
            self.assertTrue(decision.planned_only)

    def test_live_requires_execute_allowed_and_token(self):
        self.assertFalse(self.gate.evaluate(self.context(SkillMode.LIVE, execute=False, allowed=True, token="x"), self.entry, recovery=True).allowed)
        self.assertFalse(self.gate.evaluate(self.context(SkillMode.LIVE, execute=True, allowed=False, token="x"), self.entry, recovery=True).allowed)
        self.assertFalse(self.gate.evaluate(self.context(SkillMode.LIVE, execute=True, allowed=True, token=None), self.entry, recovery=True).allowed)
        old = os.environ.get("AGENTIC_SKILLS_HARDWARE_TOKEN")
        os.environ["AGENTIC_SKILLS_HARDWARE_TOKEN"] = "secret"
        try:
            self.assertFalse(self.gate.evaluate(self.context(SkillMode.LIVE, execute=True, allowed=True, token="bad"), self.entry, recovery=True).allowed)
            self.assertTrue(self.gate.evaluate(self.context(SkillMode.LIVE, execute=True, allowed=True, token="secret"), self.entry, recovery=True).allowed)
        finally:
            if old is None:
                os.environ.pop("AGENTIC_SKILLS_HARDWARE_TOKEN", None)
            else:
                os.environ["AGENTIC_SKILLS_HARDWARE_TOKEN"] = old


if __name__ == "__main__":
    unittest.main()
