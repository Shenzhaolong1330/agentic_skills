import unittest
from pathlib import Path

from agentic_skills_harness.hardware_gate import HardwareGate
from agentic_skills_harness.manifest import find_entrypoint, load_manifest
from agentic_skills_harness.types import SkillContext, SkillMode


ROOT = Path(__file__).resolve().parents[1]


class HardwareGateTests(unittest.TestCase):
    def setUp(self):
        self.manifest = load_manifest(ROOT / "skill_manifest.json")
        self.gate = HardwareGate(self.manifest)

    @staticmethod
    def context(mode, *, execute=False, hardware_allowed=False):
        return SkillContext(
            "test",
            mode=mode,
            execute=execute,
            hardware_allowed=hardware_allowed,
        )

    @staticmethod
    def entry(**overrides):
        entrypoint = {
            "name": "test-entrypoint",
            "requires_hardware": True,
            "opens_camera": False,
            "connects_robot_rpc": False,
            "moves_robot": False,
            "controls_gripper": False,
            "allowed_as_recovery": False,
        }
        entrypoint.update(overrides)
        return entrypoint

    def test_non_live_hardware_is_always_plan_only(self):
        entrypoint = self.entry(moves_robot=True)
        for mode, hardware_allowed, execute in (
            (SkillMode.MOCK, False, False),
            (SkillMode.DRY_RUN, True, True),
            (SkillMode.FROM_ARTIFACTS, True, True),
        ):
            with self.subTest(mode=mode):
                decision = self.gate.evaluate(
                    self.context(mode, hardware_allowed=hardware_allowed, execute=execute),
                    entrypoint,
                )
                self.assertFalse(decision.allowed)
                self.assertTrue(decision.planned_only)
                self.assertEqual(decision.reason, "non_live_mode_planned_only")

    def test_live_read_only_camera_requires_only_hardware_authorization(self):
        entrypoint = self.entry(opens_camera=True)
        denied = self.gate.evaluate(self.context(SkillMode.LIVE), entrypoint)
        self.assertFalse(denied.allowed)
        self.assertEqual(denied.reason, "hardware_allowed_required")
        allowed = self.gate.evaluate(
            self.context(SkillMode.LIVE, hardware_allowed=True),
            entrypoint,
        )
        self.assertTrue(allowed.allowed)
        self.assertFalse(allowed.execute_checked)

    def test_live_read_only_robot_status_requires_only_hardware_authorization(self):
        entrypoint = self.entry(connects_robot_rpc=True)
        decision = self.gate.evaluate(
            self.context(SkillMode.LIVE, hardware_allowed=True),
            entrypoint,
        )
        self.assertTrue(decision.allowed)

    def test_live_motion_requires_both_authorization_bits(self):
        entrypoint = self.entry(moves_robot=True)
        self.assertEqual(
            self.gate.evaluate(
                self.context(SkillMode.LIVE, execute=True), entrypoint
            ).reason,
            "hardware_allowed_required",
        )
        denied = self.gate.evaluate(
            self.context(SkillMode.LIVE, hardware_allowed=True), entrypoint
        )
        self.assertFalse(denied.allowed)
        self.assertEqual(denied.reason, "execute_required_for_side_effects")
        allowed = self.gate.evaluate(
            self.context(SkillMode.LIVE, hardware_allowed=True, execute=True),
            entrypoint,
        )
        self.assertTrue(allowed.allowed)

    def test_live_gripper_requires_execute(self):
        decision = self.gate.evaluate(
            self.context(SkillMode.LIVE, hardware_allowed=True),
            self.entry(controls_gripper=True),
        )
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.reason, "execute_required_for_side_effects")

    def test_recovery_requires_manifest_permission(self):
        denied = self.gate.evaluate(
            self.context(SkillMode.LIVE, hardware_allowed=True, execute=True),
            self.entry(moves_robot=True),
            recovery=True,
        )
        self.assertFalse(denied.allowed)
        self.assertEqual(denied.reason, "recovery_not_allowed")
        allowed = self.gate.evaluate(
            self.context(SkillMode.LIVE, hardware_allowed=True, execute=True),
            self.entry(moves_robot=True, allowed_as_recovery=True),
            recovery=True,
        )
        self.assertTrue(allowed.allowed)

    def test_manifest_read_only_reset_status_is_not_a_motion_capability(self):
        entrypoint = find_entrypoint(
            self.manifest,
            "atomic-state-dual-franka-reset",
            "check_and_reset_status",
        )
        decision = self.gate.evaluate(
            self.context(SkillMode.LIVE, hardware_allowed=True),
            entrypoint,
            recovery=True,
        )
        self.assertTrue(decision.allowed)

    def test_context_defaults_and_live_does_not_auto_authorize(self):
        default = SkillContext("defaults")
        self.assertFalse(default.hardware_allowed)
        self.assertFalse(default.execute)
        for context in (
            self.context(SkillMode.LIVE, execute=True),
            self.context(SkillMode.LIVE, hardware_allowed=False),
        ):
            self.assertFalse(
                self.gate.evaluate(context, self.entry(opens_camera=True)).allowed
            )


if __name__ == "__main__":
    unittest.main()
