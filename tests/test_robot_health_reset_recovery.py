import unittest
from pathlib import Path

from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.reset_recovery import ResetRecoveryController
from agentic_skills_harness.robot_health import RobotHealthMonitor
from agentic_skills_harness.types import HeldObjectState, ResetOutcome, RobotHealthState, RobotHealthStatus, SkillContext, SkillMode


ROOT = Path(__file__).resolve().parents[1]


class RobotHealthResetRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.manifest = load_manifest(ROOT / "skill_manifest.json")

    def test_mock_health_ready_and_abnormal(self):
        monitor = RobotHealthMonitor(self.manifest, ROOT)
        ready = monitor.check_health(SkillContext("ready", mode=SkillMode.MOCK), source="test")
        self.assertEqual(ready.state, RobotHealthState.READY)
        abnormal = monitor.check_health(SkillContext("bad", mode=SkillMode.MOCK, mock_robot_health="abnormal"), source="test")
        self.assertEqual(abnormal.state, RobotHealthState.ABNORMAL)
        self.assertTrue(abnormal.requires_reset)

    def test_dry_run_health_does_not_connect_rpc(self):
        monitor = RobotHealthMonitor(self.manifest, ROOT)
        health = monitor.check_health(SkillContext("dry", mode=SkillMode.DRY_RUN), source="test")
        self.assertEqual(health.state, RobotHealthState.UNKNOWN)
        self.assertIn("planned_command", health.raw_result)

    def test_mock_reset_ok(self):
        controller = ResetRecoveryController(self.manifest, ROOT)
        context = SkillContext("reset", mode=SkillMode.MOCK)
        result = controller.perform_auto_reset(context, reason="test", held_object_state=HeldObjectState.NONE)
        self.assertEqual(result.outcome, ResetOutcome.MOCK_RESET_OK)
        self.assertEqual(result.after_health.state, RobotHealthState.READY)

    def test_dry_run_reset_planned_only(self):
        controller = ResetRecoveryController(self.manifest, ROOT)
        context = SkillContext("reset", mode=SkillMode.DRY_RUN)
        result = controller.perform_auto_reset(context, reason="test", held_object_state=HeldObjectState.NONE)
        self.assertEqual(result.outcome, ResetOutcome.PLANNED_ONLY)
        self.assertTrue(result.command)

    def test_live_gate_denied(self):
        controller = ResetRecoveryController(self.manifest, ROOT)
        context = SkillContext("reset", mode=SkillMode.LIVE, execute=False, hardware_allowed=False)
        result = controller.perform_auto_reset(context, reason="test", held_object_state=HeldObjectState.NONE)
        self.assertEqual(result.outcome, ResetOutcome.RESET_DENIED_BY_GATE)

    def test_estop_never_auto_recovers(self):
        controller = ResetRecoveryController(self.manifest, ROOT)
        context = SkillContext("estop", mode=SkillMode.LIVE, execute=True, hardware_allowed=True)
        health = RobotHealthStatus(ok=False, state=RobotHealthState.ESTOP_OR_UNSAFE, source="test")
        self.assertFalse(controller.should_reset(health, None, context))
        self.assertFalse(controller.should_reset(None, {"abnormal_robot_state_detected": True, "state": "ESTOP_OR_UNSAFE"}, context))
        result = controller.perform_auto_reset(
            context,
            reason="test",
            held_object_state=HeldObjectState.NONE,
            before_health=health,
        )
        self.assertEqual(result.outcome, ResetOutcome.RESET_DENIED_BY_GATE)
        self.assertFalse(result.attempted)

    def test_attempts_exceeded_and_held_object_policy(self):
        controller = ResetRecoveryController(self.manifest, ROOT)
        exceeded = SkillContext("reset", mode=SkillMode.MOCK, reset_attempt_count=1, max_auto_reset_attempts=1)
        result = controller.perform_auto_reset(exceeded, reason="test", held_object_state=HeldObjectState.NONE)
        self.assertEqual(result.outcome, ResetOutcome.RESET_EXCEEDED_MAX_ATTEMPTS)
        held = SkillContext("held", mode=SkillMode.MOCK)
        held_result = controller.perform_auto_reset(held, reason="test", held_object_state=HeldObjectState.TUBE_HEAD_HOLDER_ARM)
        self.assertTrue(held_result.held_object_risk)
        self.assertTrue(held_result.aborted_after_reset)


if __name__ == "__main__":
    unittest.main()
