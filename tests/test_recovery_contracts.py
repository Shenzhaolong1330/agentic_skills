from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from agentic_skills_harness.contracts.enums import ErrorCode
from agentic_skills_harness.dispatch.error_mapping import make_error
from agentic_skills_harness.recovery import (
    HeldObjectEvidence,
    PlanLineage,
    RecoveryContext,
    RecoveryDisposition,
    RecoverySelector,
    RecoveryStrategy,
    RecoveryStrategyId,
    build_first_party_recovery_policy_registry,
    validate_replan_monotonicity,
)
from agentic_skills_harness.recovery.lineage import append_lineage


def context(code: ErrorCode, *, held: HeldObjectEvidence = HeldObjectEvidence.UNKNOWN, remaining: dict | None = None, mode: str = "mock") -> RecoveryContext:
    error = make_error(code, code.value, source="test")
    return RecoveryContext(
        "run", "graph", "goal", "plan0", "failed", "ACT", "motion.move_to_pose", error,
        {"revision": 1, "facts": []}, held, (), (), (),
        remaining or {"recovery_actions": 2, "same_error_retries": 1, "replans": 1},
        {"envelope_id": "e", "target_mode": mode, "allowed_capabilities": ["robot.recover_reset_home"], "forbidden_capabilities": [], "risk_ceiling": "HIGH_RISK", "max_recovery_actions": 2, "max_same_error_retries": 1, "max_replans": 1},
        mode,
    )


class RecoveryContractTests(unittest.TestCase):
    def test_strategy_contract_is_json_serializable_and_declarative(self):
        strategy = RecoveryStrategy(RecoveryStrategyId.REOBSERVE, RecoveryDisposition.RECOVERY_SUBGRAPH, priority=3, template_id="reobserve")
        self.assertEqual(strategy.to_dict()["strategy_id"], "REOBSERVE")
        with self.assertRaises(Exception):
            RecoveryStrategy.from_dict({"strategy_id": "REOBSERVE", "disposition": "RECOVERY_SUBGRAPH", "forbidden_when": {"expression": "x"}})

    def test_policy_registry_is_deterministic(self):
        registry = build_first_party_recovery_policy_registry()
        self.assertEqual([item.strategy_id.value for item in registry.list()], [item.strategy_id.value for item in sorted(registry.list(), key=lambda x: (-x.priority, x.strategy_id.value))])
        with self.assertRaises(PermissionError):
            registry.register(RecoveryStrategy(RecoveryStrategyId.ABORT, RecoveryDisposition.TERMINATE))

    def test_perception_never_selects_reset(self):
        decision = RecoverySelector(build_first_party_recovery_policy_registry()).select(context(ErrorCode.PERCEPTION_NOT_FOUND))
        self.assertNotIn(decision.selected_strategy.strategy_id if decision.selected_strategy else None, {RecoveryStrategyId.RESET_HOME, RecoveryStrategyId.CONTROLLER_RECOVERY})

    def test_estop_has_zero_automatic_recovery(self):
        decision = RecoverySelector(build_first_party_recovery_policy_registry()).select(context(ErrorCode.ROBOT_ESTOP_OR_UNSAFE))
        self.assertIn(decision.selected_strategy.strategy_id, {RecoveryStrategyId.REQUEST_HUMAN, RecoveryStrategyId.ABORT})
        self.assertEqual([item.strategy.strategy_id for item in decision.eligible_candidates if item.strategy.strategy_id not in {RecoveryStrategyId.REQUEST_HUMAN, RecoveryStrategyId.ABORT}], [])

    def test_unknown_holding_rejects_reset(self):
        decision = RecoverySelector(build_first_party_recovery_policy_registry()).select(context(ErrorCode.ROBOT_FAULT, held=HeldObjectEvidence.UNKNOWN))
        self.assertNotEqual(decision.selected_strategy.strategy_id if decision.selected_strategy else None, RecoveryStrategyId.RESET_HOME)
        self.assertNotIn(RecoveryStrategyId.RESET_HOME, [item.strategy.strategy_id for item in decision.eligible_candidates])

    def test_confirmed_no_holding_can_select_controller_recovery(self):
        decision = RecoverySelector(build_first_party_recovery_policy_registry()).select(context(ErrorCode.ROBOT_FAULT, held=HeldObjectEvidence.NONE_CONFIRMED))
        self.assertEqual(decision.selected_strategy.strategy_id, RecoveryStrategyId.CONTROLLER_RECOVERY)

    def test_budget_exhaustion_is_terminal(self):
        decision = RecoverySelector(build_first_party_recovery_policy_registry()).select(context(ErrorCode.NO_PROGRESS, remaining={"recovery_actions": 0, "same_error_retries": 0, "replans": 0}))
        self.assertEqual(decision.selected_strategy.strategy_id, RecoveryStrategyId.REQUEST_HUMAN)

    def test_lineage_requires_parent_and_changes_hash(self):
        lineage = PlanLineage("root")
        lineage = append_lineage(lineage, plan_hash="plan1", reason="alternate", failure_node_id="node", error_code="PERCEPTION_NOT_FOUND", recovery_strategy_id="ALTERNATE_CANDIDATE", remaining_budget={"replans": 0}, world_state={})
        self.assertEqual(lineage.current_plan_hash, "plan1")
        with self.assertRaises(Exception):
            PlanLineage.from_dict({"root_plan_hash": "root", "entries": [{"lineage_index": 0, "plan_hash": "plan1", "parent_plan_hash": "wrong", "reason": "x", "failure_node_id": "n", "error_code": "x", "recovery_strategy_id": "x"}]})

    def test_replan_monotonicity_rejects_expansion(self):
        original = context(ErrorCode.MOTION_IK_INFEASIBLE).original_envelope
        expanded = dict(original, allowed_capabilities=["robot.recover_reset_home", "motion.move_to_pose"], risk_ceiling="HIGH_RISK", max_replans=2)
        errors = validate_replan_monotonicity(original, expanded, original_plan_hash="a", replacement_plan_hash="b")
        self.assertIn("allowed capabilities expanded", errors)

    def test_replan_noop_is_not_progress(self):
        envelope = context(ErrorCode.MOTION_IK_INFEASIBLE).original_envelope
        self.assertIn("no-op replan has the same plan hash", validate_replan_monotonicity(envelope, envelope, original_plan_hash="same", replacement_plan_hash="same"))


if __name__ == "__main__":
    unittest.main()
