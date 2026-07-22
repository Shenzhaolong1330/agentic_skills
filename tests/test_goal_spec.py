from __future__ import annotations

import unittest

from agentic_skills_harness.planning import GoalKind, GoalSpec


class GoalSpecTests(unittest.TestCase):
    def test_structured_goal_round_trip(self):
        value = GoalSpec.from_dict({"goal_id": "g", "goal_kind": "OBSERVATION", "description": "observe", "success_predicate": {"operator": "exists", "operands": [{"predicate": "robot.health"}]}, "required_evidence": [{"evidence_id": "e", "fact_selector": {"predicate": "robot.health"}}]})
        self.assertEqual(value.goal_kind, GoalKind.OBSERVATION)
        self.assertEqual(GoalSpec.from_dict(value.to_dict()).to_dict(), value.to_dict())

    def test_unknown_predicate_and_forbidden_field_are_rejected(self):
        with self.assertRaises(Exception):
            GoalSpec.from_dict({"goal_id": "g", "goal_kind": "OBSERVATION", "description": "x", "execute": True, "success_predicate": {"operator": "eval", "operands": []}})

    def test_physical_goal_keeps_evidence_explicit(self):
        value = GoalSpec.from_dict({"goal_id": "g", "goal_kind": "PHYSICAL_STATE_CHANGE", "description": "change", "success_predicate": {"operator": "exists", "operands": [{"predicate": "robot.pose"}]}})
        self.assertFalse(value.required_evidence)

