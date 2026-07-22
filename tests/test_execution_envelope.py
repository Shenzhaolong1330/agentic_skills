from __future__ import annotations

import unittest

from agentic_skills_harness.planning import ExecutionEnvelope, TargetMode, WorkspaceConstraint


class ExecutionEnvelopeTests(unittest.TestCase):
    def test_modes_and_bounds_are_typed(self):
        envelope = ExecutionEnvelope("e", "dry_run", risk_ceiling="MOTION", workspace_constraints=(WorkspaceConstraint("w", "base", (0, -1, 0), (1, 1, 1)),))
        self.assertEqual(envelope.target_mode, TargetMode.DRY_RUN)
        self.assertEqual(envelope.workspace_constraints[0].frame, "base")

    def test_invalid_workspace_and_authorization_fields_rejected(self):
        with self.assertRaises(Exception):
            WorkspaceConstraint("w", "base", (0, 0, 0), (0, 1, 1))
        with self.assertRaises(Exception):
            ExecutionEnvelope.from_dict({"envelope_id": "e", "target_mode": "dry_run", "risk_ceiling": "MOTION", "hardware_allowed": True})
