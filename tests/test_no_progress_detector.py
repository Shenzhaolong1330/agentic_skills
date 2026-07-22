from __future__ import annotations

from agentic_skills_harness.execution.progress import ProgressDetector


def test_fingerprint_excludes_run_specific_values():
    first = ProgressDetector.fingerprint(frontier="n", arguments_digest="a", world_digest="w", error_code="E", goal_satisfied=False)
    second = ProgressDetector.fingerprint(frontier="n", arguments_digest="a", world_digest="w", error_code="E", goal_satisfied=False)
    assert first == second


def test_progress_detector_threshold():
    detector = ProgressDetector(limit=2)
    value = "stable"
    assert detector.observe(value) is False
    assert detector.observe(value) is False
    assert detector.observe(value) is True
    assert detector.observe("changed", progressed=True) is False
