"""Offline progress-guard checks, including the recorded left-arm plateau."""

import importlib.util
import json
import math
from pathlib import Path
import unittest


SCRIPTS = (Path(__file__).resolve().parents[1] /
           "task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts")
SPEC = importlib.util.spec_from_file_location("p2p_progress_test", SCRIPTS / "p2p_progress.py")
progress = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(progress)


def report(position=0.009, rotation=0.13, right_position=0.001, right_rotation=0.002, index=0):
    return {"left_translation_error_m": position, "left_rotation_error_rad": rotation,
            "right_translation_error_m": right_position, "right_rotation_error_rad": right_rotation,
            "correction_index": index}


class P2PProgressGuardTests(unittest.TestCase):
    def guard(self):
        return progress.P2PProgressGuard(0.003, 0.03)

    def test_recorded_plateau_is_allowed_without_declaring_arrival(self):
        guard = self.guard()
        first = guard.observe(report(0.009120984753754613, 0.13145914007029108,
                                     0.0016226265662288171, 0.0023209018606169408, 2))
        second = guard.observe(report(0.009308781283858996, 0.13146681146448727,
                                      0.0022528775417438823, 0.0031347972923773024, 3))
        self.assertEqual(first["status"], "baseline")
        self.assertEqual(second["status"], "plateau")
        self.assertFalse(second["within_tolerance"])
        self.assertEqual(second["best"], first["current"])
        self.assertEqual(second["deadbands"], {"position_m": 0.0005, "rotation_rad": 0.005})
        self.assertEqual(second["tolerances"], {"position_m": 0.003, "rotation_rad": 0.03})
        json.dumps(second, allow_nan=False)

    def test_significant_position_and_rotation_growth_are_rejected(self):
        for worsening in (report(position=0.00951), report(rotation=0.13501)):
            with self.subTest(worsening=worsening):
                guard = self.guard()
                guard.observe(report())
                before = guard.best
                with self.assertRaisesRegex(RuntimeError, "p2p_error_increased"):
                    guard.observe(worsening)
                self.assertEqual(guard.best, before)

    def test_non_dominant_component_cannot_hide_behind_improved_max_score(self):
        # Rotation/tolerance still dominates even as a smaller position error grows.
        guard = self.guard()
        guard.observe(report(position=0.004, rotation=0.20))
        with self.assertRaisesRegex(RuntimeError, "left_translation_error_m"):
            guard.observe(report(position=0.0046, rotation=0.10, index=1))
        # The same protection applies to the nominally stationary partner arm.
        guard = self.guard()
        guard.observe(report(right_position=0.0031, right_rotation=0.031))
        with self.assertRaisesRegex(RuntimeError, "right_translation_error_m"):
            guard.observe(report(right_position=0.0037, index=1))
        with self.assertRaisesRegex(RuntimeError, "right_rotation_error_rad"):
            guard.observe(report(right_position=0.0031, right_rotation=0.0361, index=1))

    def test_small_cumulative_drift_is_compared_with_historical_best(self):
        guard = self.guard()
        guard.observe(report(position=0.009))
        self.assertEqual(guard.observe(report(position=0.0092, index=1))["status"], "plateau")
        self.assertEqual(guard.observe(report(position=0.0094, index=2))["status"], "plateau")
        with self.assertRaisesRegex(RuntimeError, "p2p_error_increased"):
            guard.observe(report(position=0.0096, index=3))
        self.assertEqual(guard.best["left_translation_error_m"], 0.009)

    def test_new_best_is_retained_even_after_a_small_improvement(self):
        guard = self.guard()
        guard.observe(report(position=0.009))
        self.assertEqual(guard.observe(report(position=0.0088, index=1))["status"], "plateau")
        self.assertEqual(guard.best["left_translation_error_m"], 0.0088)
        with self.assertRaisesRegex(RuntimeError, "p2p_error_increased"):
            guard.observe(report(position=0.00931, index=2))

    def test_exact_deadband_boundary_is_allowed_but_exceeding_it_is_not(self):
        guard = self.guard()
        guard.observe(report(position=0.009))
        boundary = 0.009 + 0.0005
        self.assertEqual(guard.observe(report(position=boundary, index=1))["status"], "plateau")
        with self.assertRaisesRegex(RuntimeError, "p2p_error_increased"):
            guard.observe(report(position=math.nextafter(boundary, math.inf), index=2))

    def test_within_tolerance_components_never_trigger_growth_guard(self):
        guard = self.guard()
        guard.observe(report(right_position=0.0001, right_rotation=0.0001))
        result = guard.observe(report(right_position=0.003, right_rotation=0.03, index=1))
        self.assertEqual(result["status"], "plateau")
        result = guard.observe(report(position=0.003, rotation=0.03,
                                      right_position=0.003, right_rotation=0.03, index=2))
        self.assertEqual(result["status"], "within_tolerance")
        self.assertTrue(result["within_tolerance"])
        self.assertEqual(result["outside_tolerance_components"], [])

    def test_baseline_already_arrived_and_significant_improvement_status(self):
        arrived = self.guard().observe(report(0.003, 0.03, 0.003, 0.03))
        self.assertTrue(arrived["baseline"])
        self.assertEqual(arrived["status"], "within_tolerance")
        guard = self.guard()
        guard.observe(report())
        improved = guard.observe(report(position=0.007, rotation=0.12, index=1))
        self.assertEqual(improved["status"], "improving")
        self.assertEqual(set(improved["improved_components"]),
                         {"left_translation_error_m", "left_rotation_error_rad"})
        self.assertFalse(improved["within_tolerance"])

    def test_invalid_reports_never_update_any_best_component(self):
        guard = self.guard()
        guard.observe(report())
        before = guard.best
        for key in before:
            for value in (-1, math.nan, math.inf, -math.inf, None, True, "0.01"):
                invalid = report(position=0.005, rotation=0.1)
                invalid[key] = value
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    guard.observe(invalid)
                self.assertEqual(guard.best, before)
            incomplete = report()
            del incomplete[key]
            with self.assertRaises(ValueError):
                guard.observe(incomplete)
        for index in (-1, 1.5, True, None, math.nan):
            with self.assertRaises(ValueError):
                guard.observe(report(index=index))
        with self.assertRaises(ValueError):
            guard.observe(None)
        incomplete = report()
        del incomplete["correction_index"]
        with self.assertRaises(ValueError):
            guard.observe(incomplete)
        self.assertEqual(guard.best, before)

    def test_configuration_rejects_invalid_tolerances_and_deadbands(self):
        for bad in (0, -1, math.nan, math.inf, True):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    progress.P2PProgressGuard(bad, 0.03)
                with self.assertRaises(ValueError):
                    progress.P2PProgressGuard(0.003, bad)
        for bad in (-1, math.nan, math.inf, True):
            with self.assertRaises(ValueError):
                progress.P2PProgressGuard(0.003, 0.03, position_deadband_m=bad)
            with self.assertRaises(ValueError):
                progress.P2PProgressGuard(0.003, 0.03, rotation_deadband_rad=bad)
        strict = progress.P2PProgressGuard(0.003, 0.03, 0, 0)
        strict.observe(report())
        with self.assertRaisesRegex(RuntimeError, "p2p_error_increased"):
            strict.observe(report(position=0.0090001))

    def test_returned_logging_dicts_cannot_mutate_history(self):
        guard = self.guard()
        result = guard.observe(report())
        result["best"]["left_translation_error_m"] = 100
        result["current"]["left_translation_error_m"] = 100
        result["deadbands"]["position_m"] = 100
        external_best = guard.best
        external_best["left_translation_error_m"] = 100
        with self.assertRaisesRegex(RuntimeError, "p2p_error_increased"):
            guard.observe(report(position=0.0096))


if __name__ == "__main__":
    unittest.main()
