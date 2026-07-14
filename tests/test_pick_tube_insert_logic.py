import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT / "task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts"
MOCK_DIR = ROOT / "task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/mock"
sys.path.insert(0, str(SCRIPT_DIR))

import task_pick_tube_insert_rack_logic as logic


class PickTubeInsertLogicTests(unittest.TestCase):
    def test_extract_tube_geometry_and_select_arm(self):
        geom = logic.extract_tube_geometry(MOCK_DIR / "mock_tube_detection.json")
        self.assertTrue(geom["ok"])
        selected = logic.select_arm_for_tail_side(geom)
        self.assertTrue(selected["ok"])
        self.assertIn(selected["selected_arm"], ("left", "right"))
        self.assertNotEqual(selected["selected_arm"], selected["opposite_arm"])

    def test_select_highest_confidence_hole(self):
        candidates = logic.wrap_hole_candidates(MOCK_DIR / "mock_hole_detection.json")
        selected = logic.select_highest_confidence_hole(candidates, min_confidence=0.0)
        self.assertTrue(selected["ok"])
        self.assertEqual(selected["candidate"]["label"], "hole_B")
        self.assertFalse(logic.select_highest_confidence_hole(candidates, min_confidence=0.99)["ok"])


if __name__ == "__main__":
    unittest.main()
