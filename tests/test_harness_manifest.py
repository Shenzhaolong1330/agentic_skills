import unittest
from pathlib import Path

from agentic_skills_harness.manifest import find_entrypoint, find_skill, load_manifest


ROOT = Path(__file__).resolve().parents[1]


class HarnessManifestTests(unittest.TestCase):
    def test_manifest_loads_and_contains_required_skills(self):
        manifest = load_manifest(ROOT / "skill_manifest.json")
        for name in ("atomic-state-dual-franka-reset", "procedure-robot-reset-home", "task-pick-tube-insert-rack"):
            self.assertEqual(find_skill(manifest, name)["name"], name)

    def test_reset_entrypoint_allowed_as_recovery(self):
        manifest = load_manifest(ROOT / "skill_manifest.json")
        entry = find_entrypoint(manifest, "atomic-state-dual-franka-reset", "check_and_reset_ensure")
        self.assertTrue(entry["allowed_as_recovery"])
        self.assertTrue(entry["moves_robot"])

    def test_task_live_stage_entrypoints_declare_actual_side_effects(self):
        manifest = load_manifest(ROOT / "skill_manifest.json")
        locate = find_entrypoint(manifest, "task-pick-tube-insert-rack", "task_live_locate_tube")
        grasp = find_entrypoint(manifest, "task-pick-tube-insert-rack", "task_live_grasp_handover")
        insert = find_entrypoint(manifest, "task-pick-tube-insert-rack", "task_live_insert_flow")
        self.assertTrue(locate["opens_camera"])
        self.assertFalse(locate["moves_robot"])
        self.assertTrue(grasp["moves_robot"])
        self.assertTrue(grasp["controls_gripper"])
        self.assertTrue(insert["opens_camera"])
        self.assertTrue(insert["moves_robot"])
        self.assertTrue(insert["controls_gripper"])
        self.assertFalse(locate["allowed_as_recovery"])
        self.assertFalse(grasp["allowed_as_recovery"])
        self.assertFalse(insert["allowed_as_recovery"])


if __name__ == "__main__":
    unittest.main()
