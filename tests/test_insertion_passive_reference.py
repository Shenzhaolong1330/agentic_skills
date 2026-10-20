"""Legacy RPC omission semantics; offline transport-contract regression."""
import copy
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
    "task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts"))
import insert_down_from_current_pose as module


class LegacyTargets:
    """Omitted motion reanchors; explicit zero preserves the saved reference.

    Protocol evidence: robot_agent_lab/precision_research/README.md and the
    source run web-20261020-181501-7b15dea3, counts 10766 -> 11267.
    This models the command store, not the physical response of either arm.
    """
    def __init__(self, passive):
        self.passive = passive
        self.saved = [0.5732707613218686, 0.16546773595589825, 0.03924888671590531]
        self.measured = [0.5628513693809509, 0.14259538152873139, 0.023665484414606218]
        self.actions = []

    def step(self, action):
        self.actions.append(copy.deepcopy(action))
        motion = action.get(self.passive, {}).get("motion")
        if motion is None:
            self.saved = list(self.measured)
        else:
            self.saved = [p + d for p, d in zip(self.saved, motion["translation"])]
        return {"ok": True, "step_count": 10766 + len(self.actions)}


class InsertionPassiveReferenceTests(unittest.TestCase):
    def check_side(self, side):
        passive = "left_arm" if side == "right" else "right_arm"
        client = LegacyTargets(passive)
        saved = list(client.saved)
        start = np.array([.43, -.04, -.09, 0., 0., 0.])
        samples = []
        for depth, force in ((.001, 0.), (.050, 7.)):
            pose = start.copy()
            pose[2] -= depth
            samples.append(({side + "_arm": {"robot_state": {
                "wrench": {"force": [0., 0., force]}}}}, {side: pose}))
        args = module.build_parser().parse_args([
            "--position-tolerance-m", ".003", "--rotation-tolerance-rad", ".1",
            "--settle-time-sec", "0", "--seated-min-depth-m", ".047"])
        with patch.object(module.skill, "read_ee_poses", side_effect=samples), \
                patch.object(module.time, "sleep"), \
                patch("socket.socket", side_effect=AssertionError("network forbidden")), \
                patch("subprocess.Popen", side_effect=AssertionError("process forbidden")):
            result = module.guarded_vertical_insert(
                client=client, side=side, start_pose=start,
                base_force=np.zeros(3), depth_m=.055, args=args)
        self.assertTrue(result["ok"])
        self.assertEqual(len(client.actions), 2)
        self.assertEqual(client.saved, saved, "Insertion changed the passive arm's saved target")
        for action in client.actions:
            self.assertEqual(set(action), {"left_arm", "right_arm"})
            self.assertEqual(action[passive], {"motion": {
                "translation": [0.] * 3, "rotation_rotvec": [0.] * 3}})
            self.assertLess(action[side + "_arm"]["motion"]["translation"][2], 0.)
            self.assertTrue(all(set(command) == {"motion"} for command in action.values()))

    def test_right_insertion_preserves_left_reference(self):
        self.check_side("right")

    def test_left_insertion_preserves_right_reference(self):
        self.check_side("left")


if __name__ == "__main__":
    unittest.main()
