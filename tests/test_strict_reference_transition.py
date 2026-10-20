"""Recorded phase-transition regression; no hardware or source receipt rewrite."""
import copy
from dataclasses import replace
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from test_external_tracking import adapter, g
import test_fast_tracking as fast_tests


class StrictReferenceTransitionTests(unittest.TestCase):
    def setUp(self):
        self.data = json.loads((Path(__file__).parent/"fixtures/strict_orientation_transition.json").read_text())
        self.receipt = copy.deepcopy(self.data["receipt"])
        self.observation = self.data["strict_start_report"]["state_before"]
        self.saved = {s:g.Pose(**v) for s,v in self.observation["command_target_state"]["arms"].items()}
        self.measured = {s:g.Pose(tuple(self.observation[s]["native_robot_state"]["torso_eef_pose"]["position"]),
            tuple(self.observation[s]["native_robot_state"]["torso_eef_pose"]["orientation_xyzw"])) for s in adapter.SIDES}
        self.config = g.ServoConfig(max_translation_bias_m=None,max_rotation_bias_rad=.3,
                                   rotation_tolerance_rad=.035,position_tolerance_m=.001)
        for p in (patch("socket.socket",side_effect=AssertionError("offline")),
                  patch("subprocess.Popen",side_effect=AssertionError("offline"))):
            p.start();self.addCleanup(p.stop)

    def references(self, receipt):
        return adapter.continuity.matching_references(receipt,
            session=self.receipt["server_session_id"],count=self.receipt["step_count"],
            saved=self.saved,measured=self.measured,config=self.config,
            nominal_rotation=.035,geometry=g)

    def test_recorded_legacy_receipt_cannot_invent_previous_acceptance(self):
        self.assertNotIn("accepted_rotation_tolerance_rad",self.receipt)
        self.assertNotIn("left_arm",self.references(self.receipt))
        goal=g.Pose(self.measured["left_arm"].position_m,
                    self.receipt["arms"]["left_arm"]["reference"]["quaternion_xyzw"])
        with self.assertRaisesRegex(ValueError,"bias cap"):
            adapter.tracking.TrackingServo(self.measured["left_arm"],goal,self.saved["left_arm"],
                self.config,1.,profile="fast_tracking")

    def test_previous_acceptance_binds_start_without_enlarging_strict_endpoint(self):
        # Simulate what the corrected preceding SUCCESS writer now persists.
        # This explicit replay annotation is never written into the real record.
        receipt={**self.receipt,"accepted_rotation_tolerance_rad":self.data["previous_policy"]["rotation_tolerance_rad"]}
        references=self.references(receipt)
        self.assertIn("left_arm",references)
        goal=g.Pose(self.measured["left_arm"].position_m,references["left_arm"])
        servo=adapter.tracking.TrackingServo(self.measured["left_arm"],goal,self.saved["left_arm"],
            self.config,1.,profile="fast_tracking",path_rotation_tolerance_rad=.035,
            confirmed_orientation_reference=references["left_arm"],
            confirmed_rotation_tolerance_rad=adapter.continuity.reference_tolerance(receipt,self.config,.035))
        self.assertEqual(servo.config.rotation_tolerance_rad,.035)
        self.assertEqual(servo.config.max_rotation_bias_rad,.3)
        self.assertGreater(g._error(self.measured["left_arm"],goal)["rotation_rad"],.035)
        for _ in range(100):
            update=servo.update(self.measured["left_arm"],.05)
            self.assertFalse(update.arrival_latched)
            self.assertLessEqual(update.bias["rotation_rad"],.3+4e-12)
            servo.confirm_command(update.command)

    def test_changed_command_or_exceeded_previous_acceptance_cannot_import(self):
        receipt={**self.receipt,"accepted_rotation_tolerance_rad":.2}
        receipt["step_count"]+=1
        self.assertEqual(self.references(receipt),{})
        receipt["step_count"]-=1
        receipt["accepted_rotation_tolerance_rad"]=.03
        self.assertNotIn("left_arm",self.references(receipt))
        receipt["accepted_rotation_tolerance_rad"]=.2
        self.saved["left_arm"]=g.Pose(self.saved["left_arm"].position_m,(0,0,0,1))
        self.assertEqual(self.references(receipt),{})


class SuccessfulReceiptTests(unittest.TestCase):
    setUp = fast_tests.FastTrackingTests.setUp
    run_servo = fast_tests.FastTrackingTests.run_servo
    warning_band = fast_tests.FastTrackingTests.warning_band
    def test_normal_success_records_its_own_tolerance(self):
        self.warning_band()
        result=self.run_servo(goal=(.01,0.,0.,0.,0.,0.))
        self.assertTrue(result["ok"])
        self.assertEqual(adapter.continuity.read(self.client)["accepted_rotation_tolerance_rad"],.2)


if __name__=="__main__":unittest.main()
