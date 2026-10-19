import importlib.util
from pathlib import Path
import unittest
import subprocess
import sys
import json

SCRIPT = Path(__file__).resolve().parents[1] / 'task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/insertion_depth_contract.py'
spec = importlib.util.spec_from_file_location('insertion_depth_contract_test', SCRIPT)
contract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(contract)


class InsertionDepthContractTests(unittest.TestCase):
    def test_isolated_cli_dry_run_records_derived_minimum_without_rpc(self):
        result = subprocess.run([sys.executable,'-I',str(SCRIPT.with_name('insert_down_from_current_pose.py')),
            '--monitor-force-during-insert','--insert-depth-m','0.055','--compact'],
            capture_output=True,text=True,timeout=20)
        self.assertEqual(result.returncode,0,result.stderr)
        report=json.loads(result.stdout)
        self.assertTrue(report['planned_only'])
        self.assertFalse(report['execute'])
        self.assertAlmostEqual(report['insert_force_guard']['seated_min_depth_m'],.047)

    def test_historical_early_contacts_are_below_default_seating_minimum(self):
        minimum = contract.seated_minimum_depth(.055, .008)
        self.assertAlmostEqual(minimum, .047)
        # Source: overnight 06:12 recovery and 11:37 action-019 receipt.
        for contact_depth in (.00507, .029236357386137404, .04510):
            self.assertLess(contact_depth, minimum)
        self.assertGreater(.052959, minimum)

    def test_default_scales_with_shorter_task_instead_of_fixed_three_mm(self):
        self.assertAlmostEqual(contract.seated_minimum_depth(.01,.008),.0085)
        self.assertAlmostEqual(contract.seated_minimum_depth(.055,.002),.053)

    def test_explicit_calibrated_minimum_is_preserved_and_invalid_contracts_rejected(self):
        self.assertEqual(contract.seated_minimum_depth(.055,.008,.003),.003)
        for depth, tolerance, explicit in ((0,.008,None),(.055,float('nan'),None),
                                          (.055,.008,.06),(.055,.008,True),(.055,.008,-.01)):
            with self.subTest(values=(depth,tolerance,explicit)), self.assertRaises(ValueError):
                contract.seated_minimum_depth(depth,tolerance,explicit)
