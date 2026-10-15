"""Placement abort behavior with a fake RPC; never connects to hardware."""
import importlib.util
import os
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
PATH = ROOT / 'task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/place_held_object_on_table.py'
spec = importlib.util.spec_from_file_location('continuous_placement_test', PATH)
place = importlib.util.module_from_spec(spec)
spec.loader.exec_module(place)


class ContinuousPlacementTests(unittest.TestCase):
    def test_failed_servo_never_releases_or_returns_home(self):
        client = Mock()
        args = SimpleNamespace(execute=True, release_on_motion_failure=True,
                               go_home_after_motion_failure=True)
        with patch.dict(os.environ, FRANKA_P2P_CONTROLLER='continuous_servo'), \
                patch.object(place, '_move', side_effect=RuntimeError('unknown RPC outcome')):
            with self.assertRaisesRegex(RuntimeError, 'unknown RPC'):
                place._move_with_failure_release(client, side='left', active_target=None,
                    current_left=None, current_right=None, args=args, stage='place', report={})
        client.open_gripper.assert_not_called()
        client.go_home.assert_not_called()

    def test_incompatible_flags_fail_before_client_creation(self):
        for option in ('--continue-on-tolerance-miss', '--release-on-motion-failure',
                       '--go-home-after-motion-failure'):
            with self.subTest(option=option), \
                    patch.dict(os.environ, FRANKA_P2P_CONTROLLER='continuous_servo'), \
                    patch.object(place, '_load_rpc_module') as load:
                with self.assertRaisesRegex(ValueError, 'Continuous servo'):
                    place.main(['--side', 'left', '--place-xyz', '[0.5,0.2,0.1]',
                                '--execute', option])
                load.assert_not_called()

    def test_invalid_mode_fails_before_client_creation(self):
        with patch.dict(os.environ, FRANKA_P2P_CONTROLLER='typo'), \
                patch.object(place, '_load_rpc_module') as load:
            with self.assertRaisesRegex(ValueError, 'Invalid FRANKA'):
                place.main(['--side', 'left', '--place-xyz', '[0.5,0.2,0.1]', '--execute'])
            load.assert_not_called()


if __name__ == '__main__':
    unittest.main()
