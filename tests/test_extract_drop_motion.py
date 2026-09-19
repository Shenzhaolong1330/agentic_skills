"""Placement regression tests with fake RPC; never connect to hardware."""
import importlib.util
import io
import json
from pathlib import Path
from contextlib import redirect_stdout
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

PATH = Path(__file__).resolve().parents[1] / 'task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/place_held_object_on_table.py'
spec = importlib.util.spec_from_file_location('extract_drop', PATH)
place = importlib.util.module_from_spec(spec)
spec.loader.exec_module(place)


class DropMotionTests(unittest.TestCase):
    def run_fake(self, xyz, *, retract='0', rise='0.005'):
        client = MagicMock()
        client.ping.return_value = True
        client.open_gripper.return_value = {'ok': True}
        rpc = MagicMock()
        rpc.DualFrankaRobotiqRpcClient.return_value = client
        poses = {'left': np.array([.5, .3, -.1, 3., 0., 0.]),
                 'right': np.array([.5, 0., -.02, 2.2, 2.2, .01])}
        moves = []
        def move(_client, **kw):
            moves.append(kw)
            poses[kw['side']] = kw['active_target'].copy()
            return {'stage': kw['stage'], 'ok': True}
        args = ['--side', 'right', '--place-xyz', json.dumps(xyz), '--execute',
                '--keep-current-rotation', '--max-release-rise-m', rise,
                '--retract-distance-m', retract, '--release-sleep-sec', '0']
        with patch.object(place, '_load_rpc_module', return_value=rpc), \
             patch.object(place, '_pose_from_observation', side_effect=lambda r, o, s: poses[s].copy()), \
             patch.object(place, '_move', side_effect=move), redirect_stdout(io.StringIO()):
            try:
                result = place.main(args)
            except ValueError:
                self.assertEqual(moves, [])
                client.open_gripper.assert_not_called()
                client.close.assert_called_once()
                raise
        return result, moves, client

    def test_no_extra_rise_or_rotation_after_extraction(self):
        result, moves, client = self.run_fake([.5, -.20, -.13])
        self.assertEqual(result, 0)
        self.assertEqual([m['stage'] for m in moves], ['move_xy_keep_current_z', 'move_to_release_xyz'])
        np.testing.assert_allclose(moves[0]['active_target'], [.5, -.20, -.02, 2.2, 2.2, .01])
        np.testing.assert_allclose(moves[1]['active_target'], [.5, -.20, -.13, 2.2, 2.2, .01])
        client.open_gripper.assert_called_once_with('right_arm')
        client.go_home.assert_not_called()

    def test_bad_release_height_fails_before_xy_motion_and_release(self):
        with self.assertRaisesRegex(ValueError, 'Release target would rise'):
            self.run_fake([.5, -.20, .20])

    def test_positive_retract_still_available_to_existing_callers(self):
        _, moves, _ = self.run_fake([.5, -.20, -.13], retract='0.1')
        self.assertEqual(moves[-1]['stage'], 'retract_up')
        self.assertAlmostEqual(moves[-1]['active_target'][2], -.03)


if __name__ == '__main__':
    unittest.main()
