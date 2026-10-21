import importlib.util
import io
import json
import re
import shlex
import sys
import subprocess
from contextlib import redirect_stdout
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'task_skills/demo_NO/skills/task-configurable-manipulation-demos/scripts/demo_3_checkpoint.py'
spec = importlib.util.spec_from_file_location('demo3_checkpoint', SCRIPT)
checkpoint = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checkpoint)


class Demo3CheckpointTests(unittest.TestCase):
    def test_default_stage_failure_stops_without_reset_or_next_tube(self):
        entries = [SCRIPT.with_name('demo_3_all_vials_to_rack.sh'),
                   ROOT / 'task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/run_full_pick_tube_insert_rack.sh']
        for entry in entries:
            shell = entry.read_text()
            self.assertIn('RESET_AFTER_TUBE_ERROR=0', shell)
            function = shell[shell.index('reset_after_tube_error() {'):shell.index('\nfor tube_array_index')]
            result = subprocess.run(['bash', '-c', 'RESET_AFTER_TUBE_ERROR=0\nARTIFACT_DIR=/tmp\n' + function +
                '\nreset_after_tube_error 1 grasp_handover\n'], capture_output=True, text=True)
            self.assertEqual(result.returncode, 1)
            self.assertIn('stopping without further motion', result.stderr)

    def test_motion_defaults_match_leaf_parsers(self):
        shell = SCRIPT.with_name('demo_3_all_vials_to_rack.sh').read_text()
        old = ROOT / 'task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts'
        legacy_shell = (old / 'run_full_pick_tube_insert_rack.sh').read_text()
        sys.path.insert(0, str(old))
        try:
            import locate_then_grasp_by_tail_side as selector
            for array, filename, required in (
                ('GRASP_ARGS', 'grasp_right_arm_xyz.py', ['--xyz', '[0.5,0,0.1]']),
                ('INSERTION_ARGS', 'run_manual_grip_to_insert.py', []),
            ):
                args = shlex.split(re.search(rf'{array}=\((.*?)\)', shell, re.S).group(1))
                self.assertEqual(args, shlex.split(re.search(rf'{array}=\((.*?)\)', legacy_shell, re.S).group(1)))
                spec = importlib.util.spec_from_file_location('demo3_contract_' + array, old / filename)
                module = importlib.util.module_from_spec(spec)
                sys.modules[spec.name] = module
                spec.loader.exec_module(module)
                parsed = module.build_parser().parse_args(required + args)
                self.assertTrue(parsed.continue_on_tolerance_miss)
                if array == 'GRASP_ARGS':
                    self.assertEqual(parsed.pregrasp_xy_position_tolerance_m, 0.005)
                    forwarded = selector.build_parser().parse_args(
                        ['--result-json', '/tmp/offline-tube.json'] + ['--grasp-arg=' + a for a in args])
                    self.assertEqual(forwarded.grasp_arg, args)
                    for entry in (shell, legacy_shell):
                        self.assertIn('GRASP_CMD+=("--grasp-arg=$arg")', entry)
                self.assertEqual(parsed.max_tracking_translation_bias_m, 0.05)
                self.assertEqual(parsed.max_tracking_rotation_bias_rad, 1.0)
                for key, value in vars(parsed).items():
                    if 'max_correction_iters' in key or 'max_posture_correction_iters' in key:
                        self.assertEqual(value, 15, key)
        finally:
            sys.path.pop(0)

    def call(self, *args):
        with redirect_stdout(io.StringIO()) as output:
            checkpoint.main([str(a) for a in args])
        return output.getvalue().strip()

    def prepare(self, base):
        config = base / 'config'
        config.write_text('test')
        run = base / 'run'
        signature = ['legacy-vlm', 'y', 'ccw', str(run / 'inventory.json'), str(run / 'rack.json')]
        options = ['--files', str(config), '--signature', *signature]
        self.assertEqual(self.call('prepare', run, 'new', *options), '1')
        return run, options

    def test_completed_tube_skipped_and_cached_inventory_preserved(self):
        with tempfile.TemporaryDirectory() as temp:
            run, options = self.prepare(Path(temp))
            tube = run / 'tube.json'
            tube.write_text('{}')
            (run / 'inventory.json').write_text(json.dumps({'tubes': [{'result_json': str(tube)}]}))
            (run / 'rack.json').write_text('{}')
            self.call('inventory', run)
            self.call('begin', run, 1)
            self.call('complete', run, 1)
            self.assertEqual(self.call('prepare', run, 'resume', *options), '2')
            tube.write_text('{"changed": true}')
            with self.assertRaisesRegex(ValueError, '已变化'):
                self.call('prepare', run, 'resume', *options)

    def test_inflight_failed_finished_and_changed_config_rejected(self):
        for status in ('begin', 'failed', 'finish', 'config'):
            with self.subTest(status=status), tempfile.TemporaryDirectory() as temp:
                base = Path(temp)
                run, options = self.prepare(base)
                if status == 'config':
                    (base / 'config').write_text('changed')
                else:
                    self.call(status, run, 1)
                with self.assertRaises(ValueError):
                    self.call('prepare', run, 'resume', *options)

    def test_old_run_and_nonempty_new_directory_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            run, options = self.prepare(Path(temp))
            with self.assertRaises(ValueError):
                self.call('prepare', run, 'new', *options)
            (run / 'demo3_state.json').unlink()
            with self.assertRaises(FileNotFoundError):
                self.call('prepare', run, 'resume', *options)

    def test_failure_latch_survives_later_successful_tube(self):
        with tempfile.TemporaryDirectory() as temp:
            run, options = self.prepare(Path(temp))
            self.call('begin', run, 1)
            self.call('failed', run)
            self.call('begin', run, 2)
            self.call('complete', run, 2)
            with self.assertRaises(ValueError):
                self.call('prepare', run, 'resume', *options)


if __name__ == '__main__':
    unittest.main()
