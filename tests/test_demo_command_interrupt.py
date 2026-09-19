"""Local child-process lifecycle tests. No robot or camera commands."""
import importlib.util
import io
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from unittest.mock import MagicMock, patch

import yaml

SKILL = Path(__file__).resolve().parents[1] / 'task_skills/demo_NO/skills/task-configurable-manipulation-demos'
spec = importlib.util.spec_from_file_location('interrupt_demo', SKILL / 'scripts/demo_common_runner.py')
demo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(demo)


class CommandInterruptTests(unittest.TestCase):
    def test_successful_child_stdout_and_stderr_are_persisted(self):
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp)
            result = demo.run_logged_command(
                [sys.executable, '-c', "import sys; print('target before motion'); print('diagnostic', file=sys.stderr)"],
                out, out/'stdout', out/'stderr')
            self.assertEqual(result.returncode, 0)
            self.assertIn('target before motion', (out/'stdout').read_text())
            self.assertIn('diagnostic', (out/'stderr').read_text())

    def test_interrupt_preserves_logs_and_terminates_child_group_without_robot_call(self):
        with tempfile.TemporaryDirectory() as temp:
            config = yaml.safe_load((SKILL/'config/demo_4_vial_extract.yaml').read_text())
            task = demo.Demo(config, SKILL/'config/demo_4_vial_extract.yaml', temp, 'live')
            process = MagicMock(pid=987654)
            process.wait.side_effect = [KeyboardInterrupt(), subprocess.TimeoutExpired('fake', 3), 0]
            def start(*args, **kwargs):
                self.assertTrue(kwargs['start_new_session'])
                self.assertEqual(kwargs['env']['PYTHONUNBUFFERED'], '1')
                kwargs['stdout'].write('partial target trace\n')
                kwargs['stdout'].flush()
                kwargs['stderr'].write('partial correction trace\n')
                kwargs['stderr'].flush()
                return process
            with patch.object(demo.subprocess, 'Popen', side_effect=start) as popen, \
                 patch.object(demo.os, 'killpg') as killpg, redirect_stdout(io.StringIO()), self.assertRaises(KeyboardInterrupt):
                task.command('extract_observe_1', ['fake-no-robot'])
            self.assertEqual(popen.call_count, 1)
            self.assertEqual([call.args for call in killpg.call_args_list],
                             [(987654, signal.SIGTERM), (987654, signal.SIGKILL)])
            text = (Path(temp)/'demo.log').read_text()
            self.assertIn('partial target trace', text)
            self.assertIn('partial correction trace', text)
            self.assertIn('robot stop unconfirmed', text)


if __name__ == '__main__':
    unittest.main()
