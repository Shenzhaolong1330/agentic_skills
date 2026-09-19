"""Capture readiness must gate a motion command; failures must never replay it."""
import importlib.util
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
FILE = ROOT/'task_skills/demo_NO/skills/task-configurable-manipulation-demos/scripts/trace_rpc_command.py'
spec = importlib.util.spec_from_file_location('rpc_trace_command', FILE)
trace = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trace)

class TraceTests(unittest.TestCase):
    def fake_ssh(self, ready=True):
        status = {'ready': ready, 'server_pid': 123, 'remote_trace': '/tmp/example/server_wire.txt'}
        return SimpleNamespace(stdout=io.StringIO(json.dumps(status)+'\n'),
                               communicate=Mock(return_value=('wire data\n', None)), kill=Mock())

    def test_failure_exit_is_preserved_and_command_is_executed_exactly_once(self):
        ssh = self.fake_ssh()
        with tempfile.TemporaryDirectory() as folder, \
             patch.object(trace.shutil, 'which', return_value='/usr/bin/tool'), \
             patch.object(trace.subprocess, 'Popen', return_value=ssh), \
             patch.object(trace.select, 'select', return_value=([ssh.stdout], [], [])), \
             patch.object(trace.subprocess, 'call', return_value=7) as call:
            out=Path(folder)/'capture'
            self.assertEqual(trace.run('franka', out, ['python3', 'motion.py', '--execute']), 7)
            call.assert_called_once()
            self.assertEqual(call.call_args.args[0][-3:], ['python3', 'motion.py', '--execute'])
            self.assertEqual((out/'server_wire.txt').read_text(), 'wire data\n')
            ssh.communicate.assert_called_once_with('stop\n', timeout=10)

    def test_not_ready_prevents_command_and_collects_trace(self):
        ssh = self.fake_ssh(False)
        with tempfile.TemporaryDirectory() as folder, \
             patch.object(trace.shutil, 'which', return_value='/usr/bin/tool'), \
             patch.object(trace.subprocess, 'Popen', return_value=ssh), \
             patch.object(trace.select, 'select', return_value=([ssh.stdout], [], [])), \
             patch.object(trace.subprocess, 'call') as call:
            with self.assertRaisesRegex(RuntimeError, 'not ready'):
                trace.run('franka', Path(folder)/'capture', ['motion.py'])
            call.assert_not_called()
            ssh.communicate.assert_called_once()

    def test_invalid_host_prevents_ssh_and_command(self):
        with patch.object(trace.subprocess, 'Popen') as popen, patch.object(trace.subprocess, 'call') as call:
            with self.assertRaisesRegex(ValueError, 'SSH host'):
                trace.run('-oProxyCommand=bad', '/tmp/unused', ['motion.py'])
            popen.assert_not_called()
            call.assert_not_called()

if __name__ == '__main__':
    unittest.main()
